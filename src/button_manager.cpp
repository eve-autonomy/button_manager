// Copyright 2020 eve autonomy inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License


#include <chrono>
#include <memory>
#include <utility>
#include "button_manager/button_manager.hpp"

namespace button_manager
{

ButtonManager::ButtonManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("button_manager", options),
  updater_(this)
{
  /* Publisher is set to 'volatile' durability QoS.
      So subscriber needs to be set to 'volatile' for ensuring compatibility. */
  sub_dio_info_ = this->create_subscription<dio_ros_driver::msg::DIOPort>(
    "button_in",
    rclcpp::QoS{3},
    std::bind(&ButtonManager::Callback, this, std::placeholders::_1)
  );

  pub_button_ = this->create_publisher<autoware_state_machine_msgs::msg::VehicleButton>(
    "button_out",
    rclcpp::QoS{1}.transient_local());

  pub_button_pressed_ = this->create_publisher<std_msgs::msg::Bool>(
    "button_pressed_out",
    rclcpp::QoS{1}.transient_local());

  not_pressed_period_threshold_ = this->declare_parameter<double>(
    "not_pressed_period_threshold", 1.0);
  pressed_period_threshold_ = this->declare_parameter<double>("pressed_period_threshold", 1.0);
  not_pressed_period_threshold_after_pressed_ = this->declare_parameter<double>(
    "not_pressed_period_threshold_after_pressed", 1.0);
  max_allowable_period_of_pressing_ = this->declare_parameter<double>(
    "max_allowable_period_of_pressing", 1.0);
  active_polarity_ = this->declare_parameter<bool>("active_polarity", false);

  // ---- abnormal input detection (chattering / stuck while driving) ----
  enable_abnormal_input_detection_ = this->declare_parameter<bool>(
    "enable_abnormal_input_detection", true);
  AbnormalInputParams detector_params;
  detector_params.stuck_detect_sec = this->declare_parameter<double>("stuck_detect_sec");
  detector_params.mrm_after_press_sec = this->declare_parameter<double>("mrm_after_press_sec");
  detector_params.chatter_short_press_max_sec =
    this->declare_parameter<double>("chatter_short_press_max_sec");
  detector_params.chatter_count_threshold =
    this->declare_parameter<int>("chatter_count_threshold");
  detector_params.chatter_window_sec = this->declare_parameter<double>("chatter_window_sec");
  detector_params.chatter_clear_sec = this->declare_parameter<double>("chatter_clear_sec");
  abnormal_input_detector_ = std::make_unique<AbnormalInputDetector>(detector_params);
  const double detection_period = this->declare_parameter<double>("abnormal_detection_period");
  const double diagnostics_period = this->declare_parameter<double>("diagnostics_period");

  sub_state_machine_ = this->create_subscription<autoware_state_machine_msgs::msg::StateMachine>(
    "state_machine_in", rclcpp::QoS{1}.transient_local(),
    std::bind(&ButtonManager::onStateMachine, this, std::placeholders::_1));

  // The diagnostics are always published (level OK while the detection is disabled)
  // because the system_error_monitor treats a missing diagnostics as an error.
  updater_.setHardwareID(std::string(get_name()));
  updater_.add("button_input_monitoring", this, &ButtonManager::checkButtonInput);
  updater_.setPeriod(diagnostics_period);

  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<double>(detection_period),
    std::bind(&ButtonManager::onTimer, this));

  base_time_ = this->now();
  publishPressed(false);
}

ButtonManager::~ButtonManager()
{
}

void ButtonManager::Callback(const dio_ros_driver::msg::DIOPort::ConstSharedPtr msg)
{
  bool input_port_value = active_polarity_ ? msg->value : !msg->value;

  bool button_status_change = checkStateChangeWithRemovingChattering(input_port_value);

  if (button_status_change == true) {
    port_value_ = input_port_value;
    publishPressed(port_value_);
  }

  ButtonState(button_status_change);
  onTimer();

  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    *get_clock(), 1.0,
    "[ButtonManager::Callback][button:%s]port raw/fix: %d/%d status:%d",
    get_name(), input_port_value, port_value_, button_status_);
}

void ButtonManager::ButtonState(const bool button_status_change)
{
  rclcpp::Time current_time = this->now();

  const double elapsed_time = (current_time - base_time_).seconds();

  switch (button_status_) {
    case BUTTON_OFF_WAIT:
      // wait to confirm button off
      if ((elapsed_time >= not_pressed_period_threshold_) && (port_value_ == false)) {
        button_status_ = BUTTON_OFF;
        RCLCPP_DEBUG_THROTTLE(
          get_logger(),
          *get_clock(), 1.0,
          "[ButtonManager::ButtonState][button:%s]state change elpased time %f",
          get_name(), elapsed_time);
      }
      break;
    case BUTTON_OFF:
      if (button_status_change == true) {
        button_status_ = BUTTON_ON_WAIT;
        RCLCPP_DEBUG_THROTTLE(
          get_logger(),
          *get_clock(), 1.0,
          "[ButtonManager::ButtonState][button:%s]state change  : %d",
          get_name(), button_status_);
      }
      break;
    case BUTTON_ON_WAIT:
      if (button_status_change == true) {
        button_status_ = BUTTON_OFF_WAIT;
        RCLCPP_DEBUG_THROTTLE(
          get_logger(),
          *get_clock(), 1.0,
          "[ButtonManager::ButtonState][button:%s]button_status_change %d",
          get_name(), button_status_);
      } else {
        if (elapsed_time >= pressed_period_threshold_) {
          button_status_ = BUTTON_ON;
          RCLCPP_DEBUG_THROTTLE(
            get_logger(),
            *get_clock(), 1.0,
            "[ButtonManager::ButtonState][button:%s]state change elpased time %f",
            get_name(), elapsed_time);
        }
      }
      break;
    case BUTTON_ON:
      if (elapsed_time >= max_allowable_period_of_pressing_) {
        button_status_ = BUTTON_ON_STACK;
        RCLCPP_ERROR(
          get_logger(),
          "[ButtonManager::ButtonState][button:%s]button stack error",
          get_name());
      }

      if (button_status_change == true) {
        button_status_ = BUTTON_OFF_AFTER_ON_WAIT;
        RCLCPP_DEBUG_THROTTLE(
          get_logger(),
          *get_clock(), 1.0,
          "[ButtonManager::ButtonState][button:%s]state change  : %d",
          get_name(), button_status_);
      } else {
        hold_down_time_ = elapsed_time;
      }
      break;
    case BUTTON_OFF_AFTER_ON_WAIT:
      if (button_status_change == true) {
        button_status_ = BUTTON_OFF_WAIT;
        RCLCPP_DEBUG_THROTTLE(
          get_logger(),
          *get_clock(), 1.0,
          "[ButtonManager::ButtonState][button:%s]button_status_change %d",
          get_name(), button_status_);
      } else {
        if (elapsed_time >= not_pressed_period_threshold_after_pressed_) {
          auto msg = std::make_unique<autoware_state_machine_msgs::msg::VehicleButton>();
          msg->stamp = this->now();
          msg->data = true;
          msg->hold_down_time = hold_down_time_;
          button_status_ = BUTTON_OFF_WAIT;
          pub_button_->publish(std::move(msg));
          RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(), 1.0,
            "[ButtonManager::ButtonState][button:%s]button on detect",
            get_name());
        }
      }
      break;
    case BUTTON_ON_STACK:
      if (button_status_change == true) {
        button_status_ = BUTTON_OFF_WAIT;
        RCLCPP_DEBUG_THROTTLE(
          get_logger(),
          *get_clock(), 1.0,
          "[ButtonManager::ButtonState][button:%s]button stack error recovered",
          get_name());
      }
      break;
    default:
      // error
      break;
  }
  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    *get_clock(), 1.0,
    "[ButtonManager::ButtonState][button:%s]state_ : %d",
    get_name(), button_status_);
}

void ButtonManager::publishPressed(const bool pressed)
{
  std_msgs::msg::Bool msg;
  msg.data = pressed;
  pub_button_pressed_->publish(msg);
}

void ButtonManager::onStateMachine(
  const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr msg)
{
  using StateMachine = autoware_state_machine_msgs::msg::StateMachine;
  // "Driving": from RUNNING to INFORM_RESTART (the vehicle is operating / temporarily stopped
  // in the middle of the route). Waiting for engage, arrived goal and emergency stop are excluded.
  is_driving_ =
    (msg->service_layer_state >= StateMachine::STATE_RUNNING) &&
    (msg->service_layer_state <= StateMachine::STATE_INFORM_RESTART);
}

void ButtonManager::onTimer()
{
  if (!enable_abnormal_input_detection_) {
    return;
  }
  abnormal_input_detector_->update(this->now().seconds(), port_value_, is_driving_);
}

void ButtonManager::checkButtonInput(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  const auto & s = abnormal_input_detector_->status();
  if (s.mrm) {
    stat.summary(DiagnosticStatus::ERROR, "button is stuck: MRM");
  } else if (s.warning) {
    stat.summary(
      DiagnosticStatus::WARN, s.stuck ? "button is stuck" : "button is chattering");
  } else {
    stat.summary(DiagnosticStatus::OK, "OK");
  }
  stat.add("chatter", s.chatter);
  stat.add("stuck", s.stuck);
  stat.add("press_duration_sec", s.press_duration_sec);
}

bool ButtonManager::checkStateChangeWithRemovingChattering(const bool is_button_on)
{
  bool check;

  port_value_history_.pop_front();
  port_value_history_.push_back(is_button_on);

  check = !port_value_;
  for (const auto tmp_val : port_value_history_) {
    if (check != tmp_val) {
      return false;
    }
  }

  base_time_ = this->now();
  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    *get_clock(), 1.0,
    "[ButtonManager::DioChatteringRemoval][button:%s]port value confirm: %d",
    get_name(), check);

  return true;
}

}  // namespace button_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(button_manager::ButtonManager)
