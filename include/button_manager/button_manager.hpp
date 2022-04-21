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

#ifndef BUTTON_MANAGER__BUTTON_MANAGER_HPP_
#define BUTTON_MANAGER__BUTTON_MANAGER_HPP_

#include <string>
#include <list>
#include <vector>
#include <climits>

#include "rclcpp/rclcpp.hpp"

#include "autoware_state_machine_msgs/msg/vehicle_button.hpp"
#include "dio_ros_driver/msg/dio_port.hpp"

namespace button_manager
{

class ButtonManager : public rclcpp::Node
{
public:
  explicit ButtonManager(const rclcpp::NodeOptions & options);
  ~ButtonManager();

private:
  enum NodeStatus
  {
    BUTTON_OFF_WAIT = 0,
    BUTTON_OFF,
    BUTTON_ON_WAIT,
    BUTTON_ON,
    BUTTON_OFF_AFTER_ON_WAIT,
    BUTTON_ON_STACK
  };

  rclcpp::Publisher<autoware_state_machine_msgs::msg::VehicleButton>::SharedPtr pub_button_;
  rclcpp::Subscription<dio_ros_driver::msg::DIOPort>::SharedPtr sub_dio_info_;

  enum NodeStatus button_status_ = BUTTON_OFF_WAIT;
  rclcpp::Time base_time_;

  std::string button_name_ = "";
  float not_pressed_period_threshold_ = 1.0;
  float pressed_period_threshold_ = 1.0;
  float not_pressed_period_threshold_after_pressed_ = 0.5;
  float max_allowable_period_of_pressing_ = 30.0;
  bool active_polarity_ = true;

  // Initial "port_value_" sets to false as OFF. This is going to overwritten by a first DIO input.
  bool port_value_ = false;
  std::list<bool> port_value_history_{false, false, false};

  void Callback(const dio_ros_driver::msg::DIOPort::ConstSharedPtr msg);
  void ButtonState(const bool button_status_change);
  bool checkStateChangeWithRemovingChattering(const bool is_button_on);
};

}  // namespace button_manager
#endif  // BUTTON_MANAGER__BUTTON_MANAGER_HPP_
