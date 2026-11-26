// Copyright 2026 eve autonomy inc. All Rights Reserved.
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

#ifndef BUTTON_MANAGER__ABNORMAL_INPUT_DETECTOR_HPP_
#define BUTTON_MANAGER__ABNORMAL_INPUT_DETECTOR_HPP_

#include <deque>

namespace button_manager
{

/**
 * Parameters of the abnormal input detection while the vehicle is driving.
 * All times are in seconds. No default values are defined here: every value comes from
 * config/button_manager.param.yaml.
 */
struct AbnormalInputParams
{
  // [D] A press that is released within this time (measured from the press start) is not abnormal.
  // A press that continues longer than this is a "stuck" button (warning).
  double stuck_detect_sec;
  // [E] Press duration (measured from the press start) over which the MRM is requested.
  double mrm_after_press_sec;

  // [F] A press shorter than this is counted as a chattering event.
  double chatter_short_press_max_sec;
  // [F] Number of chattering events within chatter_window_sec to detect chattering.
  int chatter_count_threshold;
  double chatter_window_sec;
  // [F] Chattering is regarded as stopped when no event occurred for this time.
  double chatter_clear_sec;
};

struct AbnormalInputStatus
{
  bool warning{false};
  bool mrm{false};
  // Details (for diagnostics)
  bool chatter{false};
  bool stuck{false};
  double press_duration_sec{0.0};
};

/**
 * Detects chattering and stuck of a (debounced) button input.
 *
 * Level of the status ("a higher level turns the lower level OFF"):
 *   warning   : chattering or stuck is detected (OFF when the input recovers)
 *   mrm       : the press continues for more than mrm_after_press_sec (from the press start)
 *               (OFF as soon as the button is released)
 * The escalation from the MRM to the emergency stop is done by the system_error_monitor.
 *
 * Only a press which STARTED while the vehicle was driving is monitored.
 * Once the stuck is detected it is monitored until the button is released, so that
 * the MRM continues even if the vehicle arrives at the goal meanwhile.
 *
 * This class does not depend on ROS, time is injected by the caller.
 */
class AbnormalInputDetector
{
public:
  explicit AbnormalInputDetector(const AbnormalInputParams & params)
  : params_(params)
  {
  }

  void update(const double now_sec, const bool pressed, const bool driving)
  {
    // ---- edge handling ----
    if (pressed && !prev_pressed_) {
      press_start_ = now_sec;
      armed_ = driving;
    }
    if (!pressed && prev_pressed_) {
      if (armed_ && (now_sec - press_start_) < params_.chatter_short_press_max_sec) {
        chatter_events_.push_back(now_sec);
      }
      stuck_ = false;
      armed_ = false;
    }
    prev_pressed_ = pressed;

    // ---- chattering ----
    while (!chatter_events_.empty() &&
      (now_sec - chatter_events_.front()) > params_.chatter_window_sec)
    {
      chatter_events_.pop_front();
    }
    if (static_cast<int>(chatter_events_.size()) >= params_.chatter_count_threshold) {
      chatter_ = true;
      last_chatter_event_ = chatter_events_.back();
    }
    if (chatter_ && (now_sec - last_chatter_event_) > params_.chatter_clear_sec) {
      chatter_ = false;
      chatter_events_.clear();
    }

    // ---- stuck ----
    if (pressed && armed_ && !stuck_ &&
      (now_sec - press_start_) > params_.stuck_detect_sec)
    {
      stuck_ = true;
    }

    double elapsed = 0.0;
    bool mrm = false;
    if (stuck_) {
      elapsed = now_sec - press_start_;
      mrm = elapsed > params_.mrm_after_press_sec;
    }

    status_.mrm = mrm;
    status_.warning = (stuck_ || chatter_) && !mrm;
    status_.chatter = chatter_;
    status_.stuck = stuck_;
    status_.press_duration_sec = elapsed;
  }

  const AbnormalInputStatus & status() const {return status_;}

private:
  AbnormalInputParams params_;
  AbnormalInputStatus status_;

  bool prev_pressed_{false};
  bool armed_{false};
  double press_start_{0.0};

  bool stuck_{false};

  bool chatter_{false};
  double last_chatter_event_{0.0};
  std::deque<double> chatter_events_;
};

}  // namespace button_manager

#endif  // BUTTON_MANAGER__ABNORMAL_INPUT_DETECTOR_HPP_
