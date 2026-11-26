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

#include <gtest/gtest.h>

#include "button_manager/abnormal_input_detector.hpp"

using button_manager::AbnormalInputDetector;
using button_manager::AbnormalInputParams;

namespace
{
constexpr double kDt = 0.1;

// Same values as config/button_manager.param.yaml
AbnormalInputParams testParams()
{
  AbnormalInputParams p;
  p.stuck_detect_sec = 1.0;
  p.mrm_after_press_sec = 30.0;
  p.chatter_short_press_max_sec = 1.0;
  p.chatter_count_threshold = 5;
  p.chatter_window_sec = 5.0;
  p.chatter_clear_sec = 2.0;
  return p;
}

// Test helper which advances the simulated time.
struct Sim
{
  AbnormalInputDetector det{testParams()};
  double t{100.0};
  bool pressed{false};
  bool driving{true};

  void run(const double sec, const bool p)
  {
    pressed = p;
    const int n = static_cast<int>(sec / kDt + 0.5);
    for (int i = 0; i < n; ++i) {
      det.update(t, pressed, driving);
      t += kDt;
    }
  }
};
}  // namespace

// A press released within 1 s is not abnormal
TEST(AbnormalInputDetector, ShortPressIsNormal)
{
  Sim s;
  s.run(1.0, false);
  s.run(0.8, true);
  EXPECT_FALSE(s.det.status().warning);
  s.run(3.0, false);
  EXPECT_FALSE(s.det.status().warning);
  EXPECT_FALSE(s.det.status().mrm);
}

// 1 s < press <= 30 s: warning, OFF when released
TEST(AbnormalInputDetector, WarningWhilePressedThenRelease)
{
  Sim s;
  s.run(1.0, false);
  s.run(1.5, true);
  EXPECT_TRUE(s.det.status().warning);
  EXPECT_FALSE(s.det.status().mrm);
  s.run(13.5, true);  // 15 s
  EXPECT_TRUE(s.det.status().warning);
  EXPECT_FALSE(s.det.status().mrm);
  s.run(0.5, false);
  EXPECT_FALSE(s.det.status().warning);
  EXPECT_FALSE(s.det.status().mrm);
}

// Released at 30 s -> warning only (never MRM)
TEST(AbnormalInputDetector, ReleaseAt30sIsWarningOnly)
{
  Sim s;
  s.run(1.0, false);
  s.run(29.9, true);
  EXPECT_TRUE(s.det.status().warning);
  EXPECT_FALSE(s.det.status().mrm);
  s.run(0.5, false);
  EXPECT_FALSE(s.det.status().warning);
  EXPECT_FALSE(s.det.status().mrm);
}

// > 30 s: MRM (warning OFF), continues while pressed, OFF as soon as released
TEST(AbnormalInputDetector, MrmAfter30sAndClearedOnRelease)
{
  Sim s;
  s.run(1.0, false);
  s.run(31.0, true);
  EXPECT_TRUE(s.det.status().mrm);
  EXPECT_FALSE(s.det.status().warning);
  s.run(20.0, true);  // continues (system_error_monitor escalates it to the emergency)
  EXPECT_TRUE(s.det.status().mrm);
  s.run(0.5, false);
  EXPECT_FALSE(s.det.status().mrm);
  EXPECT_FALSE(s.det.status().warning);
}

// A new press restarts the timer from its own press start
TEST(AbnormalInputDetector, NewPressRestartsTimer)
{
  Sim s;
  s.run(1.0, false);
  s.run(10.0, true);
  s.run(1.0, false);
  EXPECT_FALSE(s.det.status().warning);
  s.run(1.5, true);
  EXPECT_TRUE(s.det.status().warning);
  EXPECT_LT(s.det.status().press_duration_sec, 1.0 + 1.0);
  EXPECT_FALSE(s.det.status().mrm);
}

// MRM continues across the goal arrival
TEST(AbnormalInputDetector, ContinuesAcrossGoalArrival)
{
  Sim s;
  s.run(1.0, false);
  s.run(10.0, true);
  s.driving = false;  // arrived at the goal
  s.run(21.0, true);  // 31 s after the press start
  EXPECT_TRUE(s.det.status().mrm);
}

// A press which started while not driving is never monitored (e.g. engage press 1-15 s)
TEST(AbnormalInputDetector, PressStartedWhileStoppedIsIgnored)
{
  Sim s;
  s.driving = false;
  s.run(1.0, false);
  s.run(14.0, true);
  EXPECT_FALSE(s.det.status().warning);
  s.driving = true;
  s.run(40.0, true);
  EXPECT_FALSE(s.det.status().warning);
  EXPECT_FALSE(s.det.status().mrm);
}

// T-22/23/24/25: chattering -> warning only, cleared after it stops
TEST(AbnormalInputDetector, ChatterWarning)
{
  Sim s;
  s.run(1.0, false);
  for (int i = 0; i < 10; ++i) {
    s.run(0.3, true);
    s.run(0.3, false);
  }
  EXPECT_TRUE(s.det.status().warning);
  EXPECT_TRUE(s.det.status().chatter);
  EXPECT_FALSE(s.det.status().mrm);
  s.run(1.0, false);
  EXPECT_TRUE(s.det.status().warning);
  s.run(2.0, false);
  EXPECT_FALSE(s.det.status().warning);
}

// T-25: chattering continues for 35 s or more -> never MRM
TEST(AbnormalInputDetector, LongChatterNeverEscalates)
{
  Sim s;
  s.run(1.0, false);
  for (int i = 0; i < 70; ++i) {
    s.run(0.3, true);
    s.run(0.3, false);
    EXPECT_FALSE(s.det.status().mrm);
  }
  EXPECT_TRUE(s.det.status().warning);
}

// A press shorter than 1 s (e.g. 0.8 s) is a chatter event, not a stuck
TEST(AbnormalInputDetector, PressUnder1sIsChatter)
{
  Sim s;
  s.run(1.0, false);
  for (int i = 0; i < 6; ++i) {
    s.run(0.8, true);
    s.run(0.2, false);
  }
  EXPECT_TRUE(s.det.status().chatter);
  EXPECT_TRUE(s.det.status().warning);
  EXPECT_FALSE(s.det.status().stuck);
  EXPECT_FALSE(s.det.status().mrm);
}

// Chattering is not detected while the vehicle is stopped
TEST(AbnormalInputDetector, NoChatterWhileStopped)
{
  Sim s;
  s.driving = false;
  s.run(1.0, false);
  for (int i = 0; i < 10; ++i) {
    s.run(0.3, true);
    s.run(0.3, false);
  }
  EXPECT_FALSE(s.det.status().warning);
}
