// MIT License
//
// Copyright (c) 2026 Scott Horton
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <Arduino.h>

#include "logging.h"
#include "gun_hw_if.h"
#include "gun_control_sm.h"
#include "aiming_control.h"
#include "cmd_api.h"
#include "testing.h"

#if USE_WIFI == 1  
  #pragma message "Building for WIFI"
#else
  #pragma message "Not building for WIFI"
#endif

namespace {

SerialServo servo_fire(Serial1, 1, 240, 1000, 0.0f, false);
// +degrees tilts up
SerialServo servo_tilt(Serial1, 2, 240, 1000, 123.3f, false);
// +degrees pans left
SerialServo servo_pan(Serial1, 3, 240, 1000, 143.3f, false);

GunHardwareInterface hw_if(servo_fire, servo_tilt, servo_pan);
AimingControl aiming_control(hw_if);
GunControlStateMachine control_sm(hw_if, aiming_control);
CommandAPI cmd_api(hw_if, aiming_control, control_sm);

} // namespace

void setup()
{
  Serial.begin(115200);

  // Serial 1 is used for servo control.  Invert signals for compatibility
  // with hardware interface.
  Serial1.begin(115200, SERIAL_8N1, 9, 8, true);  

  // Serial 2 is used for debugging (header on control board)
  Serial2.begin(115200, SERIAL_8N1, A6, A7);  

  Logging::set_log_level(LOG_LVL_INFO);
  Logging::log_message(LOG_LVL_INFO, "Setup");

  hw_if.init();

  if (hw_if.wait_for_button_press(2000)) {
    Testing::run_all_test(hw_if, aiming_control);
  }

  cmd_api.init();

  // Flash the barrel led to indicate the gun is ready
  hw_if.set_barrel_led(true);
  delay(500);
  hw_if.set_barrel_led(false);
  delay(250);

  Logging::log_message(LOG_LVL_INFO, "Ready");
}

void loop()
{
  cmd_api.update();
  control_sm.update();
}
