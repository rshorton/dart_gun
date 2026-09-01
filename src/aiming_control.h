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

#ifndef AIMING_CONTROL_H
#define AIMING_CONTROL_H

#include "Arduino.h"
#include <ArduinoJson.h>

#include "gun_hw_if.h"

class AimingControl
{
public:
    AimingControl(GunHardwareInterface &hw_if);

    void set_target_pan_angle(float pan_angle);
    void set_target_tilt_angle(float tilt_angle);

    bool aim(bool wait);

    void update_aiming(bool &new_target);

    bool is_aimed() const {
        return aimed_;
    }

    float get_cur_pan_angle() const {
        return cur_pan_angle_;
    }

    float get_cur_tilt_angle() const {
        return cur_tilt_angle_;
    }

private:
    bool is_at_target_position();
    bool read_aiming_servos();

    GunHardwareInterface &hw_if_;

    float req_pan_angle_{0.0f};
    float req_tilt_angle_{0.0f};

    float target_pan_angle_{0.0f};
    float target_tilt_angle_{0.0f};

    float cur_pan_angle_{0.0f};
    float cur_tilt_angle_{0.0f};

    bool aimed_{false};
    uint32_t last_position_ck_{0};
};

#endif // AIMING_CONTROL_H