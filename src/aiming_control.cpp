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

#include "Arduino.h"
#include "logging.h"
#include "aiming_control.h"

namespace
{
const uint32_t AIMING_POSITION_CK_MS = 200;

const float TILT_ABS_MAX_DEG = 15.0f;
const float PAN_ABS_MAX_DEG = 90.0f;

const uint32_t PAN_MOVE_MS_PER_DEG = 16;
const uint32_t TILT_MOVE_MS_PER_DEG = 16;

const uint32_t PAN_TILT_MOVE_MIN_MS = 75;

const float ANGLE_AIM_THRESH_DEG = 1.5f;
}

template <typename T>
constexpr int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

AimingControl::AimingControl(GunHardwareInterface &hw_if):
    hw_if_(hw_if)
{
}

void AimingControl::set_target_pan_angle(float pan_angle)
{
    if (abs(pan_angle) > PAN_ABS_MAX_DEG) {
        pan_angle = PAN_ABS_MAX_DEG*sgn(pan_angle);
    }
    req_pan_angle_ = pan_angle;
}

void AimingControl::set_target_tilt_angle(float tilt_angle)
{
    if (abs(tilt_angle) > TILT_ABS_MAX_DEG) {
        tilt_angle = TILT_ABS_MAX_DEG*sgn(tilt_angle);
    }
    req_tilt_angle_ = tilt_angle;
}

bool AimingControl::read_aiming_servos()
{
    float angle;
    bool valid = hw_if_.read_pan_servo_angle(angle);
    if (!valid) {
        return false;
    }
    cur_pan_angle_ = angle;

    valid = hw_if_.read_tilt_servo_angle(angle);
    if (!valid) {
        return false;
    }
    cur_tilt_angle_ = angle;

    Logging::log_message(LOG_LVL_DEBUG, "read_aiming_servos: cur_pan angle: %f, cur_tilt_angle: %f",
                         cur_pan_angle_, cur_tilt_angle_);

    return true;
}

bool AimingControl::is_at_target_position()
{
    if (!read_aiming_servos()) {
        return false;
    }

    // At the target position?
    return abs(cur_pan_angle_ - target_pan_angle_) < ANGLE_AIM_THRESH_DEG &&
           abs(cur_tilt_angle_ - target_tilt_angle_) < ANGLE_AIM_THRESH_DEG;
}

// Aim as needed.  Returns true if aiming was needed.
bool AimingControl::aim(bool wait)
{
    // Already moving to/pointing at the target?
    if (target_pan_angle_ == req_pan_angle_ &&
        target_tilt_angle_ == req_tilt_angle_) {
        return false;
    }

    aimed_ = false;

    // The movement needs the current position to accurately calculate the
    // desired movement duration.
    read_aiming_servos();

    uint32_t pan_duration = 0;
    if (target_pan_angle_ != req_pan_angle_) {
        pan_duration = max(PAN_TILT_MOVE_MIN_MS, static_cast<uint32_t>(PAN_MOVE_MS_PER_DEG*abs(cur_pan_angle_ - req_pan_angle_)));
        hw_if_.move_pan_servo_to_angle(req_pan_angle_, pan_duration);
        Logging::log_message(LOG_LVL_DEBUG, "aim: pan angle: %f, cur angle: %f, duration: %d",
                             req_pan_angle_, cur_pan_angle_, pan_duration);
        target_pan_angle_ = req_pan_angle_;
    }

    uint32_t tilt_duration = 0;
    if (target_tilt_angle_ != req_tilt_angle_) {
        tilt_duration = max(PAN_TILT_MOVE_MIN_MS, static_cast<uint32_t>(TILT_MOVE_MS_PER_DEG*abs(cur_tilt_angle_ - req_tilt_angle_)));
        hw_if_.move_tilt_servo_to_angle(req_tilt_angle_, tilt_duration);
        Logging::log_message(LOG_LVL_DEBUG, "aim: tilt angle: %f, cur angle: %f, duration: %d",
                             req_tilt_angle_, cur_tilt_angle_, tilt_duration);
        target_tilt_angle_ = req_tilt_angle_;
    }


    if (wait) {
        auto delay_ms = max(pan_duration, tilt_duration);
        Logging::log_message(LOG_LVL_DEBUG, "aim: waiting for pan/tilt, delay: %d", delay_ms);
        delay(delay_ms);
    }
    return true;
}

void AimingControl::update_aiming(bool &new_target)
{
    // Update aiming as needed on all updates
    new_target = aim(false);

    // If no change in the targeting, then check if pointed at target
    if (new_target) {
        aimed_ = false;
        Logging::log_message(LOG_LVL_INFO, "AimingControl, new target");

    } else if (!aimed_) {
        auto now_ms = millis();
        if (now_ms > last_position_ck_ + AIMING_POSITION_CK_MS) {
            last_position_ck_ = now_ms;

            if (is_at_target_position()) {
                aimed_ = true;
                Logging::log_message(LOG_LVL_INFO, "AimingControl, now aimed");
            }
        }
    }
}


