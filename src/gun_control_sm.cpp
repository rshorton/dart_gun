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
#include "gun_control_sm.h"

namespace {
}

GunControlStateMachine::GunControlStateMachine(GunHardwareInterface &hw_if, AimingControl &aiming_control):
    hw_if_(hw_if),
    aiming_control_(aiming_control)
{
    enter_init_state();
}

const char* GunControlStateMachine::get_cmd_result_str(GunControlStateMachine::CmdResult result)
{
    switch(result) {
        case CmdResult::CMD_RESULT_BUSY:
            return "busy";
        case CmdResult::CMD_RESULT_SUCCESS:
            return "success";
        case CmdResult::CMD_RESULT_FAILED:
            return "failed";
        default:
            return "?";
    }            
}

const char* GunControlStateMachine::firing_state_to_str(GunControlStateMachine::FiringState state)
{
    switch(state) {
        case FiringState::FIRING_STATE_INIT:
            return "init";
        case FiringState::FIRING_STATE_FLYWHEEL_SPINUP:
            return "spinup";
        case FiringState::FIRING_STATE_WAIT_FOR_AIMED:
            return "aiming";
        case FiringState::FIRING_STATE_PUSH_DART:
            return "push_dart";
        case FiringState::FIRING_STATE_RESET:
            return "reset";
        case FiringState::FIRING_STATE_WAIT_DART_READY:
            return "wait_dart_ready";
        case FiringState::FIRING_STATE_FLYWHEEL_TIMEOUT:
            return "flywheel_to";
        default:
            return "?";            
    }            
}

// Returns True if command accepted (ie not busy with previous cmd)
GunControlStateMachine::CmdResult GunControlStateMachine::fire_cmd(uint8_t speed, int8_t pan_angle, int8_t tilt_angle, uint8_t count)
{
    // Fail if no more darts
    if (dart_magazine_empty_) {
        return CmdResult::CMD_RESULT_FAILED;

    // Reject if fire pending
    } else  if (fire_cnt_) {
        return CmdResult::CMD_RESULT_BUSY;
    }

    if (speed == 3) {
        req_fly_wheel_speed_ = hw_if_.FLYWHEEL_SPEED_FULL;
    } else if (speed == 2) {
        req_fly_wheel_speed_ = hw_if_.FLYWHEEL_SPEED_MED;
    } else {
        req_fly_wheel_speed_ = hw_if_.FLYWHEEL_SPEED_LOW;
    }

    fire_cnt_ = count;

    aiming_control_.set_target_pan_angle(pan_angle);
    aiming_control_.set_target_tilt_angle(tilt_angle);
    update_aiming();

    return CmdResult::CMD_RESULT_SUCCESS;
}

GunControlStateMachine::CmdResult GunControlStateMachine::reset_cmd()
{
    aiming_control_.set_target_pan_angle(0.0f);
    aiming_control_.set_target_tilt_angle(0.0f);

    fire_cnt_ = 0;
    return CmdResult::CMD_RESULT_SUCCESS;
}

void GunControlStateMachine::update_aiming_laser_state(bool should_enable)
{
    if (should_enable) {
        hw_if_.set_targeting_laser(true);
        targeting_laser_on_time_ = millis();
        return;

    } else if (millis() > targeting_laser_on_time_ + TARGETING_LASER_ON_TIMEOUT_MS) {
        hw_if_.set_targeting_laser(false);
    }
}

void GunControlStateMachine::enter_state(FiringState new_state)
{
    firing_state_ = new_state;
    state_timer_start_ = millis();

    Logging::log_message(LOG_LVL_INFO, "GSSM: new state: %s", firing_state_to_str(firing_state_));
}

void GunControlStateMachine::enter_init_state()
{
    enter_state(FiringState::FIRING_STATE_INIT);
}

void GunControlStateMachine::enter_flywheel_timeout_state()
{
    enter_state(FiringState::FIRING_STATE_FLYWHEEL_TIMEOUT);
}

void GunControlStateMachine::enter_wait_dart_ready_state()
{
    if (fire_cnt_ > 0) {
    --fire_cnt_;
    }
    enter_state(FiringState::FIRING_STATE_WAIT_DART_READY);
}

void GunControlStateMachine::enter_reset_state()
{
    hw_if_.set_push_dart_into_flywheel(false, PUSH_DART_DURATION);
    hw_if_.set_barrel_led(false);
    enter_state(FiringState::FIRING_STATE_RESET);
}

void GunControlStateMachine::enter_push_dart_state()
{
    hw_if_.set_push_dart_into_flywheel(true, PUSH_DART_DURATION);
    hw_if_.set_barrel_led(true);
    enter_state(FiringState::FIRING_STATE_PUSH_DART);
}

void GunControlStateMachine::enter_wait_for_aimed_state()
{
    enter_state(FiringState::FIRING_STATE_WAIT_FOR_AIMED);
}

void GunControlStateMachine::enter_flywheel_spinup_state()
{
    if (hw_if_.set_flywheel_on(req_fly_wheel_speed_)) {
        enter_state(FiringState::FIRING_STATE_FLYWHEEL_SPINUP);
    } else {
        enter_wait_for_aimed_state();
    }    
}

void GunControlStateMachine::update_aiming()
{
    bool new_target = false;
    aiming_control_.update_aiming(new_target);
    update_aiming_laser_state(new_target);
}

void GunControlStateMachine::update()
{
    update_aiming();

    switch (firing_state_) {
        // Wait for a new firing command    
        case FiringState::FIRING_STATE_INIT:
            if (!hw_if_.is_dart_present()) {
                Logging::log_message(LOG_LVL_NEVER, "empty");
                hw_if_.set_flywheel_off();
                fire_cnt_ = 0;
                dart_magazine_empty_ = true;
                return;
            }
        dart_magazine_empty_ = false;

        if (fire_cnt_ == 0) {
            return;
        }

        update_aiming_laser_state(true);

        enter_flywheel_spinup_state();
        break;

    // Wait for the flywheel to spinup
    case FiringState::FIRING_STATE_FLYWHEEL_SPINUP:
        if (millis() - state_timer_start_ >= FLYWHEEL_SPIN_UP_DELAY) {
            enter_wait_for_aimed_state();
        }
        break;

    // Wait for aiming to complete
    case FiringState::FIRING_STATE_WAIT_FOR_AIMED:
        if (aiming_control_.is_aimed()) {
            Logging::log_message(LOG_LVL_INFO, "GSSM: aimed, pan: %f, tilt: %f, count: %d",
                                 aiming_control_.get_cur_pan_angle(), aiming_control_.get_cur_tilt_angle());   
            enter_push_dart_state();
        }
        break;

    // Push the next dart into the flywheel and give the flywheel
    // a bit of time to propel the dart
    case FiringState::FIRING_STATE_PUSH_DART:
        if (millis() - state_timer_start_ >= PUSH_DART_DELAY) {
            enter_reset_state();
        }
        break;
      
    // Pull back the dart pusher      
    case FiringState::FIRING_STATE_RESET:
        if (millis() - state_timer_start_ >= RESET_DELAY) {
            enter_wait_dart_ready_state();
        }
        break;

    // Wait for the next dart to drop into firing position
    case FiringState::FIRING_STATE_WAIT_DART_READY:
    {
        auto dart_present = hw_if_.is_dart_present();
        if (dart_present ||
            millis() - state_timer_start_ >= WAIT_DART_DELAY) {
            dart_magazine_empty_ = !dart_present;
            enter_flywheel_timeout_state();
        }
        break;
    }

    // If not commanded to fire again, stop spinning the flywheel
    // after a timeout
    case FiringState::FIRING_STATE_FLYWHEEL_TIMEOUT:
        if (fire_cnt_ > 0) {
            enter_init_state();
            break;
        }

        if (millis() - state_timer_start_ >= FLYWHEEL_TIMEOUT_DELAY) {
            hw_if_.set_flywheel_off();
            enter_init_state();
        }
        break;

    default:
        break;
  }      
}