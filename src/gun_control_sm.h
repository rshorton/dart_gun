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

#ifndef GUN_CONTROL_SM_H
#define GUN_CONTROL_SM_H

#include "Arduino.h"
#include "gun_hw_if.h"
#include "aiming_control.h"

// Gun control state machine

class GunControlStateMachine
{
public:
    enum class CmdResult {CMD_RESULT_SUCCESS, CMD_RESULT_FAILED, CMD_RESULT_BUSY};

private:
    const uint32_t FLYWHEEL_SPIN_UP_DELAY = 800;
    const uint32_t PUSH_DART_DURATION = 500;
    const uint32_t PUSH_DART_DELAY = (PUSH_DART_DURATION + 100);
    const uint32_t RESET_DELAY = (PUSH_DART_DURATION + 100);
    const uint32_t WAIT_DART_DELAY = 2000;
    const uint32_t FLYWHEEL_TIMEOUT_DELAY = 3000;

    const uint32_t TARGETING_LASER_ON_TIMEOUT_MS = 10000;

    enum class FiringState {FIRING_STATE_INIT, FIRING_STATE_FLYWHEEL_SPINUP, FIRING_STATE_WAIT_FOR_AIMED,
                            FIRING_STATE_PUSH_DART, FIRING_STATE_RESET, FIRING_STATE_WAIT_DART_READY,
                            FIRING_STATE_FLYWHEEL_TIMEOUT};  

public:
    GunControlStateMachine(GunHardwareInterface &hw_if, AimingControl &aiming_control);

    void update();
    CmdResult fire_cmd(uint8_t speed, int8_t pan_angle, int8_t tilt_angle, uint8_t count);
    CmdResult reset_cmd();

    bool is_magazine_empty() const {
        return dart_magazine_empty_;
    }

    bool is_firing_pending() const {
        return fire_cnt_ > 0;
    }

    static const char* get_cmd_result_str(GunControlStateMachine::CmdResult result);
    static const char* firing_state_to_str(GunControlStateMachine::FiringState state);

private:
    void update_aiming();
    void update_aiming_laser_state(bool should_enable);

    void enter_state(FiringState new_state);
    void enter_init_state();
    void enter_flywheel_timeout_state();
    void enter_wait_for_aimed_state();
    void enter_wait_dart_ready_state();
    void enter_reset_state();
    void enter_push_dart_state();
    void enter_flywheel_spinup_state();

    GunHardwareInterface &hw_if_;
    AimingControl &aiming_control_;

    bool firing_pending_{false};
    bool dart_magazine_empty_{true};

    FiringState firing_state_{FiringState::FIRING_STATE_INIT};
    uint32_t state_timer_start_{0};

    uint32_t targeting_laser_on_time_{0};

    uint8_t fire_cnt_{0};
    uint8_t req_fly_wheel_speed_{0};
};

#endif // GUN_CONTROL_SM_H