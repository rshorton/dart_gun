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

#ifndef GUN_HW_IF_H
#define GUN_HW_IF_H

#include "Arduino.h"
#include "serial_bus_servo.h"

#undef USE_GUN_ENABLE_PIN

class GunHardwareInterface
{
public:
    const uint8_t FLYWHEEL_SPEED_FULL = 255;
    const uint8_t FLYWHEEL_SPEED_MED= 150;
    const uint8_t FLYWHEEL_SPEED_LOW = 125;
    const uint8_t FLYWHEEL_SPEED_STOP = 0;

    GunHardwareInterface(SerialServo &servo_fire, SerialServo &servo_tilt, SerialServo &servo_pan);

    void init();

    bool set_flywheel_off(bool force = true);
    bool set_flywheel_on(uint8_t speed);

    void set_push_dart_into_flywheel(bool push_in, unsigned long move_duration);
    
    void set_barrel_led(bool on);
    
    void set_targeting_laser(bool on);
    
    bool is_dart_present();
    bool is_gun_enabled();
    
    bool wait_for_button_press(uint32_t max_wait_ms);
    bool wait_for_button_release(uint32_t max_wait_ms);
    
    void move_pan_servo_to_angle(float angle_deg, uint32_t duration);
    void move_tilt_servo_to_angle(float angle_deg, uint32_t duration);
    
    bool read_pan_servo_angle(float &angle_deg);
    bool read_tilt_servo_angle(float &angle_deg);

private:
    void set_flywheel_speed(uint8_t speed);

    SerialServo& servo_fire_;
    SerialServo& servo_tilt_;
    SerialServo& servo_pan_;

    const bool invert_pwm_{true};

    bool flywheel_on_{false};
    uint8_t fly_wheel_speed_{FLYWHEEL_SPEED_STOP};
};

#endif // GUN_HW_IF_H