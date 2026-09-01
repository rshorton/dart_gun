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
#include "serial_bus_servo.h"
#include "logging.h"
#include "gun_hw_if.h"

#define DART_PRESENT_IN       3   // Active low
#define GUN_ENABLE_IN         5   // Active low  (not current used)
#define SWITCH_IN             7   // Active low momentary switch

#define MOTOR_PWM_OUT         11  // Active low
#define BARREL_LED_STRIP_OUT  4   // Active hi
#define TARGETING_LASER_OUT   6   // Aiming laser

namespace {
const float FIRE_SERVO_FIRE_DEG = 165.0f;
const float FIRE_SERVO_RESET_DEG = 0.0f;
}

GunHardwareInterface::GunHardwareInterface(SerialServo &servo_fire, SerialServo &servo_tilt, SerialServo &servo_pan):
    servo_fire_(servo_fire),
    servo_tilt_(servo_tilt),
    servo_pan_(servo_pan)
{}    

void GunHardwareInterface::init()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(DART_PRESENT_IN, INPUT_PULLUP);
    pinMode(GUN_ENABLE_IN, INPUT_PULLUP);
    pinMode(MOTOR_PWM_OUT, OUTPUT);
    pinMode(BARREL_LED_STRIP_OUT, OUTPUT);
    pinMode(TARGETING_LASER_OUT, OUTPUT);
    pinMode(SWITCH_IN, INPUT_PULLUP);

    set_barrel_led(false);
    set_targeting_laser(false);
    set_flywheel_off(true);

    delay(100);

    move_pan_servo_to_angle(0.0, 1000);
    move_tilt_servo_to_angle(0.0, 1000);

    set_push_dart_into_flywheel(false, 1000);

    delay(1000);
}

void GunHardwareInterface::set_flywheel_speed(uint8_t speed)
{
    analogWrite(MOTOR_PWM_OUT, invert_pwm_? 255 - speed: speed);
}

bool GunHardwareInterface::set_flywheel_off(bool force)
{
    if (force || flywheel_on_) {
        set_flywheel_speed(FLYWHEEL_SPEED_STOP);
        flywheel_on_ = false;
        return true;
    }
    return false;
}

bool GunHardwareInterface::set_flywheel_on(uint8_t speed)
{
    if (flywheel_on_ &&
        fly_wheel_speed_ == speed) {
        return false;
    }
    fly_wheel_speed_ = speed;
    set_flywheel_speed(fly_wheel_speed_);
    flywheel_on_ = true;
    return true;
}

void GunHardwareInterface::set_push_dart_into_flywheel(bool push_in, unsigned long move_duration)
{
    servo_fire_.move_to_angle(push_in? FIRE_SERVO_FIRE_DEG: FIRE_SERVO_RESET_DEG, move_duration);
}

void GunHardwareInterface::set_barrel_led(bool on)
{
    digitalWrite(BARREL_LED_STRIP_OUT, on);
}

void GunHardwareInterface::set_targeting_laser(bool on)
{
    digitalWrite(TARGETING_LASER_OUT, on);
}

bool GunHardwareInterface::is_dart_present()
{
    return digitalRead(DART_PRESENT_IN);
}

bool GunHardwareInterface::is_gun_enabled()
{
#if defined(USE_GUN_ENABLE_PIN)  
    if (digitalRead(GUN_ENABLE_IN)) {
        set_flywheel_off();
        return false;
    }
    return true;
#else
    return true;
#endif  
}

bool GunHardwareInterface::wait_for_button_press(uint32_t max_wait_ms)
{
    auto start = millis();

    do {
        if (!digitalRead(SWITCH_IN)) {
            return true;
        }
    } while (millis() < start + max_wait_ms);
    return false;
}

bool GunHardwareInterface::wait_for_button_release(uint32_t max_wait_ms)
{
    auto start = millis();

    do {
        if (digitalRead(SWITCH_IN)) {
            return true;
        }
    } while (millis() < start + max_wait_ms);
    return false;
}

void GunHardwareInterface::move_pan_servo_to_angle(float angle_deg, uint32_t duration)
{
    servo_pan_.move_to_angle(angle_deg, duration);
}

void GunHardwareInterface::move_tilt_servo_to_angle(float angle_deg, uint32_t duration)
{
    servo_tilt_.move_to_angle(angle_deg, duration);
}

bool GunHardwareInterface::read_pan_servo_angle(float &angle_deg)
{
    return servo_pan_.read_angle(angle_deg);
}

bool GunHardwareInterface::read_tilt_servo_angle(float &angle_deg)
{
    return servo_tilt_.read_angle(angle_deg);
}