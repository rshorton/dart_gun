#include "Arduino.h"
#include "logging.h"
#include "testing.h"
#include "gun_hw_if.h"

namespace Testing
{
const char* str_continue = "Press button to continue\n";

bool wait_for_button_press(GunHardwareInterface &hw_if, const char* msg, uint32_t timeout)
{
    Logging::log_message(LOG_LVL_DEBUG, msg);
    if (hw_if.wait_for_button_press(timeout)) {
        return true;
    }
    return false;
}

bool test_barrel_led(GunHardwareInterface &hw_if)
{
    Logging::log_message(LOG_LVL_DEBUG, "Testing barrel LED");

    for (int i = 0; i < 3; ++i) {
        hw_if.set_barrel_led(true);
        delay(500);
        hw_if.set_barrel_led(false);
        delay(250);
    }

    return wait_for_button_press(hw_if, str_continue, 20000);
}

bool test_targeting_laser(GunHardwareInterface &hw_if)
{
    Logging::log_message(LOG_LVL_DEBUG, "Testing targeting laser");

    for (int i = 0; i < 3; ++i) {
        hw_if.set_targeting_laser(true);
        delay(500);
        hw_if.set_targeting_laser(false);
        delay(250);
    }

    return wait_for_button_press(hw_if, str_continue, 20000);
}

bool test_dart_pusher(GunHardwareInterface &hw_if)
{
    Logging::log_message(LOG_LVL_DEBUG, "Testing dart pusher");

    hw_if.set_push_dart_into_flywheel(true, 1000);
    delay(1000);
    hw_if.set_push_dart_into_flywheel(false, 1000);
    delay(1000);

    return wait_for_button_press(hw_if, str_continue, 20000);
}

bool test_flywheel(GunHardwareInterface &hw_if)
{
    Logging::log_message(LOG_LVL_DEBUG, "Testing flywheel");

    Logging::log_message(LOG_LVL_DEBUG, "Full speed");
    hw_if.set_flywheel_on(hw_if.FLYWHEEL_SPEED_FULL);
    delay(3000);

    Logging::log_message(LOG_LVL_DEBUG, "Medium speed");
    hw_if.set_flywheel_on(hw_if.FLYWHEEL_SPEED_MED);
    delay(3000);

    Logging::log_message(LOG_LVL_DEBUG, "Low speed");
    hw_if.set_flywheel_on(hw_if.FLYWHEEL_SPEED_LOW);
    delay(3000);
    hw_if.set_flywheel_off();

    return wait_for_button_press(hw_if, str_continue, 20000);
}

bool test_dart_sensor(GunHardwareInterface &hw_if)
{
    Logging::log_message(LOG_LVL_DEBUG, "Testing dart sensor. Manually block sensor");

    auto start = millis();
    do {
        if (hw_if.is_dart_present()) {
            hw_if.set_push_dart_into_flywheel(true, 1000);
            delay(1000);
            hw_if.set_push_dart_into_flywheel(false, 1000);
            delay(1000);
            break;
          }
    } while (millis() - start < 10000);

    return wait_for_button_press(hw_if, str_continue, 20000);
}

bool test_pan_servo(GunHardwareInterface &hw_if, AimingControl &aiming_control)
{
    Logging::log_message(LOG_LVL_DEBUG, "Testing pan servo");
    delay(2000);

    const float pan_test_angles[] = {-45.0f, 15.0f, 45.0f, 0.0f};
    int num_pan_test_angles = sizeof(pan_test_angles)/sizeof(pan_test_angles[0]);

    aiming_control.set_target_tilt_angle(0.0f);

    for (int i = 0; i < num_pan_test_angles; i++) {
        Logging::log_message(LOG_LVL_DEBUG, "Set pan angle: %f", pan_test_angles[i]);
        aiming_control.set_target_pan_angle(pan_test_angles[i]);

        Logging::log_message(LOG_LVL_DEBUG, "Aiming...");
        do {
          bool new_target;
          aiming_control.update_aiming(new_target);
        } while (!aiming_control.is_aimed());
        Logging::log_message(LOG_LVL_DEBUG, "Aimed\n");
    }

    return wait_for_button_press(hw_if, str_continue, 20000);
}

bool test_tilt_servo(GunHardwareInterface &hw_if, AimingControl &aiming_control)
{
    Logging::log_message(LOG_LVL_DEBUG, "Testing tilt servo");
    delay(2000);

    const float tilt_test_angles[] = {-13.0f, -5.0f, 12.0f, 0.0f};
    int num_tilt_test_angles = sizeof(tilt_test_angles)/sizeof(tilt_test_angles[0]);

    aiming_control.set_target_pan_angle(0.0f);

    for (int i = 0; i < num_tilt_test_angles; i++) {
        Logging::log_message(LOG_LVL_DEBUG, "Set tilt angle: %f", tilt_test_angles[i]);
        aiming_control.set_target_tilt_angle(tilt_test_angles[i]);

        Logging::log_message(LOG_LVL_DEBUG, "Aiming...");
        do {
          bool new_target;
          aiming_control.update_aiming(new_target);
        } while (!aiming_control.is_aimed());
        Logging::log_message(LOG_LVL_DEBUG, "Aimed\n");
    }

    return wait_for_button_press(hw_if, str_continue, 20000);
}

void run_all_test(GunHardwareInterface &hw_if, AimingControl &aiming_control)
{
    auto saved_log_level = Logging::get_log_level();
    Logging::set_log_level(LOG_LVL_DEBUG);

    Logging::log_message(LOG_LVL_DEBUG, "Test mode.  Release button if pressed.");
    hw_if.wait_for_button_release(20000);

    if (!wait_for_button_press(hw_if, "Press button to start testing", 20000)) {
        return;
    }

    if (!test_pan_servo(hw_if, aiming_control)) {
       return;
    }

    if (!test_tilt_servo(hw_if, aiming_control)) {
       return;
    }

    if (!test_barrel_led(hw_if)) {
        return;
    }

    if (!test_targeting_laser(hw_if)) {
       return;
    }

    if (!test_dart_pusher(hw_if)) {
       return;
    }

    if (!test_flywheel(hw_if)) {
       return;
    }

    if (!test_dart_sensor(hw_if)) {
       return;
    }

    Logging::log_message(LOG_LVL_DEBUG, "Test complete.");
    if (!wait_for_button_press(hw_if, str_continue, 20000)) {
        return;
    }

    // Acknowledge end of test
    for (int i = 0; i < 6; ++i) {
        hw_if.set_barrel_led(true);
        delay(500);
        hw_if.set_barrel_led(false);
        delay(250);
    }

    Logging::set_log_level(saved_log_level);
}

}
