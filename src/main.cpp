#include <Arduino.h>
#include <SoftwareSerial.h>
#include "serial_bus_servo.h"

#undef USE_GUN_ENABLE_PIN

#define DART_PRESENT_IN       3   // Active low
#define GUN_ENABLE_IN         5   // Active low  (not current used)
#define SWITCH_IN             7   // Active low momentary switch

#define MOTOR_PWM_OUT         11  // Active low
#define BARREL_LED_STRIP_OUT  4   // Active hi
#define TARGETING_LASER_OUT   6   // Aiming laser

namespace {

const float PAN_TILT_MAX_ANGLE_DEG = 270.0f;
const float TILT_0_DEG = (-11.7f + PAN_TILT_MAX_ANGLE_DEG/2.0f);
const float PAN_0_DEG = (8.3f + PAN_TILT_MAX_ANGLE_DEG/2.0f);
const float TILT_ABS_MAX_DEG = 25.0f;
const float PAN_ABS_MAX_DEG = 45.0f;

const float FIRE_SERVO_FIRE_DEG = 160.0f;
const float FIRE_SERVO_RESET_DEG = 0.0f;

const uint8_t FLYWHEEL_SPEED_FULL = 255;
const uint8_t FLYWHEEL_SPEED_MED= 150;
const uint8_t FLYWHEEL_SPEED_LOW = 125;
const uint8_t FLYWHEEL_SPEED_STOP = 0;

const uint32_t FLYWHEEL_SPIN_UP_DELAY = 1000;
const uint32_t DART_PRESENT_CK_DELAY = 3000;
const uint32_t PUSH_BULLET_DURATION = 500;
const uint32_t PUSH_BULLET_DELAY = (PUSH_BULLET_DURATION + 100);

const uint32_t PAN_TILT_MOVE_MS_PER_DEG = 10;

SerialServo servo_fire(Serial1, 1, 240, 1000, false);
// +degrees tilts up
SerialServo servo_tilt(Serial1, 2, 240, 1000, false);
// 
SerialServo servo_pan(Serial1, 3, 240, 1000, false);

bool invert_pwm = true;
bool flywheel_on = false;
bool firing = false;

uint8_t fly_wheel_speed = FLYWHEEL_SPEED_FULL;

float cur_pan_deg = -1.0f;
float cur_tilt_deg = -1.0f;

// Pan/Tilt testing related
int pan_test_idx = 0;
int tilt_test_idx = 0;

float pan_test_angles[] = {0.0f, 20.0f, -20.0f, 0.0f, 0.0f, 0.0f, 10.0f};
const int num_pan_test_angles = sizeof(pan_test_angles)/sizeof(float);
float tilt_test_angles[] = {0.0f, 0.0f, 0.0f, 0.0f, 10.0f, -10.0f, 10.0f};
const int num_tilt_test_angles = sizeof(tilt_test_angles)/sizeof(float);

template <typename T>
constexpr int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

void set_flywheel_speed(uint8_t speed)
{
  analogWrite(MOTOR_PWM_OUT, invert_pwm? 255 - speed: speed);
}

void set_flywheel_off()
{
  set_flywheel_speed(FLYWHEEL_SPEED_STOP);
  flywheel_on = false;
}

void set_flywheel_on()
{
  if (flywheel_on) {
    return;
  }
  set_flywheel_speed(fly_wheel_speed);
  flywheel_on = true;
  delay(FLYWHEEL_SPIN_UP_DELAY);
}

void set_push_bullet_into_flywheel(bool push_in)
{
  servo_fire.move(push_in? FIRE_SERVO_FIRE_DEG: FIRE_SERVO_RESET_DEG, PUSH_BULLET_DURATION);
  delay(PUSH_BULLET_DELAY);
}

void set_barrel_led(bool on)
{
  digitalWrite(BARREL_LED_STRIP_OUT, on);
}

void set_targeting_laser(bool on)
{
  digitalWrite(TARGETING_LASER_OUT, on);
}

bool is_dart_present()
{
  return digitalRead(DART_PRESENT_IN);
}

bool is_gun_enabled()
{
#if defined(USE_GUN_ENABLE_PIN)  
  if (digitalRead(GUN_ENABLE_IN)) {
    set_flywheel_off();
    Serial.println("gun disabled");
    return false;
  }
  return true;
#else
  return true;
#endif  
}

bool wait_for_button_press(uint32_t max_wait_ms)
{
  auto start = millis();

  do {
    if (!digitalRead(SWITCH_IN)) {
      return true;
    }
  } while (millis() - start < max_wait_ms);
  return false;
}

void aim(float pan_deg, float tilt_deg)
{
  if (abs(pan_deg) > PAN_ABS_MAX_DEG) {
    pan_deg = PAN_ABS_MAX_DEG*sgn(pan_deg);
  }

  if (abs(tilt_deg) > TILT_ABS_MAX_DEG) {
    tilt_deg = TILT_ABS_MAX_DEG*sgn(tilt_deg);
  }

  if (cur_pan_deg == pan_deg &&
      cur_tilt_deg == tilt_deg) {
    Serial.println("aim: at position");
    return;
  }

#if 0  
  auto pan_duration = PAN_TILT_MOVE_MIN_DURATION + static_cast<uint32_t>(PAN_TILT_MOVE_MAX_DURATION*
                      abs(cur_pan_deg - pan_deg)/PAN_TILT_MAX_ANGLE_DEG);
  auto tilt_duration = PAN_TILT_MOVE_MIN_DURATION + static_cast<uint32_t>(PAN_TILT_MOVE_MAX_DURATION*
                      abs(cur_tilt_deg - tilt_deg)/PAN_TILT_MAX_ANGLE_DEG);
#endif
  auto pan_duration = static_cast<uint32_t>(PAN_TILT_MOVE_MS_PER_DEG*abs(cur_pan_deg - pan_deg));
  auto tilt_duration = static_cast<uint32_t>(PAN_TILT_MOVE_MS_PER_DEG*abs(cur_tilt_deg - tilt_deg));

  auto abs_pan_deg = min(max(0.0f, PAN_0_DEG + pan_deg), PAN_TILT_MAX_ANGLE_DEG);
  auto abs_tilt_deg = min(max(0.0f, TILT_0_DEG + tilt_deg), PAN_TILT_MAX_ANGLE_DEG);

  servo_pan.move(abs_pan_deg, pan_duration);
  servo_tilt.move(abs_tilt_deg, tilt_duration);

  auto delay_ms = max(pan_duration, tilt_duration);
  delay(delay_ms);

  Serial.print("Pan angle: ");
  Serial.println(pan_deg);
  Serial.print("Pan duration: ");
  Serial.println(pan_duration);

  Serial.print("Tilt angle: ");
  Serial.println(tilt_deg);
  Serial.print("Tilt duration: ");
  Serial.println(tilt_duration);

  Serial.print("Pan/tilt delay: ");
  Serial.println(delay_ms);

  cur_pan_deg = pan_deg;
  cur_tilt_deg = tilt_deg;
}

void exercise_aiming()
{
  aim(pan_test_angles[pan_test_idx % num_pan_test_angles], tilt_test_angles[tilt_test_idx % num_tilt_test_angles]);

  Serial.print("pan idx ");
  Serial.println(pan_test_idx % num_pan_test_angles);

  Serial.print("tilt idx ");
  Serial.println(tilt_test_idx % num_tilt_test_angles);

  ++pan_test_idx;
  ++tilt_test_idx;
}

void run_aiming_test()
{
  exercise_aiming();
  delay(2000);
}

void run_test()
{
  Serial.println("Test barrel LED");

  for (int i = 0; i < 3; ++i) {
    set_barrel_led(true);
    delay(500);
    set_barrel_led(false);
    delay(250);
  }

  if (!wait_for_button_press(15000)) {
    return;
  }

  Serial.println("Test targeting laser");

  for (int i = 0; i < 3; ++i) {
    set_targeting_laser(true);
    delay(500);
    set_targeting_laser(false);
    delay(250);
  }

  if (!wait_for_button_press(5000)) {
    return;
  }

  Serial.println("Test bullet pusher");

  set_push_bullet_into_flywheel(true);
  delay(1000);
  set_push_bullet_into_flywheel(false);
  delay(1000);

  if (!wait_for_button_press(5000)) {
    return;
  }

  Serial.println("Test flywheel");

  // Test flywheel
  auto saved_speed = fly_wheel_speed;

  fly_wheel_speed = FLYWHEEL_SPEED_FULL;
  set_flywheel_on();
  delay(3000);
  set_flywheel_off();

  fly_wheel_speed = FLYWHEEL_SPEED_MED;
  set_flywheel_on();
  delay(3000);
  set_flywheel_off();

  fly_wheel_speed = FLYWHEEL_SPEED_LOW;
  set_flywheel_on();
  delay(3000);
  set_flywheel_off();

  fly_wheel_speed = saved_speed;

  if (!wait_for_button_press(5000)) {
    return;
  }

  Serial.println("Test bullet sensor. Manually block sensor");

  {
    auto start = millis();
    do {
      if (is_dart_present()) {
        set_push_bullet_into_flywheel(true);
        delay(1000);
        set_push_bullet_into_flywheel(false);
        delay(1000);
        break;
      }
    } while (millis() - start < 10000);
  }

  if (!wait_for_button_press(5000)) {
    return;
  }

  Serial.println("Test panning");

  delay(2000);

  // Test panning
  aim(-45.0f, 0.0f);
  delay(2000);
  aim(0.0f, 0.0f);
  delay(2000);
  aim(45.0f, 0.0f);
  delay(2000);
  aim(-45.0f, 0.0f);
  delay(2000);
  aim(0.0f, 0.0f);

  if (!wait_for_button_press(5000)) {
    return;
  }

  Serial.println("Test tilting");

  delay(2000);

  // Test tilting
  aim(0.0f, -25.0f);
  delay(2000);
  aim(0.0f, 0.0f);
  delay(2000);
  aim(0.0f, 25.0f);
  delay(2000);
  aim(0.0f, -25.0f);
  delay(2000);
  aim(0.0f, 0.0f);

  Serial.println("Press button to finish");

  if (!wait_for_button_press(5000)) {
    return;
  }

  // Acknowledge end of test
  for (int i = 0; i < 6; ++i) {
    set_barrel_led(true);
    delay(500);
    set_barrel_led(false);
    delay(250);
  }
}

void run_firing()
{
  if (!is_gun_enabled()) {
    return;
  }

  // Wait for a dart to be present
  if (!is_dart_present()) {
    Serial.println("empty");
    set_flywheel_off();
    delay(DART_PRESENT_CK_DELAY);
    return;
  }
  
  exercise_aiming();

  set_flywheel_on();

  Serial.println("firing");
  set_barrel_led(true);

  set_push_bullet_into_flywheel(true);
  set_barrel_led(false);

  Serial.println("reseting");
  set_push_bullet_into_flywheel(false);
}

} // namespace

void setup()
{
  Serial.begin(9600);
  // Serial 1 is used for servo control.  Invert signals for compatibility
  // with hardware interface.
  Serial1.begin(115200, SERIAL_8N1, 7, 8, true);  

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DART_PRESENT_IN, INPUT_PULLUP);
  pinMode(GUN_ENABLE_IN, INPUT_PULLUP);
  pinMode(MOTOR_PWM_OUT, OUTPUT);
  pinMode(BARREL_LED_STRIP_OUT, OUTPUT);
  pinMode(TARGETING_LASER_OUT, OUTPUT);
  
  set_barrel_led(false);
  set_targeting_laser(true);
  set_flywheel_off();

  delay(100);

  aim(0.0f, 0.0f);
  set_push_bullet_into_flywheel(false);

  delay(1000);

  if (wait_for_button_press(2000)) {
    run_test();
  }

  Serial.println("running normal mode");
  set_barrel_led(true);
  delay(500);
  set_barrel_led(false);
  delay(250);
}

void loop()
{
  run_firing();
}
