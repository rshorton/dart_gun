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
#include <ArduinoJson.h> // Requires the "ArduinoJson" library by Benoit Blanchon
#include <SoftwareSerial.h>
#include "serial_bus_servo.h"

#undef USE_GUN_ENABLE_PIN

#define MAX_LOG_MSG_LEN       100
#define LOG_LVL_DEBUG         1
#define LOG_LVL_NONE          99

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
const uint32_t DART_PRESENT_CK_DELAY = 1000;
const uint32_t PUSH_BULLET_DURATION = 500;
const uint32_t PUSH_BULLET_DELAY = (PUSH_BULLET_DURATION + 100);

const uint32_t PAN_TILT_MOVE_MS_PER_DEG = 10;

SerialServo servo_fire(Serial1, 1, 240, 1000, false);
// +degrees tilts up
SerialServo servo_tilt(Serial1, 2, 240, 1000, false);
// +degrees pans left
SerialServo servo_pan(Serial1, 3, 240, 1000, false);

bool invert_pwm = true;
bool flywheel_on = false;
bool firing = false;

uint8_t fire_cnt = 0;
float req_pan_angle = 0.0f;
float req_tilt_angle = 0.0f;

String inputBuffer = "";

uint8_t fly_wheel_speed = FLYWHEEL_SPEED_FULL;

float cur_pan_deg = -1.0f;
float cur_tilt_deg = -1.0f;

int log_level = LOG_LVL_NONE;

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

void log_message(int level, const char * fmt, ...)
{
  if (level < log_level) {
    return;
  }
	char buf[MAX_LOG_MSG_LEN];
  va_list args;
  va_start(args, fmt);

  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  //Serial.println(buf);
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
    log_message("gun disabled");
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
    log_message(LOG_LVL_DEBUG, "aim: at position");
    return;
  }

  auto pan_duration = static_cast<uint32_t>(PAN_TILT_MOVE_MS_PER_DEG*abs(cur_pan_deg - pan_deg));
  auto tilt_duration = static_cast<uint32_t>(PAN_TILT_MOVE_MS_PER_DEG*abs(cur_tilt_deg - tilt_deg));

  auto abs_pan_deg = min(max(0.0f, PAN_0_DEG + pan_deg), PAN_TILT_MAX_ANGLE_DEG);
  auto abs_tilt_deg = min(max(0.0f, TILT_0_DEG + tilt_deg), PAN_TILT_MAX_ANGLE_DEG);

  servo_pan.move(abs_pan_deg, pan_duration);
  servo_tilt.move(abs_tilt_deg, tilt_duration);

  auto delay_ms = max(pan_duration, tilt_duration);
  delay(delay_ms);

  log_message(LOG_LVL_DEBUG, "Pan angle: %f, duration: %d", pan_deg, pan_duration);
  log_message(LOG_LVL_DEBUG, "Tilt angle: %f, duration: %d", tilt_deg, tilt_duration);
  log_message(LOG_LVL_DEBUG, "Pan/tilt delay: %d", delay_ms);

  cur_pan_deg = pan_deg;
  cur_tilt_deg = tilt_deg;
}

void exercise_aiming()
{
  aim(pan_test_angles[pan_test_idx % num_pan_test_angles], tilt_test_angles[tilt_test_idx % num_tilt_test_angles]);

  log_message(LOG_LVL_DEBUG, "pan idx %d", pan_test_idx % num_pan_test_angles);
  log_message(LOG_LVL_DEBUG, "tilt idx %d", tilt_test_idx % num_tilt_test_angles);

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
  auto saved_log_level = log_level;
  log_level = LOG_LVL_DEBUG;

  log_message(LOG_LVL_DEBUG, "Test barrel LED");

  for (int i = 0; i < 3; ++i) {
    set_barrel_led(true);
    delay(500);
    set_barrel_led(false);
    delay(250);
  }

  if (!wait_for_button_press(15000)) {
    return;
  }

  log_message(LOG_LVL_DEBUG, "Test targeting laser");

  for (int i = 0; i < 3; ++i) {
    set_targeting_laser(true);
    delay(500);
    set_targeting_laser(false);
    delay(250);
  }

  if (!wait_for_button_press(5000)) {
    return;
  }

  log_message(LOG_LVL_DEBUG, "Test bullet pusher");

  set_push_bullet_into_flywheel(true);
  delay(1000);
  set_push_bullet_into_flywheel(false);
  delay(1000);

  if (!wait_for_button_press(5000)) {
    return;
  }

  log_message(LOG_LVL_DEBUG, "Test flywheel");

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

  log_message(LOG_LVL_DEBUG, "Test bullet sensor. Manually block sensor");

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

  log_message(LOG_LVL_DEBUG, "Test panning");

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

  log_message(LOG_LVL_DEBUG, "Test tilting");

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

  log_message(LOG_LVL_DEBUG, "Press button to finish");

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

  log_level = saved_log_level;
}

void run_firing()
{
  if (!is_gun_enabled()) {
    return;
  }

  if (fire_cnt == 0) {
    set_flywheel_off();
    return;
  }
  --fire_cnt;

  // Wait for a dart to be present
  if (!is_dart_present()) {
    log_message(LOG_LVL_DEBUG, "empty");
    set_flywheel_off();
    delay(DART_PRESENT_CK_DELAY);
    return;
  }
  
  //exercise_aiming();
  aim(req_pan_angle, req_tilt_angle);

  set_flywheel_on();

  log_message(LOG_LVL_DEBUG, "firing");
  set_barrel_led(true);

  set_push_bullet_into_flywheel(true);
  set_barrel_led(false);

  log_message(LOG_LVL_DEBUG, "reseting");
  set_push_bullet_into_flywheel(false);
}

// Packs current system status variables into a JSON object and prints to Serial
void send_status_response() {
  StaticJsonDocument<128> responseDoc;
  
  responseDoc["empty"] = !is_dart_present();
  responseDoc["pan_angle"] = cur_pan_deg;
  responseDoc["tilt_angle"] = cur_tilt_deg;

  serializeJson(responseDoc, Serial);
  Serial.println(); // Send newline terminator
}

// Parses the JSON command and executes the matching logic
void execute_command(String jsonString)
{
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
    return; // Ignore malformed JSON packets
  }

  const char* command_name = doc["name"];
  if (!command_name) return;

  if (strcmp(command_name, "fire") == 0) {
    // Extract parameters
    uint8_t speed = doc["args"]["speed"] | 0;
    int8_t pan_angle = doc["args"]["pan_angle"] | 0;
    int8_t tilt_angle = doc["args"]["tilt_angle"] | 0;
    uint8_t count = doc["args"]["count"] | 0;

    req_pan_angle = pan_angle;
    req_tilt_angle = tilt_angle;

    if (speed == 3) {
      fly_wheel_speed = FLYWHEEL_SPEED_FULL;
    } else if (speed == 2) {
      fly_wheel_speed = FLYWHEEL_SPEED_MED;
    } else {
      fly_wheel_speed = FLYWHEEL_SPEED_LOW;
    }

    fire_cnt += count;

    send_status_response();
  } 
  else if (strcmp(command_name, "reset") == 0) {
    // Reset state to defaults
    aim(0.0f, 0.0f);
    send_status_response();
  } 
  else if (strcmp(command_name, "get_status") == 0) {
    send_status_response();
  }
}

// Reads incoming serial data until a newline character is found
void process_commands()
{
  while (Serial.available() > 0) {
    char incomingChar = (char)Serial.read();
    
    if (incomingChar == '\n') {
      execute_command(inputBuffer);
      inputBuffer = "";
    } else if (incomingChar != '\r') {
      inputBuffer += incomingChar;
    }
  }
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

  log_message(LOG_LVL_DEBUG, "running normal mode");
  set_barrel_led(true);
  delay(500);
  set_barrel_led(false);
  delay(250);
}

void loop()
{
  if (fire_cnt == 0) {
    process_commands();
  }    
  run_firing();
}
