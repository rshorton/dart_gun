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
#include "cmd_api.h"

namespace
{
    CommandAPI *api = nullptr;
}

CommandAPI::CommandAPI(GunHardwareInterface &hw_if, AimingControl &aiming_control,
                       GunControlStateMachine &control_sm):
    hw_if_(hw_if),
    aiming_control_(aiming_control),
    control_sm_(control_sm)
{
    api = this;
}    

void CommandAPI::init()
{
#if USE_WIFI==1  
    init_server();
#endif  
}
void CommandAPI::update()
{
    process_commands();
#if USE_WIFI==1  
    server_.handleClient(); 
#endif
}

void CommandAPI::get_status_json(JsonDocument& doc)
{
    doc["pending"] = control_sm_.is_firing_pending();
    doc["empty"] = control_sm_.is_magazine_empty();
    doc["aimed"] = aiming_control_.is_aimed();
    doc["pan_angle"] = aiming_control_.get_cur_pan_angle();
    doc["tilt_angle"] = aiming_control_.get_cur_tilt_angle();
}

void CommandAPI::send_status_response()
{
    StaticJsonDocument<128> response_doc;
    get_status_json(response_doc);
    serializeJson(response_doc, Serial);
    Serial.println();
}

void CommandAPI::set_command_result(JsonDocument& doc, GunControlStateMachine::CmdResult result)
{
    doc["cmd_result"] = GunControlStateMachine::get_cmd_result_str(result);
}

void CommandAPI::send_cmd_response(GunControlStateMachine::CmdResult result)
{
    StaticJsonDocument<128> response_doc;
    get_status_json(response_doc);
    set_command_result(response_doc, result);
    serializeJson(response_doc, Serial);
    Serial.println();
}

void CommandAPI::execute_command(String jsonString)
{
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonString);

    if (error) {
        return;
    }

    const char* command_name = doc["name"];
    if (!command_name) {
        return;
    }

    if (strcmp(command_name, "fire") == 0) {
        // Extract parameters
        uint8_t speed = doc["args"]["speed"] | 0;
        int8_t pan_angle = doc["args"]["pan_angle"] | 0;
        int8_t tilt_angle = doc["args"]["tilt_angle"] | 0;
        uint8_t count = doc["args"]["count"] | 0;

        auto result = control_sm_.fire_cmd(speed, pan_angle, tilt_angle, count);
        Logging::log_message(LOG_LVL_INFO, "Api, serial fire: speed: %d, pan: %d, tilt: %d, count: %d, result: %s",
                             speed, pan_angle, tilt_angle, count,
                             GunControlStateMachine::get_cmd_result_str(result));

        send_cmd_response(result);

    } else if (strcmp(command_name, "reset") == 0) {
        auto result = control_sm_.reset_cmd();
        send_cmd_response(result);

    } else if (strcmp(command_name, "get_status") == 0) {
        send_status_response();
    }
}

// Reads incoming serial data until a newline character is found
void CommandAPI::process_commands()
{
    if (Serial.available() <= 0) {
        return;
    }

    while (Serial.available() > 0) {
        char incomingChar = (char)Serial.read();

        if (incomingChar == '\n') {
            execute_command(input_buffer);
            input_buffer = "";
            break;

        } else if (incomingChar != '\r') {
            input_buffer += incomingChar;
        }
    }
}

#if USE_WIFI == 1  

void CommandAPI::handle_rest_get_status()
{
    if (!api) {
        return;
    }
    api->server_.sendHeader("Access-Control-Allow-Origin", "*");
    StaticJsonDocument<128> responseDoc;
    api->get_status_json(responseDoc);
  
    String responseBuffer;
    serializeJson(responseDoc, responseBuffer);
    api->server_.send(200, "application/json", responseBuffer);
}

void CommandAPI::handle_rest_reset()
{
    if (!api) {
        return;
    }
    auto result = api->control_sm_.reset_cmd();

    api->server_.sendHeader("Access-Control-Allow-Origin", "*");

    StaticJsonDocument<128> responseDoc;
    api->get_status_json(responseDoc);
    api->set_command_result(responseDoc, result);
  
    String responseBuffer;
    serializeJson(responseDoc, responseBuffer);
    api->server_.send(200, "application/json", responseBuffer);
}

void CommandAPI::handle_rest_fire()
{
    if (!api) {
        return;
    }
    api->server_.sendHeader("Access-Control-Allow-Origin", "*");

    if (!api->server_.hasArg("plain")) {
        api->server_.send(400, "application/json", "{\"error\":\"Missing body\"}");
        return;
    }

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, api->server_.arg("plain"));

    if (error) {
        api->server_.send(400, "application/json", "{\"error\":\"Malformed JSON\"}");
        return;
    }

    // Extract parameters securely from the incoming REST request body
    uint8_t speed = doc["speed"] | 0;
    int8_t pan_angle = doc["pan_angle"] | 0;
    int8_t tilt_angle = doc["tilt_angle"] | 0;
    uint8_t count = doc["count"] | 0;

    auto result = api->control_sm_.fire_cmd(speed, pan_angle, tilt_angle, count);
    Logging::log_message(LOG_LVL_INFO, "Api, rest fire: speed: %d, pan: %d, tilt: %d, count: %d, result: %s",
                         speed, pan_angle, tilt_angle, count,
                         GunControlStateMachine::get_cmd_result_str(result));

    StaticJsonDocument<128> responseDoc;
    api->get_status_json(responseDoc);
    api->set_command_result(responseDoc, result);
  
    String responseBuffer;
    serializeJson(responseDoc, responseBuffer);
    api->server_.send(200, "application/json", responseBuffer);
}

void CommandAPI::handle_options()
{
    if (!api) {
        return;
    }
    api->server_.sendHeader("Access-Control-Allow-Origin", "*");
    api->server_.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    api->server_.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    api->server_.send(204); // No content needed for preflight
}

void CommandAPI::init_server()
{
    // Initialize Network Connection
    Logging::log_message(LOG_LVL_INFO, "Connecting to Wi-Fi Network: %s", ssid_);
    WiFi.begin(ssid_, password_);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Logging::log_message(LOG_LVL_INFO, "\nWi-Fi Connected Successfully!");
    Logging::log_message(LOG_LVL_INFO, "Local IP Address to query REST: %s", WiFi.localIP().toString());

    // Define API Router Endpoints
    server_.on("/api/fire", HTTP_OPTIONS, handle_options);  
    server_.on("/api/status", HTTP_GET, handle_rest_get_status);
    server_.on("/api/reset", HTTP_POST, handle_rest_reset);
    server_.on("/api/fire", HTTP_POST, handle_rest_fire);
  
    server_.begin();
    Logging::log_message(LOG_LVL_INFO, "REST Web Server Engine started on port 80.");
}
#endif

