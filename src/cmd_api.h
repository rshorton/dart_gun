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

#ifndef CMD_API_H
#define CMD_API_H

#include "Arduino.h"
#include <ArduinoJson.h>

#if USE_WIFI == 1  
#include <WiFi.h>
#include <WebServer.h>
#endif

#include "gun_hw_if.h"
#include "gun_control_sm.h"

class CommandAPI
{
public:
    CommandAPI(GunHardwareInterface &hw_if, AimingControl &aiming_control, GunControlStateMachine &control_sm);
    void init();
    void update();

private:
    void get_status_json(JsonDocument& doc);
    void send_status_response();
    void set_command_result(JsonDocument& doc, GunControlStateMachine::CmdResult result);
    void send_cmd_response(GunControlStateMachine::CmdResult result);

    void execute_command(String jsonString);
    void process_commands();

#if USE_WIFI == 1  
    static void handle_rest_get_status();
    static void handle_rest_reset();
    static void handle_rest_fire();
    static void handle_options();
    void init_server();
#endif

    GunHardwareInterface &hw_if_;
    AimingControl &aiming_control_;
    GunControlStateMachine &control_sm_; 

    String input_buffer;

#if USE_WIFI == 1  
    WebServer server_;

    // These are defined in platformio.ini and ENV vars
    const char* ssid_ = WIFI_SSID;
    const char* password_ = WIFI_PW;
#endif
};

#endif // CMD_API_H
