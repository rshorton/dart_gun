# Dart Gun

This project implements a nerf dart gun controller for a custom dart gun.  This code runs on an Arduino Nano ESP32.

The dart gun consists of these mechanical components:
  *  Propeller flywheels spun using two 9v DC motors.  These flywheels are separated such that a foam dart can be pushed between them and then propelled-outward by the spinning motion of the wheels.  The flywheel assembly from a Zuru brand XShot Insanity was used.
  *  Dart magazine which holds up to 16 darts.  It uses a spring to push the darts to the gun chamber. The magazine(s) from the Xshot was also used.
  *  Dart pusher which moves the top-most dart of the magazine into the flywheel module.  This was implemented by a 3D-printed rack and pinon assembly.  The pinon gear is turned by a servo.
  *  Pan-tilt base using two servos for aiming the gun.
  *  Filament LED for cosmetic effect.
  *  Laser pointer for a general indication of where the gun is pointed.

  Parts:
    *  Servos - Hiwonder LX-16A
    *  Laser - 650nm red laser module with adjustable focal length
    *  Arduino Nano ESP32
    *  LED drivers
    *  MOSFET for controller the motors
    *  Custom 3D printed parts


## Control Interfaces

The dart gun can be controlled by either a serial port REST interface via WIFI.  Both use similar JSON-based command structures.

### Serial port

Configure your serial monitor to 115200 baud and set the line ending option to Newline (LF).

#### Commands

**Fire** - Sends command to fire one or more darts

* **Send:**
    
  {"name":"fire", "args":{"speed": (3-full, 2-med, 1-low),"pan_angle": (-45 to 45 degress),"tilt_angle": (-30 to 30 degrees),"count": (number of times to fire>)}}
    
* **Receives:**

  {"empty": (true if no more darts in magazine),"pan_angle": (angle),"tilt_angle": (angle)}
    
**Reset** - Resets pan-tilt position

* **Send:**

  {"name":"reset"}

* **Receive:** same as fire command
    
**Get Status** - Gets the status

* **Send:**

  {"name":"get_status"}

* **Receive:** same as fire command

### REST

The build looks for two ENV variables to specify how to connect WIFI:

* USE_WIFI  (0 or 1)
* WIFI_SSID
* WIFI_PW

Set USE_WIFI in the platform.ini file.  You can also set WIFI_SSID and WIFI_PW in that file. Or, if using VSCode, you can specify the values in the settings.ini file using:

    "terminal.integrated.env.linux": {
       "DART_GUN_WIFI_SSID": "<set this>",
       "DART_GUN_WIFI_PW": "<set this>"
    },

The latter avoids accidently checking in the pw in the platform.ini file.

**Get Status**

* **Method:** GET /api/status

* **Sent:** None

* **Response (200 OK):** same as for serial commands

* **Test command:**

```
 curl -X GET http://192.168.1.159/api/status
```

#### Reset

* **Method:** POST /api/resetPayload

* **Sent:** None

* **Response (200 OK):** same as for serial commands

* **Test command:**

```
 curl -X POST http://192.168.1.159/api/reset
```

#### Fire

* **Method:** POST /api/fire

* **Header Required:** Content-Type: application/jsonPayload

* **Sent:**

* json{
  "speed": <see serial fire command>,
  "pan_angle": <see serial fire command>,
  "tilt_angle":<see serial fire command>,
  "count": <see serial fire command>
}

* **Response (200 OK):** same as for serial commands

* **Test command:**

```
curl -X POST http://192.168.1.159/api/fire \
     -H "Content-Type: application/json" \
     -d '{"speed":3,"pan_angle":-30,"tilt_angle":15,"count":1}'
```

