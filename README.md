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

### TODO
  * Add control via REST interface and a simple webui.

