#ifndef TESTING__H
#define TESTING__H

#include "Arduino.h"
#include "gun_hw_if.h"
#include "aiming_control.h"

namespace Testing
{
    void run_all_test(GunHardwareInterface &hw_if, AimingControl &aiming_control);
}

#endif // TESTING__H