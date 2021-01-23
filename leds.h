#ifndef LEDS_MODULE
#define LEDS_MODULE

#include <Arduino.h>
#include "definitions.h"

struct LedsStruct
{
    // CONST_PIN_TYPE Unused = 1;

    CONST_PIN_TYPE SensorLeft = 18;
    CONST_PIN_TYPE SensorRight = 19;

    //CONST_PIN_TYPE Solenoid = 9;

    // CONST_PIN_TYPE ActuatorPush = 10;
    // CONST_PIN_TYPE ActuatorPull = 11;
} Leds;

#endif
