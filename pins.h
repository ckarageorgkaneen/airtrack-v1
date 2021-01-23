#ifndef PINS_MODULE
#define PINS_MODULE

#import <Arduino.h>
#include "definitions.h"

struct PinStruct
{

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) // If Arduiono Uno is used
    #pragma message("Compiling airtrack for Arduino Uno - PWM pins has no effect")
    #define NO_EFFECT_PIN 13
    #define USE_PIN(PIN_NUM)  NO_EFFECT_PIN
    #define ARDUINO_UNO
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // If Arduino Mega is used
    #pragma message("Compiling airtrack for Arduino Mega - Using PWM pins")
    #define USE_PIN(PIN_NUM)  PIN_NUM
    #define ARDUINO_MEGA
#else
    #warning "Unknown Arduino board type, PWM works might not work or conflict"
    #define USE_PIN(PIN_NUM)  PIN_NUM
#endif


#ifdef ARDUINO_UNO
    // Pins configuration for arduino uno
    CONST_PIN_TYPE Sensor = 2;

    CONST_PIN_TYPE ActuatorPush = 4;
    CONST_PIN_TYPE ActuatorPull = 3;

    CONST_PIN_TYPE SolenoidLeft = 5;
    CONST_PIN_TYPE SolenoidRight = 8;

    CONST_PIN_TYPE LaneLight = 6;

    CONST_PIN_TYPE PeizoTone = 7;

#else #if def ARDUINO_MEGA
    // Pins configuration for arduino mega
    CONST_PIN_TYPE Sensor = 38;

    CONST_PIN_TYPE ActuatorPush = 30;
    CONST_PIN_TYPE ActuatorPull = 32;

    // 26 and 28 are free

    CONST_PIN_TYPE SolenoidLeft = 22;
    CONST_PIN_TYPE SolenoidRight = 24;

    CONST_PIN_TYPE LaneLight = 27;
    CONST_PIN_TYPE LaneLight2 = A14;

    CONST_PIN_TYPE PeizoTone = 29;
#endif #endif


    // Has no effect except on arduino mega
    CONST_PIN_TYPE PWM_CurrentLaneNum = USE_PIN(8);
    CONST_PIN_TYPE PWM_CorrectLaneNum = USE_PIN(8);
    CONST_PIN_TYPE PWM_LaneState = USE_PIN(4);
    CONST_PIN_TYPE PWM_PixyX = USE_PIN(8);
    CONST_PIN_TYPE PWM_PixyY = USE_PIN(8);
    CONST_PIN_TYPE PWM_PixyAngle = USE_PIN(8);
    CONST_PIN_TYPE PWM_LaneLed = USE_PIN(8);

    CONST_PIN_TYPE PWM_RotationDirection = USE_PIN(9);
    CONST_PIN_TYPE PWM_LickDirection = USE_PIN(10);
    CONST_PIN_TYPE PWM_DecisionResult = USE_PIN(11);
    CONST_PIN_TYPE PWM_RewardResult = USE_PIN(12);
    //CONST_PIN_TYPE PWM_RewardState = USE_PIN(13);
    CONST_PIN_TYPE PWM_PiezoAlarm = USE_PIN(44);
    CONST_PIN_TYPE PWM_Solenoid = USE_PIN(45);
    CONST_PIN_TYPE PWM_Actuator = USE_PIN(46);

    CONST_PIN_TYPE OutOfFirstLane = USE_PIN(39);
    CONST_PIN_TYPE InCorrecctLane = USE_PIN(41);
} Pins;

#endif
