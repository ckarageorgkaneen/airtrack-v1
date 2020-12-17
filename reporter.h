#ifndef REPORTER_MODULE
#define REPORTER_MODULE

#import <Arduino.h>
#include "definitions.h"

#define PWM_(INDEX, TOTAL)  (byte)((255.0/TOTAL)*INDEX)


void reportPWM (PIN_TYPE pwm_pin, byte value)
{
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // If Arduino Mega is used
        analogWrite(pwm_pin, value);
    #endif
}


struct CurrentLaneNumPWM { enum : byte
{
    Unknown = 0,
    Lane1 = PWM_(1,4),
    Lane2 = PWM_(2,4),
    Lane3 = PWM_(3,4),
    Lane4 = PWM_(4,4)
};};

struct CorrectLaneNumPWM { enum : byte
{
    Unknown = 0,
    Lane1 = PWM_(1,4),
    Lane2 = PWM_(2,4),
    Lane3 = PWM_(3,4),
    Lane4 = PWM_(4,4)
};};

struct LaneStatePwm { enum : byte
{
    Unknown = 0,
    InsideLane = PWM_(1,2),
    OutsideLane = PWM_(2,2),
};};

// Copied from "TPixy.h"
// Pixy x-y position values
#define PIXY_MIN_X                  0L
#define PIXY_MAX_X                  319L
#define PIXY_MIN_Y                  0L
#define PIXY_MAX_Y                  199L
struct PixyXPwmStruct
{
    byte convert (float x) /*x is uint16_t*/
    {
        return (byte)((x / PIXY_MAX_X) * 255);
    }
} PixyXPwm;
struct PixyYPwmStruct
{
    byte convert (float y) /*y is uint16_t*/
    {
        return (byte)((y / PIXY_MAX_Y) * 255);
    }
} PixyYPwm;
struct PixyAnglePwmStruct
{
    byte convert (int angle) /*angle is int16_t*/
    {
        angle = (angle + 360) % 360;
        return (byte)((angle / 360.0) * 255);
    }
} PixyAnglePwm;

struct LaneLedPwm { enum : byte
{
    Off = PWM_(0,1),
    On = PWM_(1,1),
};};

struct RotationDirectionPwm { enum : byte
{
    Unknown = 0,
    Clockwise = PWM_(1,2),
    AntiClockwise = PWM_(2,2),
    // MIXED = PWM_(3,3),
};};

struct LickDirectionPwm { enum : byte
{
    Unknown = 0,
    Untouched = PWM_(1,3),
    Right = PWM_(2,3),
    Left = PWM_(3,3),
};};

struct DecisionResultPwm { enum : byte
{
    Unknown = 0,
    Correct = PWM_(1,3),
    Wrong = PWM_(2,3),
    Miss = PWM_(3,3),
};};

struct RewardResultPwm { enum : byte
{
    Unknown = 0,
    Given = PWM_(1,3),
    NotGiven = PWM_(2,3),
    AutomatedGiven = PWM_(3,3),
};};

struct RewardStatePwm { enum : byte
{
    Unknown = 0,
    Normal = PWM_(1,3),
    ForcedLeft = PWM_(2,3),
    ForcedRight = PWM_(3,3),
};};

struct PiezoAlarmPwm { enum : byte
{
    Off = PWM_(0,1),
    On = PWM_(1,1),
};};

struct SolenoidPwm { enum : byte
{
    Unknown = 0,
    LeftOpen = PWM_(1,3),
    BothOff = PWM_(2,3),
    RightOpen = PWM_(3,3),
};};

struct ActuatorPwm { enum : byte
{
    Unknown = 0,
    MaxDistance = PWM_(1,4),
    Pushing = PWM_(2,4),
    Retracting = PWM_(3,4),
    MinDistance = PWM_(4,4),
};};

#endif
