#ifndef STATS_MODULE
#define STATS_MODULE


#include <Arduino.h>
#include "definitions.h"
#include "pins.h"
#include "reporter.h"

struct StatsStruct
{
    StatsMessage ENTERED_LANE(short lane)
    {
      reportPWM(Pins.PWM_LaneState, LaneStatePwm::InsideLane);
      return StatsMessage(0, "Entered lane\0", lane + 1);
    }

    StatsMessage EXITED_LANE(short lane)
    {
        reportPWM(Pins.PWM_LaneState, LaneStatePwm::OutsideLane);
        return StatsMessage(1, "Exited lane\0", lane + 1);
    }

    StatsMessage MOTOR_PUSHED()
    {
        reportPWM(Pins.PWM_Actuator, ActuatorPwm::Pushing);
        return StatsMessage(10, "Motor is pushed\0");
    }

    StatsMessage MOTOR_PULLED()
    {
        reportPWM(Pins.PWM_Actuator, ActuatorPwm::Retracting);
        return StatsMessage(11, "Motor is pulled\0");
    }

    StatsMessage MOTOR_MAX_RANGE()
    {
        reportPWM(Pins.PWM_Actuator, ActuatorPwm::MaxDistance);
        return StatsMessage(12, "Motor is at max distance\0");
    }
    StatsMessage MOTOR_MIN_RANGE()
    {
        reportPWM(Pins.PWM_Actuator, ActuatorPwm::MinDistance);
        return StatsMessage(13, "Motor is at min distance\0");
    }
    StatsMessage MOTOR_WAIT_DONE()
    {
        return StatsMessage(14, "Motor wait time is is done.\0");
    }

    StatsMessage ENTERED_LANE_RANGE(short lane)
    {
        lane++; // Make lanes start from 1 instead of zero
        // TODO: Create a function to take care of this
        switch (lane)
        {
            case 1:
                reportPWM(Pins.PWM_CurrentLaneNum, CurrentLaneNumPWM::Lane1);
                break;
            case 2:
                reportPWM(Pins.PWM_CurrentLaneNum, CurrentLaneNumPWM::Lane2);
                break;
            case 3:
                reportPWM(Pins.PWM_CurrentLaneNum, CurrentLaneNumPWM::Lane3);
                break;
            case 4:
                reportPWM(Pins.PWM_CurrentLaneNum, CurrentLaneNumPWM::Lane4);
                break;
            default:
                reportPWM(Pins.PWM_CurrentLaneNum, CurrentLaneNumPWM::Unknown);
        }
        return StatsMessage(20, "Entered rotation range of lane\0", lane);
    }

    StatsMessage CORRECT_SENSOR_TOUCHED()
    {
        reportPWM(Pins.PWM_DecisionResult, DecisionResultPwm::Correct);
        return StatsMessage(30, "Correct sensor touched\0");
    }
    StatsMessage WRONG_SENSOR_TOUCHED()
    {
        reportPWM(Pins.PWM_DecisionResult, DecisionResultPwm::Wrong);
        return StatsMessage(31, "Wrong sensor touched\0");
    }

    StatsMessage RIGHT_SENSOR_TOUCHED()
    {
        reportPWM(Pins.PWM_LickDirection, LickDirectionPwm::Right);
        return StatsMessage(35, "R-located sensor touched\0");
    }
    StatsMessage LEFT_SENSOR_TOUCHED()
    {
        reportPWM(Pins.PWM_LickDirection, LickDirectionPwm::Left);
        return StatsMessage(36, "L-located sensor touched\0");
    }
    void REPORT_SENSORS_UNTOUCHED ()
    {
        reportPWM(Pins.PWM_LickDirection, LickDirectionPwm::Untouched);
    }

    // Next one is not reported yet
    StatsMessage OTHER_SENSOR_TOUCHED(short sensor)
    {
        return StatsMessage(37, "Other sensor touched - Sensor nr.\0", sensor);
    }

    StatsMessage REWARD_GIVEN(bool automated_mode)
    {
        if (automated_mode)
        {
            reportPWM(Pins.PWM_RewardResult, RewardResultPwm::AutomatedGiven);
            return StatsMessage(44, "Giving reward automatically\0");
        }
        else
        {
            reportPWM(Pins.PWM_RewardResult, RewardResultPwm::Given);
            return StatsMessage(40, "Giving reward\0");
        }
    }
    StatsMessage REWARD_NOT_GIVEN()
    {
        reportPWM(Pins.PWM_RewardResult, RewardResultPwm::NotGiven);
        return StatsMessage(41, "Not giving reward\0");
    }
    StatsMessage MISS_DECISION()
    {
        reportPWM(Pins.PWM_DecisionResult, DecisionResultPwm::Miss);
        return StatsMessage(42, "No Decision was made in time\0");
    }

    StatsMessage NEW_LANE(short lane)
    {
        // reset trial based PWMs
        reportPWM(Pins.PWM_DecisionResult, DecisionResultPwm::Unknown);
        reportPWM(Pins.PWM_RewardResult, RewardResultPwm::Unknown);

        lane++; // Make lanes start from 1 instead of zero
        reportPWM(Pins.PWM_CorrectLaneNum, lane);
        return StatsMessage(50, "New lane chosen\0", lane);
    }
    StatsMessage NEW_TRIAL(short trial)
    {
        return StatsMessage(55, "Trial number\0", trial);
    }

    StatsMessage LIGHT_ON()
    {
        // TEMP: Move this to its own stats
        reportPWM(Pins.InCorrecctLane, LaneLedPwm::Off);
        reportPWM(Pins.PWM_LaneLed, LaneLedPwm::On);
        return StatsMessage(60, "Cue light turned on\0");
    }
    StatsMessage LIGHT_OFF()
    {
        // TEMP: Move this to its own stats
        reportPWM(Pins.InCorrecctLane, LaneLedPwm::On);
        reportPWM(Pins.PWM_LaneLed, LaneLedPwm::Off);
        return StatsMessage(61, "Cue light turned off\0");
    }

    StatsMessage SOLENOID_RIGHT_ON()
    {
        reportPWM(Pins.PWM_Solenoid, SolenoidPwm::RightOpen);
        return StatsMessage(70, "R-located solenoid turned on\0");
    }
    StatsMessage SOLENOID_RIGHT_OFF()
    {
        reportPWM(Pins.PWM_Solenoid, SolenoidPwm::BothOff);
        return StatsMessage(71, "R-located solenoid turned off\0");
    }
    StatsMessage SOLENOID_LEFT_ON()
    {
        reportPWM(Pins.PWM_Solenoid, SolenoidPwm::LeftOpen);
        return StatsMessage(72, "L-located solenoid turned on\0");
    }
    StatsMessage SOLENOID_LEFT_OFF()
    {
        reportPWM(Pins.PWM_Solenoid, SolenoidPwm::BothOff);
        return StatsMessage(73, "L-located solenoid turned off\0");
    }

    StatsMessage FORCE_SENSOR_ON(bool is_left)
    {
        char* msg;
        if (is_left)
        {
            // TODO: change PWM in next trial
            //reportPWM(Pins.PWM_RewardState, RewardStatePwm::ForcedLeft);
            msg = (char*)"Force sensor mode on (left)\0";
        }
        else
        {
            //reportPWM(Pins.PWM_RewardState, RewardStatePwm::ForcedRight);
            msg = (char*)"Force sensor mode on (right)\0";
        }
        return StatsMessage(80, msg, is_left);
    }
    StatsMessage FORCE_SENSOR_OFF()
    {
        //reportPWM(Pins.PWM_RewardState, RewardStatePwm::Normal);
        return StatsMessage(81, "Force sensor mode is off\0");
    }
    StatsMessage FEEDBACK_AUTOMATED_ON()
    {
        return StatsMessage(82, "Feedback automated on\0");
    }
    StatsMessage FEEDBACK_AUTOMATED_OFF()
    {
        return StatsMessage(83, "Feedback automated off\0");
    }

    void REPORT_PIXY_POSITION (SubjectLocation location)
    {
        reportPWM(Pins.PWM_PixyX, PixyXPwm.convert(location.x));
        reportPWM(Pins.PWM_PixyY, PixyYPwm.convert(location.y));
        reportPWM(Pins.PWM_PixyAngle, PixyAnglePwm.convert(location.angle));
    }
} Stats;

#endif
