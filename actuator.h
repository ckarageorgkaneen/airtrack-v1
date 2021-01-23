#ifndef ACTUATOR_MODULE
#define ACTUATOR_MODULE

#include "Arduino.h"
#include "definitions.h"

struct Actuator
{
  public:
    enum State {
      STILL,
      PUSH,
      PULL
    };

  private:

    PIN_TYPE motor_push_pin;
    PIN_TYPE motor_pull_pin;

    int max_distance_pwm;

    State current_state;
    bool change_required;

    GlobalState* global_state;

  public:

    CONST_PIN_TYPE ANALOUGE_PIN = 1;

    Actuator (PIN_TYPE motor_push_pin, PIN_TYPE motor_pull_pin,
              int max_distance_pwm, GlobalState* global_state)
    {
        this->current_state = STILL;
        this->motor_push_pin = motor_push_pin;
        this->motor_pull_pin = motor_pull_pin;
        this->max_distance_pwm = max_distance_pwm;
        this->global_state = global_state;
    }

    void setup()
    {
        this->global_state->actuator_at_max_push = false;
        // Serial.println("Setting up actiator");
        this->setState(PULL);
        // printState();
        while (this->current_state != STILL)
        {
            this->motorLoop();
            //Serial.print("Setup loop - current state: ");
            //Serial.println(this->current_state);
        }
        //Serial.println("Setup loop done");
    }

    void setMaxDistance(int max_distance_pwm)
    {
      this->max_distance_pwm = max_distance_pwm;
    }

    void printState()
    {
      Serial.print("Actuator current state: ");
      Serial.print(this->current_state);
      Serial.print(" - and PWM value: ");
      Serial.println(analogRead(ANALOUGE_PIN));
    }

    void setState(State state)
    {
      // Serial.print("Current state: ");
      // Serial.println(this->current_state);
      if (state != this->current_state)
      {
        this->current_state = state;
        this->change_required = true;
      }
    }

    int getMotorDist()
    {
      return analogRead(ANALOUGE_PIN);
    }

    void motorLoop()
    {
      // Serial.print("Motor loop - ");
      //printState();
      if (this->current_state == STILL)
      {
        return;
      }

      #define MIN_DISTANCE 20
      int sensor_value = this->getMotorDist();
      // if (sensor_value >= this->max_distance_pwm - 30) // Leave a bit of buffer
      if (sensor_value >= this->max_distance_pwm)
      {
        this->global_state->actuator_at_max_push = true;
      }
      else
      {
        this->global_state->actuator_at_max_push = false;
        if (sensor_value <= MIN_DISTANCE)
        {
          this->global_state->actuator_at_min_pull = true;
        }
        else
        {
          this->global_state->actuator_at_min_pull = false;
        }

      }

      if (this->current_state == PUSH)
      {
          if (sensor_value >= this->max_distance_pwm)
          {
              this->enableStill();
              this->current_state = STILL;
              this->change_required = false;
          }

          if (this->change_required)
          {
              this->enablePush();
              this->change_required = false;
          }
      }
      else if (this->current_state == PULL)
      {
          if (sensor_value <= MIN_DISTANCE) // Doesn't have to be completely retracted
          {
              this->enableStill();
              this->current_state = STILL;
              this->change_required = false;
          }

          if (this->change_required)
          {
              this->enablePull();
              this->change_required = false;
          }
      }
    }

    void enablePush()
    {
       digitalWrite(this->motor_push_pin, HIGH);
       digitalWrite(this->motor_pull_pin, LOW);
    }

    void enablePull()
    {
        digitalWrite(this->motor_push_pin, LOW);
        digitalWrite(this->motor_pull_pin, HIGH);
    }

    void enableStill()
    {
       digitalWrite(this->motor_push_pin, LOW);
       digitalWrite(this->motor_pull_pin, LOW);
    }
};

#endif
