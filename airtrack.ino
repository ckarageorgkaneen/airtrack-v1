#include <Arduino.h>
#include <Wire.h> // Need by sensor.h
#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>
#include <Fsm.h>

#include "definitions.h"
#include "leds.h"
#include "pins.h"
#include "stats.h"
#include "sensor.h"
#include "actuator.h"
#include "reporter.h"

#define DEBUG false
#define DEBUG_STATE_FUNCTIONS DEBUG || false
#define DEBUG_TRIGGER_EVENT_MSGS DEBUG || false
#define DEBUG_W_VIRTUAL_MOUSE DEBUG || false
#define AUTOMATED_REWARD true
#define SINGLE_REWARD true
#define FEEDBACK_AUTOMATED_REWARD false
#define TRAINING_MODE true
#define TIME_UNTIL_TRAINING 8000
#define THRESHOLD 75
#define PIN_PUMP_ACTIVATOR_PULSE 29
#define SERVO_FACTOR 100
#define IS_INSIDE_LANE_FLEXIBLE_RANGE 10
#define NO_DELAY 0

Maus maus;
GlobalState global_state;
Sensor sensor = Sensor(Pins.Sensor);
Actuator actuator = Actuator(Pins.ActuatorPush, Pins.ActuatorPull,
                             global_state.actuator_max_pwm_distance,
                             &global_state);
Pixy pixy;
//Servo myservo1;
//Servo myservoleft;
//Servo myservoright;
int time_counter = 0;
int opto_activation_trial = -1;
bool is_within_reward_lane_angle = false;
bool is_inside_lane = false;
bool motor_pushed = false;
bool is_correct_sensor = false;
bool is_delay_timed_out = false;
long int actuator_at_rest_time_now;
SensorTouched touched_sensor = SensorTouched(false, false);
SubjectLocation subject_location;

void initSystem();
void makeNewRewardLane();
void resetSystem();
void reportExitedLane();
void turnOffPiezo();
void beOutsideLane();
void enterLane();
void pushActuator();
void reportActuatorAtMaxPush();
void reportActuatorAtRest();
void resetIsCorrectSensor();
void reportSensorTouched();
void reward();
void signalNoReward();
void beInsideLane();
void pullActuator();
void resetMotors();
void turnOnActuatorAtMinPull();
void turnOnActuatorAfterDelay();

struct StateStruct
{
  State* INITIALIZE = new State(initSystem, NULL, NULL);
  State* RESET_SYSTEM = new State(resetSystem, NULL, NULL);
  State* EXIT_LANE = new State(reportExitedLane, NULL, NULL);
  State* TURN_OFF_PIEZO = new State(turnOffPiezo, NULL, NULL);
  State* OUTSIDE_LANE = new State(beOutsideLane, NULL, NULL);
  State* ENTER_LANE = new State(enterLane, NULL, NULL);
  State* PUSH_ACTUATOR = new State(pushActuator, NULL, NULL);
  State* ACTUATOR_AT_MAX_PUSH = new State(reportActuatorAtMaxPush, NULL, NULL);
  State* ACTUATOR_AT_REST = new State(reportActuatorAtRest, NULL, NULL);
  State* RESET_IS_CORRECT_SENSOR = new State(resetIsCorrectSensor, NULL, NULL);
  State* SENSOR_TOUCHED = new State(reportSensorTouched, NULL, NULL);
  State* REWARD = new State(reward, NULL, NULL);
  State* NO_REWARD = new State(signalNoReward, NULL, NULL);
  State* INSIDE_LANE = new State(beInsideLane, NULL, NULL);
  State* PULL_ACTUATOR = new State(pullActuator, NULL, NULL);
  State* RESET_MOTORS = new State(resetMotors, NULL, NULL);
  State* TURN_ON_ACTUATOR_AT_MIN_PULL = new State(turnOnActuatorAtMinPull, NULL, NULL);
  State* TURN_ON_ACTUATOR_AFTER_DELAY = new State(turnOnActuatorAfterDelay, NULL, NULL);

};

enum EventEnum
{
  EVENT_RESET_SYSTEM,
  EVENT_BE_OUTSIDE_LANE,
  EVENT_EXIT_LANE,
  EVENT_TURN_OFF_PIEZO,
  EVENT_ENTER_LANE,
  EVENT_PUSH_ACTUATOR,
  EVENT_ACTUATOR_AT_MAX_PUSH,
  EVENT_ACTUATOR_AT_REST,
  EVENT_RESET_IS_CORRECT_SENSOR,
  EVENT_SENSOR_TOUCHED,
  EVENT_REWARD,
  EVENT_NO_REWARD,
  EVENT_PULL_ACTUATOR,
  EVENT_RESET_MOTORS,
  EVENT_ACTUATOR_AT_MIN_PULL_UNREPORTED,
  EVENT_TURN_ON_ACTUATOR_AFTER_DELAY,
};

struct StateStruct state;
enum EventEnum event;
Fsm fsm(state.INITIALIZE);

void setup()
{
  Serial.begin(115200);
  fsm.add_transition(
    state.INITIALIZE,
    state.RESET_SYSTEM,
    EVENT_RESET_SYSTEM,
    NULL);
  fsm.add_transition(
    state.RESET_SYSTEM,
    state.OUTSIDE_LANE,
    EVENT_BE_OUTSIDE_LANE,
    NULL);
  fsm.add_transition(
    state.RESET_SYSTEM,
    state.EXIT_LANE,
    EVENT_EXIT_LANE,
    NULL);
  fsm.add_transition(
    state.EXIT_LANE,
    state.TURN_OFF_PIEZO,
    EVENT_TURN_OFF_PIEZO,
    NULL);
  fsm.add_transition(
    state.OUTSIDE_LANE,
    state.PULL_ACTUATOR,
    EVENT_PULL_ACTUATOR,
    NULL);
  fsm.add_timed_transition(
    state.EXIT_LANE,
    state.OUTSIDE_LANE,
    NO_DELAY,
    NULL);
  fsm.add_timed_transition(
    state.TURN_OFF_PIEZO,
    state.OUTSIDE_LANE,
    NO_DELAY,
    NULL);
  fsm.add_transition(
    state.OUTSIDE_LANE,
    state.RESET_MOTORS,
    EVENT_RESET_MOTORS,
    NULL);
  fsm.add_transition(
    state.RESET_SYSTEM,
    state.ENTER_LANE,
    EVENT_ENTER_LANE,
    NULL);
  fsm.add_timed_transition(
    state.ENTER_LANE,
    state.INSIDE_LANE,
    NO_DELAY,
    NULL);
  fsm.add_transition(
    state.ENTER_LANE,
    state.PUSH_ACTUATOR,
    EVENT_PUSH_ACTUATOR,
    NULL);
  fsm.add_timed_transition(
    state.PUSH_ACTUATOR,
    state.INSIDE_LANE,
    NO_DELAY,
    NULL);
  fsm.add_transition(
    state.PUSH_ACTUATOR,
    state.ACTUATOR_AT_MAX_PUSH,
    EVENT_ACTUATOR_AT_MAX_PUSH,
    NULL);
  fsm.add_timed_transition(
    state.ACTUATOR_AT_MAX_PUSH,
    state.INSIDE_LANE,
    NO_DELAY,
    NULL);
  fsm.add_transition(
    state.ACTUATOR_AT_MAX_PUSH,
    state.ACTUATOR_AT_REST,
    EVENT_ACTUATOR_AT_REST,
    NULL);
  fsm.add_transition(
    state.ACTUATOR_AT_REST,
    state.RESET_IS_CORRECT_SENSOR,
    EVENT_RESET_IS_CORRECT_SENSOR,
    NULL);
  fsm.add_timed_transition(
    state.ACTUATOR_AT_REST,
    state.INSIDE_LANE,
    NO_DELAY,
    NULL);
  fsm.add_transition(
    state.RESET_IS_CORRECT_SENSOR,
    state.SENSOR_TOUCHED,
    EVENT_SENSOR_TOUCHED,
    NULL);
  fsm.add_transition(
    state.RESET_IS_CORRECT_SENSOR,
    state.REWARD,
    EVENT_REWARD,
    NULL);
  fsm.add_transition(
    state.RESET_IS_CORRECT_SENSOR,
    state.NO_REWARD,
    EVENT_NO_REWARD,
    NULL);
  fsm.add_timed_transition(
    state.RESET_IS_CORRECT_SENSOR,
    state.INSIDE_LANE,
    NO_DELAY,
    NULL);
  fsm.add_transition(
    state.SENSOR_TOUCHED,
    state.REWARD,
    EVENT_REWARD,
    NULL);
  fsm.add_transition(
    state.SENSOR_TOUCHED,
    state.NO_REWARD,
    EVENT_NO_REWARD,
    NULL);
  fsm.add_timed_transition(
    state.SENSOR_TOUCHED,
    state.INSIDE_LANE,
    NO_DELAY,
    NULL);
  fsm.add_timed_transition(
    state.REWARD,
    state.INSIDE_LANE,
    NO_DELAY,
    NULL);
  fsm.add_timed_transition(
    state.NO_REWARD,
    state.INSIDE_LANE,
    NO_DELAY,
    NULL);
  fsm.add_transition(
    state.INSIDE_LANE,
    state.PULL_ACTUATOR,
    EVENT_PULL_ACTUATOR,
    NULL);
  fsm.add_timed_transition(
    state.INSIDE_LANE,
    state.RESET_MOTORS,
    EVENT_RESET_MOTORS,
    NULL);
  fsm.add_transition(
    state.PULL_ACTUATOR,
    state.RESET_MOTORS,
    EVENT_RESET_MOTORS,
    NULL);
  fsm.add_transition(
    state.PULL_ACTUATOR,
    state.TURN_ON_ACTUATOR_AT_MIN_PULL,
    EVENT_ACTUATOR_AT_MIN_PULL_UNREPORTED,
    NULL);
  fsm.add_transition(
    state.TURN_ON_ACTUATOR_AT_MIN_PULL,
    state.RESET_MOTORS,
    EVENT_RESET_MOTORS,
    NULL);
  fsm.add_transition(
    state.RESET_MOTORS,
    state.TURN_ON_ACTUATOR_AFTER_DELAY,
    EVENT_TURN_ON_ACTUATOR_AFTER_DELAY,
    NULL);
  fsm.add_transition(
    state.TURN_ON_ACTUATOR_AFTER_DELAY,
    state.RESET_SYSTEM,
    EVENT_RESET_SYSTEM,
    NULL);
  fsm.add_transition(
    state.RESET_MOTORS,
    state.RESET_SYSTEM,
    EVENT_RESET_SYSTEM,
    NULL);
}

void loop()
{
  fsm.run_machine();
  triggerNewEvents();
}

void initSystem()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In initSystem().");
  #endif
  Serial.println("Initializing system.");
  sensor.setup();
  actuator.setup();
  setupLeds();
  setupPins();
  setupLanes();
  setupPixy();
  setupGlobalState();
  // We are seeding the random so it would give us reproducible results
  // randomSeed(0);// call randomSeed(analogRead(A3)) for random order on each run
  randomSeed(analogRead(A3));
  createShuffledChoice();
  makeNewRewardLane();
  Serial.println("Done initializing system.");
}

void resetSystem()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In resetSystem().");
  #endif
  if(digitalRead(25) == LOW)
    resetActuator();
  digitalWrite(A9,LOW);
  if(digitalRead(23) == HIGH)
    pulsePumpActivator();
  resetSubjectLocation();
  resetIsInsideLaneFlags();
  //move_training_servos(subject_location.angle, is_within_reward_lane_angle);
  motor_pushed = false;
  // if(time_counter < TIME_UNTIL_TRAINING + 10 && !is_within_reward_lane_angle){
  //   time_counter++;
  // }
  // if(time_counter > TIME_UNTIL_TRAINING && TRAINING_MODE && global_state.Servo1_pos < 160*SERVO_FACTOR && !is_within_reward_lane_angle){
  //   global_state.Servo1_pos += 1;
  //   myservo1.write(global_state.Servo1_pos/SERVO_FACTOR);
  // }
  touched_sensor = sensor.readInput();
  if (!touched_sensor.change_happened)
    Stats.REPORT_SENSORS_UNTOUCHED();
}

void turnOnActuator()
{
  // TODO: Do it cleanly
  turnOnMotor(13, 20);
}

void turnOnActuatorAtMinPull()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In turnOnActuatorAtMinPull().");
  #endif
  writeStats(Stats.MOTOR_MIN_RANGE());
  global_state.reported_motor_min_distance = true;
  turnOnActuator();
}

void turnOnActuatorAfterDelay()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In turnOnActuatorAfterDelay().");
  #endif
  turnOnActuator();
  global_state.delayed_report = 0;
}

void resetMotors()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In resetMotors().");
  #endif
  turnOffMotors();
  actuator.motorLoop();
}

void triggerResetSystemEvent()
{
  #if DEBUG_TRIGGER_EVENT_MSGS
  Serial.println("Triggering EVENT_RESET_SYSTEM");
  #endif
  fsm.trigger(EVENT_RESET_SYSTEM);
}

void triggerEnterLaneEvent()
{
  if (is_inside_lane) {
    #if DEBUG_TRIGGER_EVENT_MSGS
    Serial.println("Triggering EVENT_ENTER_LANE");
    #endif
    fsm.trigger(EVENT_ENTER_LANE);
  }
}

void triggerPushActuatorEvent()
{
  #if DEBUG_W_VIRTUAL_MOUSE
  bool push_actuator_condition = true;
  #else
  bool push_actuator_condition = is_within_reward_lane_angle && shouldTriggerMotor();
  #endif
  if (push_actuator_condition) {
    #if DEBUG_TRIGGER_EVENT_MSGS
    Serial.println("Triggering EVENT_PUSH_ACTUATOR");
    #endif
    fsm.trigger(EVENT_PUSH_ACTUATOR);
  }
}

void triggerActuatorAtMaxPushEvent()
{
  #if DEBUG_W_VIRTUAL_MOUSE
  bool actuator_is_at_max_push = true;
  #else
  bool actuator_is_at_max_push = global_state.actuator_at_max_push;
  #endif
  if (actuator_is_at_max_push) {
    #if DEBUG_TRIGGER_EVENT_MSGS
    Serial.println("Triggering EVENT_ACTUATOR_AT_MAX_PUSH");
    #endif
    fsm.trigger(EVENT_ACTUATOR_AT_MAX_PUSH);
  }
}

void triggerActuatorAtRestEvent()
{
  // Allow a bit of buffer time for sensor vibration to
  // rest before reading
  long int time_now = millis();
  #if DEBUG_W_VIRTUAL_MOUSE
  bool max_push_wait_exceeded = true;
  #else
  bool max_push_wait_exceeded = global_state.max_push_current_duration <= time_now - global_state.MAX_PUSH_WAIT;
  #endif
  if (max_push_wait_exceeded) {
    actuator_at_rest_time_now = time_now;
    #if DEBUG_TRIGGER_EVENT_MSGS
    Serial.println("Triggering EVENT_ACTUATOR_AT_REST");
    #endif
    fsm.trigger(EVENT_ACTUATOR_AT_REST);
  }
}

void triggerResetIsCorrectSensor()
{
  #if DEBUG_TRIGGER_EVENT_MSGS
  Serial.println("Triggering EVENT_RESET_IS_CORRECT_SENSOR");
  #endif
  fsm.trigger(EVENT_RESET_IS_CORRECT_SENSOR);
}

void triggerSensorTouchedEvent()
{
  // #if DEBUG_W_VIRTUAL_MOUSE
  // bool sensor_was_touched = true;
  // #else
  // bool sensor_was_touched = touched_sensor.change_happened && !global_state.sensor_was_touched;
  // #endif
  // if (sensor_was_touched) {
  //   #if DEBUG_TRIGGER_EVENT_MSGS
  //   Serial.println("Triggering EVENT_SENSOR_TOUCHED");
  //   #endif
  //   fsm.trigger(EVENT_SENSOR_TOUCHED);
  // }
}

void triggerRewardEvents()
{
  #if DEBUG_W_VIRTUAL_MOUSE
  bool decide_reward_condition = true;
  #else
  bool decide_reward_condition = (global_state.is_automated_reward || touched_sensor.change_happened) && !global_state.reward_given;
  #endif
  if (decide_reward_condition) {
    bool reward_condition = (is_correct_sensor || global_state.is_automated_reward);
    if (reward_condition) {
      #if DEBUG_TRIGGER_EVENT_MSGS
      Serial.println("Triggering EVENT_REWARD");
      #endif
      fsm.trigger(EVENT_REWARD);
    } else {
      #if DEBUG_TRIGGER_EVENT_MSGS
      Serial.println("Triggering EVENT_NO_REWARD");
      #endif
      fsm.trigger(EVENT_NO_REWARD);
    }
  }
}

void triggerOutsideLaneEvents()
{
  if (!is_inside_lane && global_state.was_inside_lane) {
    #if DEBUG_TRIGGER_EVENT_MSGS
    Serial.println("Triggering EVENT_EXIT_LANE");
    #endif
    fsm.trigger(EVENT_EXIT_LANE);
  } else if (!is_inside_lane) {
    #if DEBUG_TRIGGER_EVENT_MSGS
    Serial.println("Triggering EVENT_BE_OUTSIDE_LANE");
    #endif
    fsm.trigger(EVENT_BE_OUTSIDE_LANE);
  }
}

void triggerTurnOffPiezoEvent()
{
  #if DEBUG_W_VIRTUAL_MOUSE
  bool turn_off_piezo_condition = true;
  #else
  bool turn_off_piezo_condition = global_state.piezo_motor_entry != NULL;
  #endif
  if (turn_off_piezo_condition) {
    #if DEBUG_TRIGGER_EVENT_MSGS
    Serial.println("Triggering EVENT_TURN_OFF_PIEZO");
    #endif
    fsm.trigger(EVENT_TURN_OFF_PIEZO);
  }
}

void triggerPullActuatorEvent()
{
  #if DEBUG_W_VIRTUAL_MOUSE
  bool pull_actuator_condition = true;
  #else
  bool pull_actuator_condition = subject_location.block_detected && !motor_pushed;
  #endif
  if (pull_actuator_condition)
  {
    #if DEBUG_TRIGGER_EVENT_MSGS
    Serial.println("Triggering EVENT_PULL_ACTUATOR");
    #endif
    fsm.trigger(EVENT_PULL_ACTUATOR);
  }
}

void triggerActuatorAtMinPullEvent()
{
  #if DEBUG_W_VIRTUAL_MOUSE
  bool actuator_at_min_pull_unreported = true;
  #else
  bool actuator_at_min_pull_unreported = global_state.actuator_at_min_pull && !global_state.reported_motor_min_distance;
  #endif
  if (actuator_at_min_pull_unreported)
  {
    #if DEBUG_TRIGGER_EVENT_MSGS
    Serial.println("Triggering EVENT_ACTUATOR_AT_MIN_PULL_UNREPORTED");
    #endif
    fsm.trigger(EVENT_ACTUATOR_AT_MIN_PULL_UNREPORTED);
  }
}

void triggerResetMotorsEvent()
{
  #if DEBUG_TRIGGER_EVENT_MSGS
  Serial.println("Triggering EVENT_RESET_MOTORS");
  #endif
  fsm.trigger(EVENT_RESET_MOTORS);
}

void triggerTurnOnActuatorAfterDelayEvent()
{
  #if DEBUG_W_VIRTUAL_MOUSE
  is_delay_timed_out = true;
  #else
  long int time_now = millis();
  is_delay_timed_out = global_state.delayed_report && (time_now >= global_state.delayed_report);
  #endif
  if (is_delay_timed_out) {
    #if DEBUG_TRIGGER_EVENT_MSGS
    Serial.println("Triggering EVENT_TURN_ON_ACTUATOR_AFTER_DELAY");
    #endif
    fsm.trigger(EVENT_TURN_ON_ACTUATOR_AFTER_DELAY);
  }
}

void triggerNewEvents()
{
  triggerResetSystemEvent();
  triggerOutsideLaneEvents();
  triggerTurnOffPiezoEvent();
  triggerEnterLaneEvent();
  triggerPushActuatorEvent();
  triggerActuatorAtMaxPushEvent();
  triggerActuatorAtRestEvent();
  triggerResetIsCorrectSensor();
  triggerSensorTouchedEvent();
  triggerRewardEvents();
  triggerPullActuatorEvent();
  triggerActuatorAtMinPullEvent();
  triggerResetMotorsEvent();
  triggerTurnOnActuatorAfterDelayEvent();
}

bool isInsideLane()
{
  bool subject_location_y_gte_max_thres = subject_location.y >= Distances.y_threshold_max &&
    subject_location.y - Distances.y_threshold_max <= IS_INSIDE_LANE_FLEXIBLE_RANGE;
  bool subject_location_y_lte_max_thres = subject_location.y <= Distances.y_threshold_max &&
    Distances.y_threshold_max - subject_location.y <= IS_INSIDE_LANE_FLEXIBLE_RANGE;
  if (global_state.was_inside_lane && subject_location_y_gte_max_thres)
    subject_location.y = Distances.y_threshold_max - 1;
  else if (subject_location_y_lte_max_thres)
    subject_location.y = Distances.y_threshold_max + 1;
  return Distances.x_threshold_min < subject_location.x &&
   subject_location.x        < Distances.x_threshold_max &&
   Distances.y_threshold_min < subject_location.y &&
   subject_location.y        < Distances.y_threshold_max;
}

void resetIsInsideLaneFlags()
{
  is_within_reward_lane_angle = isWithinRewardLaneAngle();
  #if DEBUG_W_VIRTUAL_MOUSE
  is_inside_lane = !global_state.was_inside_lane;
  #else
  is_inside_lane = isInsideLane();
  #endif
}

bool shouldTriggerMotor()
{
  bool is_within_distance = Distances.y_threshold_min < subject_location.y &&
                            subject_location.y < Distances.y_motor_threshold;
  if (!is_within_distance)
  {
    global_state.actuator_duration_activated = false;
    if (global_state.reward_given && !global_state.chose_new_lane)
    {
      makeNewRewardLane();
      global_state.chose_new_lane = true;
    }
    return false;
  }
  else
  {
    long int time_now = millis();
    if (!global_state.actuator_duration_activated)
    {
      global_state.chose_new_lane = false;
      // Only start counting when we are at max pwm
      setActuatorTimeout(global_state.MAX_PUSH_TIMEOUT);
      return true;
    }
    // We are still counting, retunr true until we time out
    else if (global_state.max_push_current_duration +
             global_state.motor_timeout_duration >= time_now)
    {
      return true;
    }
    else
    {
      if (isWithinRewardLaneAngle() && !global_state.chose_new_lane)
      {
          if (!global_state.reward_given ||
              (AUTOMATED_REWARD && !global_state.sensor_was_touched))
          {
              writeStats(Stats.MISS_DECISION());
              global_state.miss_or_wrong_touch_count += 1;
          }
          makeNewRewardLane();
          global_state.chose_new_lane = true;
      }

      return false;
    }
  }
}

void resetSubjectLocation()
{
  subject_location = global_state.last_subject_location;
  // Serial.println("Gettin blocks");
  uint16_t num_of_blocks = pixy.getBlocks();
  //Serial.print("Pixy array size: ");
  //Serial.println(num_of_blocks);
  SubjectLocation location = SubjectLocation();
  location.block_detected = num_of_blocks == 1;
  if (location.block_detected)
  {
    location.angle = pixy.blocks[0].angle;
    location.x = pixy.blocks[0].x;
    location.y = pixy.blocks[0].y;
    if (location.angle > 180)
        location.angle -= 360;
    maus = calculateVelocity(maus, location.x, location.y, location.angle);
    //Serial.println(location.angle);
    Stats.REPORT_PIXY_POSITION(location);
    const bool ENABLE_POSITION_PRINT = false;
    if (ENABLE_POSITION_PRINT)
    {
      Serial.print("subject_location: x: ");
      Serial.print(location.x);
      Serial.print(" y: ");
      Serial.print(location.y);
      Serial.print(" angle: ");
      Serial.print(location.angle);
      Serial.print(" Mouse velocity: ");
      Serial.print(maus.velocity);
      Serial.print(" Num. of blocks: ");
      Serial.println(num_of_blocks);
    }
    global_state.last_subject_location = subject_location = location;
  }
}

bool isCorrectSensor()
{
  Lane lane = global_state.lanes[global_state.current_lane];
  if (touched_sensor.left_sensor == true)
  {
    // Bug here: switch name
    writeStats(Stats.LEFT_SENSOR_TOUCHED());
    digitalWrite(Leds.SensorLeft, HIGH);

    checkForceSensorMode(true);

    if (lane.reward_sensor == sensor.LEFT_ANALOUGE_PIN)
    {
        writeStats(Stats.CORRECT_SENSOR_TOUCHED());
        return true;
    }
    else
    {
        writeStats(Stats.WRONG_SENSOR_TOUCHED());
        return false;
    }
  }
  else
  {
    digitalWrite(Leds.SensorLeft, LOW);
  }
  if (touched_sensor.right_sensor == true)
  {
    // Bug here: switch name
    writeStats(Stats.RIGHT_SENSOR_TOUCHED());
    digitalWrite(Leds.SensorRight, HIGH);
    checkForceSensorMode(false);
    if (lane.reward_sensor == sensor.RIGHT_ANALOUGE_PIN)
    {
      writeStats(Stats.CORRECT_SENSOR_TOUCHED());
      return true;
    }
    else
    {
      writeStats(Stats.WRONG_SENSOR_TOUCHED());
      return false;
    }
  }
  else
    digitalWrite(Leds.SensorRight, LOW);
  return false;
}

void checkForceSensorMode(bool is_left_sensor_touched)
{
  if (SINGLE_REWARD)
    return;
  if (global_state.actuator_at_max_push && !global_state.sensor_was_touched)
  {
    if (is_left_sensor_touched == global_state.last_sensor_touched_left)
    {
      global_state.same_sensor_touch_count++;
    }
    else
    {
      global_state.last_sensor_touched_left = is_left_sensor_touched;
      global_state.same_sensor_touch_count = 1;
    }

    unsigned int touch_count = global_state.same_sensor_touch_count;
    if (global_state.in_force_sensor_mode)
    {
      if (is_left_sensor_touched == global_state.is_force_sensor_left)
      {
        if (touch_count == global_state.FORCE_OTHER_SENSOR)
        {
            global_state.in_force_sensor_mode = false;
            // Should we reset the counter or not?
            global_state.same_sensor_touch_count = 1;
            writeStats(Stats.FORCE_SENSOR_OFF());
        }
      }

      if (FEEDBACK_AUTOMATED_REWARD)
      {
        if (is_left_sensor_touched == global_state.is_force_sensor_left)
        {
          if (global_state.is_automated_reward)
            writeStats(Stats.FEEDBACK_AUTOMATED_OFF());
          global_state.is_automated_reward = false;
        }
        else if (global_state.miss_or_wrong_touch_count >=
                 global_state.FEEDBACK_AUTOMATED_REWARD_THRESHOLD)
        {
          if (!global_state.is_automated_reward)
            writeStats(Stats.FEEDBACK_AUTOMATED_ON());
          global_state.is_automated_reward = true;
        }
      }
    }
    else if (touch_count == global_state.SAME_SENSOR_MAX_THRESHOLD)
    {
      global_state.in_force_sensor_mode = true;
      global_state.is_force_sensor_left = !is_left_sensor_touched;
      writeStats(Stats.FORCE_SENSOR_ON(!is_left_sensor_touched));
    }
  }
}

void resetIsCorrectSensor()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In resetIsCorrectSensor()");
  #endif
  // Call isCorrectSensor() anyway so it'd call
  // writeStats() on the touched sensor
  is_correct_sensor = isCorrectSensor();
}

void reward()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In reward()");
  #endif
  // Report if reward was given due to correct sensor was touched or
  // due to wrong sensore touched but automated reward is enabled
  writeStats(Stats.REWARD_GIVEN(is_correct_sensor == false));
  // TODO: Clean hacky way
  global_state.delayed_report = millis() + 2000;

  Lane lane = global_state.lanes[global_state.reward_lane_id];
  if (lane.reward_sensor == sensor.RIGHT_ANALOUGE_PIN)
  {
    writeStats(Stats.SOLENOID_RIGHT_ON());
    turnOnMotor(Pins.SolenoidRight, global_state.SOLENOID_DURATION);
  }
  else if (lane.reward_sensor == sensor.LEFT_ANALOUGE_PIN)
  {
    writeStats(Stats.SOLENOID_LEFT_ON());
    Serial.write("Reward given");
    digitalWrite(PIN_PUMP_ACTIVATOR_PULSE, HIGH);
    delayMicroseconds(global_state.SOLENOID_DURATION);
    digitalWrite(PIN_PUMP_ACTIVATOR_PULSE, LOW);
    //turnOnMotor(Pins.SolenoidLeft, global_state.SOLENOID_DURATION);
  }
  else
  {
    Serial.println("Want to give reward - Unknown Solenoid");
  }
  setActuatorTimeout(global_state.ALLOWED_REWARD_TIMEOUT);
  global_state.reward_given = true;
}

void signalNoReward()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In signalNoReward()");
  #endif
  //Serial.println("No reward");
  writeStats(Stats.REWARD_NOT_GIVEN());
  setActuatorTimeout(global_state.NO_REWARD_TIMEOUT);
  global_state.piezo_motor_entry = turnOnMotor(Pins.PiezoTone, global_state.PEIZO_TIMEOUT);
  global_state.reward_given = true;
}

void makeNewRewardLane()
{
  Serial.println("Making new reward lane.");
  // Assign to a non existing lane initially
  LANE_ID new_lane_id = global_state.NUM_OF_LANES + 1;
  if (global_state.in_force_sensor_mode)
  {
    bool found_lane = false;
    do
    {
      LANE_ID potential_lane_id = random(global_state.NUM_OF_LANES);
      if (potential_lane_id == global_state.reward_lane_id)
        continue;
      Lane lane = global_state.lanes[potential_lane_id];
      bool is_left = lane.reward_sensor == sensor.LEFT_ANALOUGE_PIN;
      if (is_left != global_state.is_force_sensor_left)
        continue;
      new_lane_id = potential_lane_id;
      found_lane = true;
    }
    while (!found_lane);
  }
  else
  {
    LANE_ID* list_ptr = global_state.lane_shuffle_list;
    size_t index = global_state.shuffle_list_index;
    new_lane_id = list_ptr[index];
    // This might happen if the last lane was due to a forced sensor mode
    if (new_lane_id == global_state.reward_lane_id)
    {
      // IF there is enough space then swap
      if (index < global_state.GUARANTEED_RANDOM_BOUND - 1)
      {
        LANE_ID temp = list_ptr[index];
        list_ptr[index] = list_ptr[index + 1];
        list_ptr[index + 1] = temp;
      }
      else
      {
        createShuffledChoice();
        new_lane_id = list_ptr[0];
      }
    }
    global_state.shuffle_list_index++;
  }
  //#####################################################easy trial
  global_state.reward_direction = random(0,2);
  if(global_state.reward_direction == 0){
    if(global_state.current_lane == 3){
      new_lane_id = 0;
    }
    else
    new_lane_id = global_state.current_lane + 1;
  }

  if(global_state.reward_direction == 1){
    if(global_state.current_lane == 0){
      new_lane_id = 3;
    }
    else
    new_lane_id = global_state.current_lane - 1;
  }
  //######################################################
  global_state.reward_lane_id = new_lane_id;
  Serial.print("New reward lane is: ");
  Serial.println(global_state.reward_lane_id);
  Serial.print("New reward rotation direction is: ");
  Serial.println(global_state.reward_direction);
  global_state.reward_given = false;
  // We must assign first thenew lane before calling createShuffledChoice()
  if (global_state.shuffle_list_index == global_state.GUARANTEED_RANDOM_BOUND)
  {
      createShuffledChoice();
  }
  writeStats(Stats.NEW_LANE(new_lane_id));
  writeStats(Stats.NEW_TRIAL(global_state.trial_number));
  //#########################################################################################################################
  //Video end trigger
  digitalWrite(A8, HIGH);
  digitalWrite(A9, HIGH);
  digitalWrite(A10, HIGH);
  delay(100);
  digitalWrite(A8, LOW);
  digitalWrite(A9, LOW);
  digitalWrite(A10, LOW);
  global_state.trial_number += 1;
  digitalWrite(A9, HIGH);
  delay(20);
  digitalWrite(A9,LOW);
  printRewardLane();
  //pylonPD execution Trigger
  delay(100);
  digitalWrite(A11, HIGH);
  delay(20);
  digitalWrite(A11, LOW);
  delay(500);
  //Video start trigger
  digitalWrite(A12, HIGH);
  digitalWrite(A15, HIGH);
  delay(100);
  digitalWrite(A12, LOW);
  digitalWrite(A15, LOW);
  delay(50);
  //Video Mark trigger (removed, because new camera has issues with channel 4 and ZR-View only supports 3 channels due to reasons)
  digitalWrite(A15, HIGH);
  //digitalWrite(44, HIGH);  // Video alignment trigger
  delay(50);
  digitalWrite(A15, LOW);
  //digitalWrite(44, LOW);
  time_counter = 0;
  global_state.Servo1_pos = 200;
  Serial.println("Done making new reward lane.");
}

void failedTrial(){
  LANE_ID new_lane_id = global_state.NUM_OF_LANES + 1;
  if(global_state.reward_direction == 0){
    if(global_state.current_lane == 3)
      new_lane_id = 0;
    else
      new_lane_id = global_state.current_lane + 1;
  }
  if(global_state.reward_direction == 1){
    if(global_state.current_lane == 0)
      new_lane_id = 3;
    else
      new_lane_id = global_state.current_lane - 1;
  }
  global_state.reward_lane_id = new_lane_id;
  Serial.print("New reward lane is: ");
  Serial.println(global_state.reward_lane_id);
  Serial.print("New reward rotation direction is: ");
  Serial.println(global_state.reward_direction);
  global_state.reward_given = false;
}


void createShuffledChoice()
{
  if (global_state.GUARANTEED_RANDOM_BOUND % global_state.NUM_OF_LANES != 0)
    Serial.println("Num of lanes is not divisble by random bound");
  // Create a list containing the lanes values (repeated in order). This list
  // will be shuffled in the next step.
  int j_max = global_state.GUARANTEED_RANDOM_BOUND/global_state.NUM_OF_LANES;
  for (int i = 0; i < global_state.NUM_OF_LANES; i++)
  {
    for (int j = 0; j < j_max; j++)
    {
      size_t index = j + (i*j_max);
      global_state.lane_shuffle_list[index] = i;
    }
  }
  int last_lane_value = global_state.reward_lane_id;
  // Serial.print("Starting with last lane value: ");
  // Serial.println(last_lane_value);
  for (int i = 0; i < global_state.GUARANTEED_RANDOM_BOUND; i++)
  {
    int r_index;
    int new_value = global_state.lane_shuffle_list[i];
    int count = 0;
    bool print = true;
    do
    {
      r_index = random(i, global_state.GUARANTEED_RANDOM_BOUND);
      new_value = global_state.lane_shuffle_list[r_index];
      // Don't get stuck if it's the last element. Roll backwards
      if (count > global_state.GUARANTEED_RANDOM_BOUND - 1)
      {
        int decrement = global_state.GUARANTEED_RANDOM_BOUND /
                        global_state.NUM_OF_LANES;
        if (decrement < i)
            i -= decrement;
        else
            i = 0;
        if (i)
            last_lane_value = global_state.lane_shuffle_list[i - 1];
        else
            last_lane_value = global_state.reward_lane_id;
        // Serial.print("Stepped in error handling - ");
        // Serial.print("Resetting i to: ");
        // Serial.print(i);
        // Serial.print(" and last lane value to: ");
        // Serial.println(last_lane_value);
        count = 0;
      }
      count++;
    }
    while (last_lane_value == new_value);

    int old_value = global_state.lane_shuffle_list[i];
    global_state.lane_shuffle_list[i] = new_value;
    global_state.lane_shuffle_list[r_index] = old_value;

    last_lane_value = new_value;
  }
  global_state.shuffle_list_index = 0;
}

void printShuffleList()
{
  LANE_ID lane_counter[global_state.NUM_OF_LANES];
  for (int i = 0; i < global_state.NUM_OF_LANES; i++)
      lane_counter[i] = 0;
  Serial.print("[");
  for (int i = 0; i < global_state.GUARANTEED_RANDOM_BOUND; i++)
  {
    Serial.print(global_state.lane_shuffle_list[i] + 1);
    if (i !=global_state.GUARANTEED_RANDOM_BOUND - 1)
        Serial.print(", ");
    lane_counter[global_state.lane_shuffle_list[i]]++;
  }
  Serial.println("]");
  for (int i = 0; i < global_state.NUM_OF_LANES; i++)
  {
    Serial.print("Count for lane ");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.println(lane_counter[i]);
  }
}


void setActuatorTimeout(long int actuator_time_out)
{
  if (global_state.actuator_at_max_push)
  {
    long int time_now = millis();
    global_state.motor_timeout_duration = actuator_time_out;
    global_state.max_push_current_duration = time_now;
    global_state.actuator_duration_activated = true;
  }
}

void setupPins()
{
  Serial.println("Setting up Pins.");
  // Solenoid
  pinMode(Pins.SolenoidLeft, OUTPUT);
  pinMode(Pins.SolenoidRight, OUTPUT);
  digitalWrite(Pins.SolenoidLeft, LOW);
  digitalWrite(Pins.SolenoidRight, LOW);
  // Actuator
  pinMode(Pins.ActuatorPush, OUTPUT);
  pinMode(Pins.ActuatorPull, OUTPUT);
  digitalWrite(Pins.ActuatorPush, LOW);
  digitalWrite(Pins.ActuatorPush, LOW);
  // Lane Light
  pinMode(Pins.LaneLight, OUTPUT);
  digitalWrite(Pins.LaneLight, LOW);
  // Piezo Tone
  pinMode(Pins.PiezoTone, OUTPUT);
  digitalWrite(Pins.PiezoTone, LOW);
  // Initially set all used PWM to OUTPUT and to value Unkown or off (== 0)
  #ifdef PWM_ENABLED
    for (int i = 2; i <= 13; i++)
    {
      pinMode(i, OUTPUT);
      analogWrite(i, 0);
    }
    for (int i = 44; i <= 46; i++)
    {
      pinMode(i, OUTPUT);
      analogWrite(i, 0);
    }
    // temp change for TTL
    pinMode(31, OUTPUT);
    digitalWrite(31, LOW);
    pinMode(32, OUTPUT);
    digitalWrite(32, LOW);
  #endif
  pinMode(25, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);
  pinMode(35, INPUT_PULLUP);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(23, INPUT);
  pinMode(PIN_PUMP_ACTIVATOR_PULSE, OUTPUT);
  //GPIO Outputs for trigger
  pinMode(A8, OUTPUT);
  pinMode(A9, OUTPUT);
  pinMode(A10, OUTPUT);
  pinMode(A11, OUTPUT);
  pinMode(A12, OUTPUT);
  pinMode(A13, OUTPUT);
  pinMode(A14, OUTPUT);
  pinMode(A15, OUTPUT);
  Serial.println("Done setting up Pins.");
}

void setupLeds()
{
  Serial.println("Setting up LEDs.");
  //pinMode(Leds.Solenoid, OUTPUT);
  //digitalWrite(Leds.Solenoid, LOW);
  // pinMode(Leds.ActuatorPush, OUTPUT);
  // digitalWrite(Leds.ActuatorPush, HIGH);
  // pinMode(Leds.ActuatorPull, OUTPUT);
  // digitalWrite(Leds.ActuatorPull, HIGH);
  // pinMode(Leds.Unused, OUTPUT);
  // digitalWrite(Leds.Unused, LOW);
  pinMode(Leds.SensorLeft, OUTPUT);
  digitalWrite(Leds.SensorLeft, LOW);
  pinMode(Leds.SensorRight, OUTPUT);
  digitalWrite(Leds.SensorRight, LOW);
  Serial.println("Done setting up LEDs.");
}

void setupLanes()
{
  Serial.println("Setting up Lanes.");
  Lane lane;
  lane.lane_id = 0;
  if (SINGLE_REWARD)
    lane.reward_sensor = sensor.LEFT_ANALOUGE_PIN;
  else
    lane.reward_sensor = sensor.RIGHT_ANALOUGE_PIN;
  lane.region_start_angle = 45;
  lane.region_end_angle =  135;
  global_state.lanes[lane.lane_id] = lane;

  lane.lane_id = 1;
  if (SINGLE_REWARD)
    lane.reward_sensor = sensor.LEFT_ANALOUGE_PIN;
  else
    lane.reward_sensor = sensor.LEFT_ANALOUGE_PIN;
  lane.region_start_angle = 136;
  lane.region_end_angle =  -136;
  global_state.lanes[lane.lane_id] = lane;
  lane.lane_id = 2;
  if (SINGLE_REWARD)
    lane.reward_sensor = sensor.LEFT_ANALOUGE_PIN;
  else
    lane.reward_sensor = sensor.LEFT_ANALOUGE_PIN;
  lane.region_start_angle = -135;
  lane.region_end_angle =  -46;
  global_state.lanes[lane.lane_id] = lane;
  lane.lane_id = 3;
  if (SINGLE_REWARD)
    lane.reward_sensor = sensor.LEFT_ANALOUGE_PIN;
  else
    lane.reward_sensor = sensor.RIGHT_ANALOUGE_PIN;
  lane.region_start_angle = -45;
  lane.region_end_angle =  44;
  global_state.lanes[lane.lane_id] = lane;
  Serial.println("Done setting up Lanes.");
}

void setupGlobalState()
{
  Serial.println("Setting up global state.");
  for (int i = 0; i < global_state.MOTOR_DURATION_ENTERIES_SIZE; i++)
  {
    global_state.motor_duration_entries[i].activated = false;
  }
  global_state.is_automated_reward = AUTOMATED_REWARD;
  global_state.trial_number = 0;
  // Initially report non existing lane
  global_state.last_reported_lane = global_state.NUM_OF_LANES + 1;
  // Assign any random initial value
  global_state.last_reported_light_status = 250;
  global_state.actuator_duration_activated = false;
  global_state.was_inside_lane = false;
  // Assign last lane id to soeme random value so that it'd be possible to
  // choose lane 0 as the first lane
  global_state.reward_lane_id = -1;
  global_state.reward_direction = -1;
  Serial.println("Done setting up global state.");
}

void setupPixy()
{
  Serial.println("Setting up Pixy.");
  pixy.init();
  Serial.println("Done setting up Pixy.");
}

void resetActuator()
{
  Serial.println("Resetting actuator.");
  digitalWrite(A9,HIGH);
  delay(300);
  while(digitalRead(25) == HIGH)
  {
    digitalWrite(3, HIGH);
    if(digitalRead(35) == LOW)
    {
      actuator.enablePush();
    }
    else if(digitalRead(31) == LOW)
    {
      actuator.enablePull();
    }
    else
    {
      actuator.enableStill();
    }
  }
  actuator.setMaxDistance(actuator.getMotorDist());
  actuator.setup();
  digitalWrite(3, LOW);
  delay(1000);
  Serial.println("Actuator reset.");
}

void pulsePumpActivator()
{
  //turnOnMotor(Pins.SolenoidLeft, global_state.SOLENOID_DURATION);
  Serial.println("Pulsing pump activator.");
  digitalWrite(PIN_PUMP_ACTIVATOR_PULSE, HIGH);
  delayMicroseconds(100);
  digitalWrite(PIN_PUMP_ACTIVATOR_PULSE,LOW);
  Serial.println("Pump activator pulsed.");
}

bool isWithinRewardLaneAngle()
{
  if (!subject_location.block_detected)
  {
    //Serial.println("Current location: No block detected");
    return false;
  }

  for (int j = 0; j < global_state.NUM_OF_LANES; j++)
  {
    // Sometimes the angle jumps forward and backwards between the  current
    // lane and the next lane. So we need to give a bit of extra buffer to
    // the current until we are sure it's in the next lane. To do so, start
    // looping from the current lane;
    int i = (global_state.current_lane + j) % global_state.NUM_OF_LANES;
    int flexible_range = 0;
    if (i == global_state.current_lane)
        flexible_range = 10;
    Lane lane = global_state.lanes[i];
    int start = lane.region_start_angle - flexible_range;
    int end = lane.region_end_angle + flexible_range;
    int angle = subject_location.angle;
    if (end < start)
      end += 360;
    if (angle < start)
      angle += 360;
    if (start <= angle && angle <= end)
    {
      global_state.current_lane = i;
      //Serial.println(global_state.reward_direction);
      //Serial.print(" ");
      //Serial.println(angle);
      if (global_state.last_reported_lane != i)
      {
        switch(i)
        {  //Here it is decided if the animal went right(0) or left(1)
          case 0 :
          if(global_state.last_reported_lane == 1)
          global_state.rotation = 1;
          else if(global_state.last_reported_lane == 3)
          global_state.rotation = 0;
          else
          global_state.rotation = -1;
          break;

          case 1 :
          if(global_state.last_reported_lane == 2)
          global_state.rotation = 1;
          else if(global_state.last_reported_lane == 0)
          global_state.rotation = 0;
          else
          global_state.rotation = -1;
          break;

          case 2 :
          if(global_state.last_reported_lane == 3)
          global_state.rotation = 1;
          else if(global_state.last_reported_lane == 1)
          global_state.rotation = 0;
          else
          global_state.rotation = -1;
          break;

          case 3 :
          if(global_state.last_reported_lane == 0)
          global_state.rotation = 1;
          else if(global_state.last_reported_lane == 2)
          global_state.rotation = 0;
          else
          global_state.rotation = -1;
          break;
        }
        if(global_state.rotation != global_state.reward_direction){
          failedTrial();
          Serial.println("Trial has been failed");
        }
        writeStats(Stats.ENTERED_LANE_RANGE(i));
        global_state.last_reported_lane = i;
      }

      //Serial.print(lane.lane_id);     //Debug stuff
      //Serial.print(global_state.rotation);
      //Serial.println(global_state.was_inside_lane);
      if (lane.lane_id == global_state.reward_lane_id && global_state.rotation == global_state.reward_direction)
      {
        digitalWrite(Pins.LaneLight, LOW);
        digitalWrite(Pins.LaneLight2, LOW);
        //myservo1.write(10);
        if (global_state.last_reported_light_status != LOW)
        {
          writeStats(Stats.LIGHT_OFF());
          global_state.last_reported_light_status = LOW;
        }
        return true;
      }
      else if(global_state.reward_direction == 0) {
        digitalWrite(Pins.LaneLight, LOW);
        digitalWrite(Pins.LaneLight2, HIGH);
        return false;
      }
      else if(global_state.reward_direction == 1) {
        digitalWrite(Pins.LaneLight2, LOW);
        digitalWrite(Pins.LaneLight, HIGH);
        return false;
      }
      else
      {
        if (global_state.last_reported_light_status != HIGH)
        {
            writeStats(Stats.LIGHT_ON());
            global_state.last_reported_light_status = HIGH;
        }
        return false;
      }
    }
  }
  Serial.print("Unexpected code redirection. Subject location angle: ");
  Serial.println(subject_location.angle);
  return false;
}

// void move_training_servos(int angle, bool is_within_reward_lane_angle){
//   if(global_state.reward_direction == 0){
//     if(inRange(angle, -120,-90) || inRange(angle, -30,0) || inRange(angle, 60,80) || inRange(angle, 160,180)){
//       myservoleft.write(170);
//       if(is_within_reward_lane_angle){
//         myservoright.write(20);
//       }
//     }
//     else{
//       myservoleft.write(20);
//       myservoright.write(170);
//     }
//   }
//   else{
//     if(inRange(angle, -120,-90) || inRange(angle, -30,0) || inRange(angle, 60,80) || inRange(angle, 160,180)){
//       myservoright.write(20);
//       if(is_within_reward_lane_angle){
//         myservoleft.write(170);
//       }
//     }
//     else{
//       myservoleft.write(20);
//       myservoright.write(170);
//     }

//   }
// }

bool inRange(int val, int minimum, int maximum)
{
  return ((minimum <= val) && (val <= maximum));
}

void printRewardLane()
{
    // Already printed by the stats
    //Lane reward_lane = global_state.lanes[global_state.reward_lane_id];
    //Serial.print("Next reward lane id: ");
    //Serial.println(reward_lane.lane_id);
}

MotorDurationEntry* turnOnMotor(PIN_TYPE motor_id, long int activation_period)
{
  long int time_now = millis();
  for(int i = 0; i < global_state.MOTOR_DURATION_ENTERIES_SIZE; i++)
  {
    MotorDurationEntry* motor_entry = &global_state.motor_duration_entries[i];
    if (!motor_entry->activated)
    {
      do
      {
        pinMode(motor_id, OUTPUT);
        digitalWrite(motor_id, HIGH);
        // if (motor_id == 31 || motor_id == 32)
        //     pinMode(motor_id, INPUT);
      }
      while (!digitalRead(motor_id));
      pinMode(motor_id, OUTPUT);
      motor_entry->activated = true;
      motor_entry->motor_id = motor_id;
      motor_entry->activation_time = time_now;
      motor_entry->timeout_period = activation_period;
      Serial.print("Setting high on pin: ");
      Serial.println(motor_id);
      return motor_entry;
    }
  }

  Serial.print("Didn't find a non-empty motor slot to turn on motor pin: ");
  Serial.println(motor_id);
  return NULL;
}

void turnOffMotors()
{
  long int time_now = millis();
  for(int i = 0; i < global_state.MOTOR_DURATION_ENTERIES_SIZE; i++)
  {
    MotorDurationEntry* motor_entry = &global_state.motor_duration_entries[i];
    if (motor_entry->activated)
    {
      if (motor_entry->activation_time + motor_entry->timeout_period < time_now)
      {
        do
        {
          Serial.print("Turning off pin: ");
          Serial.println(motor_entry->motor_id);
          pinMode(motor_entry->motor_id, OUTPUT);
          digitalWrite(motor_entry->motor_id, LOW);
          // if (motor_entry->motor_id == 31 || motor_entry->motor_id == 32)
          //     pinMode(motor_entry->motor_id, INPUT);
        }
        while (digitalRead(motor_entry->motor_id));
        pinMode(motor_entry->motor_id, OUTPUT);
        motor_entry->activated = false;
        Serial.print("Turned off pin: ");
        Serial.println(motor_entry->motor_id);
        if (motor_entry->motor_id == Pins.SolenoidRight)
          writeStats(Stats.SOLENOID_RIGHT_OFF());
        else if (motor_entry->motor_id == Pins.SolenoidLeft)
          writeStats(Stats.SOLENOID_LEFT_OFF());
      }
    }
  }
}

void enterLane()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In enterLane()");
  #endif
  if (!global_state.was_inside_lane)
    writeStats(Stats.ENTERED_LANE(global_state.current_lane));
}

void pushActuator()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In pushActuator()");
  #endif
  actuator.setState(Actuator::PUSH);
  if (global_state.last_reported_actuator_status != Actuator::PUSH)
  {
    writeStats(Stats.MOTOR_PUSHED());
    global_state.last_reported_actuator_status = Actuator::PUSH;
    global_state.sensor_was_touched = false;
  }
  motor_pushed = true;
}

void reportActuatorAtMaxPush()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In reportActuatorAtMaxPush()");
  #endif
  if (!global_state.reported_motor_max_distance)
  {
    writeStats(Stats.MOTOR_MAX_RANGE());
    global_state.reported_motor_max_distance = true;
  }
}

void reportActuatorAtRest()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In reportActuatorAtRest()");
  #endif
  if (!global_state.reported_motor_max_wait)
  {
    Serial.print("max_push_current_duration: ");
    Serial.print(global_state.max_push_current_duration);
    Serial.print(" - MAX_PUSH_WAIT: ");
    Serial.print(global_state.MAX_PUSH_WAIT);
    Serial.print("- time_now: ");
    Serial.println(actuator_at_rest_time_now);
    writeStats(Stats.MOTOR_WAIT_DONE());
    global_state.reported_motor_max_wait = true;
  }
}

void reportSensorTouched()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In reportSensorTouched()");
  #endif
  // global_state.sensor_was_touched = true;
  // if (is_correct_sensor)
  //   global_state.miss_or_wrong_touch_count = 0;
  // else
  //   global_state.miss_or_wrong_touch_count += 1;
}

void beInsideLane(){
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In beInsideLane()");
  #endif
  global_state.was_inside_lane = true;
}

void reportExitedLane()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In reportExitedLane()");
  #endif
  writeStats(Stats.EXITED_LANE(global_state.current_lane));
}

void turnOffPiezo()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In turnOffPiezo()");
  #endif
  do
    digitalWrite(global_state.piezo_motor_entry->motor_id, LOW);
  #if DEBUG_W_VIRTUAL_MOUSE
  while (false);
  #else
  while (digitalRead(global_state.piezo_motor_entry->motor_id));
  #endif
  global_state.piezo_motor_entry->activated = false;
  global_state.piezo_motor_entry = NULL;
}

void beOutsideLane()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In beOutsideLane()");
  #endif
  global_state.was_inside_lane = false;
  global_state.reported_motor_max_distance = false;
}

void pullActuator()
{
  #if DEBUG_STATE_FUNCTIONS
  Serial.println("In pullActuator()");
  #endif
  //digitalWrite(29,LOW); //########################################################################################
  actuator.setState(Actuator::PULL);
  if (global_state.last_reported_actuator_status != Actuator::PULL)
  {
    writeStats(Stats.MOTOR_PULLED());
    global_state.last_reported_actuator_status = Actuator::PULL;
  }
  if (!global_state.actuator_at_min_pull)
    global_state.reported_motor_min_distance = false;
}


void writeStats(StatsMessage stat)
{
  long int time_now = millis();
  Serial.print("s:");
  Serial.print(time_now);
  Serial.print("\\id:");
  Serial.print(stat.event_id);
  Serial.print("\\msg:");
  Serial.print(stat.msg);
  if (stat.parameter != -1)
  {
    Serial.print("\\parameter:");
    Serial.print(stat.parameter);
  }
  Serial.println("");
  Serial.flush();
}

Maus calculateVelocity(Maus maus, int newX, int newY, int angle)
{
  maus.posx[maus.index] = newX;
  maus.posy[maus.index] = newY;
  //maus.angle[maus.index] = sin(angle)*1000;
  //Serial.println((sin((angle/57.296))+1)*120);
  float mean[2] = {0.0,0.0};
  //float mean_angle = 0;
  for(int i = 0; i<10 ; i++){
    mean[0] += maus.posx[i];
    mean[1] += maus.posy[i];
    //mean_angle += maus.angle[i];
    //Serial.print(maus.angle[i]);
    //Serial.print(", ");
  }
  //Serial.println();
  mean[0] /= 10;
  mean[1] /= 10;
  //maus.mean_angle = asin(mean_angle/10);
  //Serial.println(maus.mean_angle);
  float diff = sqrt(sq(mean[0] - maus.pos_old[0]) + sq(mean[1] - maus.pos_old[1]) );
  maus.difference[0] = mean[0]-maus.pos_old[0];
  maus.difference[1] = mean[1]-maus.pos_old[1];

  if(diff < 0.2)
    diff=0;

  //Serial.println(maus.difference[1]);
  // Optogenetic pulse randomly every 7 occurences

  if(maus.difference[1]>2 && newY > THRESHOLD - 5 && newY < THRESHOLD + 5){
    int random_number = random(100);
    Serial.println(random_number);
    if(random_number > 70 && opto_activation_trial != global_state.trial_number){
      digitalWrite(A13, HIGH);
      Serial.println("Opto-activation)");
      opto_activation_trial = global_state.trial_number;
    }
  }
  else
    digitalWrite(A13, LOW);
  /*
   * if(maus.optogenetics == false &&
  if(mean[1] < THRESHOLD){
    maus.below_threshold = true;
  }
  else{
    maus.below_threshold = false;
  }
   */
  analogWrite(13, (sin((angle/57.296))+1)*120);
  //Serial.println(maus.mean_angle);
  //Serial.println(diff);
  //Serial.println(maus.difference[0]);
  //Serial.println((maus.difference[1]+14)*9);

  //analogWrite(9, (maus.difference[1]+14)*9);  //uncomment for platform velocity
  analogWrite(9, mean[1]);
  /*
  Serial.print(" = sqrt(");
  Serial.print(mean[0]);
  Serial.print(" - ");
  Serial.print(maus.pos_old[0]);
  Serial.print(")^2 + (");
  Serial.print(mean[1]);
  Serial.print(" - ");
  Serial.print(maus.pos_old[1]);
  Serial.print(")^2");
  Serial.print(")");
  Serial.println();
    */
  maus.velocity = diff;
  maus.pos_old[0] = mean[0];
  maus.pos_old[1] = mean[1];
  if(maus.index < 9)
    maus.index++;
  else
    maus.index = 0;
  return maus;
}
