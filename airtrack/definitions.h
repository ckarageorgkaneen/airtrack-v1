#ifndef DEFINITIONS_MODULE
#define DEFINITIONS_MODULE

#include <Arduino.h>

// CONST_PIN_TYPE types cannot change their values after first declartion
// typedef static const unsigned int CONST_PIN_TYPE;
#define CONST_PIN_TYPE static const unsigned int // typedef is not working, dedbug later

typedef unsigned int PIN_TYPE;
typedef int16_t ANGLE;
typedef unsigned int LANE_ID;

struct SensorTouched
{
  bool change_happened;
  bool left_sensor;
  bool right_sensor;

  SensorTouched(bool left_touched, bool right_touched)
  {
    this->change_happened = false;
    this->left_sensor = left_touched;
    this->right_sensor = right_touched;
  }
};

struct SubjectLocation
{
    bool block_detected;
    ANGLE angle;
    uint16_t x;
    uint16_t y;
};

struct Lane
{
   LANE_ID lane_id;
   PIN_TYPE reward_sensor;
   ANGLE region_start_angle;
   ANGLE region_end_angle;

};

struct MotorDurationEntry
{
    bool activated;
    PIN_TYPE motor_id;
    long int activation_time;
    long int timeout_period;
};

struct Maus
{
  float posx[10];
  float posy[10];
  float pos_new[2];
  float pos_old[2];
  float difference[2];
  int angle[10];
  float mean_angle;
  float velocity;
  int index;
  bool optogenetics;
  bool below_threshold;
};

struct GlobalState
{
  int Servo1_pos = 0;
  bool Servo1_dir = true;


   static const LANE_ID NUM_OF_LANES = 4;
   Lane lanes[NUM_OF_LANES];
   // Guaranteed bound value must be divisble by num of lanes
   static const int GUARANTEED_RANDOM_BOUND = NUM_OF_LANES*1;
   LANE_ID lane_shuffle_list[GUARANTEED_RANDOM_BOUND];
   size_t shuffle_list_index;
   bool was_inside_lane;
   LANE_ID current_lane;
   LANE_ID reward_lane_id;
   bool reward_direction;
   bool rotation;
   SubjectLocation last_subject_location;
   bool actuator_at_max_push;
   bool actuator_at_min_pull;
   static const long int MAX_PUSH_TIMEOUT = 1000;
   static const long int MAX_PUSH_WAIT = 100;
   static const long int ALLOWED_REWARD_TIMEOUT = 1000;
   static const long int NO_REWARD_TIMEOUT = 100;
   static const long int PEIZO_TIMEOUT = 1000;
   long int motor_timeout_duration;
   long int max_push_current_duration;
   bool actuator_duration_activated;
   bool chose_new_lane;
   bool reward_given;
   bool in_first_lane;
   MotorDurationEntry *piezo_motor_entry;

   LANE_ID last_reported_lane; // Initially report non existing lane
   byte last_reported_light_status; // Assign any random initial value
   byte last_reported_actuator_status; // It's type is Actuator::State
   bool reported_motor_max_distance;
   bool reported_motor_min_distance;
   bool reported_motor_max_wait;
   bool sensor_was_touched;

   // Change to fit the number of motors that you have in your system
   static const size_t MOTOR_DURATION_ENTERIES_SIZE = 6;
   MotorDurationEntry motor_duration_entries[MOTOR_DURATION_ENTERIES_SIZE];

   int actuator_max_pwm_distance = 700;
   static const long int SOLENOID_DURATION = 200;

   unsigned int trial_number;

   static const long int SAME_SENSOR_MAX_THRESHOLD = 5;
   static const long int FORCE_OTHER_SENSOR = 3;
   static const long int FEEDBACK_AUTOMATED_REWARD_THRESHOLD = 3;
   bool last_sensor_touched_left;
   unsigned int same_sensor_touch_count;
   bool in_force_sensor_mode;
   bool is_force_sensor_left;
   int miss_or_wrong_touch_count;

   bool is_automated_reward;

   long int delayed_report;

   bool is_within_reward_lane_angle;
   bool is_inside_lane;
   bool motor_pushed;
   bool is_correct_sensor;
   bool is_delay_timed_out;
   long int actuator_at_rest_time_now;
};

struct DistancesStruct
{
    uint16_t x_threshold_min;
    uint16_t x_threshold_max;
    uint16_t y_threshold_min;
    uint16_t y_threshold_max;
    uint16_t y_motor_threshold;

    DistancesStruct ()
    {
        this->x_threshold_min = 100;
        this->x_threshold_max = 350;
        this->y_threshold_min = 2;
        this->y_threshold_max = 80;
        this->y_motor_threshold = 45;
    }
} Distances;



struct StatsMessage
{
  byte event_id;
  char const* msg;
  short parameter;

  StatsMessage(byte event_id, char const* msg, short parameter)
  {
      this->event_id = event_id;
      this->msg = msg;
      this->parameter = parameter;
  }

  StatsMessage(byte event_id, char const* msg)
  {
      this->event_id = event_id;
      this->msg = msg;
      this->parameter = -1;
    }
};

#endif
