#include <Arduino.h>
#include <Wire.h> // Need by sensor.h
#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>

#include "definitions.h"
#include "leds.h"
#include "pins.h"
#include "stats.h"
#include "sensor.h"
#include "actuator.h"
#include "reporter.h"

const bool AUTOMATED_REWARD = true;
const bool SINGLE_REWARD = true;
const bool FEEDBACK_AUTOMATED_REWARD = false;
const float threshold = 75;
const int pumpActivatorPulse = 29;

const bool training = true;
const int time_until_training = 8000;
int time_counter=0;

int opto_activation_trial = -1;

typedef struct Maus{
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

}Maus;

struct Maus maus;

GlobalState global_state;
Sensor sensor = Sensor(Pins.Sensor);
Actuator actuator = Actuator(Pins.ActuatorPush, Pins.ActuatorPull,
                             global_state.actuator_max_pwm_distance,
                             &global_state);
Pixy pixy;
Servo myservo1;
Servo myservoleft;
Servo myservoright;

// the setup routine runs once when you press reset:
void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing");
    Serial.println("Setting up sensor");
    sensor.setup();
    Serial.println("Setting up LEDs");
    setupLeds();
    Serial.println("Setting up Pins");
    setupPins();
    Serial.println("Setting up Lanes");
    setupLanes();
    Serial.println("Setting up Pixy");
    pixy.init();
    Serial.println("Pixy is set up");


    pinMode(25, INPUT_PULLUP);
    pinMode(31,INPUT_PULLUP);
    pinMode(35, INPUT_PULLUP);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(44, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(23, INPUT);
    pinMode(pumpActivatorPulse, OUTPUT);


    //GPIO Outputs for trigger
    pinMode(A8, OUTPUT);
    pinMode(A9, OUTPUT);
    pinMode(A10, OUTPUT);
    pinMode(A11, OUTPUT);
    pinMode(A12, OUTPUT);
    pinMode(A13, OUTPUT);
    pinMode(A14, OUTPUT);
    pinMode(A15, OUTPUT);

    myservo1.attach(2);
    myservoright.attach(6);
    myservoleft.attach(7);
    myservoright.attach(44);

    //pinMode(29, OUTPUT);

    for (int i = 0; i < global_state.MOTOR_DURATION_ENTERIES_SIZE; i++)
    {
        global_state.motor_duration_entries[i].activated = false;
    }

    global_state.is_automated_reward = AUTOMATED_REWARD;
    global_state.trial_number = 0;
    global_state.was_inside_lane = false;
    // Initially report non existing lane
    global_state.last_reported_lane = global_state.NUM_OF_LANES + 1;
    // Assign any random initial value
    global_state.last_reported_light_status = 250;

    global_state.actuator_duration_activated = false;
    Serial.println("Setting up actuator");
    actuator.setup();
    Serial.println("Actuator is set up");

    // We are seeding the random so it would give us reproducible results
    // randomSeed(0);// call randomSeed(analogRead(A3)) for random order on each run
    randomSeed(analogRead(A3));
    global_state.was_inside_lane = false;

    // Assign last lane id to soeme random value so that it'd be possible to
    // choose lane 0 as the first lane
    global_state.reward_lane_id = -1;
    global_state.reward_direction = -1;
    createShuffledChoice();
    makeNewRewardLane();
}


// the loop routine runs over and over again forever:
void loop ()
{
//Serial.println(global_state.reward_direction);
  //#######################################################################################Begin
  if(digitalRead(25)==LOW)   //Enter Setup
  {
    Serial.println("Setup entered");
     digitalWrite(A9,HIGH);
    delay(300);
    while(digitalRead(25)==HIGH)
    {
      digitalWrite(3, HIGH);
      if(digitalRead(35)==LOW)
      {
        actuator.enablePush();
      }
      else if(digitalRead(31)==LOW)
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
  }
  digitalWrite(A9,LOW);

  if(digitalRead(23)==HIGH){
    //turnOnMotor(Pins.SolenoidLeft, global_state.SOLENOID_DURATION);
    Serial.println("Pump pulse");
    digitalWrite(pumpActivatorPulse, HIGH);
    delayMicroseconds(100);
    digitalWrite(pumpActivatorPulse,LOW);
  }


  //#######################################################################################End



    SubjectLocation subject_location = getSubjectLocation();
    if (subject_location.block_detected)
    {
        global_state.last_subject_location = subject_location;
    }
    else
    {
        subject_location = global_state.last_subject_location;
    }

    bool is_within_reward_lane_angle = isWithinRewardLaneAngle(subject_location);
    bool is_inside_lane = isInsideLane(subject_location);

    move_training_servos(subject_location.angle, is_within_reward_lane_angle);

    bool motor_pushed = false;




  if(time_counter<time_until_training+10 && !is_within_reward_lane_angle){
    time_counter++;
  }


int factor = 100;
   if(time_counter > time_until_training && training && global_state.Servo1_pos < 160*factor && !is_within_reward_lane_angle){
    global_state.Servo1_pos += 1;
    myservo1.write(global_state.Servo1_pos/factor);
    }








    SensorTouched touched_sensor = sensor.readInput();
    if ( !touched_sensor.change_happened )
    {
        Stats.REPORT_SENSORS_UNTOUCHED();
    }

    // Serial.print("We are within reward lane");
    if (is_inside_lane)
    {
        if (!global_state.was_inside_lane)
        {
            writeStats(Stats.ENTERED_LANE(global_state.current_lane));
        }
        global_state.was_inside_lane = true;

        // Serial.print( " and inside lane ");
        if (is_within_reward_lane_angle)
        {
            if (shouldTriggerMotor(subject_location))
            {
                // Serial.print( " and we should fire motor ");
                //Serial.println("");
                actuator.setState(Actuator::PUSH);

                if (global_state.last_reported_actuator_status != Actuator::PUSH)
                {
                    writeStats(Stats.MOTOR_PUSHED());
                    global_state.last_reported_actuator_status = Actuator::PUSH;
                    global_state.sensor_was_touched = false;
                }
                motor_pushed = true;

                if (global_state.actuator_at_max_push)
                {
                    if (!global_state.reported_motor_max_distance)
                    {
                        writeStats(Stats.MOTOR_MAX_RANGE());
                        global_state.reported_motor_max_distance = true;
                    }

                    // Allow a bit of buffer time for sensor vibration to
                    // rest before reading
                    long int time_now = millis();
                    if (global_state.max_push_current_duration <=
                        time_now - global_state.MAX_PUSH_WAIT)
                    {
                        if (!global_state.reported_motor_max_wait)
                        {
                            Serial.print("max_push_current_duration: ");
                            Serial.print(global_state.max_push_current_duration);
                            Serial.print(" - MAX_PUSH_WAIT: ");
                            Serial.print(global_state.MAX_PUSH_WAIT);
                            Serial.print("- time_now: ");
                            Serial.println(time_now);
                            writeStats(Stats.MOTOR_WAIT_DONE());
                            global_state.reported_motor_max_wait = true;
                        }

                        // Call isCorrectSensor() anyway so it'd call
                        // writeStats() on the touched sensor
                        bool is_correct_sensor = isCorrectSensor(touched_sensor);
                        if (touched_sensor.change_happened &&
                            !global_state.sensor_was_touched)
                        {
                            global_state.sensor_was_touched = true;
                            if (is_correct_sensor)
                                global_state.miss_or_wrong_touch_count = 0;
                            else
                                global_state.miss_or_wrong_touch_count += 1;
                        }

                        if (global_state.is_automated_reward ||
                            touched_sensor.change_happened)
                        {
                            checkGiveReward(is_correct_sensor,
                                            global_state.is_automated_reward);
                        }
                    }
                }
            }
        }
        else
        {
            // Count that it's a bad trial
            // if (touched_sensor.change_happened)
            // {
            //    checkGiveReward(false);
            // }
        }
        //Serial.println("");
    }
    else // Outside a lane
    {
        if (global_state.was_inside_lane)
        {
            writeStats(Stats.EXITED_LANE(global_state.current_lane));

            // Turn off peizo if it was on
            if (global_state.peizo_motor_entry != NULL)
            {
                do
                {
                    digitalWrite(global_state.peizo_motor_entry->motor_id, LOW);
                }
                while (digitalRead(global_state.peizo_motor_entry->motor_id));
                global_state.peizo_motor_entry->activated = false;
                global_state.peizo_motor_entry = NULL;
            }
        }

        global_state.was_inside_lane = false;
        global_state.reported_motor_max_distance = false;
    }

    if (subject_location.block_detected && !motor_pushed)
    {
      //digitalWrite(29,LOW); //########################################################################################

        actuator.setState(Actuator::PULL);
        if (global_state.last_reported_actuator_status != Actuator::PULL)
        {
            writeStats(Stats.MOTOR_PULLED());
            global_state.last_reported_actuator_status = Actuator::PULL;
        }

        if (global_state.actuator_at_min_pull)
        {
            if (!global_state.reported_motor_min_distance)
            {
                writeStats(Stats.MOTOR_MIN_RANGE());
                global_state.reported_motor_min_distance = true;
                // TODO: Do it cleanly
                turnOnMotor(13, 20);
                //digitalWrite(45, HIGH);
            }
        }
        else
        {
            global_state.reported_motor_min_distance = false;
        }
    }

    global_state.was_inside_lane = is_inside_lane;
    turnOffMotor();
    actuator.motorLoop();

    if (global_state.delayed_report)
    {
        long int time_now = millis();
        if (time_now >= global_state.delayed_report)
        {
            // TODO: Do it cleanly
            turnOnMotor(13, 20);
            global_state.delayed_report = 0;
        }
    }
}

bool isInsideLane(SubjectLocation subject_location)
{
    int flexible_range = 10;
    if (global_state.was_inside_lane)
    {
        if (subject_location.y >= Distances.y_threshold_max &&
            subject_location.y - Distances.y_threshold_max <= flexible_range)
        {
            subject_location.y = Distances.y_threshold_max - 1;
        }
    }
    else
    {
        if (subject_location.y <= Distances.y_threshold_max &&
            Distances.y_threshold_max - subject_location.y <= flexible_range)
        {
            subject_location.y = Distances.y_threshold_max + 1;
        }
    }

    return Distances.x_threshold_min < subject_location.x &&
           subject_location.x        < Distances.x_threshold_max &&
           Distances.y_threshold_min < subject_location.y &&
           subject_location.y        < Distances.y_threshold_max;
}

bool shouldTriggerMotor(SubjectLocation subject_location)
{
    bool is_within_distance = Distances.y_threshold_min < subject_location.y &&
                              subject_location.y < Distances.y_motor_threshold;

    if (!is_within_distance)
    {
        // Serial.println("Not withing distance");
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
            if (isWithinRewardLaneAngle(subject_location) &&
                !global_state.chose_new_lane)
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

SubjectLocation getSubjectLocation()
{
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
    }

    return location;
}

bool isCorrectSensor(SensorTouched touched_sensor)
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
    {
        digitalWrite(Leds.SensorRight, LOW);
    }

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

void checkGiveReward(bool is_correct_sensor, bool is_automated_reward)
{
    if (global_state.reward_given)
        return;

    if (is_correct_sensor || is_automated_reward)
    {
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
            digitalWrite(pumpActivatorPulse, HIGH);
            delayMicroseconds(global_state.SOLENOID_DURATION);
            digitalWrite(pumpActivatorPulse, LOW);
            //turnOnMotor(Pins.SolenoidLeft, global_state.SOLENOID_DURATION);
        }
        else
        {
            Serial.println("Want to give reward - Unknown Solenoid");
        }

        setActuatorTimeout(global_state.ALLOWED_REWARD_TIMEOUT);
    }
    else
    {
        //Serial.println("No reward");
        writeStats(Stats.REWARD_NOT_GIVEN());
        setActuatorTimeout(global_state.NO_REWARD_TIMEOUT);
        global_state.peizo_motor_entry = turnOnMotor(Pins.PeizoTone,
                                                    global_state.PEIZO_TIMEOUT);
    }

    global_state.reward_given = true;
}

void makeNewRewardLane()
{
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

    //##########################################################################################################################


}
void failedTrial(){
  LANE_ID new_lane_id = global_state.NUM_OF_LANES + 1;
      //##################################################### same as easy trial but without assigning a new reward direction

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
}


void createShuffledChoice()
{
    if (global_state.GUARANTEED_RANDOM_BOUND % global_state.NUM_OF_LANES != 0)
    {
        Serial.println("Num of lanes is not divisble by random bound");
    }

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

    //printShuffleList();
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

    pinMode(Pins.LaneLight, OUTPUT);
    digitalWrite(Pins.LaneLight, LOW);

    pinMode(Pins.PeizoTone, OUTPUT);
    digitalWrite(Pins.PeizoTone, LOW);

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
}

void setupLeds()
{
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
}

void setupLanes()
{
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
}

bool isWithinRewardLaneAngle(SubjectLocation subject_location)
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
             switch(i){  //Here it is decided if the animal went right(0) or left(1)
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

                myservo1.write(10);

                if (global_state.last_reported_light_status != LOW)
                {
                    writeStats(Stats.LIGHT_OFF());
                    global_state.last_reported_light_status = LOW;
                }
                return true;
            }
            else if(global_state.reward_direction == 0){
                  digitalWrite(Pins.LaneLight, LOW);
                  digitalWrite(Pins.LaneLight2, HIGH);
                  return false;
            }
              else if(global_state.reward_direction == 1){
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
void move_training_servos(int angle, bool is_within_reward_lane_angle){
  if(global_state.reward_direction == 0){
    if(inRange(angle, -120,-90) || inRange(angle, -30,0) || inRange(angle, 60,80) || inRange(angle, 160,180)){
      myservoleft.write(170);
      if(is_within_reward_lane_angle){
        myservoright.write(20);
      }
    }
    else{
      myservoleft.write(20);
      myservoright.write(170);
    }
  }
  else{
    if(inRange(angle, -120,-90) || inRange(angle, -30,0) || inRange(angle, 60,80) || inRange(angle, 160,180)){
      myservoright.write(20);
      if(is_within_reward_lane_angle){
        myservoleft.write(170);
      }
    }
    else{
      myservoleft.write(20);
      myservoright.write(170);
    }

  }
}

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

void turnOffMotor()
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

Maus calculateVelocity(Maus maus, int newX, int newY, int angle){
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

if(diff<0.2){
  diff=0;
}

//Serial.println(maus.difference[1]);
// Optogenetic pulse randomly every 7 occurences

if(maus.difference[1]>2 && newY > threshold - 5 && newY < threshold + 5){
  int random_number = random(100);
  Serial.println(random_number);
  if(random_number > 70 && opto_activation_trial != global_state.trial_number){
    digitalWrite(A13, HIGH);
    Serial.println("Opto-activation)");
    opto_activation_trial = global_state.trial_number;
  }

}
else{
  digitalWrite(A13, LOW);
}

/*
 * if(maus.optogenetics == false &&


if(mean[1] < threshold){
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
  if(maus.index<9){
    maus.index++;
  }
  else{
    maus.index = 0;
  }
  return maus;
}
