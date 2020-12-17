#ifndef SENSOR_MODULE
#define SENSOR_MODULE

#import <Arduino.h>
// #include "I2C/I2C.cpp"
#include <Wire.h>
#include "mpr121.h"
#include "definitions.h"

struct Sensor
{
  private:

    PIN_TYPE interrupt_pin;

  public:

    CONST_PIN_TYPE LEFT_ANALOUGE_PIN = 9;
    CONST_PIN_TYPE RIGHT_ANALOUGE_PIN = 2;


    Sensor (PIN_TYPE sensor_interrupt_pin)
    {
        this->interrupt_pin = sensor_interrupt_pin;
    }

    SensorTouched readInput()
    {
      // Serial.print("Checking for interrupt on pin: ");
      // Serial.println(this->interrupt_pin);

      SensorTouched sensor_touched = SensorTouched(false, false);
      if (!digitalRead(this->interrupt_pin))
      {
        //read the touch state from the MPR121
        Wire.requestFrom(0x5A,2);
        //I2c.read(0x5A,2);

        byte LSB = Wire.read(); // I2c.receive();
        byte MSB = Wire.read(); // I2c.receive();

        uint16_t touched = ((MSB << 8) | LSB); //16bits that make up the touch states

        // Serial.print("Checking for interrupt. variable touched: ");
        // Serial.println(touched);
        // We run the next commented out for loop to check for each pin,
        // but we already know which pins are possible so no need for the loop
        /*
        for (int i=0; i < 12; i++)
        {  // Check what electrodes were pressed
            if(touched & (1<<i))
            {
            }
        }
        */

        sensor_touched.change_happened = false;
        if(touched & (1<<LEFT_ANALOUGE_PIN))
        {
          sensor_touched.left_sensor = true;
        }
        else
        {
          sensor_touched.left_sensor = false;
        }

        if(touched & (1<<RIGHT_ANALOUGE_PIN))
        {
          sensor_touched.right_sensor = true;
        }
        else
        {
          sensor_touched.right_sensor = false;
        }

        if (sensor_touched.left_sensor || sensor_touched.right_sensor)
        {
          sensor_touched.change_happened = true;
        }
      }
      if (sensor_touched.change_happened)
      {
        //Serial.print("Returning sensor_touched: change_happened: ");
        //Serial.print(sensor_touched.change_happened ? "true" : "false");
        //Serial.print(" left_sensor: ");
        //Serial.print(sensor_touched.left_sensor ? "touched" : "untouched");
        //Serial.print(" right_sensor: ");
        //Serial.println(sensor_touched.right_sensor ? "touch" : "untouched");
      }
      return sensor_touched;
    }

    void setup()
    {
      static bool initialize_once = false;
      if (!initialize_once)
      {
        this->setupOnce();
        initialize_once = true;
      }
    }

  private:
    void setupOnce(void)
    {
      //Serial.print("Setting up sensor on pin: ");
      //Serial.println(this->interrupt_pin);
      pinMode(this->interrupt_pin, INPUT);
      digitalWrite(this->interrupt_pin, HIGH); //enable pullup resistor

      Wire.begin();
      // I2c.begin();
      // I2c.timeOut(3000);

      //Serial.println("Setting 0x00 register");
      set_register(0x5A, ELE_CFG, 0x00);

      //Serial.println("Setting Section A");
      // Section A - Controls filtering when data is > baseline.
      set_register(0x5A, MHD_R, 0x01);
      set_register(0x5A, NHD_R, 0x01);
      set_register(0x5A, NCL_R, 0x00);
      set_register(0x5A, FDL_R, 0x00);

      //Serial.println("Setting Section B");
      // Section B - Controls filtering when data is < baseline.
      set_register(0x5A, MHD_F, 0x01);
      set_register(0x5A, NHD_F, 0x01);
      set_register(0x5A, NCL_F, 0xFF);
      set_register(0x5A, FDL_F, 0x02);

      // Section C - Sets touch and release thresholds for each electrode
      // #define NEW_TOU_THRESH TOU_THRESH // 0x10
      // #define NEW_REL_THRESH REL_THRESH // 0x02
      #define NEW_TOU_THRESH 0x09
      #define NEW_REL_THRESH 0x35

      set_register(0x5A, ELE0_T, NEW_TOU_THRESH);
      set_register(0x5A, ELE0_R, NEW_REL_THRESH);

      set_register(0x5A, ELE1_T, NEW_TOU_THRESH);
      set_register(0x5A, ELE1_R, NEW_REL_THRESH);

      set_register(0x5A, ELE2_T, NEW_TOU_THRESH);
      set_register(0x5A, ELE2_R, NEW_REL_THRESH);

      set_register(0x5A, ELE3_T, NEW_TOU_THRESH);
      set_register(0x5A, ELE3_R, NEW_REL_THRESH);

      set_register(0x5A, ELE4_T, NEW_TOU_THRESH);
      set_register(0x5A, ELE4_R, NEW_REL_THRESH);

      set_register(0x5A, ELE5_T, NEW_TOU_THRESH);
      set_register(0x5A, ELE5_R, NEW_REL_THRESH);

      set_register(0x5A, ELE6_T, NEW_TOU_THRESH);
      set_register(0x5A, ELE6_R, NEW_REL_THRESH);

      set_register(0x5A, ELE7_T, NEW_TOU_THRESH);
      set_register(0x5A, ELE7_R, NEW_REL_THRESH);

      set_register(0x5A, ELE8_T, NEW_TOU_THRESH);
      set_register(0x5A, ELE8_R, NEW_REL_THRESH);

      #define NEW_TOU_THRESH_SPECIAL 0x01
      #define NEW_REL_THRESH_SPECIAL 0x35

      set_register(0x5A, ELE9_T, NEW_TOU_THRESH_SPECIAL);
      set_register(0x5A, ELE9_R, NEW_REL_THRESH_SPECIAL);

      set_register(0x5A, ELE10_T, NEW_TOU_THRESH);
      set_register(0x5A, ELE10_R, NEW_REL_THRESH);

      set_register(0x5A, ELE11_T, NEW_TOU_THRESH);
      set_register(0x5A, ELE11_R, NEW_REL_THRESH);

      //Serial.println("Setting Section D");
      // Section D
      // Set the Filter Configuration
      // Set ESI2
      set_register(0x5A, FIL_CFG, 0x04);

      // Section E
      // Electrode Configuration
      // Set ELE_CFG to 0x00 to return to standby mode
      set_register(0x5A, ELE_CFG, 0x0C);  // Enables all 12 Electrodes


      // Section F
      // Enable Auto Config and auto Reconfig
      /*set_register(0x5A, ATO_CFG0, 0x0B);
      set_register(0x5A, ATO_CFGU, 0xC9);  // USL = (Vdd-0.7)/vdd*256 = 0xC9 @3.3V   set_register(0x5A, ATO_CFGL, 0x82);  // LSL = 0.65*USL = 0x82 @3.3V
      set_register(0x5A, ATO_CFGT, 0xB5);*/  // Target = 0.9*USL = 0xB5 @3.3V
      set_register(0x5A, ELE_CFG, 0x0C);
    }

    void set_register(int address, unsigned char r, unsigned char v)
    {
        // Serial.println("Beginnig transimittion");
        Wire.beginTransmission(address);
        //Serial.print("Writing r:");
        Wire.write(r);
        //Serial.println( I2c.write((uint8_t)address, r) );
        //Serial.print("Writing v: ");
        Wire.write(v);
        //Serial.println( I2c.write((uint8_t)address, v) );
        //Serial.println("ending transimittion");
        Wire.endTransmission();
    }
};

#endif
