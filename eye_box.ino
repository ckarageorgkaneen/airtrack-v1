/*
  LiquidCrystal Library - Hello World

 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.

 This sketch prints "Hello World!" to the LCD
 and shows the time.

  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe
 modified 7 Nov 2016
 by Arturo Guadalupi

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystalHelloWorld

*/

// include the library code:
#include <LiquidCrystal.h>

int counter=0;
String upper = "Experiment      ";
String lower = "Trials done: " + String(counter);
bool setup_mode = false;
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
const int info_1 = 6;
const int taster_2 = 9;

void setup() {
  pinMode(7, INPUT);
  pinMode(8,INPUT_PULLUP);
  pinMode(taster_2,INPUT_PULLUP);
  pinMode(info_1, OUTPUT);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  //lcd.print("hello, world!");
  Serial.begin(9600);
  update_screen();
}

void loop() {
  if(digitalRead(8)==LOW){
  counter=0;
  lower = "Trials done:    ";
  update_screen();
  lower = "Trials done: " + String(counter);
  update_screen();
  }
  if(digitalRead(7)==HIGH){
    delay(200);
    if(digitalRead(7)==HIGH){
      upper = "Setup-mode      ";
      update_screen();
      while(digitalRead(7)==HIGH){

      }
    }
    else
    counter++;
    upper = "Experiment      ";
    lower = "Trials done: " + String(counter);
    update_screen();
  }
  if (digitalRead(taster_2) == LOW){
    Serial.println("output");
    digitalWrite(info_1, HIGH);
  }
  else{
    digitalWrite(info_1, LOW);
  }




  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
 // lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  //lcd.print(millis() / 1000);
}
void update_screen(){
  lcd.setCursor(0, 0);
  lcd.print(upper);
  lcd.setCursor(0, 1);
  lcd.print(lower);

}
