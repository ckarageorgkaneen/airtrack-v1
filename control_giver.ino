#include <Keyboard.h>
char ctrlKey = KEY_LEFT_CTRL;
const int ZR_stop_key = 9;
const int ZR_start_key = 7;

void setup() {
  pinMode(15, INPUT_PULLUP);
  pinMode(ZR_stop_key, OUTPUT);
  pinMode(ZR_start_key, OUTPUT);
  pinMode(2, INPUT); //Start
  pinMode(3, INPUT); //Stop
  Keyboard.begin();
}

void loop() {
  if(digitalRead(2)==HIGH){
    digitalWrite(ZR_start_key, HIGH);
    Keyboard.press(ctrlKey);
    Keyboard.press('i');
    Keyboard.releaseAll();
    delay(100);
    digitalWrite(ZR_start_key, LOW);
  }
    if(digitalRead(3)==HIGH){
    digitalWrite(ZR_stop_key, HIGH);
    Keyboard.press(ctrlKey);
    Keyboard.press('o');
    Keyboard.releaseAll();
    delay(100);
    digitalWrite(ZR_stop_key, LOW);
  }

    if(digitalRead(15)==LOW){
    Keyboard.press(ctrlKey);
    Keyboard.press('p');
    Keyboard.releaseAll();
    delay(100);
  }
  digitalWrite(7, LOW);


}
