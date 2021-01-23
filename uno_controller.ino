
const int cam_eye1 = 3;
const int cam_eye2 = 11;
const int cam_top = 10;
const int start_trigger = 13;
const int stop_trigger = 9;

volatile byte state = LOW;
volatile byte state2 = LOW;
int counter =0;


void setup() {
  pinMode(start_trigger, INPUT);
  pinMode(stop_trigger, INPUT);
  pinMode(cam_eye1, OUTPUT);
  pinMode(cam_eye2, OUTPUT);
  pinMode(cam_top, OUTPUT);

   TIMSK2 = (TIMSK2 & B11111110) | 0x01;
   TCCR2B = (TCCR2B & B11111000) | 0x02;
   Serial.begin(9600);
}


void loop() {


   if(digitalRead(start_trigger) == HIGH){
    do{
        digitalWrite(cam_top, state);
        digitalWrite(cam_eye1, state2);
        digitalWrite(cam_eye2, state2);
      }while(digitalRead(stop_trigger) != HIGH);
      delay(5);
      digitalWrite(cam_top, LOW);
      digitalWrite(cam_eye1, LOW);
      digitalWrite(cam_eye2, LOW);
      }
   }



ISR(TIMER2_OVF_vect){
  counter++;
  //if(counter2 >19){ // 200Hz
  //if(counter >9){ //200HZ
   // state = !state;
   // counter = 0;
  //}

  if (counter == 10){
    state = LOW;
    state2 = LOW;
  }
  else if (counter == 20){
    state = HIGH;
  }
  else if (counter == 30){
    state = LOW;
  }
  else if (counter >= 40){
    state = HIGH;
    state2 = HIGH;
    counter = 0;
  }
}
