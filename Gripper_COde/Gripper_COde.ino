#include <Servo.h>
#include "Filter.h"                  //megunolink exponential filter libary
#include "ArduinoTimer.h"    // for periodic measurement
//120
//down   20,13
// up 120,120
// diff 100,107

Servo servo1;               //initializing the sevos
Servo servo2;
Servo servo3;
int pos = 0;
int analogInPin = A0;      //defining A0  
int Opening = 900;        //setting threshold values 
int closing = 830;

int motorGrip1 = 4;        // assigning arduino digital pins to variables 
int motorGrip2 = 45;
int motorGrip3 = 43;


bool state = false;
int CurrentsensorValue = analogRead(analogInPin);  //updating current sensor value 
ExponentialFilter<float> FilteredCurrent(20, 0);      /// exponential filter

int CSValue = 0;
int filterCSValue = 0;
int trimConstantOpeing = 10;
int trimConstantClosing = 6;

float filterOpeing(int value);  
void openArm();
void closeArm();
void armDown();
int  ReadCurrentSensor();

void setup() {
  servo1.attach(4);
  servo2.attach(3);
  servo3.attach(2);
  Serial.begin(9600);

  servo1.write(120);      //moving the arm up and opening the gripper 
  servo2.write(13);
  servo3.write(0);
  openArm();
}

void loop()
{
  static ArduinoTimer MeasurementTimer;

  if (MeasurementTimer.TimePassed_Milliseconds(100)) {    //exponential filter 
    CSValue = ReadCurrentSensor();
    filterCSValue = filterOpeing(CSValue);
    Serial.println(pos);
    Serial.println(filterCSValue);
  }
  armDown();
  while (state) {
    CSValue = ReadCurrentSensor();
    if (filterCSValue > 830) {  //object only picked up after threshold value is reached 
      // stop griper
      closeArm();
    }
  }
   armUp();      //rotating the arm after picking object 
  armRotateLeft();
  openArm();
  armUp();
  armRotateRight;
  openArm();
  delay(5000);
}
void armDown() {
  for (pos = 120; pos >= 20; pos -= 5) {   //moving the am down to place the object 
    servo1.write(pos);
    servo2.write(pos - 8);
    Serial.println(pos);
    delay(80);
  }
}
void armUp() {
  for (pos = 20; pos <= 120; pos += 5) {
    // in steps of 1 degree
    servo1.write(pos);
    servo2.write(pos - 8);
  }
}
void armRotateRight() {
  for (pos = 10; pos <= 180; pos += 5) {
    // in steps of 1 degree
    servo3.write(pos);
  }
}
void armRotateLeft() {
  for (pos = 180; pos <= 10; pos -= 5) {
    // in steps of 1 degree
    servo3.write(pos);
  }
}
int  ReadCurrentSensor() {
  CurrentsensorValue = analogRead(analogInPin);
  if (CurrentsensorValue > Opening) {
    state = false;
  }
  if (CurrentsensorValue > closing) {
    state = true;
  }
  return CurrentsensorValue;
}
float filterOpeing(int value) {
  FilteredCurrent.Filter(value + 10 - trimConstantOpeing);
  float Smoothcurrentopeing = FilteredCurrent.Current();
  return (Smoothcurrent);
}
float filterClosing(int value) {
  FilteredCurrent.Filter(value - 15 + trimConstantClosing);
  float SmoothcurrentClosing = FilteredCurrent.Current();
  return (SmoothcurrentClosing);
}
void closeArm() {
  while (!state) {
    closeArmM();
    speedArmM(120);
    ReadCurrentSensor();
  }
  speedArmM(0);
  stopArmM();
}
void openArm() {
  while (!state) {
    openArmM();
    speedArmM(120);
    ReadCurrentSensor();
  }
  speedArmM(0);
  stopArmM();
}
void speedArmM(int speedValue) {
  analogWrite(motorGrip1, speedValue);
}
void closeArmM() {
  digitalWrite(motorGrip2, HIGH);
  digitalWrite(motorGrip3, LOW);
}
void openArmM() {
  digitalWrite(motorGrip2, LOW);
  digitalWrite(motorGrip3, HIGH);
}
void stopArmM() {
  digitalWrite(motorGrip2, LOW);
  digitalWrite(motorGrip3, LOW);
}
