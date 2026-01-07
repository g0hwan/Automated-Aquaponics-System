#include "set_motor.h"
#include "encoder.h"
#include "pid.h"

void setup() {
  Serial.begin(115200);
  motor_pin();
  set_tim();
  set_int();
}

void loop() {
  move_j1(700);  
  //move_j2(200);
  //move_j3(400);
  //move_j4(500);
}