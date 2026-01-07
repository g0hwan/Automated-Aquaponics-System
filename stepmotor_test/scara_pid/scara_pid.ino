#include "set_motor.h"
#include "encoder.h"
#include "pid.h"
//#include "kinematic.h"

unsigned long t1=0, t2=0, t3=0, t4=0;
const unsigned long PID_DT_MS = 10; 

void setup() {
  Serial.begin(115200);
  motor_pin();
  set_tim();
  set_int();
}

void loop() {
  
  unsigned long now = millis();
  if (now - t1 >= PID_DT_MS) { t1 = now; move_j1(-1000); }
  if (now - t3 >= PID_DT_MS) { t3 = now; move_j3(-1400); }
  if (now - t4 >= PID_DT_MS) { t4 = now; move_j4(800); }
}