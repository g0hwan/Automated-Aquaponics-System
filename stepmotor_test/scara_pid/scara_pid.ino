#include "set_motor.h"
#include "encoder.h"
#include "pid.h"
#include "move.h"

void move_j3_wait(float targetAngle,
                  float tolDeg = 1.0f,
                  unsigned long stable_ms = 150,
                  unsigned long timeout_ms = 8000);


void setup() {
  Serial.begin(115200);
  motor_pin();
  set_tim();
  set_int();
}

void loop() {
  /*
  move_j3_wait(50);
  home();
  delay(10000);
  */

  
}
