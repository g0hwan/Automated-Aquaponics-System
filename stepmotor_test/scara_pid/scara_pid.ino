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
  delay(8000);
  move_j4_wait(90,2000);
  move_j3_wait(50);
  home();
  delay(10000);

  //if (digitalRead(stop_j4)==LOW) Serial.println("스위치 인식");
  //else Serial.println("스위치 인식x");
}
