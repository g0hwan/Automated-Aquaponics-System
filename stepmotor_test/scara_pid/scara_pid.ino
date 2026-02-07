#include "set_motor.h"
#include "encoder.h"
#include "pid.h"
#include "move.h"


void setup() {
  Serial.begin(115200);
  motor_pin();
  set_tim();
  set_int();
}

void loop() {
  //Serial.println(digitalRead(stop_z)); // 안누르면 1, 누르면 0 이 나오면 정상
  delay(5000);
  home();
  delay(200);
  goXY(120, 150);
  Serial.println("절대 이동1 완료");
  delay(100);
  moveXY_rel(10,30);
  Serial.println("상대이동1 완료");
  //delay(1000);
  moveXY_rel(-30, -50);
  Serial.println("상대이동2 완료");
  //delay(1000);
}
