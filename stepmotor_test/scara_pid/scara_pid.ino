#include "set_motor.h"
#include "encoder.h"
#include "pid.h"
#include "move.h"
#include "path.h"

void setup() {
  serialProtocolBegin(115200);
  motor_pin();
  set_tim();
  set_int();
}

void loop() {
  serialReceiveTask();
  pathTask();
  /*
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
  */
  //delay(5);
  //move_j3_wait(30);
  //move_j4_wait(-30);
  //move_j2_continuous(false, 80000);
  
  //j4_home_stop_on_switch_safe(true, 2000);
  //delay(200);
  //move_j4_wait(30);
  //move_j4_wait(-30);


  //move_j2_cm(-1, 80000);
  //delay(3000);
  //move_j2_cm(1, 80000);
  //delay(3000);  
  /*
  Serial.print("A=");
  Serial.print(digitalRead(j3_A));
  Serial.print("  B=");
  Serial.print(digitalRead(j3_B));
  Serial.print("  pos=");
  Serial.println(j3_enc.pos);
  delay(50);
  */
  //Serial.println(digitalRead(stop_j3)); // 안누르면 1, 누르면 0 이 나오면 정상
  //delay(5);
  //tool(true);
  //delay(15000);
  //moveRail_untilStop(false, 3000, stop2_rail);
  //move_j3_wait(900);
  home();
  //j3_home_stop_on_switch(true,5000);
  //goXY(80,0);
  //delay(50);
  //goXY(80,80);
  //delay(50);
  //goXY(0,80);
  //delay(50);
  delay(100000);
}


