#include "set_motor.h"
#include "encoder.h"
#include "pid.h"
#include "move.h"
#include "path.h"
#include "Serial.h"

void setup() {
  serialProtocolBegin(115200);
  setflag();
  motor_pin();
  set_tim();
  set_int();
}

void loop() {
  //serialReceiveTask();
  //pathTask();
  //move_j1_wait(9000);
  //moveRail(3000,0);   
  //delay(900);
  //move_j1_wait(9000);
  //move_j2_cm(2);
  //move_j1_wait(-9000);
  
  j3_home_stop_on_switch(true, 5000);
  move_j2_cm(-5);
  //moveRail(3000,1);
  //delay(3000);
  //stopRail();
  j1_home_stop_on_switch(false, 4000);

  home();
  delay(1000);
  moveRail(3000,0);   
  delay(900);
  stopRail();
  move_j2_cm(-1.2);
  j1_home_stop_on_switch(true, 4200);
  delay(50);
  moveRail_untilStop(true, 4000, stop_rail);
  moveRail(2000,1);
  delay(2000);
  stopRail();
  enc_reset_j3();
  move_j3_wait(220);
  move_j1_wait(30);
  move_j2_cm(-9);
  delay(1000);
  move_j2_cm(9);

  move_j1_wait(-40);
  moveRail_untilStop(false, 4000, stop_rail);
  moveRail(3000,0);
  delay(700);
  stopRail();
  delay(100);
  moveRail_untilStop(true, 3000, stop_rail);
  delay(50);
  moveRail(3000,0);   
  delay(1800);
  stopRail();

  move_j2_cm(4.5);
  move_j1_wait(10);
  if (uv == 0)
  {
  moveRail(3000,0);   
  delay(2000);
  stopRail();
  move_j1_wait(30);
  enc_reset_j3();
  move_j3_wait(30);
  move_j2_cm(-3);
  delay(1000);
  move_j2_cm(3);
  move_j3_wait(-30);
  move_j1_wait(-30);
  uv++;
  }
  else if (uv == 1)
  {
    moveRail(3000,1);   
    delay(1000);
    stopRail();
    uv++; 
  }
  //moveRail_untilStop(true, 1000, stop_rail);
  //moveRail(1500, true);

  //delay(100000);
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
  //Serial.println(digitalRead(stop_rail)); // 안누르면 1, 누르면 0 이 나오면 정상
  //delay(5);
  //Serial.println(digitalRead(stop2_rail)); // 안누르면 1, 누르면 0 이 나오면 정상
  //delay(5);
  //tool(true);
  //delay(15000);
  //moveRail_untilStop(false, 3000, stop2_rail);
  //move_j3_wait(900);
  //home();
  //j3_home_stop_on_switch(true,5000);
  //goXY(80,0);
  //delay(50);
  //goXY(80,80);
  //delay(50);
  //goXY(0,80);
  //delay(50);
}


