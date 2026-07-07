#include "set_motor.h"
#include "encoder.h"
#include "pid.h"
#include "move.h"
#include "path.h"
#include "Serial.h"
#include "servo_config.h"

void setup() {
  serialProtocolBegin(115200);
  setServo();
  delay(50);
  motor_pin();
  set_tim();
  set_int();
  setflag();
}

void loop() {
  //serialReceiveTask();
  //pathTask();

  //move_sync_13(30, 30, 1000,1000, 1.0, 150, 8000);
  home();
  Serial.println("home fin");
  delay(500);

  moveRail(3000, 0);   
  delay(900);
  stopRail();

  move_j2_cm(-4.2);

  moveRail_untilStop(true, 4000, stop_rail);

  moveRail(2000, 1);
  delay(2025);
  stopRail();

  delay(300);

  setSmf(0);
  sendSmf();

  setHm(0);
  sendHm();
  
  enc_reset_j3();
  move_j3_wait(155+45+5-360+75);
  //delay(100000);
  j1_home_stop_on_switch(false, 3200);
  delay(1300);
  enc_reset_j1();
  delay(10);
  move_j1_wait(45,2000);
  enc_reset_j1();
  delay(500);

  j4_move(true, 2000);
  delay(140);
  j4_stop();
  grip(true);//그립 펴고
  //delay(100);
  delay(300);
  move_j2_cm(-12.5); // 내려가서
  grip(false); //잡고
  //delay(100);
  delay(300);
  //delay(300);
  move_j2_cm(1); // 살짝 올라가서
  enc_reset_j1();
  move_j1_wait(-15, 1000); // 뒤로 좀 빼고
  delay(10);
  move_j2_cm(12); // 올라가고
  //move_j1_wait(-10);
  //move_j2_cm(3);

  setCrf(0); // 직교로봇 리셋
  sendCrf();
  
  j4_move(false, 2000);
  delay(140);
  j4_stop();
  enc_reset_j3();
  delay(200);
  move_j3_wait(30,2000);
  uv++;
  j1_home_stop_on_switch(true, 1200);
  //move_j2_cm(1.5);
  if (uv == 0)
  {
    moveRail_untilStop(false, 4500, stop3_rail);
    moveRail(1500,1);   
    delay(1000);
    stopRail();
    enc_reset_j1();
    move_j1_wait(55);
    enc_reset_j1();
    enc_reset_j3();
    move_j3_wait(10);
    enc_reset_j3();
    j4_move(true, 780);
    moveRail(2700,1);   
    delay(2000);
    j4_stop();
    moveRail(1300,0);
    delay(1400);
    stopRail();
    move_j2_cm(-3.2);
    grip(true);
    delay(10);
    move_j2_cm(3.2);
    j1_home_stop_on_switch(true, 3500);
    uv++;
  }
  else 
  {
    moveRail_untilStop(false, 4000, stop3_rail);
    moveRail(2000,0);   
    delay(1000);
    stopRail();
    enc_reset_j1();
    move_j1_wait(45);
    enc_reset_j1();
    enc_reset_j3();
    //move_j3_wait(10);
    enc_reset_j3();
    j4_move(true, 745);
    moveRail(250,0);   
    delay(2000);
    j4_stop();
    //moveRail(700,1);
    //delay(2000);
    stopRail();
    move_j2_cm(-2.2);
    grip(true);
    delay(10);
    move_j2_cm(2.2);
    j1_home_stop_on_switch(true, 3500);
    uv++;
  }

/* sect3
  home();
  urf++;
  ulf=0;
  moveRail_untilStop(false, 4000, stop3_rail);
  if (((urf == 0)&&(ulf == 1))||((urf == 1)&&(ulf == 1)))
  {
    enc_reset_j3();
    move_j3_wait(-100);
    move_j2_cm(-4.5);
    moveRail(2000,0);   
    delay(1300);
    stopRail();
    j1_home_stop_on_switch(false, 4200);
    delay(1000);
    enc_reset_j1();
    move_j1_wait(50);
    enc_reset_j1();
    enc_reset_j3();
    move_j3_wait(-50);
    enc_reset_j3();
    j4_move(true, 800);
    moveRail(1500,0);   
    delay(2000);
    j4_stop();
    moveRail(800,1);
    delay(2000);
    grip(true);
    stopRail();
    ulf --;
  }
  else if ((urf == 1)&&(ulf == 0))
  {
    enc_reset_j3();
    move_j3_wait(-100);
    move_j2_cm(-4.5);
    moveRail(2000,1);   
    delay(1000);
    stopRail();
    j1_home_stop_on_switch(false, 4200);
    delay(1500);
    enc_reset_j1();
    delay(100);
    move_j1_wait(40);
    enc_reset_j1();
    enc_reset_j3();
    move_j3_wait(-60);
    enc_reset_j3();
    grip(true);
    delay(10);
    j4_move(true, 1500);
    moveRail(2300,1);   
    delay(2000);
    j4_stop();
    moveRail(830,0);
    delay(2000);
    grip(true);
    stopRail();
    urf --;
  }

  move_j2_cm(-5.2);
  grip(false); //발아실의 트레이를 집음
  delay(50);
  move_j2_cm(5.2);
  j4_move(true, 800);
  delay(800);
  j4_stop();
  enc_reset_j3();
  delay(10);
  move_j3_wait(20+50);
  delay(30);
  enc_reset_j3();
  moveRail_untilStop(false, 4000, stop2_rail);
  enc_reset_j1();
  move_j1_wait(130);
  enc_reset_j1(); 
  j4_move(false, 800);
  delay(800);
  j4_stop();
  
  if (((wlf == 1)&&(wrf == 0))||((wlf==0)&&(wrf==0)))
  {
    move_j1_wait(30+40);
    enc_reset_j1();
    enc_reset_j3();
    move_j3_wait(-30-50);
    j4_move(true, 800);
    moveRail(1400,0);
    delay(2000);
    stopRail();
    j4_stop();
    move_j1_wait(-45);
    enc_reset_j1();
    j4_move(true, 380);
    delay(2000);
    j4_stop();
    move_j2_cm(-14.5);
    delay(10);
    move_j1_wait(-15);
    enc_reset_j1();
    j4_move(true, 500);
    delay(1500);
    j4_stop();
    j4_move(false, 650);
    delay(1500);
    j4_stop();
    move_j2_cm(-1);
    grip(true); 
  }
  else if ((wlf == 0)&&(wrf == 1))
  {
    move_j1_wait(50);
  }
  move_j2_cm(15);
  delay(10);
  moveRail_untilStop(true, 4200, stop3_rail);
  move_j2_cm(-1);
  j1_home_stop_on_switch(true, 4200);
  move_j1_wait(30);
  j2_home_stop_on_switch(true, 4000);
  delay(50);
  j4_move(true, 2500);
  delay(1000);
  j4_stop();  
  j4_home();
  j3_home_stop_on_switch(false, 5000);
  enc_reset_j3();
  move_j3_wait(30);
  delay(1000000);
*/
  /*
  Serial.print("A=");
  Serial.print(digitalRead(j1_A));
  Serial.print("  B=");
  Serial.print(digitalRead(j1_B));
  Serial.print("  pos=");
  Serial.println(j1_enc.pos);
  delay(50);
  */
  
  //Serial.println(digitalRead(stop_j4)); // 안누르면 1, 누르면 0 이 나오면 정상
  //delay(15);
  
  //Serial.println(digitalRead(stop_rail)); // 안누르면 1, 누르면 0 이 나오면 정상
  //delay(5);
}


