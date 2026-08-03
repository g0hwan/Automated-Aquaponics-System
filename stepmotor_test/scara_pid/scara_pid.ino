#include "set_motor.h"
#include "encoder.h"
#include "pid.h"
#include "move.h"
#include "path.h"
#include "Serial.h"
#include "servo_config.h"

void setup() {
  serialProtocolBegin(115200);
  delay(50);
  setServo();
  delay(50);
  motor_pin();
  delay(50);
  set_tim();
  delay(50);
  set_int();
  delay(50);
  setflag();
  delay(50);
}

void loop() {
  //serialReceiveTask();
  //pathTask();  
  //home();
  ///*
  sect0();
  sect1();
  sect2();
  sect3();
  delay(40000000000);
  //*/
//*/
  /*
  Serial.print("A=");
  Serial.print(digitalRead(j1_A));
  Serial.print("  B=");
  Serial.print(digitalRead(j1_B));
  Serial.print("  pos=");
  Serial.println(j1_enc.pos);
  delay(50);
  //*/
  
  //Serial.println(digitalRead(stop_j1)); // 안누르면 1, 누르면 0 이 나오면 정상
  //delay(15);
  
  //Serial.println(digitalRead(stop_z)); // 안누르면 1, 누르면 0 이 나오면 정상
  //delay(5);
/*
  static long last = 0;

  noInterrupts();
  long now = j1_enc.pos;
  interrupts();

  if (now != last) {
    Serial.println(now);
    last = now;
  }

  delay(10);
  */
}


