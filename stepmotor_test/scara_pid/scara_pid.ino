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
  updatePrevFlags();
  sendState(SCARA_STATE_IDLE);  
}

void loop() {
<<<<<<< HEAD
  //home();
  sect0();
  sect1();
  //move_j2_cm(1);
  //delay(1000000);
  //delay(500000);
  /*
    serialReceiveTask();
=======
  serialReceiveTask();
>>>>>>> 19894be591562613b543e6a628dae01d2e1c257c

  if (processScaraControlRequests()) {
      return;
  }

  pathTask();

<<<<<<< HEAD
    pathTask();
  */
  //sect0();
  //sect1();
  //delay(10000000000);
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
  */
  
  //Serial.println(digitalRead(stop_rail)); // 안누르면 1, 누르면 0 이 나오면 정상
  //delay(15);
  
  //Serial.println(digitalRead(stop_z)); // 안누르면 1, 누르면 0 이 나오면 정상
  //delay(5);
=======
>>>>>>> 19894be591562613b543e6a628dae01d2e1c257c
}


