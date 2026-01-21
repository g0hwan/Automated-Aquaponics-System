#include "utils.h"
#include "step.h"
#include "usb.h"  

void setup()
{
  
  Serial.begin(115200);
  initManipulator();
  usb_begin(Serial);
  setstep();

  Serial.println("==== OpenManipulator Started! ====");
}


void loop()
{
  /*
  float ang;
  if (usb_poll_angle(ang)) {
    Serial.print("[RX] angle = ");
    Serial.println(ang, 2);

    // 각도 수신되면 홈으로 이동
    moveHome();
    for (int i = 0; i < 1000; i++) stepPulse(0, 500);
    //Serial.println("[INFO] moveHome() called");
    
  }*/

  //moveHome();
  //for (int i = 0; i < 1000; i++) stepPulse(0, 1000);
  //delay(1000);
  
  /*
  moveHome();

   if (stop_req) {
    STOP_NOW();
    while(1) { delay(10); }  // 테스트용: 완전 정지 유지
  }

  // 예: 이동 루프에서도 stop_req 체크
  for (int i=0; i<50000; i++) {
    if (stop_req) { STOP_NOW(); break; }
    stepPulse(0, 200);
  }
  */
  //readJoint();
  moveHome();
  delay(100000);
}




