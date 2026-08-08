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
  serialReceiveTask();

  if (processScaraControlRequests()) {
      return;
  }

  pathTask();

}


