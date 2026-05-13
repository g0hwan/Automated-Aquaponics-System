#include "utils.h"
#include "step.h"
#include "usb.h"  

void setup()
{
  
  Serial.begin(115200);
  initManipulator();
  //usb_begin(Serial);
  setstep();

  Serial.println("==== OpenManipulator Started! ====");
}


void loop()
{
  moveHome();
  delay(300);
  setGripper(false);
  moveJointAbs(87.54, 106.88, -2.11, -103.14);
  setGripper(true);
  delay(5000);

}




