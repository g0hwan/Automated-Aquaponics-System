#include "pump_motor.h"
#include "water_level.h"

void setup() {
  Serial.begin(9600);

  pump_pin();
  lev_pin();

  pump(off, off, off, off);
}

void loop() {
  lev();
  Serial.print("WL,");
  Serial.print(off_wl_mm1, 1);
  Serial.print(",");
  Serial.println(off_wl_mm2, 1);

  if (off_wl_mm1 > 180) {
    pump(on, on, on, on);
  }
  else {
    pump(off, off, off, off);
  }
}