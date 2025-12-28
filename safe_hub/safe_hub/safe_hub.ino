#include "pump_motor.h"
#include "water_level.h"

void setup() {
  pump_pin();

}
void loop() {
  pump(on, on, on, off);
  lev();
}
