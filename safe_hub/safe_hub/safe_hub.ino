#include "pump_motor.h"

void setup() {
  pump_pin();

}
void loop() {
  pump(on, on, on, off);
}
