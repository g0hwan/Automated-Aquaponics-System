#include "water_level.h"

const int wl_pin[2] = {A0,A1};
const uint8_t buz_pin = 5;

void lev_pin()
{
  pinMode(buz_pin, OUTPUT);
}
void lev()
{
  int rawval1 = analogRead(wl_pin[0]);
  float wl_mm1 = 42.0 + ((rawval1 - 722.0) * 288.0 / (1022.0 - 722.0));
  Serial.print("Level(mm): ");
  Serial.print(wl_mm1, 1);

  if (wl_mm1 < 150) digitalWrite(buz_pin, HIGH);
  else digitalWrite(buz_pin, LOW);
  delay(1000);  // 1초 주기
}