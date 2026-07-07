#include "water_level.h"
#include "emergency.h"
#include "pump_motor.h"

const int wl_pin[2] = {A0, A1};
const uint8_t buz_pin = 8;

float off_wl_mm1;
float off_wl_mm2;

int rawval1;
int rawval2;

float wl_mm1;
float wl_mm2;

const float raw_min1 = 0.0;
const float raw_max1 = 300.0;
const float offset1 = 50.0;  // 15cm 센서 설치 높이 보정값
const float offset2 = 50.0;  // 25cm 센서 설치 높이 보정값
const float raw_min2 = 0.0;
const float raw_max2 = 300.0;

const float level_max1 = 150.0;
const float level_max2 = 250.0;

void lev_pin()
{
  pinMode(wl_pin[0], INPUT);
  pinMode(wl_pin[1], INPUT);

  pinMode(buz_pin, OUTPUT);
  digitalWrite(buz_pin, LOW);
}

void lev()
{
  rawval1 = analogRead(wl_pin[0]);
  rawval2 = analogRead(wl_pin[1]);

  wl_mm1 = (rawval1 - raw_min1) * level_max1 / (raw_max1 - raw_min1);

  wl_mm2 = (rawval2 - raw_min2) * level_max2 / (raw_max2 - raw_min2);

  wl_mm1 = constrain(wl_mm1, 0.0, level_max1);
  wl_mm2 = constrain(wl_mm2, 0.0, level_max2);

  off_wl_mm1 = wl_mm1 + offset1;
  off_wl_mm2 = wl_mm2 + offset2;

  
  delay(200);
}