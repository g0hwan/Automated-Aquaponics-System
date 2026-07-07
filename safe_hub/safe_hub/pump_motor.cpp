#include "pump_motor.h"

const uint8_t rel_pin[4] = {4, 5, 6, 7};

void pump_pin()                                          // 펌프모터 핀 설정
{
  for (int i=0; i<4; i++)
  {
    pinMode(rel_pin[i], OUTPUT);
  }
  //pump(off, off, off, off);
}

void pump(bool a, bool b, bool c, bool d)               //펌프모터 구동 함수
{
  if (a == 1) digitalWrite(rel_pin[0], HIGH);
  else digitalWrite(rel_pin[0], LOW);
  if (b == 1) digitalWrite(rel_pin[1], HIGH);
  else digitalWrite(rel_pin[1], LOW);
  if (c == 1) digitalWrite(rel_pin[2], HIGH);
  else digitalWrite(rel_pin[2], LOW);
  if (d == 1) digitalWrite(rel_pin[3], HIGH);
  else digitalWrite(rel_pin[3], LOW);
  }
