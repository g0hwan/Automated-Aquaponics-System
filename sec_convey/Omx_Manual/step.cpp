#include "step.h"
#include <math.h>

const int PUL = 5;
const int DIR = 7;
bool dir_state = 0;

void stepPulse(bool dir, int speed) {
  if (dir == 0) // 후진
  {
    digitalWrite(DIR, dir);
    digitalWrite(PUL, LOW);          // ON (옵토 켬)
    delayMicroseconds(speed);
    digitalWrite(PUL, HIGH);         // OFF
    delayMicroseconds(10);          // 속도(작을수록 빠름)
    Serial.println("후진");
  }
  else if(dir == 1)
  {
    digitalWrite(DIR, dir);
    digitalWrite(PUL, LOW);          // ON (옵토 켬)
    delayMicroseconds(10);
    digitalWrite(PUL, HIGH);         // OFF
    delayMicroseconds(speed);          // 속도(작을수록 빠름)
    Serial.println("전진");
  }
}

///////////////////////////////////////////////////////////////// 01.21 리니어레일 구동, 드라이버 마이크로스텝 1600일때
constexpr float LEAD_MM = 5.0f;          // 1회전당 5mm
constexpr int   PULSES_PER_REV = 1600;   // 드라이버 마이크로스텝 설정
constexpr float PULSES_PER_MM = PULSES_PER_REV / LEAD_MM; // 320

void moveMM(bool dir, float mm, int speed) // mm단위로 구동
{
  if (mm < 0) mm = -mm;                       // 안전
  long pulses = lround(mm * PULSES_PER_MM/2);   // mm → pulses

  for (long i = 0; i < pulses; i++)
  {
    stepPulse(dir, speed);
  }
}
void moveCM(bool dir, float cm, int speed) // cm단위로 구동
{
  moveMM(dir, cm * 10.0f, speed);
}
//////////////////////////////////////////////////////////////