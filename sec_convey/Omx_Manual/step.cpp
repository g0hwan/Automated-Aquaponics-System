#include "step.h"

const int sw = 2;
const int PUL = 5;
const int DIR = 7;
bool dir_state = 0;

bool stop_req = false;

void setstep(){
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(sw, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sw), STOP_ISR, FALLING);
  digitalWrite(PUL, HIGH); // 기본 OFF (옵토 끔)
}

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

void STOP_ISR() {
  stop_req = true;
}

void STOP_NOW() {
  digitalWrite(PUL, HIGH);   // 펄스 끊기(옵토 끔)
  // ENA 핀이 있으면: digitalWrite(ENA, HIGH/LOW)로 disable
}