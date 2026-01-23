#pragma once
#include <Arduino.h>

extern const int PUL;
extern const int DIR;
extern bool dir_state;

void stepPulse(bool dir, int speed);


//////////////////////////// 01.21 리니어레일 구동
void moveMM(bool dir, float mm, int speed);
void moveCM(bool dir, float cm, int speed);
////////////////////////////