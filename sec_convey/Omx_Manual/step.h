#pragma once
#include <Arduino.h>

extern const int sw;
extern const int PUL;
extern const int DIR;
extern bool dir_state;

extern bool stop_req;

void STOP_ISR();
void STOP_NOW();
void setstep();
void stepPulse(bool dir, int speed);