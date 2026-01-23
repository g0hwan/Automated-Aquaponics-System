#ifndef MOVE_H
#define MOVE_H

#include "encoder.h"
#include "kinematic.h"

//unsigned long J2_DEFAULT_PP;

void move_j2_mm(float mm, unsigned long pps =  J2_DEFAULT_PPS);
void move_j2_cm(float cm, unsigned long pps =  J2_DEFAULT_PPS);
void home();
#endif