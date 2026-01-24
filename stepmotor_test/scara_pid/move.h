#ifndef MOVE_H
#define MOVE_H

#include "encoder.h"
#include "set_motor.h"
#include "kinematic.h"

//unsigned long J2_DEFAULT_PP;

void move_j2_mm(float mm, unsigned long pps =  J2_DEFAULT_PPS);
void move_j2_cm(float cm, unsigned long pps =  J2_DEFAULT_PPS);
void j3_home_stop_on_switch(bool dir_to_switch, unsigned long pps);
void home();
#endif