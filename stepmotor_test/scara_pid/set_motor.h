#ifndef SET_MOTOR_H
#define SET_MOTOR_H

#include <Arduino.h>

//베이스
#define j1_pul  4
#define j1_dir  5
#define j1_en   6
#define j1_A    18
#define j1_B    8
#define j1_Z    9
#define stop_j1 52

//z축
#define STEP_PIN  37
#define DIR_PIN   27
#define j2_en     28
#define PEND_PIN  21
#define ALM_PIN   29
#define stop_z    3

//중간 관절
#define j3_pul  10
#define j3_dir  11
#define j3_en   12
#define j3_A    19
#define j3_B    14
#define j3_Z    15
#define stop_j3 50

//엔드이펙터 회전
#define j4_pul  7
#define j4_dir  17
#define j4_en   31
#define j4_A    2
#define j4_B    24
#define j4_Z    25
#define stop_j4 48 // == 53으로 돼 있음

//리니어 레일
#define rail_pul  41
#define rail_dir  42
#define rail_en   43
#define stop_rail 44


//릴레이 핀 설정
#define num1 45
#define num2 46
#define num3 47

static inline void motor_pin()
{
  pinMode(j1_pul, OUTPUT); //j1 핀 설정
  pinMode(j1_dir, OUTPUT); 
  pinMode(j1_en, OUTPUT);
  
  pinMode(STEP_PIN, OUTPUT); //j2 핀설정
  pinMode(DIR_PIN, OUTPUT);

  pinMode(j3_pul, OUTPUT); //j3 핀설정
  pinMode(j3_dir, OUTPUT); 
  pinMode(j3_en, OUTPUT);
  
  pinMode(j4_pul, OUTPUT); //j4 핀설정
  pinMode(j4_dir, OUTPUT); 
  pinMode(j4_en, OUTPUT);

  pinMode(rail_pul, OUTPUT);
  pinMode(j4_dir, OUTPUT);
  pinMode(j4_en, OUTPUT); 

  digitalWrite(j1_en, HIGH); // en설정
  digitalWrite(j3_en, HIGH);
  digitalWrite(j4_en, HIGH);

  pinMode(j1_A, INPUT_PULLUP); //j1 엔코더 설정
  pinMode(j1_B, INPUT_PULLUP);

  pinMode(PEND_PIN, INPUT_PULLUP); //j2 엔코더 설정
  pinMode(ALM_PIN, INPUT_PULLUP);

  pinMode(j3_A, INPUT_PULLUP); //j3 엔코더 설정
  pinMode(j3_B, INPUT_PULLUP);
   
  pinMode(j4_A, INPUT_PULLUP); //j4 엔코더 설정
  pinMode(j4_B, INPUT_PULLUP);
  
  digitalWrite(STEP_PIN, HIGH); // z축, j2 

  pinMode(stop_z, INPUT_PULLUP); // 각 축 엔드스탑
  pinMode(stop_j1, INPUT_PULLUP);
  pinMode(stop_j3, INPUT_PULLUP);
  pinMode(stop_j4, INPUT_PULLUP);
  pinMode(stop_rail, INPUT_PULLUP);

  pinMode(num1, OUTPUT);
  pinMode(num2, OUTPUT);
  pinMode(num3, OUTPUT);

}

#endif
