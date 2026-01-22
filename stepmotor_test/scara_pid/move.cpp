#include "move.h"

void move_j2_mm(float mm, unsigned long pps)
{
  // mm -> 회전수 -> deg
  float deg = (mm / J2_LEAD_MM_PER_REV) * 360.0f;
  move_j2(deg, pps);   // 기존 함수 그대로 활용
}

// cm 단위 이동
void move_j2_cm(float cm, unsigned long pps)
{
  move_j2_mm(cm * 10.0f, pps);
}