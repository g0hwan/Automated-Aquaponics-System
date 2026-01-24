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

void home()
{
  
  //위로 올리다가 엔드스탑에서 멈추면 false
  bool ok = move_j2(7200.0f, 200000);
  // 엔드스탑에 걸렸다면 1cm 내려서 해제
  if (!ok) {
    move_j2_mm(-10.0f,15000);
  }
  // 그 다음 원하는 만큼(예: 5cm) 내려오기
  move_j2_cm(-5.0f);

  j3_home_stop_on_switch(true, 8000);
  enc_reset_j3();
  move_j3_wait(-15);

  j4_home_stop_on_switch(true, 2000);
  enc_reset_j4();
  move_j3_wait(-15,4000);
}





