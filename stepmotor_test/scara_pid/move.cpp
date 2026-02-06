#include "move.h"

void move_j2_mm(float mm, unsigned long pps)
{
  // mm -> 회전수 -> deg
  float deg = (-mm / J2_LEAD_MM_PER_REV) * 360.0f;
  move_j2(deg, pps);   // 기존 함수 그대로 활용
}

// cm 단위 이동
void move_j2_cm(float cm, unsigned long pps)
{
  move_j2_mm(cm * - 10.0f, pps);
}

void home()
{
  motors_enable_all(true);

  j1_home_stop_on_switch(true, 2000);
  delay(500);
  enc_reset_j1();
  Serial.println("j1 end");

  bool ok = move_j2(7200.0f, 600000);

  if (!ok) {
    move_j2_mm(-10.0f);
  }
  move_j2_cm(-5.0f);
  Serial.println("j2 end"); 

  j3_home_stop_on_switch(true, 3000);
  delay(500);
  enc_reset_j3();
  delay(500);
  //move_j3_wait(-25.0f);
  move_j3_wait(175.0f);
  delay(500);
  Serial.println("j3 end");

  //move_j4_wait(20);
  //delay(50);
  j4_home_stop_on_switch_safe(true, 2000);
  delay(500);
  move_j4_wait(-15.0f);
  delay(50);
  Serial.println("j4 end");
  enc_reset_all();
  
  motors_enable_all(false);
}


void goXY(float x, float y)
{
  motors_enable_all(true);
  float th1, th2;

  bool ok = inverse2R(x, y, L1_mm, L2_mm, /*elbowUp=*/true, th1, th2);
  if (!ok) {
    Serial.println("[IK] unreachable");
    return;
  }

  // 이제 조인트각으로 이동
  move_j1_wait(th1);
  move_j3_wait(th2);
  Serial.println(th2);
  Serial.println(th1);

  motors_enable_all(false);
}

void printXY(float th1_deg, float th2_deg)
{
  Pose2D p = forward2R(th1_deg, th2_deg, L1_mm, L2_mm);
  Serial.print("X="); Serial.print(p.x_mm);
  Serial.print("  Y="); Serial.println(p.y_mm);
}

void goXY_keepParallel(float x, float y)
{
  float th1, th2;
  if (!inverse2R(x, y, L1_mm, L2_mm, true, th1, th2)) return;

  float phi_offset = 0.0f; // 기구 조립 기준에 따라 보정값 필요
  float wrist_deg = wristPhiParallelX(th1, th2, phi_offset);

  move_j1_wait(th1);
  move_j3_wait(th2);
  move_j4_wait(wrist_deg);   // 너 프로젝트에 j4가 있다면
}
// move.cpp에 추가

void moveXY_rel(float dx_mm, float dy_mm)
{
  // 1) 현재 조인트 각도 읽기
  float th1_cur = j1_getJointDeg();
  float th2_cur = j3_getJointDeg();

  // 2) FK로 현재 TCP 좌표 계산
  Pose2D cur = forward2R(th1_cur, th2_cur, L1_mm, L2_mm);

  // 3) 상대이동 목표 생성
  float x_tgt = cur.x_mm + dx_mm;
  float y_tgt = cur.y_mm + dy_mm;

  // 4) 현재 팔 자세(엘보 업/다운) 유지하도록 옵션 결정(권장)
  bool elbowUp = (th2_cur < 0.0f);  // inverse2R 구현상 elbowUp이면 th2가 음수로 나오는 편

  // 5) IK -> 이동 (goXY는 elbowUp을 true로 고정이라, 여기서는 직접 풀어주는 게 더 안정적)
  motors_enable_all(true);

  float th1_tgt, th2_tgt;
  if (!inverse2R(x_tgt, y_tgt, L1_mm, L2_mm, elbowUp, th1_tgt, th2_tgt)) {
    Serial.println("[IK] unreachable (rel)");
    motors_enable_all(false);
    return;
  }

  move_j1_wait(th1_tgt);
  move_j3_wait(th2_tgt);

  motors_enable_all(false);
}

void tool(bool on) // 엔드이팩터 교체
{
  if (on)
  {
    digitalWrite(num1, HIGH);
    digitalWrite(num2, HIGH);
    digitalWrite(num3, HIGH);
  }
  else 
  {
    digitalWrite(num1, LOW);
    digitalWrite(num2, LOW);
    digitalWrite(num3, LOW);
  }

}