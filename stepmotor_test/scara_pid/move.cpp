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

  j3_home_stop_on_switch(true, 5000);
  delay(500);
  enc_reset_j3();
  delay(500);
  //move_j3_wait(-25.0f);
  move_j3_wait(175.0f);
  delay(500);
  Serial.println("j3 end");

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

  // 현재 조인트각 읽기
  float th1_cur = j1_getJointDeg();
  float th2_cur = j3_getJointDeg();

  float th1, th2;
  bool ok = inverse2R_best(x, y, th1_cur, th2_cur, th1, th2);
  if (!ok) {
    Serial.println("[IK] unreachable");
    motors_enable_all(false);
    return;
  }

  move_j1_wait(th1);
  move_j3_wait(th2);

  motors_enable_all(false);
}


void printXY(float th1_deg, float th2_deg)
{
  Pose2D p = forward2R(th1_deg, th2_deg, L1_mm, L2_mm);
  Serial.print("X="); Serial.print(p.x_mm);
  Serial.print("  Y="); Serial.println(p.y_mm);
}

void moveXY_rel(float dx_mm, float dy_mm)
{
  float th1_cur = j1_getJointDeg();
  float th2_cur = j3_getJointDeg();

  Pose2D cur = forward2R(th1_cur, th2_cur, L1_mm, L2_mm);

  float x_tgt = cur.x_mm + dx_mm;
  float y_tgt = cur.y_mm + dy_mm;

  motors_enable_all(true);

  float th1_tgt, th2_tgt;
  if (!inverse2R_best(x_tgt, y_tgt, th1_cur, th2_cur, th1_tgt, th2_tgt)) {
    Serial.println("[IK] unreachable (rel)");
    motors_enable_all(false);
    return;
  }

  move_j1_wait(th1_tgt);
  move_j3_wait(th2_tgt);

  motors_enable_all(false);
}

void goXY_keepParallel(float x, float y)
{
  float th1_cur = j1_getJointDeg();
  float th2_cur = j3_getJointDeg();

  float th1, th2;
  if (!inverse2R_best(x, y, th1_cur, th2_cur, th1, th2)) 
  {
    motors_enable_all(false);
    return;
  }
  float phi_offset = 0.0f;
  float wrist_deg = wristPhiParallelX(th1, th2, phi_offset);

  move_j1_wait(th1);
  move_j3_wait(th2);
  move_j4_wait(wrist_deg);
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

//  상대/절대 관절각 이동
// ---------------------------------------------
void moveJ_abs(float th1_deg, float th3_deg)
{
  move_j1_wait(th1_deg);
  move_j3_wait(th3_deg);
}

void moveJ_rel(float dth1_deg, float dth3_deg)
{
  float th1_cur = j1_getJointDeg();
  float th3_cur = j3_getJointDeg();

  moveJ_abs(th1_cur + dth1_deg, th3_cur + dth3_deg);
}

// j4 포함 버전 (원하면 사용)
void moveJ_abs4(float th1_deg, float th3_deg, float th4_deg)
{
  move_j1_wait(th1_deg);
  move_j3_wait(th3_deg);
  move_j4_wait(th4_deg);
}

void moveJ_rel4(float dth1_deg, float dth3_deg, float dth4_deg)
{
  float th1_cur = j1_getJointDeg();
  float th3_cur = j3_getJointDeg();
  float th4_cur = j4_getJointDeg();

  moveJ_abs4(th1_cur + dth1_deg, th3_cur + dth3_deg, dth4_deg + th4_cur);
}
