#ifndef KINEMATIC_H
#define KINEMATIC_H

// theta1, theta2: rad
x = L1*cos(theta1) + L2*cos(theta1+theta2);
y = L1*sin(theta1) + L2*sin(theta1+theta2);
phi_deg = -(th1_deg + th2_deg) + phi_offset;

bool inverse2R(float x, float y, float L1, float L2,
               bool elbowUp,
               float &th1_deg, float &th2_deg)
{
  float r2 = x*x + y*y;
  float c2 = (r2 - L1*L1 - L2*L2) / (2.0f*L1*L2);

  // 도달 가능 체크 + clamp
  if (c2 >  1.0f) c2 =  1.0f;
  if (c2 < -1.0f) c2 = -1.0f;

  float s2 = sqrtf(fmaxf(0.0f, 1.0f - c2*c2));
  if (elbowUp) s2 = -s2;                 // elbow-up/down 선택

  float th2 = atan2f(s2, c2);
  float th1 = atan2f(y, x) - atan2f(L2*s2, L1 + L2*c2);

  th1_deg = th1 * 180.0f / PI;
  th2_deg = th2 * 180.0f / PI;
  return true;
}

#endif