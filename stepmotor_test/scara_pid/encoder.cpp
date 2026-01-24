#include "encoder.h"
#include "set_motor.h"
#include "pid.h"

const float cpr    = 400.0f;
const float en_cnt = cpr * 2.0f;
volatile bool j1_run=false, j2_run=false, j3_run=false, j4_run=false;
static volatile bool j1_ps=false, j2_ps=false, j3_ps=false, j4_ps=false;

encod j1_enc = { j1_A, j1_B, 0 };
encod j3_enc = { j3_A, j3_B, 0 };
encod j4_enc = { j4_A, j4_B, 0 };

volatile bool j1pulseState = false;
volatile bool j3pulseState = false;
volatile bool j4pulseState = false;

volatile long j1pulseInterval = 100;
volatile long j3pulseInterval = 100;
volatile long j4pulseInterval = 100;

// z축 
const unsigned int PULSE_US = 15;   // 펄스폭은 넉넉히
const long PULSES_PER_REV = 3200;  // (가정) 1.8°모터 + 128분주
float J2_LEAD_MM_PER_REV = (8.0f/1.0f);
unsigned long J2_DEFAULT_PPS =300000;   // j2 기본 속도
volatile bool j2_endstop_hit = false;

float encoder_getAngleDeg(const encod* e) {
  return -(e->pos) * 360.0f / en_cnt;
}

void set_tim()
{
  Timer1.initialize(50);
  Timer1.attachInterrupt(j1stepPulse);
  Timer4.initialize(50);
  Timer4.attachInterrupt(j3stepPulse);
  Timer5.initialize(50);
  Timer5.attachInterrupt(j4stepPulse);
}

void set_int()
{
  attachInterrupt(digitalPinToInterrupt(j1_enc.pinA), j1EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j3_enc.pinA), j3EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j4_enc.pinA), j4EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(stop_z), j2EndstopISR, FALLING);
}

void j1stepPulse() {
  if(!j1_run){ digitalWrite(j1_pul, LOW); return; }
  digitalWrite(j1_pul, j1pulseState);
  j1pulseState = !j1pulseState;
}

void j4stepPulse() {
  if(!j4_run){ digitalWrite(j4_pul, LOW); return; }
  digitalWrite(j4_pul, j4pulseState);
  j4pulseState = !j4pulseState;
}


void j1EncoderA() {//엔코더 읽기
  bool a = digitalRead(j1_enc.pinA);
  bool b = digitalRead(j1_enc.pinB);
  j1_enc.pos += (a == b) ? 1 : -1;
}

void j4EncoderA() {
  bool a = digitalRead(j4_enc.pinA);
  bool b = digitalRead(j4_enc.pinB);
  j4_enc.pos += (a == b) ? 1 : -1;
}

void move_j1(float targetAngle)
{
  j1_run = true;              
  noInterrupts();
  long Count = j1_enc.pos;
  interrupts();

  encod snap = j1_enc;
  snap.pos = Count;
  float nowAngle = encoder_getAngleDeg(&snap);

  float error  = targetAngle - nowAngle;
  float pidOut = pid_update(&j1_pid, error);

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > 5000) speed = 5000;

  long interval = 1000000L / speed;

  noInterrupts();
  j1pulseInterval = interval;
  Timer1.setPeriod(j1pulseInterval);
  interrupts();

  digitalWrite(j1_dir, (pidOut > 0) ? LOW : HIGH);

  Serial.print("Angle="); Serial.print(nowAngle);
  Serial.print(" Error="); Serial.print(error);
  Serial.print(" speed="); Serial.print(speed);
  Serial.print(" interval(us)="); Serial.println(interval);
}


//관절 3에 대한 함수들////////////////////////////////////////////////////////////////////////
void j3stepPulse() {//펄스 생성 코드
  if(!j3_run) return;                
  digitalWrite(j3_pul, j3pulseState);
  j3pulseState = !j3pulseState;
}
void j3EncoderA() {
  bool a = digitalRead(j3_enc.pinA);
  bool b = digitalRead(j3_enc.pinB);
  j3_enc.pos += (a == b) ? 1 : -1;
}
bool move_j3(float targetAngle, float tolDeg = 1.0f)
{
  float Angle = 16 * targetAngle;

  j3_run = true;

  noInterrupts();
  long Count = j3_enc.pos;
  interrupts();

  encod snap = j3_enc;
  snap.pos = Count;
  float nowAngle = encoder_getAngleDeg(&snap);

  float error  = Angle - nowAngle;
  float pidOut = pid_update(&j3_pid, error);

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > 5000) speed = 5000;

  long interval = 1000000L / speed;

  noInterrupts();
  j3pulseInterval = interval;
  Timer4.setPeriod(j3pulseInterval);
  interrupts();

  digitalWrite(j3_dir, (pidOut > 0) ? LOW : HIGH);


  if (fabs(error) <= tolDeg) {
    j3_run = false;              
    digitalWrite(j3_pul, LOW);   
    return true;                 
  }
  return false;                 
}

static float j3_now_deg() // 현재 각도값 저장하는 함수
{
  noInterrupts();
  long c = j3_enc.pos;
  interrupts();

  encod snap = j3_enc;
  snap.pos = c;
  return encoder_getAngleDeg(&snap);
}

static float j3_error_deg(float targetAngle) //현재 오차값 저장하는 함수
{
  return (16.0f * targetAngle) - j3_now_deg();
} 

void move_j3_wait(float targetAngle,
                  float tolDeg = 1.0f,
                  unsigned long stable_ms = 150,
                  unsigned long timeout_ms = 8000) //연속동작 가능
{
  unsigned long t0 = millis();
  unsigned long inTolSince = 0;

  while (true) {
    move_j3(targetAngle);                 

    float e = j3_error_deg(targetAngle);
    if (fabs(e) <= tolDeg) {
      if (inTolSince == 0) inTolSince = millis();
      if (millis() - inTolSince >= stable_ms) break;
    } else {
      inTolSince = 0;
    }

    if (millis() - t0 > timeout_ms) break;
    delay(5);
  }

  // 정지(필요하면)
  j3_run = false;
  digitalWrite(j3_pul, LOW);
}
////////////////////////////////////////////////////////////////////////////////////////////


void move_j4(float targetAngle)
{
  j4_run = true;            
  noInterrupts();
  long Count = j4_enc.pos;
  interrupts();

  encod snap = j4_enc;
  snap.pos = Count;
  float nowAngle = encoder_getAngleDeg(&snap);

  float error  = targetAngle + nowAngle;
  float pidOut = pid_update(&j4_pid, error);

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > 5000) speed = 5000;

  long interval = 1000000L / speed;

  noInterrupts();
  j4pulseInterval = interval;
  Timer5.setPeriod(j4pulseInterval);
  interrupts();

  digitalWrite(j4_dir, (pidOut > 0) ? LOW : HIGH);

  Serial.print("Angle="); Serial.print(nowAngle);
  Serial.print(" Error="); Serial.print(error);
  Serial.print(" speed="); Serial.print(speed);
  Serial.print(" interval(us)="); Serial.println(interval);
}

void stepOnce(unsigned long pps){
  unsigned long period = 1000000UL / pps;
  if (period <= PULSE_US + 5) period = PULSE_US + 5;
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(PULSE_US);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(period - PULSE_US);
}


bool move_j2(float deg, unsigned long pps)
{
  j2_endstop_hit = false;

  bool toward_endstop = (deg >= 0); 
  long steps = (long)(fabs(deg) * (float)PULSES_PER_REV / 360.0f);

  digitalWrite(DIR_PIN, (deg >= 0) ? HIGH : LOW);

  for (long i = 0; i < steps; i++) {
    if (toward_endstop && (digitalRead(stop_z) == LOW)) {
      return false; // 엔드스탑 hit
    }
    stepOnce(pps);
  }
  return true;
}


void j2EndstopISR(){
  j2_endstop_hit = true;   // ISR은 플래그만!
}

// encoder.cpp (맨 아래쪽 아무데나 추가)

void enc_reset_all()
{
  noInterrupts();
  j1_enc.pos = 0;
  j3_enc.pos = 0;
  j4_enc.pos = 0;
  interrupts();
}

void enc_reset_j1()
{
  noInterrupts();
  j1_enc.pos = 0;
  interrupts();
}

void enc_reset_j3()
{
  noInterrupts();
  j3_enc.pos = 0;
  interrupts();
}

void enc_reset_j4()
{
  noInterrupts();
  j4_enc.pos = 0;
  interrupts();
}


