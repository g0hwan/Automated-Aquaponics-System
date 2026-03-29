#include "encoder.h"
#include "kinematic.h"
#include "move.h"
#include "pid.h"
#include "set_motor.h"
#include "path.h"
#include "Serial.h"

// =========================
// path 내부 상태
// =========================
enum PathState : uint8_t {
  PATH_IDLE,
  PATH_SECT1,
  PATH_SECT2
};

static PathState currentPath = PATH_IDLE;

// 실제 섹션 진행 상태
static bool sect1_started = false;
static bool sect2_started = false;

// =========================
// 실제 하드웨어 제어 함수 자리
// 지금은 뼈대만
// =========================
static void startScaraMotion(void) {
  home();
  delay(100);
}

// =========================
// 섹션 함수
// =========================
void sect1(void) {
  if (!sect1_started) {
    sect1_started = true;

    // 스카라 동작 시작
    startScaraMotion();
    // 현재 스카라 움직이는 중
    setSmf(1);
    sendSmf();
  }
    home();
  delay(1000);
  moveRail(3000,0);   
  delay(900);
  stopRail();
  move_j2_cm(-1.2);
  j1_home_stop_on_switch(true, 4200);
  delay(50);
  moveRail_untilStop(true, 4000, stop_rail);
  moveRail(2000,1);
  delay(2000);
  stopRail();
  enc_reset_j3();
  move_j3_wait(220);
  move_j1_wait(30);
  move_j2_cm(-9);
  delay(1000);

  move_j2_cm(9);

  move_j1_wait(-40);
  moveRail_untilStop(false, 4000, stop_rail);
  moveRail(3000,0);
  delay(700);
  stopRail();
  delay(100);
  moveRail_untilStop(true, 3000, stop_rail);
  delay(50);
  moveRail(3000,0);   
  delay(1800);
  stopRail();
  
  setCrf(1);
  sendCrf();
  
  move_j2_cm(4.5);
  move_j1_wait(10);
  if (uv == 0)
  {
  moveRail(3000,0);   
  delay(2000);
  stopRail();
  move_j1_wait(30);
  enc_reset_j3();
  move_j3_wait(30);
  uv++;
  }
  else if (uv == 1)
  {
    moveRail(3000,1);   
    delay(1000);
    stopRail();
    uv++; 
  }

  setSmf(0);
  sendSmf();
  sect1_started = false;
  currentPath = PATH_IDLE;
}

void sect2(void) {
  if (!sect2_started) {
    sect2_started = true;

    // 직교로봇 초기화 시작
    //startCartesianReset();
  }

  sect2_started = false;
  currentPath = PATH_IDLE;
}

// =========================
// 전체 동작 판단
// =========================
void pathTask(void) {
  uint8_t ssf = getSsf();
  uint8_t crf = getCrf();
  uint8_t prev_ssf = getPrevSsf();
  uint8_t prev_crf = getPrevCrf();

  // -------------------------
  // ssf 상승에지: 0 -> 1
  // 스카라 시작 섹션 진입
  // -------------------------
  if (prev_ssf == 0 && ssf == 1) {
    if (currentPath == PATH_IDLE) {
      currentPath = PATH_SECT1;
    }
  }

  // -------------------------
  // ssf 하강에지: 1 -> 0
  // 필요 시 강제 정지
  // -------------------------
  if (prev_ssf == 1 && ssf == 0) {
    //stopScaraMotion();
    setSmf(0);
    sendSmf();

    sect1_started = false;

    if (currentPath == PATH_SECT1) {
      currentPath = PATH_IDLE;
    }
  }

  // -------------------------
  // crf 상승에지: 0 -> 1
  // 직교로봇 초기화 섹션 진입
  // -------------------------
  if (prev_crf == 0 && crf == 1) {
    if (currentPath == PATH_IDLE) {
      currentPath = PATH_SECT2;
    }
  }

  // -------------------------
  // crf 하강에지: 1 -> 0
  // 필요 시 초기화 정지
  // -------------------------
  if (prev_crf == 1 && crf == 0) {
    //stopCartesianReset();

    sect2_started = false;

    if (currentPath == PATH_SECT2) {
      currentPath = PATH_IDLE;
    }
  }

  // -------------------------
  // 현재 섹션 수행
  // -------------------------
  switch (currentPath) {
    case PATH_IDLE:
      break;

    case PATH_SECT1:
      sect1();
      break;

    case PATH_SECT2:
      sect2();
      break;

    default:
      currentPath = PATH_IDLE;
      break;
  }

  // 마지막에 이전값 갱신
  updatePrevFlags();
}