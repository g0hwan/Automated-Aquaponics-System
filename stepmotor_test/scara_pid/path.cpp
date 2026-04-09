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
  PATH_SECT2,
  PATH_SECT3
};

enum Sect3Step : uint8_t {
  SECT3_STEP_IDLE,
  SECT3_STEP_INIT,
  SECT3_STEP_START_HARVEST,
  SECT3_STEP_HARVEST_RUNNING,
  SECT3_STEP_DONE
};
enum Sect3Phase : uint8_t {
  SECT3_PHASE_LOCKED,       // 선점 불가
  SECT3_PHASE_PREEMPTIBLE   // 선점 가능
};
enum HarvestStep : uint8_t {
  HARVEST_STEP_IDLE,
  HARVEST_STEP_MOVE_TO_CELL,
  HARVEST_STEP_CUT,
  HARVEST_STEP_ADVANCE_INDEX,
  HARVEST_STEP_DONE
};
static uint8_t harvest_row = 0;
static uint8_t harvest_col = 0;
static HarvestStep harvest_step = HARVEST_STEP_IDLE;
static PathState currentPath = PATH_IDLE;
static PathState suspendedPath = PATH_IDLE;

static bool sect1_started = false;
static bool sect2_started = false;
static bool sect3_started = false;

static bool step_command_issued = false;
static bool sect3_preemptible = false;

static bool pending_sect1 = false;
static bool pending_sect2 = false;

static Sect3Step sect3_step = SECT3_STEP_IDLE;

int base_x = 80;
int base_y = 0;
// =========================
// 실제 하드웨어 제어 함수 자리
// 지금은 뼈대만
// =========================

static void startScaraMotion(void) {
  home();
}
/*
static void pick_from_hydro(void) {
  moveRail_untilStop(false, 3000, stop2_rail);
  goXY(-80,0);
  move_j2_cm(-5);
  move_j2_cm(5);
  home();
}
static void left_tray(void){
  moveRail_untilStop(true, 3000, stop_rail);
  goXY(80,0);
}
static void move_to_conveyor(void){
  move_j2_cm(-3);
  //그리퍼 off 함수
}
static void place_on_convey(void){
  //그리퍼 on 함수
  move_j2_cm(3);
}
static void start_harvest(void){
  setFf(1); //sbc에 전송
  sendFf();
}
*/
static void sect3_init(void){
  home();
  moveRail_untilStop(false, 3000, stop2_rail);
  goXY(-80,0);
  move_j2_cm(-5);
  move_j2_cm(5);
  home();
  moveRail_untilStop(true, 3000, stop_rail);
  goXY(80,0);
  move_j2_cm(-3);
  //그리퍼 off 함수
  //그리퍼 on 함수
  move_j2_cm(3);
}
static void sect3_harvest(void){
  move_j2_cm(-3);//내려가서
  //그리퍼 동작 열기 (추가 예정)
  //그리퍼 동작 오므리기 (추가 예정)
  move_j2_cm(3); //올라가기
  //goXY(0,80); //(수확 자리로 이동)
  //그리퍼 열기 (추가 예정)

}
static void harvest_running(void)
{
  if (pending_sect1) {
    pending_sect1 = false;
    suspendedPath = PATH_SECT3;
    currentPath = PATH_SECT1;
    return;
  }

  if (pending_sect2) {
    pending_sect2 = false;
    suspendedPath = PATH_SECT3;
    currentPath = PATH_SECT2;
    return;
  }

  switch (harvest_step) {
    case HARVEST_STEP_IDLE:
      harvest_row = 0;
      harvest_col = 0;
      harvest_step = HARVEST_STEP_MOVE_TO_CELL;
      break;

    case HARVEST_STEP_MOVE_TO_CELL: {
      float x = base_x - (3.0f * harvest_col);
      float y = base_y + (3.0f * harvest_row);

      goXY(x, y); //첫 기준 좌표로 이동
      harvest_step = HARVEST_STEP_CUT;
      break;
    }

    case HARVEST_STEP_CUT:
      sect3_harvest();// 수확 1회 동작
      // cutter_on/off or ff 체크 등
      harvest_step = HARVEST_STEP_ADVANCE_INDEX;
      break;

    case HARVEST_STEP_ADVANCE_INDEX:
      harvest_row++;

      if (harvest_row >= 5) {
        harvest_row = 0;
        harvest_col++;
      }

      if (harvest_col >= 3) {
        harvest_step = HARVEST_STEP_DONE;
      } else {
        harvest_step = HARVEST_STEP_MOVE_TO_CELL;
      }
      break;

    case HARVEST_STEP_DONE:
      harvest_step = HARVEST_STEP_IDLE;
      // sect3 다음 단계 or 종료
      break;
  }
}

// =========================
// 섹션 함수
// =========================
void sect1(void) {
  if (!sect1_started) {
    sect1_started = true;

    // 스카라 동작 시작
    startScaraMotion();// 홈 위치 이동
    // 현재 스카라 움직이는 중
    setSmf(1);
    sendSmf();
  }
  //home(); // 홈 위치
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
  delay(1000); // 트레이 픽업

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
  
  setCrf(0); // 직교로봇 리셋
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
    setUv(uv);
    sendUv();
  }
  else if (uv == 1)
  {
    moveRail(3000,1);   
    delay(1000);
    stopRail();
    uv++; 
    setUv(uv);
    sendUv();
  }
  move_j1_wait(40);
  moveRail_untilStop(true, 3000, stop_rail);
  moveRail(3000,0);   
  delay(900);
  stopRail();
  //move_j2_cm(-1.2);
  j3_home_stop_on_switch(false, 5000);
  j1_home_stop_on_switch(false, 4200);
  home();
  setSmf(0); // 스카라 구동 끝
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

void sect3(void) {
  switch (sect3_step) {
    case SECT3_STEP_IDLE:
      sect3_started = true;
      step_command_issued = false;
      sect3_step = SECT3_STEP_INIT;
      break;

    case SECT3_STEP_INIT:
      if (!step_command_issued) {
        sect3_init();
        step_command_issued = true;
      }

      // 예: if (is_j1_done())
      if (true) {
        step_command_issued = false;
        sect3_step = SECT3_STEP_START_HARVEST;
      }
      break;

    case SECT3_STEP_START_HARVEST:
      if (!step_command_issued) {
        setFf(1); //sbc에 전송
        sendFf();
        step_command_issued = true;
      }

      // 예: if (is_j2_done())
      if (true) {
        step_command_issued = false;
        sect3_step = SECT3_STEP_HARVEST_RUNNING;
      }
      break;

    case SECT3_STEP_HARVEST_RUNNING:
      if (!step_command_issued){
       harvest_running();

      if (harvest_step == HARVEST_STEP_DONE) {
        sect3_step = SECT3_STEP_DONE;
      } 
      break;
      }

    case SECT3_STEP_DONE:
      sect3_started = false;
      sect3_step = SECT3_STEP_IDLE;
      currentPath = PATH_IDLE;
      break;
  }
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