#include <Arduino.h>
#include <avr/interrupt.h>

// ================= 핀 설정 =================
const uint8_t r1dirPin  = 2;
const uint8_t r1stepPin = 3;
const uint8_t r2dirPin  = 4;
const uint8_t r2stepPin = 5;

// TB6600 ENA 제어 핀
const uint8_t r1enPin = 6;
const uint8_t r2enPin = 7;

const uint8_t sensPin = 10;

// 적외선 센서가 감지되면 LOW
const bool SENSOR_ACTIVE = LOW;

// ================= TB6600 ENA 논리 설정 =================
/*
  true: 공통 애노드 방식
        PUL+, DIR+, ENA+ -> Arduino 5V
        PUL-, DIR-, ENA- -> Arduino 출력 핀

  false: 공통 캐소드 방식
         PUL-, DIR-, ENA- -> GND
         PUL+, DIR+, ENA+ -> Arduino 출력 핀
*/
const bool TB6600_COMMON_ANODE = true;

// ENA는 '오프라인/출력 차단' 입력이다.
// 공통 애노드: LOW가 ENA 유효 -> 모터 토크 해제
// 공통 캐소드: HIGH가 ENA 유효 -> 모터 토크 해제
const uint8_t DRIVER_DISABLE_LEVEL =
    TB6600_COMMON_ANODE ? LOW : HIGH;
const uint8_t DRIVER_ENABLE_LEVEL =
    TB6600_COMMON_ANODE ? HIGH : LOW;

// ENA를 활성화한 뒤 펄스를 시작하기 전 대기 시간
const unsigned long DRIVER_ENABLE_DELAY_US = 1000UL;

// 정방향 이동 완료 후 센서가 계속 감지되는 동안 토크 유지 여부
// true  : HOLD_SENSOR에서 위치 고정
// false : HOLD_SENSOR에서도 토크 해제하여 발열 감소
const bool HOLD_TORQUE_WHILE_SENSOR_ACTIVE = true;

// ================= 모터 설정 =================
// 기존 코드와 동일한 속도 해석 유지
// STEP 핀이 500us마다 반전되므로 실제 상승 에지는 약 1000us마다 발생
const unsigned int MOTOR_SPEED_US = 500;

const long FORWARD_STEPS  = 2500;
const long BACKWARD_STEPS = 2500;

const bool FORWARD_DIR  = HIGH;
const bool BACKWARD_DIR = LOW;

// ================= 타이머 설정 =================
#define TIMER_TICK_US 50

volatile unsigned int r1_count = 0;
volatile unsigned int r2_count = 0;

volatile unsigned int r1_interval = 10;
volatile unsigned int r2_interval = 10;

volatile bool r1_step_state = LOW;
volatile bool r2_step_state = LOW;

volatile bool r1_run = false;
volatile bool r2_run = false;

volatile long r1_step_count = 0;
volatile long r2_step_count = 0;

volatile long r1_target_steps = 0;
volatile long r2_target_steps = 0;

volatile bool r1_done = false;
volatile bool r2_done = false;

// ================= 상태머신 =================
enum SystemState {
  WAIT_SENSOR,
  PREPARE_FORWARD,
  FORWARD_RUN,
  HOLD_SENSOR,
  PREPARE_BACKWARD,
  BACKWARD_RUN
};

SystemState state = WAIT_SENSOR;
unsigned long driverEnableStartedUs = 0;

// ================= 함수 선언 =================
void setupTimer1();

void r1_move(bool dir, unsigned int speedUs, long steps);
void r2_move(bool dir, unsigned int speedUs, long steps);

void stopAllMotor();
bool isAllMotorDone();

void enableAllDrivers();
void disableAllDrivers();
void beginDriverEnableWait();
bool isDriverEnableDelayFinished();

// ================= 초기 설정 =================
void setup() {
  pinMode(r1dirPin, OUTPUT);
  pinMode(r1stepPin, OUTPUT);
  pinMode(r2dirPin, OUTPUT);
  pinMode(r2stepPin, OUTPUT);

  pinMode(r1enPin, OUTPUT);
  pinMode(r2enPin, OUTPUT);

  pinMode(sensPin, INPUT_PULLUP);

  Serial.begin(9600);

  digitalWrite(r1stepPin, LOW);
  digitalWrite(r2stepPin, LOW);

  // 부팅 직후에는 반드시 모터 토크 해제
  disableAllDrivers();

  setupTimer1();

  Serial.println("SYSTEM START");
  Serial.println("WAIT_SENSOR / DRIVER DISABLED");
}

// ================= 메인 상태머신 =================
void loop() {
  const bool sensorDetected =
      (digitalRead(sensPin) == SENSOR_ACTIVE);

  switch (state) {

    // 센서가 감지되지 않은 대기 상태
    // TB6600 출력 차단 -> 모터 토크 없음
    case WAIT_SENSOR:
      if (sensorDetected) {
        enableAllDrivers();
        beginDriverEnableWait();

        state = PREPARE_FORWARD;
        Serial.println("DRIVER ENABLE -> PREPARE_FORWARD");
      }
      break;

    // ENA 해제 후 드라이버가 준비될 시간을 비동기로 대기
    case PREPARE_FORWARD:
      if (isDriverEnableDelayFinished()) {
        r1_move(FORWARD_DIR, MOTOR_SPEED_US, FORWARD_STEPS);
        r2_move(FORWARD_DIR, MOTOR_SPEED_US, FORWARD_STEPS);

        state = FORWARD_RUN;
        Serial.println("FORWARD_RUN");
      }
      break;

    // 정방향 이동
    case FORWARD_RUN:
      if (isAllMotorDone()) {
        stopAllMotor();

        if (sensorDetected) {
          if (!HOLD_TORQUE_WHILE_SENSOR_ACTIVE) {
            disableAllDrivers();
          }

          state = HOLD_SENSOR;
          Serial.println("HOLD_SENSOR");
        } else {
          // 센서가 이미 해제된 경우 복귀 준비
          enableAllDrivers();
          beginDriverEnableWait();

          state = PREPARE_BACKWARD;
          Serial.println("PREPARE_BACKWARD");
        }
      }
      break;

    // 정방향 이동 완료 후 센서가 해제되기를 대기
    case HOLD_SENSOR:
      if (!sensorDetected) {
        // HOLD에서 토크를 꺼두었을 수도 있으므로 다시 활성화
        enableAllDrivers();
        beginDriverEnableWait();

        state = PREPARE_BACKWARD;
        Serial.println("DRIVER ENABLE -> PREPARE_BACKWARD");
      }
      break;

    // 역방향 이동 전 드라이버 준비 대기
    case PREPARE_BACKWARD:
      if (isDriverEnableDelayFinished()) {
        r1_move(BACKWARD_DIR, MOTOR_SPEED_US, BACKWARD_STEPS);
        r2_move(BACKWARD_DIR, MOTOR_SPEED_US, BACKWARD_STEPS);

        state = BACKWARD_RUN;
        Serial.println("BACKWARD_RUN");
      }
      break;

    // 역방향 이동
    case BACKWARD_RUN:
      if (isAllMotorDone()) {
        stopAllMotor();

        // 원위치 복귀가 끝나면 토크 해제
        disableAllDrivers();

        state = WAIT_SENSOR;
        Serial.println("WAIT_SENSOR / DRIVER DISABLED");
      }
      break;
  }
}

// ================= TB6600 ENA 제어 =================
void enableAllDrivers() {
  digitalWrite(r1enPin, DRIVER_ENABLE_LEVEL);
  digitalWrite(r2enPin, DRIVER_ENABLE_LEVEL);
}

void disableAllDrivers() {
  // 혹시 펄스 발생 중이라면 먼저 중지
  stopAllMotor();

  digitalWrite(r1enPin, DRIVER_DISABLE_LEVEL);
  digitalWrite(r2enPin, DRIVER_DISABLE_LEVEL);
}

void beginDriverEnableWait() {
  driverEnableStartedUs = micros();
}

bool isDriverEnableDelayFinished() {
  return (unsigned long)(micros() - driverEnableStartedUs)
         >= DRIVER_ENABLE_DELAY_US;
}

// ================= r1 구동 함수 =================
void r1_move(bool dir, unsigned int speedUs, long steps) {
  digitalWrite(r1dirPin, dir);

  noInterrupts();

  r1_interval = speedUs / TIMER_TICK_US;
  if (r1_interval < 1) {
    r1_interval = 1;
  }

  r1_count = 0;
  r1_step_state = LOW;
  r1_step_count = 0;
  r1_target_steps = steps;
  r1_done = false;
  r1_run = true;

  digitalWrite(r1stepPin, LOW);

  interrupts();
}

// ================= r2 구동 함수 =================
void r2_move(bool dir, unsigned int speedUs, long steps) {
  digitalWrite(r2dirPin, dir);

  noInterrupts();

  r2_interval = speedUs / TIMER_TICK_US;
  if (r2_interval < 1) {
    r2_interval = 1;
  }

  r2_count = 0;
  r2_step_state = LOW;
  r2_step_count = 0;
  r2_target_steps = steps;
  r2_done = false;
  r2_run = true;

  digitalWrite(r2stepPin, LOW);

  interrupts();
}

// ================= 전체 모터 펄스 정지 =================
void stopAllMotor() {
  noInterrupts();

  r1_run = false;
  r2_run = false;

  r1_step_state = LOW;
  r2_step_state = LOW;

  interrupts();

  digitalWrite(r1stepPin, LOW);
  digitalWrite(r2stepPin, LOW);
}

// ================= 전체 모터 완료 확인 =================
bool isAllMotorDone() {
  bool done;

  noInterrupts();
  done = r1_done && r2_done;
  interrupts();

  return done;
}

// ================= Timer1 설정 =================
void setupTimer1() {
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Arduino UNO 16MHz
  // Prescaler 8 -> 2MHz
  // 타이머 1카운트 = 0.5us
  // 50us = 100카운트 -> OCR1A = 99
  OCR1A = 99;

  // CTC 모드
  TCCR1B |= (1 << WGM12);

  // Prescaler 8
  TCCR1B |= (1 << CS11);

  // Compare Match A 인터럽트 허용
  TIMSK1 |= (1 << OCIE1A);

  interrupts();
}

// ================= Timer1 ISR =================
ISR(TIMER1_COMPA_vect) {
  if (r1_run) {
    r1_count++;

    if (r1_count >= r1_interval) {
      r1_count = 0;

      r1_step_state = !r1_step_state;
      digitalWrite(r1stepPin, r1_step_state);

      // 상승 에지를 한 스텝으로 계산
      if (r1_step_state == HIGH) {
        r1_step_count++;

        if (r1_step_count >= r1_target_steps) {
          r1_run = false;
          r1_done = true;
          r1_step_state = LOW;
          digitalWrite(r1stepPin, LOW);
        }
      }
    }
  }

  if (r2_run) {
    r2_count++;

    if (r2_count >= r2_interval) {
      r2_count = 0;

      r2_step_state = !r2_step_state;
      digitalWrite(r2stepPin, r2_step_state);

      // 상승 에지를 한 스텝으로 계산
      if (r2_step_state == HIGH) {
        r2_step_count++;

        if (r2_step_count >= r2_target_steps) {
          r2_run = false;
          r2_done = true;
          r2_step_state = LOW;
          digitalWrite(r2stepPin, LOW);
        }
      }
    }
  }
}