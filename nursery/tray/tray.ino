#include <Arduino.h>
#include <avr/interrupt.h>

// =====================================================
// 핀 설정
// =====================================================

const uint8_t R1_DIR_PIN  = 2;
const uint8_t R1_STEP_PIN = 3;

const uint8_t R2_DIR_PIN  = 4;
const uint8_t R2_STEP_PIN = 5;

const uint8_t SENSOR_PIN = 10;

// 적외선 센서가 감지되면 LOW
const bool SENSOR_ACTIVE = LOW;

// =====================================================
// 모터 설정
// =====================================================

// STEP 상승 에지와 다음 상승 에지 사이의 시간
// 500us = 2000 step/s
const uint32_t STEP_PERIOD_US = 500UL;

const long FORWARD_STEPS  = 2500;
const long BACKWARD_STEPS = 2500;

const bool FORWARD_DIR  = HIGH;
const bool BACKWARD_DIR = LOW;

// =====================================================
// Timer1 설정
// =====================================================

// Timer1 인터럽트 발생 주기
const uint16_t TIMER_TICK_US = 50;

// STEP 핀은 HIGH/LOW 두 번 반전해야 한 주기가 완성됨
const uint16_t STEP_TOGGLE_INTERVAL =
    (STEP_PERIOD_US / 2UL) / TIMER_TICK_US;

// =====================================================
// 모터 구조체
// =====================================================

struct StepMotor {
  uint8_t dirPin;
  uint8_t stepPin;

  volatile uint16_t tickCount;
  volatile uint16_t toggleInterval;

  volatile bool stepState;
  volatile bool running;
  volatile bool done;

  volatile long currentSteps;
  volatile long targetSteps;
};

// 모터 1
StepMotor motor1 = {
  R1_DIR_PIN,
  R1_STEP_PIN,
  0,
  STEP_TOGGLE_INTERVAL,
  LOW,
  false,
  false,
  0,
  0
};

// 모터 2
StepMotor motor2 = {
  R2_DIR_PIN,
  R2_STEP_PIN,
  0,
  STEP_TOGGLE_INTERVAL,
  LOW,
  false,
  false,
  0,
  0
};

// =====================================================
// 상태머신
// =====================================================

enum SystemState {
  WAIT_SENSOR,
  FORWARD_RUN,
  HOLD_SENSOR,
  BACKWARD_RUN
};

SystemState state = WAIT_SENSOR;

// =====================================================
// 함수 선언
// =====================================================

void setupTimer1();

void startMotor(
  StepMotor &motor,
  bool direction,
  uint32_t stepPeriodUs,
  long steps
);

void processMotorISR(StepMotor &motor);

void stopMotor(StepMotor &motor);
void stopAllMotors();

bool isMotorDone(const StepMotor &motor);
bool areAllMotorsDone();

// =====================================================
// setup
// =====================================================

void setup() {
  pinMode(R1_DIR_PIN, OUTPUT);
  pinMode(R1_STEP_PIN, OUTPUT);

  pinMode(R2_DIR_PIN, OUTPUT);
  pinMode(R2_STEP_PIN, OUTPUT);

  pinMode(SENSOR_PIN, INPUT_PULLUP);

  digitalWrite(R1_STEP_PIN, LOW);
  digitalWrite(R2_STEP_PIN, LOW);

  Serial.begin(9600);

  setupTimer1();

  Serial.println("SYSTEM START");
  Serial.println("WAIT_SENSOR");
}

// =====================================================
// loop: 상태머신만 처리
// =====================================================

void loop() {
  const bool sensorDetected =
      digitalRead(SENSOR_PIN) == SENSOR_ACTIVE;

  switch (state) {
    // -------------------------------------------------
    // 센서 감지 대기
    // -------------------------------------------------
    case WAIT_SENSOR:

      if (sensorDetected) {
        startMotor(
          motor1,
          FORWARD_DIR,
          STEP_PERIOD_US,
          FORWARD_STEPS
        );

        startMotor(
          motor2,
          FORWARD_DIR,
          STEP_PERIOD_US,
          FORWARD_STEPS
        );

        state = FORWARD_RUN;
        Serial.println("FORWARD_RUN");
      }

      break;

    // -------------------------------------------------
    // 정방향 이동 중
    // -------------------------------------------------
    case FORWARD_RUN:

      if (areAllMotorsDone()) {
        stopAllMotors();

        // 정방향 이동이 끝났는데 센서가 계속 감지 중
        if (sensorDetected) {
          state = HOLD_SENSOR;
          Serial.println("HOLD_SENSOR");
        }

        // 센서가 이미 해제됐다면 바로 복귀
        else {
          startMotor(
            motor1,
            BACKWARD_DIR,
            STEP_PERIOD_US,
            BACKWARD_STEPS
          );

          startMotor(
            motor2,
            BACKWARD_DIR,
            STEP_PERIOD_US,
            BACKWARD_STEPS
          );

          state = BACKWARD_RUN;
          Serial.println("BACKWARD_RUN");
        }
      }

      break;

    // -------------------------------------------------
    // 센서가 해제될 때까지 정지
    // -------------------------------------------------
    case HOLD_SENSOR:

      if (!sensorDetected) {
        startMotor(
          motor1,
          BACKWARD_DIR,
          STEP_PERIOD_US,
          BACKWARD_STEPS
        );

        startMotor(
          motor2,
          BACKWARD_DIR,
          STEP_PERIOD_US,
          BACKWARD_STEPS
        );

        state = BACKWARD_RUN;
        Serial.println("BACKWARD_RUN");
      }

      break;

    // -------------------------------------------------
    // 역방향 이동 중
    // -------------------------------------------------
    case BACKWARD_RUN:

      if (areAllMotorsDone()) {
        stopAllMotors();

        state = WAIT_SENSOR;
        Serial.println("WAIT_SENSOR");
      }

      break;
  }
}

// =====================================================
// 모터 시작
// =====================================================

void startMotor(
  StepMotor &motor,
  bool direction,
  uint32_t stepPeriodUs,
  long steps
) {
  if (steps <= 0) {
    return;
  }

  digitalWrite(motor.dirPin, direction);

  // DIR 신호가 드라이버에 안정적으로 입력되도록 대기
  delayMicroseconds(5);

  uint32_t halfPeriodUs = stepPeriodUs / 2UL;

  uint16_t interval =
      halfPeriodUs / TIMER_TICK_US;

  if (interval < 1) {
    interval = 1;
  }

  noInterrupts();

  motor.tickCount      = 0;
  motor.toggleInterval = interval;

  motor.stepState   = LOW;
  motor.currentSteps = 0;
  motor.targetSteps  = steps;

  motor.done    = false;
  motor.running = true;

  interrupts();

  digitalWrite(motor.stepPin, LOW);
}

// =====================================================
// 모터 정지
// =====================================================

void stopMotor(StepMotor &motor) {
  noInterrupts();

  motor.running   = false;
  motor.stepState = LOW;
  motor.tickCount = 0;

  interrupts();

  digitalWrite(motor.stepPin, LOW);
}

void stopAllMotors() {
  stopMotor(motor1);
  stopMotor(motor2);
}

// =====================================================
// 완료 상태 확인
// =====================================================

bool isMotorDone(const StepMotor &motor) {
  bool result;

  noInterrupts();
  result = motor.done;
  interrupts();

  return result;
}

bool areAllMotorsDone() {
  bool result;

  noInterrupts();
  result = motor1.done && motor2.done;
  interrupts();

  return result;
}

// =====================================================
// Timer1 설정
// =====================================================

void setupTimer1() {
  noInterrupts();

  // Timer1 초기화
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  /*
   * Arduino UNO 클럭: 16MHz
   * Prescaler: 8
   *
   * Timer1 카운터 주파수:
   * 16MHz / 8 = 2MHz
   *
   * 카운터 1회 증가 시간:
   * 0.5us
   *
   * TIMER_TICK_US가 50us인 경우:
   * 50us / 0.5us = 100카운트
   *
   * CTC는 0부터 OCR1A까지 세므로:
   * OCR1A = 100 - 1 = 99
   */

  OCR1A = (TIMER_TICK_US * 2UL) - 1UL;

  // CTC 모드
  TCCR1B |= (1 << WGM12);

  // Prescaler 8
  TCCR1B |= (1 << CS11);

  // Compare Match A 인터럽트 활성화
  TIMSK1 |= (1 << OCIE1A);

  interrupts();
}

// =====================================================
// 개별 모터 ISR 처리
// =====================================================

void processMotorISR(StepMotor &motor) {
  if (!motor.running) {
    return;
  }

  motor.tickCount++;

  if (motor.tickCount < motor.toggleInterval) {
    return;
  }

  motor.tickCount = 0;

  // STEP 핀 반전
  motor.stepState = !motor.stepState;
  digitalWrite(motor.stepPin, motor.stepState);

  // 상승 에지를 실제 한 스텝으로 계산
  if (motor.stepState == HIGH) {
    motor.currentSteps++;

    if (motor.currentSteps >= motor.targetSteps) {
      motor.running   = false;
      motor.done      = true;
      motor.stepState = LOW;

      digitalWrite(motor.stepPin, LOW);
    }
  }
}

// =====================================================
// Timer1 인터럽트
// =====================================================

ISR(TIMER1_COMPA_vect) {
  processMotorISR(motor1);
  processMotorISR(motor2);
}