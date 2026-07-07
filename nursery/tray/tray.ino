const int r1dirPin  = 2;
const int r1stepPin = 3;

const int r2dirPin  = 4;
const int r2stepPin = 5;

const int sensPin = 10;

// 센서가 인식될 때 0이므로 LOW
const bool SENSOR_ACTIVE = LOW;

// ================= 모터 설정 =================
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
  FORWARD_RUN,
  HOLD_SENSOR,
  BACKWARD_RUN
};

SystemState state = WAIT_SENSOR;

void setup() {
  pinMode(r1dirPin, OUTPUT);
  pinMode(r1stepPin, OUTPUT);

  pinMode(r2dirPin, OUTPUT);
  pinMode(r2stepPin, OUTPUT);

  // 센서가 오픈 컬렉터 타입이면 INPUT_PULLUP이 더 안정적
  pinMode(sensPin, INPUT_PULLUP);

  Serial.begin(9600);

  digitalWrite(r1stepPin, LOW);
  digitalWrite(r2stepPin, LOW);

  setupTimer1();

  Serial.println("SYSTEM START");
}

void loop() {
  bool sens = digitalRead(sensPin);

  switch (state) {

    case WAIT_SENSOR:
      if (sens == SENSOR_ACTIVE) {
        r1_move(FORWARD_DIR, MOTOR_SPEED_US, FORWARD_STEPS);
        r2_move(FORWARD_DIR, MOTOR_SPEED_US, FORWARD_STEPS);

        state = FORWARD_RUN;
        Serial.println("FORWARD_RUN");
      }
      break;

    case FORWARD_RUN:
      if (isAllMotorDone()) {
        stopAllMotor();

        if (sens == SENSOR_ACTIVE) {
          state = HOLD_SENSOR;
          Serial.println("HOLD_SENSOR");
        } else {
          r1_move(BACKWARD_DIR, MOTOR_SPEED_US, BACKWARD_STEPS);
          r2_move(BACKWARD_DIR, MOTOR_SPEED_US, BACKWARD_STEPS);

          state = BACKWARD_RUN;
          Serial.println("BACKWARD_RUN");
        }
      }
      break;

    case HOLD_SENSOR:
      if (sens != SENSOR_ACTIVE) {
        r1_move(BACKWARD_DIR, MOTOR_SPEED_US, BACKWARD_STEPS);
        r2_move(BACKWARD_DIR, MOTOR_SPEED_US, BACKWARD_STEPS);

        state = BACKWARD_RUN;
        Serial.println("BACKWARD_RUN");
      }
      break;

    case BACKWARD_RUN:
      if (isAllMotorDone()) {
        stopAllMotor();

        state = WAIT_SENSOR;
        Serial.println("WAIT_SENSOR");
      }
      break;
  }
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

// ================= 전체 모터 정지 =================
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
  // Prescaler 8 → 2MHz
  // 1 tick = 0.5us
  // 50us = 100 ticks
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