#include "cartesian.h"
#include "first_convey.h"
#include "sensor.h"
#include "board_pin.h"
#include <string.h>
#include <math.h>

extern TIM_HandleTypeDef  htim2;   //  직교로봇 x축 PWM: TIM2 CH1
extern UART_HandleTypeDef huart2;  //  ALIGN_DONE(매니퓰레이터 자세보정 완료) 받는 UART

// ===================== 나중에 조정할 변수값들
static float g_x_steps_per_mm = 40.0f;   // 타이밍벨트라서 값 나중에 조정

static float g_x_ref_cm = 12.0f; // x축 위치보정 값, 이 거리까지 조정한다.

static uint32_t g_x_move_hz = 4000; // X축 이동 속도

static float g_deadband_mm = 1.0f; // 거리(x축)허용 오차 범위

#define X_DIR_INVERT   0   // 방향
#define X_EN_ACTIVE    0   // EN


// ===================== 내부 상태
typedef enum {
  ST_RUN_CONVEY = 0,   // 컨베이어 구동(IR 감지 전)
  ST_WAIT_ALIGN,       // ALIGN_DONE 기다림
  ST_MEASURE_MOVE_X,   // 거리 측정 후 X 이동 시작
  ST_WAIT_X_DONE,      // X 이동 끝날 때까지 대기
  ST_DONE              // 완료(여기서 다음 파종 단계는 너가 나중에 추가)
} State;

static State state = ST_RUN_CONVEY;

static bool ir_latched = false;      // IR 감지 순간 1번만 전환
static bool busy = false;            // X축 이동 중인지
static volatile uint32_t x_remain = 0;

// UART는 "WAIT_ALIGN 상태"에서만 ALIGN_DONE을 인정
static bool rx_enabled = false;
static bool uart_started = false;
static volatile bool align_done = false;

// ===================== TIM2 클럭 계산
// TIM2는 APB1 타이머
static uint32_t TIM2_GetClockHz(void)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) return pclk1 * 2;
  return pclk1;
}

// X STEP PWM 주파수 설정(TIM2 CH1)
static void X_StepPWM_SetHz(uint32_t hz)
{
  if (hz < 1) hz = 1;

  uint32_t tim_clk = TIM2_GetClockHz();
  uint32_t arr = (tim_clk / hz) - 1;

  __HAL_TIM_SET_PRESCALER(&htim2, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, arr / 2); // 50%
  __HAL_TIM_SET_COUNTER(&htim2, 0);
}

static void X_SetDir(bool dir) // X 방향설정
{
  if (X_DIR_INVERT) dir = !dir;
  HAL_GPIO_WritePin(CAR_X_DIR_PORT, CAR_X_DIR_PIN,
                    dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void X_Enable(bool en) // X EN설정(허용)
{
#if X_EN_ACTIVE
  HAL_GPIO_WritePin(CAR_X_EN_PORT, CAR_X_EN_PIN,
                    en ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(CAR_X_EN_PORT, CAR_X_EN_PIN,
                    en ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

static void X_Stop(void)
{
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);

  X_Enable(false);

  x_remain = 0;
  busy = false;
}

static void X_StartMoveSteps(int32_t steps, uint32_t step_hz)
{
  if (steps == 0) return;

  bool dir = (steps >= 0);
  uint32_t n = (uint32_t)(dir ? steps : -steps);

  X_SetDir(dir);
  X_Enable(true);

  x_remain = n;
  busy = true;

  X_StepPWM_SetHz(step_hz);

  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

// TIM2 Update 인터럽트마다 1스텝 감소(= PWM 1주기당 1펄스라고 가정)
void Cartesian_OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM2) return;
  if (!busy) return;

  if (x_remain > 0) {
    x_remain--;
    if (x_remain == 0) {
      X_Stop();
      state = ST_DONE;
    }
  }
}

// ===================== UART로 ALIGN_DONE 받기
static uint8_t rx;
static char line[32];
static uint8_t idx = 0;

static void AlignRx_StartOnce(void)
{
  if (uart_started) return;
  uart_started = true;
  HAL_UART_Receive_IT(&huart2, &rx, 1);
}

void Cartesian_OnUartRxCplt(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2) return;

  // WAIT_ALIGN 상태가 아니면 내용 무시(그래도 다음 수신은 계속 걸어줌)
  if (!rx_enabled || state != ST_WAIT_ALIGN) {
    HAL_UART_Receive_IT(&huart2, &rx, 1);
    return;
  }

  if (rx == '\n' || rx == '\r') {
    line[idx] = 0;
    idx = 0;

    if (strcmp(line, "ALIGN_DONE") == 0) {
      align_done = true;
      rx_enabled = false;  // 한 번 받으면 끝(원하면 이 줄 지워도 됨)
    }
  } else {
    if (idx < sizeof(line) - 1) line[idx++] = (char)rx;
  }

  HAL_UART_Receive_IT(&huart2, &rx, 1);
}

// ===================== 거리 측정 후 X 이동
static void MeasureAndMoveX(void)
{
  // 거리센서 읽기
  uint16_t raw = Sensor_DMS80_ReadRawAvg(16);
  float v  = (3.3f * raw) / 4095.0f;
  float cm = Sensor_DMS80_VoltageToCm(v);

  // 기준값 대비 오차(mm)
  float err_mm = (cm - g_x_ref_cm) * 10.0f;

  // 오차가 작으면 X축 이동 안 함
  if (fabsf(err_mm) <= g_deadband_mm) {
    state = ST_DONE;
    return;
  }

  // mm -> steps
  int32_t steps = (int32_t)(err_mm * g_x_steps_per_mm);

  // X축 이동 시작
  X_StartMoveSteps(steps, g_x_move_hz);
  state = ST_WAIT_X_DONE;
}

// ===================== main에서 계속 호출하는 Task
void Cartesian_Task(void)
{
  switch (state)
  {
    case ST_RUN_CONVEY:
      // 컨베이어는 IR 감지될 때까지 네 코드 그대로 사용
      FirstConvey_Task();

      // IR 감지 순간 1번만 latch
      if (Sensor_IR_Detected() && !ir_latched) {
        ir_latched = true;

        // 이제부터만 ALIGN_DONE 받겠다
        rx_enabled = true;
        AlignRx_StartOnce();

        state = ST_WAIT_ALIGN;
      }
      break;

    case ST_WAIT_ALIGN:
      // 자세보정 완료 신호(ALIGN_DONE) 받은 경우에만 다음 단계
      if (align_done) {
        align_done = false;
        state = ST_MEASURE_MOVE_X;
      }
      break;

    case ST_MEASURE_MOVE_X:
      MeasureAndMoveX();
      break;

    case ST_WAIT_X_DONE:
      // X 이동 완료는 타이머 콜백에서 ST_DONE으로 바뀜
      break;

    case ST_DONE:
      // 여기서부터 Z 파종 단계는 너가 나중에 추가하면 됨
      break;

    default:
      state = ST_RUN_CONVEY;
      break;
  }
}

// ===================== 설정 함수
void Cartesian_SetXStepsPerMM(float steps_per_mm)
{
  if (steps_per_mm > 0.1f) g_x_steps_per_mm = steps_per_mm;
}

void Cartesian_SetXRefCm(float ref_cm)
{
  if (ref_cm > 0.1f) g_x_ref_cm = ref_cm;
}

bool Cartesian_IsBusy(void)
{
  return busy;
}
