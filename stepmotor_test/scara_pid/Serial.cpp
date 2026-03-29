#include "Serial.h"

// =========================
// 내부 상태 변수
// =========================
static uint8_t ssf = 0;
static uint8_t smf = 0;
static uint8_t crf = 0;
static uint16_t uv = 0;

static uint8_t prev_ssf = 0;
static uint8_t prev_crf = 0;

// =========================
// 수신 상태 머신 변수
// =========================
enum RxState : uint8_t {
  RX_WAIT_SOF,
  RX_WAIT_ID,
  RX_WAIT_LEN,
  RX_WAIT_DATA,
  RX_WAIT_CHK
};

static RxState rxState = RX_WAIT_SOF;
static uint8_t rxId = 0;
static uint8_t rxLen = 0;
static uint8_t rxData[MAX_DATA_LEN];
static uint8_t rxIndex = 0;

// =========================
// 체크섬 계산
// CHK = ID + LEN + DATA 합의 하위 1바이트
// =========================
static uint8_t calcChecksum(uint8_t id, uint8_t len, const uint8_t* data) {
  uint16_t sum = id + len;

  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
  }

  return (uint8_t)(sum & 0xFF);
}

// =========================
// 내부 프레임 반영
// =========================
static void handleFrame(uint8_t id, const uint8_t* data, uint8_t len) {
  switch (id) {
    case PID_SSF:
      if (len == 1) {
        ssf = data[0];
      }
      break;

    case PID_SMF:
      if (len == 1) {
        smf = data[0];
      }
      break;

    case PID_CRF:
      if (len == 1) {
        crf = data[0];
      }
      break;

    case PID_UV:
      if (len == 2) {
        uv = ((uint16_t)data[0] << 8) | data[1];
      }
      break;

    default:
      // 추후 새로운 ID 추가 시 여기 확장
      break;
  }
}

// =========================
// 외부 공개 함수
// =========================
void serialProtocolBegin(unsigned long baud) {
  Serial.begin(baud);
}

void sendFrame(uint8_t id, const uint8_t* data, uint8_t len) {
  uint8_t chk = calcChecksum(id, len, data);

  Serial.write(SOF);
  Serial.write(id);
  Serial.write(len);

  for (uint8_t i = 0; i < len; i++) {
    Serial.write(data[i]);
  }

  Serial.write(chk);
}

void sendU8(uint8_t id, uint8_t value) {
  uint8_t buf[1] = { value };
  sendFrame(id, buf, 1);
}

void sendU16(uint8_t id, uint16_t value) {
  uint8_t buf[2];
  buf[0] = (uint8_t)((value >> 8) & 0xFF);
  buf[1] = (uint8_t)(value & 0xFF);
  sendFrame(id, buf, 2);
}

void serialReceiveTask(void) {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();

    switch (rxState) {
      case RX_WAIT_SOF:
        if (b == SOF) {
          rxState = RX_WAIT_ID;
        }
        break;

      case RX_WAIT_ID:
        rxId = b;
        rxState = RX_WAIT_LEN;
        break;

      case RX_WAIT_LEN:
        rxLen = b;

        if (rxLen > MAX_DATA_LEN) {
          rxState = RX_WAIT_SOF;
          rxIndex = 0;
        } else if (rxLen == 0) {
          rxState = RX_WAIT_CHK;
        } else {
          rxIndex = 0;
          rxState = RX_WAIT_DATA;
        }
        break;

      case RX_WAIT_DATA:
        rxData[rxIndex++] = b;

        if (rxIndex >= rxLen) {
          rxState = RX_WAIT_CHK;
        }
        break;

      case RX_WAIT_CHK: {
        uint8_t recvChk = b;
        uint8_t calcChk = calcChecksum(rxId, rxLen, rxData);

        if (recvChk == calcChk) {
          handleFrame(rxId, rxData, rxLen);
        }

        rxState = RX_WAIT_SOF;
        rxIndex = 0;
        break;
      }
    }
  }
}

// =========================
// getter
// =========================
uint8_t getSsf(void) {
  return ssf;
}

uint8_t getSmf(void) {
  return smf;
}

uint8_t getCrf(void) {
  return crf;
}

uint16_t getUv(void) {
  return uv;
}

// =========================
// setter
// =========================
void setSsf(uint8_t value) {
  ssf = value;
}

void setSmf(uint8_t value) {
  smf = value;
}

void setCrf(uint8_t value) {
  crf = value;
}

void setUv(uint16_t value) {
  uv = value;
}

// =========================
// 이전 값 관련
// =========================
uint8_t getPrevSsf(void) {
  return prev_ssf;
}

uint8_t getPrevCrf(void) {
  return prev_crf;
}

void updatePrevFlags(void) {
  prev_ssf = ssf;
  prev_crf = crf;
}

// =========================
// 편의 송신 함수
// =========================
void sendSsf(void) {
  sendU8(PID_SSF, ssf);
}

void sendSmf(void) {
  sendU8(PID_SMF, smf);
}

void sendCrf(void) {
  sendU8(PID_CRF, crf);
}

void sendUv(void) {
  sendU16(PID_UV, uv);
}