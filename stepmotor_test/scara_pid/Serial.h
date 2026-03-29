#ifndef MY_SERIAL_PROTOCOL_H
#define MY_SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <stdint.h>

extern uint8_t ssf ;
extern uint8_t smf ;
extern uint8_t crf ;
extern uint16_t uv ;


static const uint8_t SOF = 0xAA;
static const uint8_t MAX_DATA_LEN = 16;

// =========================
// ID 정의
// =========================
enum ParamId : uint8_t {
  PID_SSF = 0x01,
  PID_SMF = 0x02,
  PID_CRF = 0x03,
  PID_UV  = 0x04,
};

void setflag();

// =========================
// 초기화 / 주기 실행
// =========================
void serialProtocolBegin(unsigned long baud);
void serialReceiveTask(void);

// =========================
// 송신 함수
// =========================
void sendFrame(uint8_t id, const uint8_t* data, uint8_t len);
void sendU8(uint8_t id, uint8_t value);
void sendU16(uint8_t id, uint16_t value);

// =========================
// 현재 값 getter / setter
// =========================
uint8_t getSsf(void); //ssf 읽기
uint8_t getSmf(void); // smf 읽기
uint8_t getCrf(void); //crf 읽기
uint16_t getUv(void); //uv읽기

void setSsf(uint8_t value); //ssf 설정
void setSmf(uint8_t value); //smf 설정
void setCrf(uint8_t value); //crf설정
void setUv(uint16_t value); //uv설정

// =========================
// 이전 값 관련
// path.cpp에서 에지 검출용으로 사용
// =========================
uint8_t getPrevSsf(void);
uint8_t getPrevCrf(void);
void updatePrevFlags(void);

// =========================
// 편의 송신 함수
// =========================
void sendSsf(void);  // ssf 보내기
void sendSmf(void);  // smf 보내기
void sendCrf(void);  // crf 보내기
void sendUv(void);   // uv보내기

#endif