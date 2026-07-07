#include "emergency.h"

const byte stop_pin = 3;
const byte start_pin = 2;
const byte sw_pin = 9;
const byte led_pin = 10;

volatile bool emergencyLatched = false;
volatile bool startRequest = false;

void emergency_pin()
{
  pinMode(stop_pin, INPUT_PULLUP);
  pinMode(start_pin, INPUT_PULLUP);
  pinMode(sw_pin, INPUT_PULLUP);
  pinMode(led_pin, OUTPUT);

  digitalWrite(led_pin, LOW);

  attachInterrupt(digitalPinToInterrupt(stop_pin), stopISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(start_pin), startISR, FALLING);
}

void stopISR()
{
  emergencyLatched = true;
}

void startISR()
{
  startRequest = true;
}

bool SW()
{
  return digitalRead(sw_pin) == LOW;
}