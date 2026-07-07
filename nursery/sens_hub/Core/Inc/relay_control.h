#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H

#include "main.h"
#include <stdint.h>

void Relay_Init(void);
void Relay_On(void);
void Relay_Off(void);
void Relay_Control(float ph, uint32_t tds);

#endif
