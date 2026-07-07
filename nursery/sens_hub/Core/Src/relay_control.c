#include "relay_control.h"

/*
 * Single relay control
 * PC0 -> Relay IN
 * Relay ON condition:
 *   ph < 5.8 OR ph > 8.0 OR tds < 700 OR tds > 1200
 */
#define RELAY_PORT GPIOC
#define RELAY_PIN  GPIO_PIN_0

/* If your relay module is active-low, swap these two definitions. */
#define RELAY_ON   GPIO_PIN_RESET
#define RELAY_OFF  GPIO_PIN_SET

#define PH_MIN   5.8f
#define PH_MAX   8.0f
#define TDS_MIN  700U
#define TDS_MAX  1200U

void Relay_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = RELAY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RELAY_PORT, &GPIO_InitStruct);

    Relay_Off();
}

void Relay_On(void)
{
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, RELAY_ON);
}

void Relay_Off(void)
{
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, RELAY_OFF);
}

void Relay_Control(float ph, uint32_t tds)
{
    if ((ph < PH_MIN) || (ph > PH_MAX) || (tds < TDS_MIN) || (tds > TDS_MAX))
    {
        Relay_On();
    }
    else
    {
        Relay_Off();
    }
}
