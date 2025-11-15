#ifndef __RELAY_H
#define __RELAY_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

/* º¯ÊýÉùÃ÷ */
void MX_GPIO_Init(void);
void Relay_Control(FunctionalState state);
void Relay_Init(void);
void Relay_Control_Interface(void);

#endif /* __RELAY_H */

