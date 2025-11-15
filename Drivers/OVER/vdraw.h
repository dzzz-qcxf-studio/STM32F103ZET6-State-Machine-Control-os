#ifndef __VDRAW_H
#define __VDRAW_H

#include "stm32f1xx_hal.h"

/* 外部依赖声明 */
extern TIM_HandleTypeDef htim2;

/**
  * @brief  启动PWM分析仪功能
  */
void PWM_Analyzer_Run(void);

#endif /* __VDRAW_H */
