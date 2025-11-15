#ifndef __HAND_CONTROL_H
#define __HAND_CONTROL_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

// 包含menu.h以使用其中定义的枚举和变量
#include "./OVER/menu.h"
// 包含LCD和TOUCH的头文件
#include "./BSP/LCD/lcd.h"
#include "./BSP/TOUCH/touch.h"

// 声明函数
void Hand_Control(void);
void MX_USART3_UART_Init(void);
void usart_send_string(int32_t x, int32_t y);

#endif   
