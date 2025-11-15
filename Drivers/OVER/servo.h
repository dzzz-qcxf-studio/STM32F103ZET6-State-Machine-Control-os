#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f1xx_hal.h"

// 初始化PWM
void Servo_PWM_Init(void);
// 设置舵机角度
void Servo1_SetAngle(uint8_t angle);
void Servo2_SetAngle(uint8_t angle);
// 具体功能界面
void Servo_control(void);
#endif    
