#ifndef __MPU_PID_H__
#define __MPU_PID_H__

#include "./SYSTEM/sys/sys.h"
#include "./BSP/LCD/lcd.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* PID结构体 - 为飞棍项目优化 */
typedef struct {
    float kp;              // 比例系数
    float ki;              // 积分系数
    float kd;              // 微分系数
    float setpoint;        // 目标值
    float integral;        // 误差积分
    float prev_input;      // 上一次的输入值 (用于微分计算)
    float output_min;      // 输出最小值
    float output_max;      // 输出最大值
} PID_TypeDef;

/* 接口函数 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float setpoint, float min, float max);
float PID_Compute(PID_TypeDef *pid, float input, float dt);
void L9110_Init(void);  // 使用双定时器初始化电机控制 (TIM3和TIM2)
void L9110_SetMotor(float speed);
void Display_Init(void);
void Draw_Grid(void);
void Draw_Title_And_Help(void);
void Update_PID_Graph(float current, float target, float output);
void Update_Info_Display(float current, float target, float output);
void Handle_Touch_Events(void);
void Handle_Keyboard_Input(char key);
void Set_Default_Parameters(void);
void MPU_PID_Task(void);
void MPU_PID_Control_Interface(void);
#endif /* __MPU_PID_H__ */ 
