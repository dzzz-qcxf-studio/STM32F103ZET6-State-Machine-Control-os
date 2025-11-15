/*************************************************
 * vdraw.c - PWM波形测量与可视化系统
 * 功能：
 * 1. PWM波形频率与占空比测量
 * 2. 实时波形图形化显示
 * 硬件依赖：
 * - STM32F103ZE
 * - PA0: PWM输入捕获
 * - LCD
 ************************************************/

#include "stm32f1xx_hal.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/TOUCH/touch.h"
#include "./OVER/vdraw.h"
#include "./OVER/menu.h"
#include <stdio.h>
#include <math.h>

/* 全局变量 */
volatile uint32_t pwm_high_time = 0;
volatile uint32_t pwm_period = 0;
volatile uint8_t pwm_updated = 0;
volatile uint32_t last_rise_time = 0;

/* 图形绘制参数 - 扩大绘制区域 */
#define GRAPH_WIDTH  400     
#define GRAPH_HEIGHT 200      
#define GRAPH_X      40       
#define GRAPH_Y      280      

TIM_HandleTypeDef htim10;//2

void PWM_Measure_Init(void) {
    TIM_IC_InitTypeDef sConfigIC = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    // 修改GPIO模式为输入模式
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;  // 改为AF输入模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    htim10.Instance = TIM2;
    htim10.Init.Prescaler = 72 - 1;  // 72MHz / 72 = 1MHz，计数频率为1us
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 0xFFFF;  // 16位计时器最大值
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim10) != HAL_OK) {
        // 初始化失败处理
        while(1);
    }
    
    // 配置通道1为上升沿触发
    sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0x0;
    if (HAL_TIM_IC_ConfigChannel(&htim10, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
        // 配置失败处理
        while(1);
    }
    
    // 配置通道2为下降沿触发
    sConfigIC.ICPolarity = TIM_ICPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;  // 使用间接触发
    if (HAL_TIM_IC_ConfigChannel(&htim10, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
        // 配置失败处理
        while(1);
    }
    
    // 启动输入捕获和中断
    HAL_TIM_IC_Start_IT(&htim10, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim10, TIM_CHANNEL_2);
    
    // 配置中断优先级
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim10);
}

// 添加HAL库的回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    static uint32_t rise_time = 0;
    static uint32_t last_period = 0;
    
    if (htim->Instance == TIM2) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            // 上升沿
            uint32_t current_rise = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            
            // 计算两次上升沿之间的时间差，即为周期
            if (rise_time != 0) {
                if (current_rise > rise_time) {
                    pwm_period = current_rise - rise_time;
                } else {
                    // 计时器溢出处理
                    pwm_period = (0xFFFF - rise_time) + current_rise + 1;
                }
                last_period = pwm_period;
            } else {
                // 首次捕获到上升沿，使用上一次的周期
                pwm_period = last_period;
            }
            
            rise_time = current_rise;
            last_rise_time = current_rise;
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            // 下降沿
            uint32_t fall_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            
            // 计算高电平时间
            if (fall_time >= last_rise_time) {
                pwm_high_time = fall_time - last_rise_time;
            } else {
                // 计时器溢出处理
                pwm_high_time = (0xFFFF - last_rise_time) + fall_time + 1;
            }
            
            pwm_updated = 1;  // 标记数据已更新
        }
    }
}

static void Draw_PWM_Waveform(uint32_t high_time, uint32_t period) {
    // 清除整个波形绘制区域
    lcd_fill(GRAPH_X, GRAPH_Y - GRAPH_HEIGHT, GRAPH_X + GRAPH_WIDTH, GRAPH_Y + 40, WHITE);
    
    // 绘制坐标轴
    lcd_draw_line(GRAPH_X, GRAPH_Y, GRAPH_X + GRAPH_WIDTH, GRAPH_Y, BLACK);        // X轴
    lcd_draw_line(GRAPH_X, GRAPH_Y, GRAPH_X, GRAPH_Y - GRAPH_HEIGHT, BLACK);       // Y轴
    
    // 添加刻度标记 - X轴上每100像素一个刻度
    for (int i = 100; i < GRAPH_WIDTH; i += 100) {
        lcd_draw_line(GRAPH_X + i, GRAPH_Y, GRAPH_X + i, GRAPH_Y + 5, BLACK);
    }
    
    float duty = (period > 0) ? (float)high_time / period : 0;
    // 限制占空比在0-1范围内
    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;
    
    uint16_t high_width = (uint16_t)(GRAPH_WIDTH * duty);
    uint16_t high_level = GRAPH_Y - GRAPH_HEIGHT * 0.8;
    uint16_t low_level = GRAPH_Y - GRAPH_HEIGHT * 0.2;
    
    // 绘制PWM波形
    uint16_t pos = GRAPH_X;
    
    // 绘制高电平部分
    lcd_draw_line(pos, high_level, pos + high_width, high_level, RED);
    
    // 绘制垂直下降线
    if (high_width < GRAPH_WIDTH) {
        lcd_draw_line(pos + high_width, high_level, pos + high_width, low_level, RED);
    }
    
    // 绘制低电平部分
    if (high_width < GRAPH_WIDTH) {
        lcd_draw_line(pos + high_width, low_level, pos + GRAPH_WIDTH, low_level, RED);
    }
    
    // 波形信息显示
    char info[100];
    if (period > 0) {
        float freq = 1000000.0f / period;  // 周期单位为us，频率单位为Hz
        snprintf(info, sizeof(info), "Freq: %.2fHz  Duty: %.1f%%  Period: %.2fms", 
                 freq, duty * 100, period / 1000.0f);
    } else {
        snprintf(info, sizeof(info), "No Signal!");
    }
    lcd_show_string(GRAPH_X, GRAPH_Y + 20, 400, 16, 16, info, BLUE);
}

void PWM_Analyzer_Run(void) {
    PWM_Measure_Init();
    
    // 界面初始化
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT-1, WHITE);  // 清屏
    lcd_show_string(120, 20, 240, 24, 24, "PWM ANALYZER", BLACK);
    lcd_show_string(120, 50, 350, 16, 16, "Input Signal -> PA0", BLUE);
    
    // 绘制装饰性边框
    lcd_draw_rectangle(10, 10, 470, 390, BLACK);
    
    // 绘制初始空波形
    Draw_PWM_Waveform(0, 0);
    
    uint32_t last_update = HAL_GetTick();
    while (1) {
        // 检测按键退出
        if(detect_key_press() == 'n') {
            HAL_TIM_IC_Stop_IT(&htim10, TIM_CHANNEL_1);
            HAL_TIM_IC_Stop_IT(&htim10, TIM_CHANNEL_2);
            return;
        }
        
        // 定时更新显示
        if (HAL_GetTick() - last_update > 200) {
            if (pwm_updated) {
                Draw_PWM_Waveform(pwm_high_time, pwm_period);
                pwm_updated = 0;
            }
            last_update = HAL_GetTick();
        }
    }
} 
