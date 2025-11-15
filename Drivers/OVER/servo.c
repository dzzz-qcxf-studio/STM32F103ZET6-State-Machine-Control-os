#include "./OVER/servo.h"
#include "./OVER/menu.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/TOUCH/touch.h"

// 替换为定时器3的句柄
TIM_HandleTypeDef htim4;

void Servo_PWM_Init(void) {
    // 1. 使能时钟，替换为定时器3和对应GPIO的时钟
    __HAL_RCC_GPIOA_CLK_ENABLE(); 
    __HAL_RCC_TIM3_CLK_ENABLE();

    // 2. 配置PA6(TIM3_CH1)和PA7(TIM3_CH2)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. 配置TIM3（72MHz时钟）
    htim4.Instance = TIM3;
    htim4.Init.Prescaler = 143;         // 72MHz/144 = 500kHz
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 10000 - 1;      // 20ms周期
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim4);

    // 4. 配置PWM通道
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 750;              // 1.5ms脉宽
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

    // 5. 启动PWM
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

void Servo_SetAngle(uint8_t servo_num, uint8_t angle)
{
    // 限制角度范围
    angle = (angle > 180) ? 180 : angle;
    
    // 计算脉冲宽度 (SG90: 0.5ms-2.5ms → 250-1250计数)
    uint16_t pulse = 250 + (angle * 1000 / 180);
    
    if(servo_num == 1) {
        // PA6 (TIM3_CH1)
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);
    } 
    else if(servo_num == 2) {
        // PA7 (TIM3_CH2)
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse);
    }
}

void Servo_control(void)
{
    Servo_PWM_Init();
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WHITE);
    
    // 更新显示文本，替换为定时器3对应的引脚
    lcd_show_string(50, 50, 200, 24, 24, "Servo1 (PA6):", BLACK);
    lcd_show_string(50, 100, 200, 24, 24, "Servo2 (PA7):", BLACK);
    lcd_show_string(50, 150, 200, 24, 24, "Step size:", BLACK);
    
    uint8_t servo1_angle = 90;
    uint8_t servo2_angle = 90;
    uint8_t step = 5;
    
    // 初始位置
    Servo_SetAngle(1, servo1_angle);
    Servo_SetAngle(2, servo2_angle);
    
    // 显示初始值
    lcd_show_num(210, 50, servo1_angle, 3, 24, RED);
    lcd_show_num(210, 100, servo2_angle, 3, 24, RED);
    lcd_show_num(210, 150, step, 2, 24, RED);
    
    while(1) {
        int key = detect_key_press();
        if(key == EVT_NONE) continue;
        
        // 舵机1控制 (PA6)
        if(key == '4') {  // 左转
            servo1_angle = (servo1_angle >= step) ? (servo1_angle - step) : 0;
            Servo_SetAngle(1, servo1_angle);
        }
        else if(key == '6') {  // 右转
            servo1_angle = (servo1_angle <= 180-step) ? (servo1_angle + step) : 180;
            Servo_SetAngle(1, servo1_angle);
        }
        // 舵机2控制 (PA7)
        else if(key == '2') {  // 下转
            servo2_angle = (servo2_angle >= step) ? (servo2_angle - step) : 0;
            Servo_SetAngle(2, servo2_angle);
        }
        else if(key == '8') {  // 上转
            servo2_angle = (servo2_angle <= 110-step) ? (servo2_angle + step) : 110;
            Servo_SetAngle(2, servo2_angle);
        }
        // 步长调整
        else if(key == '+') step = (step < 40) ? step + 5 : 40;
        else if(key == '-') step = (step > 5) ? step - 5 : 5;
        // 退出
        else if(key == 'n') {
            current_state = MENU_SERVO;
            return;
        }
        
        // 更新显示
        lcd_fill(210, 50, 300, 74, WHITE);
        lcd_fill(210, 100, 300, 124, WHITE);
        lcd_fill(210, 150, 300, 174, WHITE);
        lcd_show_num(210, 50, servo1_angle, 3, 24, RED);
        lcd_show_num(210, 100, servo2_angle, 3, 24, RED);
        lcd_show_num(210, 150, step, 2, 24, RED);
    }
}    
