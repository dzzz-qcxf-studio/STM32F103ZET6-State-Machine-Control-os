#include "./OVER/relay.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "./OVER/menu.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/TOUCH/touch.h"
#include <stdio.h>

/* 变量声明 */
uint8_t relay_flag = 1;

void Relay_Control_Interface(void) {
	// 初始化gpio
	MX_GPIO_Init();
	
    // 清空菜单区域（不清除键盘）
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WHITE);

    // 显示界面标题
    lcd_show_string(50, 30, 200, 24, 24, "Relay Control", BLACK);
    
    // 显示当前状态
    lcd_show_string(50, 80, 200, 24, 24, "Current State:", BLACK);
    lcd_show_string(220, 80, 100, 24, 24, relay_flag ? "OFF" : "ON", relay_flag ? RED : GREEN);
    
    // 显示操作提示
    lcd_show_string(50, 130, 250, 24, 24, "Press '1' to Toggle", BLUE);
    lcd_show_string(50, 180, 250, 24, 24, "Press 'n' to Exit", BLUE);

    while (1) {
        int key = detect_key_press();  // 监听键盘输入
        if (key == EVT_NONE) continue;

        if (key == '1') {  // 切换继电器状态
            Relay_Init();  // 调用您的继电器初始化/切换函数
            
            // 更新状态显示
            lcd_fill(220, 80, 300, 104, WHITE);
            lcd_show_string(220, 80, 100, 24, 24, relay_flag ? "OFF" : "ON", relay_flag ? RED : GREEN);
            
            // 显示操作反馈
            lcd_show_string(50, 230, 200, 24, 24, "State Changed!", GREEN);
            HAL_Delay(1000);
            lcd_fill(50, 230, 250, 254, WHITE);
        }
        else if (key == 'n') {  // 退出
            current_state = MENU_RELAY;
            return;
        }
    }
}

// GPIO初始化函数
void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
}

// 继电器控制函数
void Relay_Control(FunctionalState state) {
    if (state == ENABLE) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    }
}

// 继电器状态切换函数
void Relay_Init(void) {
    if(relay_flag) {
        relay_flag = 0;
        Relay_Control(DISABLE);
    } else {
        relay_flag = 1;
        Relay_Control(ENABLE);
    }
}
