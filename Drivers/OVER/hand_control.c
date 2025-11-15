#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "./OVER/hand_control.h"
#include "./OVER/menu.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/TOUCH/touch.h"
#include <stdio.h>

void usart_send_string(int32_t x, int32_t y);

void Hand_Control(void) {
    // 清空菜单区域（不清除键盘）
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WHITE);

    // 显示输入提示
    lcd_show_string(50, 50, 200, 24, 24, "Enter X:", BLACK);
    lcd_show_string(50, 100, 200, 24, 24, "Enter Y:", BLACK);
    lcd_show_string(50, 200, 200, 24, 24, "Press = to switch", BLUE);
    lcd_show_string(50, 250, 200, 24, 24, "Press 'y' to send", BLUE);
    lcd_show_string(50, 300, 200, 24, 24, "Press 'n' to exit", BLUE);

    char x_input[10] = {0};  // 存储X坐标字符串
    char y_input[10] = {0};  // 存储Y坐标字符串
    uint8_t x_index = 0;
    uint8_t y_index = 0;
    int input_mode = -1;  // -1表示输入X，1表示输入Y

    while (1) {
        int key = detect_key_press();  // 监听键盘输入
        if (key == EVT_NONE) continue;

		if (key == 'y') {  // 检测到输入 'y' 则发送数据
            int x_val = atoi(x_input);
            int y_val = atoi(y_input);

            usart_send_string(x_val, y_val);  // 发送数据

            lcd_show_string(50, 350, 200, 24, 24, "Sent!", GREEN);
            HAL_Delay(1000);

        }
		
        if (input_mode == -1) {  // 输入X
            if (key == '=') {  // 切换到Y输入
                input_mode = -input_mode;
            } else if (key == '<') {  // 退格
                if (x_index > 0) {
                    x_index--;
                    x_input[x_index] = '\0';
                }
            } else if (key == 'C') {  // 清除
                for (int i = 0; i < 10; i++) x_input[i] = '\0';  // 代替 memset
                x_index = 0;
            } else if (x_index < 9) {  // 限制输入长度
                x_input[x_index++] = key;
                x_input[x_index] = '\0';
            }

            // 更新 X 坐标显示
            lcd_fill(150, 50, 300, 74, WHITE);  // 清除旧值
            lcd_show_string(150, 50, 100, 24, 24, x_input, RED);
        } 
        else {  // 输入Y
            if (key == '=') {  // 切换到X输入
                input_mode = -input_mode;
            } else if (key == '<') {  // 退格
                if (y_index > 0) {
                    y_index--;
                    y_input[y_index] = '\0';
                }
            } else if (key == 'C') {  // 清除
                for (int i = 0; i < 10; i++) y_input[i] = '\0';  // 代替 memset
                y_index = 0;
            } else if (y_index < 9) {  // 限制输入长度
                y_input[y_index++] = key;
                y_input[y_index] = '\0';
            }

            // 更新 Y 坐标显示
            lcd_fill(150, 100, 300, 124, WHITE);  // 清除旧值
            lcd_show_string(150, 100, 100, 24, 24, y_input, RED);
        }

        if (key == 'n') {  // 按 'n' 退出
			current_state = MENU_HAND;
			update_display();
            return;  // 返回主菜单
        }
    }
}

UART_HandleTypeDef huart3;  // 定义 UART3 句柄

void MX_USART3_UART_Init(void) {
    // 使能 USART3 和 GPIO 时钟
    __HAL_RCC_USART3_CLK_ENABLE();  // 使能 USART3 时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();   // 使能 GPIOB 时钟（USART3 的 TX 和 RX 通常连接到 GPIOB）

    // 配置 USART3 的 GPIO 引脚
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;  // PB10: TX, PB11: RX
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;           // 复用推挽输出（TX）和浮空输入（RX）
    GPIO_InitStruct.Pull = GPIO_NOPULL;               // 无上拉/下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;     // 高速模式
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);           // 初始化 GPIOB

    // 配置 USART3
    huart3.Instance = USART3;                         // 选择 USART3
    huart3.Init.BaudRate = 115200;                    // 波特率 115200
    huart3.Init.WordLength = UART_WORDLENGTH_8B;      // 8 位数据位
    huart3.Init.StopBits = UART_STOPBITS_1;           // 1 位停止位
    huart3.Init.Parity = UART_PARITY_NONE;            // 无校验位
    huart3.Init.Mode = UART_MODE_TX_RX;               // 使能发送和接收
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;      // 无硬件流控制
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;  // 16 倍过采样

    // 初始化 USART3
    if (HAL_UART_Init(&huart3) != HAL_OK) {
		//Error_Handler();
    }
}

void usart_send_string(int32_t x, int32_t y) {
    uint8_t send_buffer[12];

    // 填充发送缓冲区
    send_buffer[0] = 0xAA;  // 起始字节
    send_buffer[1] = 0x01;  // X 坐标类型
    send_buffer[2] = (uint8_t)(x & 0xFF);
    send_buffer[3] = (uint8_t)((x >> 8) & 0xFF);
    send_buffer[4] = (uint8_t)((x >> 16) & 0xFF);
    send_buffer[5] = (uint8_t)((x >> 24) & 0xFF);
    send_buffer[6] = 0x02;  // Y 坐标类型
    send_buffer[7]  = (uint8_t)(y & 0xFF);
    send_buffer[8]  = (uint8_t)((y >> 8) & 0xFF);
    send_buffer[9]  = (uint8_t)((y >> 16) & 0xFF);
    send_buffer[10] = (uint8_t)((y >> 24) & 0xFF);
    send_buffer[11] = 0x55;  // 结束字节

    // 发送数据
    HAL_UART_Transmit(&huart3, send_buffer, sizeof(send_buffer), 100);

    // 显示完整协议帧（十六进制）
    char hex_str[35] = {0};  // 12字节 * 3字符（0xXX空格） + 终止符
    for (int i = 0; i < 12; i++) {
        snprintf(hex_str + 3*i, 4, "%02X ", send_buffer[i]);
    }
    
    // 显示坐标值和协议帧
    lcd_show_string(50, 350, 200, 24, 24, "Sent:", BLUE);
    lcd_show_string(50, 380, 200, 24, 24, hex_str, GREEN);  // 显示十六进制数据
    
    // 显示坐标值（可选）
    char coord_str[50];
    snprintf(coord_str, sizeof(coord_str), "X:%d Y:%d", x, y);
    lcd_show_string(50, 410, 400, 24, 24, coord_str, RED);
    
    HAL_Delay(2000);  // 显示 2 秒
    lcd_fill(50, 350, 300, 434, WHITE);  // 清除显示区域
}
