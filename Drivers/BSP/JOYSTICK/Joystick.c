// PC0 -> ADC1_IN10 左摇杆 Y1	A1	PC1 -> ADC1_IN11 右摇杆 X2
// PC2 -> ADC1_IN12 左摇杆 X1	A0	PC3 -> ADC1_IN13 右摇杆 Y2
// PC4 -> GPIO 左按键 K1		D2	PC5 -> GPIO 右按键 K2
// 使用PC0-PC5引脚避免与其他模块冲突
#include "./BSP/JOYSTICK/Joystick.h"
#include "./BSP/LCD/lcd.h"
#include "./OVER/menu.h"
#include <stdio.h>
#include <math.h>

/* ADC句柄定义 */
static ADC_HandleTypeDef hadc1;

/* 摇杆初始化函数 */
void Joystick_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    ADC_ChannelConfTypeDef sConfig = {0};
    
    // 使能时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    // 配置模拟输入引脚 (PC0, PC1, PC2, PC3)
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 配置按键输入引脚 (PC4, PC5)
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // ADC1配置
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);

    // ADC通道配置
    sConfig.Channel = ADC_CHANNEL_10;  // 对应PC0
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/* 读取单个ADC通道 */
static uint16_t ADC_ReadChannel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint16_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    
    return value;
}

/* 读取摇杆状态 */
void Joystick_Read(JoystickState *state) {
    // 读取模拟值
    state->y1 = ADC_ReadChannel(ADC_CHANNEL_10);  // PC0
    state->x1 = ADC_ReadChannel(ADC_CHANNEL_12);  // PC2
    state->x2 = ADC_ReadChannel(ADC_CHANNEL_11);  // PC1
    state->y2 = ADC_ReadChannel(ADC_CHANNEL_13);  // PC3

    // 读取按键状态（低电平有效）
    state->k1 = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
    state->k2 = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
}

/* 界面显示参数 */
#define INFO_Y         5       // 信息显示区Y坐标
#define HELP_Y         450     // 帮助信息Y坐标

/* 摇杆显示参数 */
#define JOY_RADIUS    70      // 摇杆圆形区域半径
#define JOY1_X        120     // 左摇杆圆心X坐标
#define JOY1_Y        280     // 左摇杆圆心Y坐标
#define JOY2_X        360     // 右摇杆圆心X坐标
#define JOY2_Y        280     // 右摇杆圆心Y坐标
#define DOT_RADIUS    5       // 位置指示点半径
#define INNER_RADIUS  20      // 内圈半径（死区指示）

/* 颜色定义 */
#define JOY1_COLOR    RED      // 左摇杆颜色
#define JOY2_COLOR    BLUE     // 右摇杆颜色
#define GRID_COLOR    GRAY     // 网格颜色
#define AXIS_COLOR    BLACK    // 轴线颜色
#define TEXT_COLOR    BLACK    // 文本颜色
#define DOT1_COLOR    MAGENTA  // 左摇杆位置点颜色
#define DOT2_COLOR    CYAN     // 右摇杆位置点颜色
#define DEADZONE_COLOR LGRAY   // 死区颜色
#define BG_COLOR      WHITE    // 背景颜色

/* 绘制摇杆底图 */
void Draw_Joystick_Layout(void) {
//    // 清除显示区域
//    lcd_fill(0, INFO_Y + 130, lcddev.width, HELP_Y - 10, BG_COLOR);
    // 清空菜单区域（不清除键盘）
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WHITE);
	
    // 绘制左摇杆外圈
    lcd_draw_circle(JOY1_X, JOY1_Y, JOY_RADIUS, JOY1_COLOR);
    // 绘制左摇杆死区圈
    lcd_draw_circle(JOY1_X, JOY1_Y, INNER_RADIUS, DEADZONE_COLOR);
    // 绘制左摇杆十字准星
    lcd_draw_line(JOY1_X - JOY_RADIUS, JOY1_Y, JOY1_X + JOY_RADIUS, JOY1_Y, GRID_COLOR);
    lcd_draw_line(JOY1_X, JOY1_Y - JOY_RADIUS, JOY1_X, JOY1_Y + JOY_RADIUS, GRID_COLOR);
    // 绘制左摇杆刻度圈
    lcd_draw_circle(JOY1_X, JOY1_Y, JOY_RADIUS/2, GRID_COLOR);

    // 绘制右摇杆外圈
    lcd_draw_circle(JOY2_X, JOY2_Y, JOY_RADIUS, JOY2_COLOR);
    // 绘制右摇杆死区圈
    lcd_draw_circle(JOY2_X, JOY2_Y, INNER_RADIUS, DEADZONE_COLOR);
    // 绘制右摇杆十字准星
    lcd_draw_line(JOY2_X - JOY_RADIUS, JOY2_Y, JOY2_X + JOY_RADIUS, JOY2_Y, GRID_COLOR);
    lcd_draw_line(JOY2_X, JOY2_Y - JOY_RADIUS, JOY2_X, JOY2_Y + JOY_RADIUS, GRID_COLOR);
    // 绘制右摇杆刻度圈
    lcd_draw_circle(JOY2_X, JOY2_Y, JOY_RADIUS/2, GRID_COLOR);

    // 绘制标签
    lcd_show_string(JOY1_X - 30, JOY1_Y + JOY_RADIUS + 15, 60, 16, 16, "Left Joy", JOY1_COLOR);
    lcd_show_string(JOY2_X - 35, JOY2_Y + JOY_RADIUS + 15, 70, 16, 16, "Right Joy", JOY2_COLOR);

    // 绘制引脚说明
    lcd_show_string(10, JOY1_Y - JOY_RADIUS - 30, 200, 12, 12, "PC2(X1) PC0(Y1) PC4(K1)", AXIS_COLOR);
    lcd_show_string(300, JOY2_Y - JOY_RADIUS - 30, 200, 12, 12, "PC1(X2) PC3(Y2) PC5(K2)", AXIS_COLOR);
}

/* 重绘摇杆元素（当位置点擦除时可能需要重绘的线条和圆圈） */
void Redraw_Joystick_Elements(int dot_x, int dot_y, int center_x, int center_y) {
    // 检查是否需要重绘十字准星
    if (abs(dot_x - center_x) <= DOT_RADIUS + 1) {
        // 重绘垂直线
        lcd_draw_line(center_x, center_y - JOY_RADIUS, center_x, center_y + JOY_RADIUS, GRID_COLOR);
    }
    if (abs(dot_y - center_y) <= DOT_RADIUS + 1) {
        // 重绘水平线
        lcd_draw_line(center_x - JOY_RADIUS, center_y, center_x + JOY_RADIUS, center_y, GRID_COLOR);
    }

    // 检查是否需要重绘圆圈
    int dist_to_center = sqrt((dot_x - center_x) * (dot_x - center_x) + (dot_y - center_y) * (dot_y - center_y));

    // 重绘死区圆圈
    if (abs(dist_to_center - INNER_RADIUS) <= DOT_RADIUS + 1) {
        lcd_draw_circle(center_x, center_y, INNER_RADIUS, DEADZONE_COLOR);
    }

    // 重绘中间刻度圆圈
    if (abs(dist_to_center - JOY_RADIUS/2) <= DOT_RADIUS + 1) {
        lcd_draw_circle(center_x, center_y, JOY_RADIUS/2, GRID_COLOR);
    }

    // 重绘外圆圈
    if (abs(dist_to_center - JOY_RADIUS) <= DOT_RADIUS + 1) {
        uint16_t color = (center_x == JOY1_X) ? JOY1_COLOR : JOY2_COLOR;
        lcd_draw_circle(center_x, center_y, JOY_RADIUS, color);
    }
}

/* 更新摇杆显示 */
void Update_Joystick_Graph(JoystickState *state) {
    static uint16_t last_x1 = 0, last_y1 = 0;
    static uint16_t last_x2 = 0, last_y2 = 0;
    static uint8_t first_run = 1;

    // 计算摇杆1的位置
    float x1_norm = (float)(state->x1 - 2048) / 2048.0f;  // 归一化到[-1, 1]
    float y1_norm = -(float)(state->y1 - 2048) / 2048.0f;

    // 限制在圆形区域内
    float r1 = sqrt(x1_norm * x1_norm + y1_norm * y1_norm);
    if (r1 > 1.0f) {
        x1_norm /= r1;
        y1_norm /= r1;
    }

    int x1_pos = JOY1_X + (int)(x1_norm * (JOY_RADIUS - DOT_RADIUS));
    int y1_pos = JOY1_Y - (int)(y1_norm * (JOY_RADIUS - DOT_RADIUS));  // Y轴向上为正

    // 计算摇杆2的位置
    float x2_norm = (float)(state->x2 - 2048) / 2048.0f;
    float y2_norm = (float)(state->y2 - 2048) / 2048.0f;

    // 限制在圆形区域内
    float r2 = sqrt(x2_norm * x2_norm + y2_norm * y2_norm);
    if (r2 > 1.0f) {
        x2_norm /= r2;
        y2_norm /= r2;
    }

    int x2_pos = JOY2_X + (int)(x2_norm * (JOY_RADIUS - DOT_RADIUS));
    int y2_pos = JOY2_Y - (int)(y2_norm * (JOY_RADIUS - DOT_RADIUS));

    // 清除上一次的位置点（仅在非首次运行时）
    if (!first_run) {
        // 清除左摇杆位置点
        lcd_fill_circle(last_x1, last_y1, DOT_RADIUS, BG_COLOR);
        // 清除右摇杆位置点
        lcd_fill_circle(last_x2, last_y2, DOT_RADIUS, BG_COLOR);

        // 重绘可能被擦除的网格线和圆圈
        Redraw_Joystick_Elements(last_x1, last_y1, JOY1_X, JOY1_Y);
        Redraw_Joystick_Elements(last_x2, last_y2, JOY2_X, JOY2_Y);
    }

    // 绘制新的位置点（实心圆）
    lcd_fill_circle(x1_pos, y1_pos, DOT_RADIUS, DOT1_COLOR);
    lcd_fill_circle(x2_pos, y2_pos, DOT_RADIUS, DOT2_COLOR);

    // 保存当前位置
    last_x1 = x1_pos;
    last_y1 = y1_pos;
    last_x2 = x2_pos;
    last_y2 = y2_pos;
    first_run = 0;
}

/* 显示标题和帮助信息 */
void Draw_Title_And_Info(void) {

}

/* 显示摇杆数据 */
void Display_Joystick_Values(JoystickState *state) {
    char buf[50];

    // 计算归一化值和距离
    float x1_norm = (float)(state->x1 - 2048) / 2048.0f;
    float y1_norm = (float)(state->y1 - 2048) / 2048.0f;
    float x2_norm = (float)(state->x2 - 2048) / 2048.0f;
    float y2_norm = (float)(state->y2 - 2048) / 2048.0f;

    float dist1 = sqrt(x1_norm * x1_norm + y1_norm * y1_norm);
    float dist2 = sqrt(x2_norm * x2_norm + y2_norm * y2_norm);

    // 显示左摇杆数据
    snprintf(buf, sizeof(buf), "L-X: %4d (%+.2f)", state->x1, x1_norm);
    lcd_show_string(10, INFO_Y + 30, 200, 16, 16, buf, JOY1_COLOR);

    snprintf(buf, sizeof(buf), "L-Y: %4d (%+.2f)", state->y1, y1_norm);
    lcd_show_string(10, INFO_Y + 50, 200, 16, 16, buf, JOY1_COLOR);

    snprintf(buf, sizeof(buf), "L-Dist: %.3f", dist1);
    lcd_show_string(10, INFO_Y + 70, 150, 16, 16, buf, JOY1_COLOR);

    // 显示右摇杆数据
    snprintf(buf, sizeof(buf), "R-X: %4d (%+.2f)", state->x2, x2_norm);
    lcd_show_string(250, INFO_Y + 30, 200, 16, 16, buf, JOY2_COLOR);

    snprintf(buf, sizeof(buf), "R-Y: %4d (%+.2f)", state->y2, y2_norm);
    lcd_show_string(250, INFO_Y + 50, 200, 16, 16, buf, JOY2_COLOR);

    snprintf(buf, sizeof(buf), "R-Dist: %.3f", dist2);
    lcd_show_string(250, INFO_Y + 70, 150, 16, 16, buf, JOY2_COLOR);

    // 显示按键状态
    lcd_show_string(10, INFO_Y + 95, 80, 16, 16, "Keys:", TEXT_COLOR);
    lcd_show_string(80, INFO_Y + 95, 40, 16, 16, state->k1 ? "[ON]" : "[OFF]",
                   state->k1 ? JOY1_COLOR : GRAY);
    lcd_show_string(130, INFO_Y + 95, 40, 16, 16, state->k2 ? "[ON]" : "[OFF]",
                   state->k2 ? JOY2_COLOR : GRAY);

    // 显示死区状态
    lcd_show_string(200, INFO_Y + 95, 100, 16, 16, "Deadzone:", TEXT_COLOR);
    lcd_show_string(290, INFO_Y + 95, 40, 16, 16, (dist1 < 0.1f) ? "L" : " ", DEADZONE_COLOR);
    lcd_show_string(310, INFO_Y + 95, 40, 16, 16, (dist2 < 0.1f) ? "R" : " ", DEADZONE_COLOR);
}

/* 主显示函数 */
void Joystick_Display(void) {
    JoystickState js;
    uint32_t last_update = 0;
    const uint32_t UPDATE_INTERVAL = 5; // 20Hz更新率
	
	    // 清空菜单区域（不清除键盘）
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WHITE);
	
    Draw_Title_And_Info();
    Draw_Joystick_Layout();
    
    while (current_state == MENU_JOYSTICK) {

        int key = detect_key_press();  // 监听键盘输入
		
		if (key == 'n') {  // 按 'n' 退出
			current_state = MENU_HAND;
			update_display();
			return;  // 返回主菜单
		}
		
		uint32_t now = HAL_GetTick();
        
		if (now - last_update >= UPDATE_INTERVAL) {
            // 读取摇杆状态
            Joystick_Read(&js);
            
            // 更新显示
            Display_Joystick_Values(&js);
            Update_Joystick_Graph(&js);
            
            last_update = now;
        }
        
        HAL_Delay(1); // 防止过度占用CPU
    }
}
