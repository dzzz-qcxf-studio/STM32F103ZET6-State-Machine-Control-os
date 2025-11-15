/*************************************************
 * mpu_pid.c - MPU6050传感器与PID控制L9110电机
 * 功能：
 * 1. MPU6050数据采集与显示
 * 2. PID控制L9110电机
 * 3. 波形显示界面
 * 硬件连接：
 * - STM32F103ZE
 * - MPU6050(I2C): SCL -> PB6, SDA -> PB7
 * - L9110电机驱动: A-IA -> PA6(TIM3_CH1), A-IB -> PA0(TIM2_CH1)
 * - LCD 4.3寸屏幕 - 竖屏显示
 ************************************************/

#include "stm32f1xx_hal.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/TOUCH/touch.h"
#include "./OVER/mpu_pid.h"
#include "./OVER/mpu6050.h"
#include "./OVER/menu.h"
#include <stdio.h>
#include <math.h>

/* 外部函数声明 */
extern int detect_key_press(void);

/* 全局变量 */
PID_TypeDef pid;
volatile float current_angle = 0.0f;
volatile float target_angle = 0.0f;
volatile float motor_output = 0.0f;
volatile uint8_t data_updated = 0;
char last_key=' ';

/* 图形波形显示FPS */
#define FPS 1000/60

/* 图形波形绘制参数 - 适配竖屏显示 */
#define SCREEN_HEIGHT 480

/* 竖屏界面布局 - 针对4.3寸屏幕优化 */
#define INFO_Y         5       // 信息显示区Y坐标
#define GRAPH_HEIGHT   260     // 波形图高度 (原 200)
#define GRAPH_WIDTH    360     // 波形图宽度 (安全值, 原 230)
#define GRAPH_X        35      // 波形图X轴起点 (安全值, 原 35)
#define GRAPH_Y        (GRAPH_HEIGHT + 70) // 波形图Y轴（0度线）, Y_start = 70, Y_axis = 330
#define KEYBOARD_Y     400     // 键盘Y坐标 (原 390)
#define HELP_TEXT_X    10      // 帮助文本X坐标
#define HELP_TEXT_Y    (KEYBOARD_Y - 20) // 帮助文本Y坐标

/* 电机PWM参数 */
#define PWM_FREQ     1000     // PWM频率1KHz
#define PWM_PERIOD   1000     // PWM周期1000

/* 波形颜色定义 */
#define GRID_COLOR   GRAY     // 网格颜色
#define AXIS_COLOR   BLACK    // 坐标轴颜色
#define WAVE1_COLOR  RED      // 当前角度波形颜色
#define WAVE2_COLOR  BLUE     // 目标角度波形颜色
#define WAVE3_COLOR  GREEN    // PID输出波形颜色
#define TEXT_COLOR   BLACK    // 文本颜色

/* 竖屏界面布局 */
#define INFO_AREA_Y  20       // 信息区域Y坐标
#define INFO_AREA_HEIGHT 50   // 信息区域高度
#define CTRL_AREA_Y  70       // 控制区域Y坐标
#define CTRL_AREA_HEIGHT 50   // 控制区域高度

/* 电机控制变量 */
TIM_HandleTypeDef htim2;  // 使用TIM2控制一个通道
TIM_HandleTypeDef htim3;  // 使用TIM3控制另一个通道
TIM_OC_InitTypeDef sConfigOC = {0};

/* PID结构体定义 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float prev_input;
    float last_measurement;   // 上一次的测量值（用于D项）
    float output_min;
    float output_max;
} PID_Controller;

/* MPU6050数据 */
short aacx, aacy, aacz;     // 加速度传感器原始数据
short gyrox, gyroy, gyroz;  // 陀螺仪传感器原始数据
float mpu_angle_y = 0.0f;   // 绕Y轴的角度

/* 时间戳 */
uint32_t last_update_time = 0;

/* 电机控制相关定义 */
#define TIM_PERIOD (7200 - 1) // 定时器周期 (72MHz / 10kHz = 7200)
#define MIN_PWM_OUT 500       // 电机最小启动PWM值

/* PID初始化函数 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float setpoint, float min, float max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;
    pid->prev_input = 0.0f; // 初始化prev_input
    pid->output_min = min;
    pid->output_max = max;
}

/* PID计算函数 - 针对推力有限的飞棍项目优化 */
float PID_Compute(PID_TypeDef *pid, float input, float dt) {
    if (dt <= 0) return 0; // 防止dt为0

    float error = pid->setpoint - input;

    // --- 增加积分死区：当误差绝对值小于0.5度时，清零积分项 ---
    if (fabsf(error) < 0.5f) {
        pid->integral = 0.0f;
    }

    // --- P项: 比例环节 ---
    float p_term = pid->kp * error;

    // --- D项: 微分先行 (作用于测量值，避免目标值跳变冲击) ---
    float derivative = (input - pid->prev_input) / dt;
    float d_term = -pid->kd * derivative;
    
    // --- 预计算I项和总输出，用于抗饱和判断 ---
    float i_term = pid->ki * pid->integral;
    float output = p_term + i_term + d_term;

    // --- 积分抗饱和 (Anti-Windup) & 输出限制 ---
    // 只有在输出未饱和的情况下才累积积分
    if (output > pid->output_max) {
        output = pid->output_max;
        // 如果输出饱和，则不进行积分累加
    } else if (output < pid->output_min) {
        output = pid->output_min;
        // 如果输出饱和，则不进行积分累加
    } else {
        // 未饱和，正常积分
        pid->integral += error * dt;
    }

    // 更新历史值
    pid->prev_input = input;

    return output;
}

/* L9110电机初始化 - 使用两个定时器 (回退到稳定版本) */
void L9110_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    
    /* 启用时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    /* 配置GPIO引脚 - TIM3_CH1(PA6) */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* 配置GPIO引脚 - TIM2_CH1(PA0) */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* 配置TIM3 - 用于A-IA */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 72-1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = PWM_PERIOD-1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim3);
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
    HAL_TIM_PWM_Init(&htim3);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
    
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    
    /* 配置TIM2 - 用于A-IB */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 72-1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = PWM_PERIOD-1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);
    HAL_TIM_PWM_Init(&htim2);
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    
    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
}

/* L9110电机驱动函数 - 使用两个定时器 (回退到稳定版本) */
void L9110_SetMotor(float speed) {
    uint32_t pwm_value;
    
    if (speed > 100.0f) speed = 100.0f;
    if (speed < -100.0f) speed = -100.0f;
    
    pwm_value = (uint32_t)(fabsf(speed) * PWM_PERIOD / 100.0f);
    
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    HAL_Delay(2);
    
    if (speed >= 0) {
        // 正转: TIM3(PA6) PWM, TIM2(PA0) Low
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
    } else {
        // 反转: TIM2(PA0) PWM, TIM3(PA6) Low
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_value);
    }
}

/* MPU6050获取角度函数 - 互补滤波 */
float Get_MPU_Angle(void) {
    short accel_x, accel_y, accel_z;
    short gyro_x, gyro_y, gyro_z;
    static float angle = 0;
    static uint32_t last_time = 0;
    float dt;
    
    // 获取加速度和陀螺仪数据
    MPU_Get_Accelerometer(&accel_x, &accel_y, &accel_z);
    MPU_Get_Gyroscope(&gyro_x, &gyro_y, &gyro_z);
    
    // 计算时间差
    uint32_t now = HAL_GetTick();
    dt = (now - last_time) / 1000.0f; // 转换为秒
    if(dt <= 0) dt = 0.001f; // 防止除零
    last_time = now;
    
    // 使用y轴计算倾角
    float accel_angle = atan2f((float)accel_y, sqrtf((float)accel_x * accel_x + (float)accel_z * accel_z)) * 180.0f / 3.14159f;
    
    // 使用互补滤波器融合陀螺仪和加速度计数据
    float gyro_rate = (float)gyro_x / 16.4f; // 转换为度/秒，使用x轴陀螺仪数据
    angle = 0.98f * (angle + gyro_rate * dt) + 0.02f * accel_angle;
    
    return angle;
}

/* 绘制网格、坐标轴和图例 - 优化版 */
void Draw_PID_Graph_Layout(void) {
    // 清除整个图形区域
    lcd_fill(0, GRAPH_Y - GRAPH_HEIGHT, GRAPH_X + GRAPH_WIDTH + 40, GRAPH_Y + 20, WHITE);
    
    // 绘制坐标轴
    lcd_draw_line(GRAPH_X, GRAPH_Y, GRAPH_X + GRAPH_WIDTH, GRAPH_Y, AXIS_COLOR);        // X轴
    lcd_draw_line(GRAPH_X, GRAPH_Y, GRAPH_X, GRAPH_Y - GRAPH_HEIGHT, AXIS_COLOR);       // Y轴
    
    // 绘制中线（零度位置）
    lcd_draw_line(GRAPH_X, GRAPH_Y - GRAPH_HEIGHT/2, GRAPH_X + GRAPH_WIDTH, GRAPH_Y - GRAPH_HEIGHT/2, GRID_COLOR);
    
    // 标记Y轴刻度 (角度)
    lcd_show_string(0, GRAPH_Y - GRAPH_HEIGHT, 20, 16, 16, "+45", TEXT_COLOR);
    lcd_show_string(0, GRAPH_Y - GRAPH_HEIGHT/2 - 8, 20, 16, 16, "0", TEXT_COLOR);
    lcd_show_string(0, GRAPH_Y - 16, 20, 16, 16, "-45", TEXT_COLOR);

    // 标记第二Y轴刻度 (输出百分比)
    lcd_show_string(GRAPH_X + GRAPH_WIDTH + 2, GRAPH_Y - GRAPH_HEIGHT, 38, 16, 16, "+100%", TEXT_COLOR);
    lcd_show_string(GRAPH_X + GRAPH_WIDTH + 2, GRAPH_Y - 16, 38, 16, 16, "-100%", TEXT_COLOR);
    
    // 绘制图例
    lcd_draw_line(GRAPH_X, GRAPH_Y + 15, GRAPH_X + 20, GRAPH_Y + 15, WAVE1_COLOR);
    lcd_show_string(GRAPH_X + 25, GRAPH_Y + 10, 40, 16, 16, "Angle", TEXT_COLOR);
    
    lcd_draw_line(GRAPH_X + 70, GRAPH_Y + 15, GRAPH_X + 90, GRAPH_Y + 15, WAVE2_COLOR);
    lcd_show_string(GRAPH_X + 95, GRAPH_Y + 10, 40, 16, 16, "Target", TEXT_COLOR);
    
    lcd_draw_line(GRAPH_X + 150, GRAPH_Y + 15, GRAPH_X + 170, GRAPH_Y + 15, WAVE3_COLOR);
    lcd_show_string(GRAPH_X + 175, GRAPH_Y + 10, 40, 16, 16, "Output", TEXT_COLOR);
}

void Draw_Title_And_Help(void) {
    lcd_show_string(10, INFO_Y, 260, 16, 16, "MPU6050 PID Control (Y-axis)", TEXT_COLOR);
    lcd_show_string(10, 400, 300, 16, 16, "1/4:P+ 2/5:I+ 3/6:D+ 7/9:Angle 0:Reset", BLUE);
}

/* 绘制PID波形图 , 绘制连续线条 */
void Update_PID_Graph(float current, float target, float output) {
    static uint16_t graph_index = 0;
    static float angle_history[GRAPH_WIDTH] = {0};
    static float target_history[GRAPH_WIDTH] = {0};
    static float output_history[GRAPH_WIDTH] = {0};
    static uint8_t initialized = 0;
    
    if (!initialized) {
        for (int i = 0; i < GRAPH_WIDTH; i++) {
            angle_history[i] = 0.0f;
            target_history[i] = 0.0f;
            output_history[i] = 0.0f;
        }
        initialized = 1;
        Draw_PID_Graph_Layout(); 
        return; 
    }
    
    // 1. 获取上一个数据点的索引
    uint16_t prev_index = (graph_index == 0) ? GRAPH_WIDTH - 1 : graph_index - 1;

    // 2. 计算上一个点的屏幕坐标
    uint16_t prev_x_pos = GRAPH_X + prev_index;
    int y_prev_current = GRAPH_Y - (int)((angle_history[prev_index] / 45.0f) * (GRAPH_HEIGHT/2) + GRAPH_HEIGHT/2);
    int y_prev_target  = GRAPH_Y - (int)((target_history[prev_index] / 45.0f) * (GRAPH_HEIGHT/2) + GRAPH_HEIGHT/2);
    int y_prev_output  = GRAPH_Y - (int)((output_history[prev_index] / 100.0f) * (GRAPH_HEIGHT/2) + GRAPH_HEIGHT/2);
    
    // 3. 更新当前点的数据到历史记录
    angle_history[graph_index] = current;
    target_history[graph_index] = target;
    output_history[graph_index] = output;
    
    // 4. 计算当前点的屏幕坐标
    uint16_t x_pos = GRAPH_X + graph_index;
    int y_current = GRAPH_Y - (int)((angle_history[graph_index] / 45.0f) * (GRAPH_HEIGHT/2) + GRAPH_HEIGHT/2);
    int y_target  = GRAPH_Y - (int)((target_history[graph_index] / 45.0f) * (GRAPH_HEIGHT/2) + GRAPH_HEIGHT/2);
    int y_output  = GRAPH_Y - (int)((output_history[graph_index] / 100.0f) * (GRAPH_HEIGHT/2) + GRAPH_HEIGHT/2);
    
    // 限制Y坐标在图形区域内
#define CLAMP_Y(y) do { \
    if (y < GRAPH_Y - GRAPH_HEIGHT + 1) y = GRAPH_Y - GRAPH_HEIGHT + 1; \
    if (y > GRAPH_Y - 1) y = GRAPH_Y - 1; \
} while(0)

    CLAMP_Y(y_prev_current);
    CLAMP_Y(y_prev_target);
    CLAMP_Y(y_prev_output);
    CLAMP_Y(y_current);
    CLAMP_Y(y_target);
    CLAMP_Y(y_output);

    // 5. 只清除当前列的像素，为绘制新线段做准备
	if(x_pos > 0 && x_pos < GRAPH_WIDTH - 10)
    lcd_fill(x_pos , GRAPH_Y - GRAPH_HEIGHT -1, x_pos + 10, GRAPH_Y - 1, WHITE);
    else lcd_fill(x_pos , GRAPH_Y - GRAPH_HEIGHT - 1, x_pos, GRAPH_Y - 1, WHITE);
	
    // 如果是网格线位置，重绘网格
    if (graph_index > 0 && graph_index % 40 == 0) {
        lcd_draw_line(x_pos, GRAPH_Y - GRAPH_HEIGHT + 1, x_pos, GRAPH_Y - 1, GRID_COLOR);
    }
    
    // 重绘中线
    lcd_draw_point(x_pos, GRAPH_Y - GRAPH_HEIGHT/2, GRID_COLOR);
    
    // 6. 绘制连接线段 (只有当索引大于0时才绘制，避免从屏幕末尾画到开头)
    if (graph_index > 0) {
        lcd_draw_line(prev_x_pos, y_prev_current, x_pos, y_current, WAVE1_COLOR);
        lcd_draw_line(prev_x_pos, y_prev_target,  x_pos, y_target,  WAVE2_COLOR);
        lcd_draw_line(prev_x_pos, y_prev_output,  x_pos, y_output,  WAVE3_COLOR);
    } else {
        // 对于第一个点，只画一个点来开始
        lcd_draw_point(x_pos, y_current, WAVE1_COLOR);
        lcd_draw_point(x_pos, y_target, WAVE2_COLOR);
        lcd_draw_point(x_pos, y_output, WAVE3_COLOR);
    }
    
    // 7. 更新索引
    graph_index = (graph_index + 1) % GRAPH_WIDTH;
}

/* Update Info Display Area with minimal flicker */
void Update_Info_Display(float current, float target, float output) {
    static float last_shown_angle = -999.0f;
    static float last_shown_target = -999.0f;
    static float last_shown_output = -999.0f;
    static float last_kp = -1.0f, last_ki = -1.0f, last_kd = -1.0f;
    
    char buf[40];

    // 跟新当前角度
    if (fabsf(current - last_shown_angle) > 0.1f) {
        snprintf(buf, sizeof(buf), "Angle: %.1f", current);
        lcd_fill(10, INFO_AREA_Y, 130, INFO_AREA_Y + 16, WHITE);
        lcd_show_string(10, INFO_AREA_Y, 120, 16, 16, buf, TEXT_COLOR);
        last_shown_angle = current;
    }

    // 跟新目标角度
    if (fabsf(target - last_shown_target) > 0.1f) {
        snprintf(buf, sizeof(buf), "Target: %.1f", target);
        lcd_fill(140, INFO_AREA_Y, 270, INFO_AREA_Y + 16, WHITE);
        lcd_show_string(140, INFO_AREA_Y, 120, 16, 16, buf, TEXT_COLOR);
        last_shown_target = target;
    }

    // 跟新pid参数
    if (fabsf(pid.kp - last_kp) > 0.01f || fabsf(pid.ki - last_ki) > 0.001f || fabsf(pid.kd - last_kd) > 0.01f) {
        snprintf(buf, sizeof(buf), "P:%.1f I:%.3f D:%.1f", pid.kp, pid.ki, pid.kd);
        lcd_fill(10, INFO_AREA_Y + 20, 165, INFO_AREA_Y + 20 + 16, WHITE);
        lcd_show_string(10, INFO_AREA_Y + 20, 150, 16, 16, buf, TEXT_COLOR);
        last_kp = pid.kp;
        last_ki = pid.ki;
        last_kd = pid.kd;
    }

    // 跟新输出
    if (fabsf(output - last_shown_output) > 1.0f) {
        snprintf(buf, sizeof(buf), "Output: %.0f%%", output);
        lcd_fill(170, INFO_AREA_Y + 20, 270, INFO_AREA_Y + 20 + 25, WHITE);
        lcd_show_string(170, INFO_AREA_Y + 20, 100, 16, 16, buf, TEXT_COLOR);
        last_shown_output = output;
    }
}

/* 处理键盘输入以调整PID参数 */
void Handle_Keyboard_Input(char key) { // 接受按键字符
    char help[50];
	
	switch(key) {
		case '1': pid.kp += 0.5f; break;
		case '4': pid.kp -= 0.5f; if (pid.kp < 0) pid.kp = 0; break;
		
		case '2': pid.ki += 0.1f; break;
		case '5': pid.ki -= 0.1f; if (pid.ki < 0) pid.ki = 0; break;
			
		case '3': pid.kd += 0.2f; break;
		case '6': pid.kd -= 0.2f; if (pid.kd < 0) pid.kd = 0; break;
			
		case '7': target_angle += 5.0f; if (target_angle > 45) target_angle = 45; pid.setpoint = target_angle; break;
		case '9': target_angle -= 5.0f; if (target_angle < -45) target_angle = -45; pid.setpoint = target_angle; break;
			
		case '8': 
			// 重置
			target_angle = 0.0f;
			pid.setpoint = 0.0f;
			pid.integral = 0.0f;
			pid.prev_input = 0.0f;
			L9110_SetMotor(0.0f);   // 停止电机
			break;
		
		case '0':
		case '.':
			// 自定义调参
			snprintf(help, sizeof(help), "Custom Adjust: %c", key);
			lcd_fill(HELP_TEXT_X, HELP_TEXT_Y, 280, HELP_TEXT_Y + 16, WHITE);
			lcd_show_string(HELP_TEXT_X, HELP_TEXT_Y, 200, 16, 16, help, RED);
			return; // 直接返回
		case 'N':
			snprintf(help, sizeof(help), "Unknown key: %c", key);
			lcd_fill(HELP_TEXT_X, HELP_TEXT_Y, 280, HELP_TEXT_Y + 16, WHITE);
			lcd_show_string(HELP_TEXT_X, HELP_TEXT_Y, 200, 16, 16, help, RED);
			return;
		default:
			// 未知按键
			return; // Directly return for unknown keys
	}
    
    // 清除旧的帮助文本并更新信息显示
    lcd_fill(HELP_TEXT_X, HELP_TEXT_Y, 280, HELP_TEXT_Y + 16, WHITE);
    Update_Info_Display(current_angle, target_angle, motor_output);
}

/* MPU PID控制主界面 - 竖屏简洁版 - 使用自带键盘调参 */
void MPU_PID_Control_Interface(void) {
    // 初始化MPU6050
    if(MPU_Init()) {
        lcd_show_string(10, 80, 240, 24, 24, "MPU6050 Error!", RED);
        HAL_Delay(1000);
        return;
    }
    
    // 初始化L9110电机
    L9110_Init();
    
    // 初始化PID控制器 - 使用为飞棍项目优化的参数
    PID_Init(&pid, 1.5f, 0.08f, 2.0f, 0.0f, -100.0f, 100.0f);
    target_angle = 0.0f;
    
    // 清屏初始化 - 调整为竖屏布局
    lcd_fill(0, 0, lcddev.width, lcddev.height, WHITE);
    
    // 绘制初始界面
    Draw_Title_And_Help();
    Draw_PID_Graph_Layout();
    Update_Info_Display(current_angle, target_angle, motor_output);
    
    // 绘制键盘
    draw_keyboard();
    
	uint32_t last_display_time = 0;
	
    while (1) {
        
        int key = detect_key_press(); // 获取按键
        if(key) {
            Handle_Keyboard_Input((char)key); // 传入按键
        }

        // 按照FPS跟新显示和控制速度
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_update_time >= FPS) { // 限制频率与FPS相同
            float dt = (current_time - last_update_time) / 1000.0f;
            last_update_time = current_time;
            
            // 获取角度
            current_angle = Get_MPU_Angle();
            
            // 计算pidf输出
            motor_output = PID_Compute(&pid, current_angle, dt);
            
            // 控制电机
            L9110_SetMotor(motor_output);
            
            // 一些帮助文本显示，限制刷新率，提高效率
            if (current_time - last_display_time >= 300) {
                Update_Info_Display(current_angle, target_angle, motor_output);
                last_display_time = current_time;
            }
            
            // 跟新波形图
            Update_PID_Graph(current_angle, target_angle, motor_output);
        }
        
        // 少量延时减少cpu负担
        HAL_Delay(5);
    }
} 

