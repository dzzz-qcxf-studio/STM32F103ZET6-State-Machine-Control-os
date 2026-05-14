/**
 * @file    mpu_pid.c
 * @brief   MPU6050 + PID + L9110电机控制模块
 *
 * MPU6050互补滤波姿态估计 + PID控制L9110电机 + LCD波形显示。
 * 模块化改造：提供 enter/tick/exit 接口，非阻塞运行。
 *
 * 硬件：
 *   MPU6050: 软件I2C (PB6=SCL, PB7=SDA)
 *   L9110 A-IA: TIM3 CH1 (PA6)
 *   L9110 A-IB: TIM2 CH1 (PA0)
 */

#include "./OVER/mpu_pid.h"
#include "./OVER/mpu6050.h"
#include "./OVER/module.h"
#include "./OVER/resource.h"
#include "./OVER/menu.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/lcd_widgets.h"
#include "./BSP/TOUCH/touch.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <math.h>

/* ======================== PID结构体 ======================== */
/**
 * PID控制器结构体
 * 采用"微分先行"(Derivative on Measurement)形式，避免设定值突变引起的微分冲击。
 *
 * kp / ki / kd : 比例、积分、微分三项增益系数
 * setpoint     : 目标角度(单位：度)，由键盘按键或外部设定
 * integral     : 积分累加器，用于消除稳态误差；在死区或抗饱和条件下会被清零
 * prev_input   : 上一次的输入值(角度)，用于微分项计算
 *                使用"微分作用于测量值"而非"微分作用于误差"，
 *                公式: d_term = -kd * (input - prev_input) / dt，
 *                这样当设定值突变时不会产生微分尖峰
 * output_min/max: 输出限幅范围 [-100, +100]，对应 PWM 占空比百分比
 */
typedef struct {
    float kp, ki, kd;       /* PID三项增益系数 */
    float setpoint;         /* 目标角度(度) */
    float integral;         /* 积分累加器 */
    float prev_input;       /* 上一次输入值，用于微分先行计算 */
    float output_min, output_max; /* 输出限幅范围 */
} PID_t;

/* ======================== 布局常量 ======================== */
/* LCD屏幕上的UI布局参数，用于波形图和信息区域的定位 */
#define INFO_Y          5               /* 信息区域Y起始坐标(未使用) */
#define GRAPH_HEIGHT    260             /* 波形图高度(像素) */
#define GRAPH_WIDTH     360             /* 波形图宽度(像素)，即滚动缓冲区容量 */
#define GRAPH_X         35              /* 波形图左边缘X坐标，留出Y轴标注空间 */
#define GRAPH_Y         (GRAPH_HEIGHT + 70) /* 波形图底边Y坐标 */
#define INFO_AREA_Y     20              /* PID参数/角度信息显示区域Y起始坐标 */

/* ======================== 模块状态 ======================== */
/* 以下为模块运行时的全局状态变量 */

static PID_t pid;               /* PID控制器实例 */
static float current_angle = 0.0f; /* 当前MPU6050测量的倾斜角度(度) */
static float target_angle  = 0.0f; /* 用户设定的目标角度(度) */
static float motor_output  = 0.0f; /* PID计算输出值，范围[-100, +100]，映射到PWM占空比 */

/* L9110电机驱动双H桥的两个定时器句柄 */
static TIM_HandleTypeDef htim_motor_a;  /* TIM3 CH1 (PA6) — 控制正转 */
static TIM_HandleTypeDef htim_motor_b;  /* TIM2 CH1 (PA0) — 控制反转 */

static uint32_t last_pid_tick   = 0; /* 上一次PID计算的时间戳(ms)，用于控制周期 */
static uint32_t last_disp_tick  = 0; /* 上一次显示刷新的时间戳(ms)，用于降低刷新频率 */
static uint8_t  hw_inited       = 0; /* 硬件初始化标志，防止重复初始化GPIO/定时器 */

/* ======================== PID计算 ======================== */
/**
 * PID_Init_Local() — 初始化PID控制器参数
 * @kp: 比例增益，决定响应速度，越大响应越快但可能振荡
 * @ki: 积分增益，消除稳态误差，过大会引起超调和积分饱和
 * @kd: 微分增益，抑制振荡，提供阻尼作用
 *
 * 初始状态：设定值=0，积分=0，前次输入=0
 * 输出限幅：[-100, +100]，对应PWM百分比
 */
static void PID_Init_Local(float kp, float ki, float kd)
{
    pid.kp = kp;  pid.ki = ki;  pid.kd = kd;  /* 设置PID三项增益 */
    pid.setpoint   = 0.0f;   /* 目标角度初始为0(水平位置) */
    pid.integral   = 0.0f;   /* 积分累加器清零 */
    pid.prev_input = 0.0f;   /* 前次输入清零 */
    pid.output_min = -100.0f; /* 输出下限：-100%占空比(反转最大) */
    pid.output_max =  100.0f; /* 输出上限：+100%占空比(正转最大) */
}

/**
 * PID_Compute() — 执行一次PID计算
 * @input: 当前测量值(角度，单位：度)
 * @dt:    距上次计算的时间间隔(单位：秒)
 *
 * 算法流程：
 *   1. 计算误差 = 设定值 - 测量值
 *   2. 积分死区：当误差绝对值 < 0.5度时，清零积分累加器
 *      ——避免小误差持续累积导致过冲，提高系统稳定性
 *   3. 计算P、I、D三项：
 *      P项 = kp * error (比例响应)
 *      D项 = -kd * (input - prev_input) / dt (微分先行，作用于测量值)
 *            ——负号是因为微分先行公式，避免设定值突变时微分尖峰
 *      I项 = ki * integral (积分累积)
 *   4. 输出 = P + I + D，并进行限幅
 *   5. 抗积分饱和(Anti-windup)：
 *      只有当输出未饱和时才累积积分项
 *      ——防止输出已经到达±100%时积分继续增长，导致退饱和延迟
 *   6. 保存当前输入为prev_input，供下次微分计算使用
 */
static float PID_Compute(float input, float dt)
{
    if (dt <= 0) return 0;  /* 防御性检查：时间间隔必须为正 */

    float error = pid.setpoint - input;  /* 计算误差：目标值 - 当前值 */

    /* 积分死区：误差小于0.5度时清零积分，防止小偏差持续累积 */
    if (fabsf(error) < 0.5f)
        pid.integral = 0.0f;

    /* 计算PID三项 */
    float p_term = pid.kp * error;                          /* 比例项：误差越大，输出越大 */
    float d_term = -pid.kd * (input - pid.prev_input) / dt; /* 微分项(微分先行)：抑制变化率 */
    float i_term = pid.ki * pid.integral;                    /* 积分项：消除稳态误差 */
    float output = p_term + i_term + d_term;                 /* 三项求和得到总输出 */

    /* 抗积分饱和(Anti-windup)：
     * 当输出已经饱和(超出限幅范围)时，不再累积积分项，
     * 避免积分项无限增长导致退饱和时间过长 */
    if (output > pid.output_max) {
        output = pid.output_max;       /* 上限限幅 */
    } else if (output < pid.output_min) {
        output = pid.output_min;       /* 下限限幅 */
    } else {
        pid.integral += error * dt;    /* 仅在未饱和时累积积分项 */
    }

    pid.prev_input = input;  /* 保存当前输入，供下次微分计算使用 */
    return output;
}

/* ======================== L9110电机 ======================== */
/**
 * L9110_Init_Local() — 初始化L9110双H桥电机驱动的PWM输出
 *
 * L9110电机驱动模块工作原理：
 *   A-IA (PA6/TIM3-CH1) 和 A-IB (PA0/TIM2-CH1) 两个输入控制一个电机
 *   - A-IA有PWM、A-IB为低 → 电机正转
 *   - A-IB有PWM、A-IA为低 → 电机反转
 *   - 两个都为低 → 电机停止/制动
 *
 * 定时器配置：
 *   预分频器 = 72-1 → 计数时钟 = 72MHz / 72 = 1MHz (1us计数一次)
 *   自动重装载值 = 1000-1 → PWM周期 = 1000us = 1kHz
 *   因此比较值范围 [0, 1000]，直接对应占空比千分比
 *
 * 使用资源管理器(pin_request/tim_request)申请GPIO和定时器资源，
 * 防止与其他模块冲突。
 */
static void L9110_Init_Local(void)
{
    if (hw_inited) return;  /* 防止重复初始化 */

    /* 向资源管理器申请GPIO引脚和定时器，模块标识为"MPU_PID" */
    pin_request(GPIOA, GPIO_PIN_6, "MPU_PID");  /* PA6: TIM3_CH1, 控制电机A-IA */
    pin_request(GPIOA, GPIO_PIN_0, "MPU_PID");  /* PA0: TIM2_CH1, 控制电机A-IB */
    tim_request(TIM3, "MPU_PID");                /* TIM3: 产生A-IA的PWM */
    tim_request(TIM2, "MPU_PID");                /* TIM2: 产生A-IB的PWM */

    /* 使能GPIOA、TIM3、TIM2的时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* 配置PA6和PA0为复用推挽输出(定时器PWM输出模式) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_6 | GPIO_PIN_0;   /* 同时配置两个引脚 */
    gpio.Mode  = GPIO_MODE_AF_PP;            /* 复用推挽输出 */
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;       /* 高速模式 */
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PWM输出通道配置：PWM模式1，初始占空比=0，高电平有效 */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;          /* PWM模式1：CNT < CCR时输出高电平 */
    oc.Pulse      = 0;                         /* 初始比较值=0，占空比=0% */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;       /* 高电平有效 */
    oc.OCFastMode = TIM_OCFAST_DISABLE;        /* 禁用快速模式 */

    /* --- TIM3 初始化：控制Motor A正转 (PA6) --- */
    htim_motor_a.Instance = TIM3;
    htim_motor_a.Init.Prescaler         = 72 - 1;               /* 预分频: 72MHz/72=1MHz */
    htim_motor_a.Init.CounterMode       = TIM_COUNTERMODE_UP;    /* 向上计数模式 */
    htim_motor_a.Init.Period            = 1000 - 1;              /* 周期: 1000个计数=1kHz PWM */
    htim_motor_a.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;/* 不分频 */
    htim_motor_a.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; /* 使能自动重装载预装 */
    HAL_TIM_PWM_Init(&htim_motor_a);
    HAL_TIM_PWM_ConfigChannel(&htim_motor_a, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim_motor_a, TIM_CHANNEL_1);            /* 启动PWM输出 */

    /* --- TIM2 初始化：控制Motor B反转 (PA0) --- */
    htim_motor_b.Instance = TIM2;
    htim_motor_b.Init.Prescaler         = 72 - 1;               /* 预分频: 72MHz/72=1MHz */
    htim_motor_b.Init.CounterMode       = TIM_COUNTERMODE_UP;    /* 向上计数模式 */
    htim_motor_b.Init.Period            = 1000 - 1;              /* 周期: 1000个计数=1kHz PWM */
    htim_motor_b.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;/* 不分频 */
    htim_motor_b.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; /* 使能自动重装载预装 */
    HAL_TIM_PWM_Init(&htim_motor_b);
    HAL_TIM_PWM_ConfigChannel(&htim_motor_b, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim_motor_b, TIM_CHANNEL_1);            /* 启动PWM输出 */

    /* 初始占空比设为0，确保电机上电时静止 */
    __HAL_TIM_SET_COMPARE(&htim_motor_a, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim_motor_b, TIM_CHANNEL_1, 0);

    hw_inited = 1;  /* 标记硬件已初始化，防止重复初始化 */
}

/**
 * L9110_SetMotor() — 设置L9110电机转速和方向
 * @speed: 速度值，范围[-100, +100]
 *         正值 → 正转(TIM3/PA6输出PWM，TIM2/PA0保持低)
 *         负值 → 反转(TIM2/PA0输出PWM，TIM3/PA6保持低)
 *         0    → 停止(两个通道都输出0)
 *
 * 映射关系：|speed| * 10 → PWM比较值 [0, 1000]
 * 例如：speed=50 → pwm=500 → 50%占空比正转
 */
static void L9110_SetMotor(float speed)
{
    /* 限幅到 [-100, +100] 范围 */
    if (speed > 100.0f)  speed = 100.0f;
    if (speed < -100.0f) speed = -100.0f;

    /* 将速度百分比映射到PWM比较值：|speed| / 100 * 1000 */
    uint32_t pwm = (uint32_t)(fabsf(speed) * 1000.0f / 100.0f);

    /* 先将两个通道都置零(互斥，同一时刻只有一个通道输出PWM) */
    __HAL_TIM_SET_COMPARE(&htim_motor_a, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim_motor_b, TIM_CHANNEL_1, 0);

    /* 根据方向选择对应的通道输出PWM */
    if (speed >= 0)
        __HAL_TIM_SET_COMPARE(&htim_motor_a, TIM_CHANNEL_1, pwm); /* 正转：TIM3(PA6)输出 */
    else
        __HAL_TIM_SET_COMPARE(&htim_motor_b, TIM_CHANNEL_1, pwm); /* 反转：TIM2(PA0)输出 */
}

/* ======================== MPU6050互补滤波 ======================== */
/**
 * Get_MPU_Angle() — 使用互补滤波融合加速度计和陀螺仪数据，计算倾斜角度
 *
 * 互补滤波原理：
 *   陀螺仪：积分得到角度，短期精度高，但长期会漂移
 *   加速度计：通过重力分量计算角度，长期稳定，但受振动/加速度干扰
 *   互补滤波将两者结合：高通(陀螺仪) + 低通(加速度计)
 *
 * 公式: angle = 0.98 * (angle + gyro_rate * dt) + 0.02 * accel_angle
 *   - 0.98权重给陀螺仪积分路径(高频/短期可信)
 *   - 0.02权重给加速度计(低频/长期修正漂移)
 *   - 时间常数约 tau = RC = alpha/(1-alpha) * dt ≈ 几秒
 *
 * 陀螺仪灵敏度：±250°/s量程下，LSB = 131 LSB/(°/s)
 *   实际代码用16.4，可能是±2000°/s量程(灵敏度=16.4 LSB/(°/s))
 *   或者是实验调校值
 */
static float Get_MPU_Angle(void)
{
    short ax, ay, az, gx, gy, gz;  /* 原始传感器数据：加速度(16bit)、角速度(16bit) */
    static float angle = 0;         /* 融合后的角度(静态变量，跨调用保持) */
    static uint32_t last_time = 0;  /* 上次采样时间戳 */

    /* 读取MPU6050六轴原始数据 */
    MPU_Get_Accelerometer(&ax, &ay, &az);  /* 加速度计：ax/ay/az */
    MPU_Get_Gyroscope(&gx, &gy, &gz);      /* 陀螺仪：gx/gy/gz */

    /* 计算时间间隔dt(秒)，用于陀螺仪积分 */
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) / 1000.0f;
    if (dt <= 0) dt = 0.001f;  /* 防御性检查，避免dt<=0 */
    last_time = now;

    /* 加速度计角度：通过atan2(ay, sqrt(ax^2+az^2))计算绕X轴的倾斜角
     * 这里用ay(前后方向)和ax/az平面的合力来求倾角
     * 结果转换为度(atan2返回弧度) */
    float accel_angle = atan2f((float)ay, sqrtf((float)ax * ax + (float)az * az)) * 180.0f / 3.14159f;

    /* 陀螺仪角速度：gx原始值除以灵敏度得到°/s */
    float gyro_rate  = (float)gx / 16.4f;

    /* 互补滤波融合：
     * 98%权重用陀螺仪积分(短期精确)，2%权重用加速度计修正(长期稳定)
     * 这样既避免了加速度计噪声，又抑制了陀螺仪漂移 */
    angle = 0.98f * (angle + gyro_rate * dt) + 0.02f * accel_angle;

    return angle;  /* 返回融合后的倾斜角度(度) */
}

/* ======================== 波形显示 ======================== */
/**
 * Draw_Graph_Layout() — 绘制波形图的静态框架
 *
 * 布局结构：
 *   左侧Y轴标注角度刻度：+45°(顶) / 0°(中) / -45°(底)
 *   右侧Y轴标注输出百分比：+100%(顶) / -100%(底)
 *   中间水平虚线为零线(0°参考线)
 *   底部图例：Angle=红色, Target=蓝色, Output=绿色
 *
 * 整个图表背景为黑色，网格线为灰色(0x7BEF)，便于区分波形
 */
static void Draw_Graph_Layout(void)
{
    uint32_t old_bg = g_back_color;  /* 保存原始背景色，绘制完后恢复 */
    g_back_color = 0x0000;           /* 设置背景色为黑色 */

    /* 用黑色填充整个图表区域(清除旧内容) */
    lcd_fill(0, GRAPH_Y - GRAPH_HEIGHT, GRAPH_X + GRAPH_WIDTH + 40, GRAPH_Y + 20, 0x0000);

    /* 绘制坐标轴：X轴(底部水平线)和Y轴(左侧垂直线)，颜色=灰色(0x7BEF) */
    lcd_draw_line(GRAPH_X, GRAPH_Y, GRAPH_X + GRAPH_WIDTH, GRAPH_Y, 0x7BEF);           /* X轴 */
    lcd_draw_line(GRAPH_X, GRAPH_Y, GRAPH_X, GRAPH_Y - GRAPH_HEIGHT, 0x7BEF);           /* Y轴 */
    lcd_draw_line(GRAPH_X, GRAPH_Y - GRAPH_HEIGHT / 2, GRAPH_X + GRAPH_WIDTH, GRAPH_Y - GRAPH_HEIGHT / 2, 0x2104); /* 零线(深灰色) */

    /* 左侧Y轴刻度标注：角度值 */
    lcd_show_string(0, GRAPH_Y - GRAPH_HEIGHT, 20, 16, 16, "+45", 0x7BEF);    /* 顶：+45度 */
    lcd_show_string(0, GRAPH_Y - GRAPH_HEIGHT / 2 - 8, 20, 16, 16, "0", 0x7BEF); /* 中：0度 */
    lcd_show_string(0, GRAPH_Y - 16, 20, 16, 16, "-45", 0x7BEF);              /* 底：-45度 */

    /* 右侧Y轴刻度标注：PWM输出百分比(角度和输出共用Y轴，但量程不同) */
    lcd_show_string(GRAPH_X + GRAPH_WIDTH + 2, GRAPH_Y - GRAPH_HEIGHT, 38, 16, 16, "+100%", 0x7BEF); /* 顶：+100% */
    lcd_show_string(GRAPH_X + GRAPH_WIDTH + 2, GRAPH_Y - 16, 38, 16, 16, "-100%", 0x7BEF);           /* 底：-100% */

    /* 图例(Legend)：用彩色线段+文字标识三条波形 */
    lcd_draw_line(GRAPH_X, GRAPH_Y + 15, GRAPH_X + 20, GRAPH_Y + 15, WGT_CLR_KEY_NEG);   /* 红色线段 */
    lcd_show_string(GRAPH_X + 25, GRAPH_Y + 10, 40, 16, 16, "Angle", WGT_CLR_KEY_NEG);   /* "Angle" = 当前角度 */
    lcd_draw_line(GRAPH_X + 70, GRAPH_Y + 15, GRAPH_X + 90, GRAPH_Y + 15, WGT_CLR_ACCENT); /* 蓝色线段 */
    lcd_show_string(GRAPH_X + 95, GRAPH_Y + 10, 40, 16, 16, "Target", WGT_CLR_ACCENT);    /* "Target" = 目标角度 */
    lcd_draw_line(GRAPH_X + 150, GRAPH_Y + 15, GRAPH_X + 170, GRAPH_Y + 15, 0x07E0);      /* 绿色线段 */
    lcd_show_string(GRAPH_X + 175, GRAPH_Y + 10, 40, 16, 16, "Output", 0x07E0);            /* "Output" = PID输出 */

    g_back_color = old_bg;  /* 恢复原始背景色 */
}

/**
 * Update_Graph() — 滚动波形显示更新
 * @current: 当前角度值
 * @target:  目标角度值
 * @output:  PID输出值
 *
 * 实现原理：
 *   使用循环缓冲区(长度=GRAPH_WIDTH=360)存储历史数据。
 *   每次调用在当前列位置绘制新的波形点，并擦除旧列内容。
 *   idx以取模方式递增，实现滚动效果。
 *
 * 坐标映射：
 *   角度 ±45° → 映射到图表全高度(±GRAPH_HEIGHT/2)
 *   输出 ±100% → 映射到图表全高度
 *   公式: pixel_y = GRAPH_Y - (value/scale * half_height + half_height)
 *   CLAMP确保像素坐标不超出图表区域
 *
 * 绘制策略：
 *   1. 先擦除当前列(白色填充)
 *   2. 每隔40像素重绘垂直网格线
 *   3. 从前一列到当前列画线段(连接相邻采样点)
 *   三条波形分别用RED(角度)、BLUE(目标)、GREEN(输出)绘制
 */
static void Update_Graph(float current, float target, float output)
{
    static uint16_t idx = 0;                    /* 当前写入列索引(循环递增) */
    static float hist_angle[GRAPH_WIDTH]  = {0}; /* 角度历史缓冲区 */
    static float hist_target[GRAPH_WIDTH] = {0}; /* 目标值历史缓冲区 */
    static float hist_output[GRAPH_WIDTH] = {0}; /* 输出值历史缓冲区 */
    static uint8_t inited = 0;                   /* 首次调用标志 */

    /* 首次调用：绘制图表框架，不绘制数据(此时无历史数据) */
    if (!inited) {
        Draw_Graph_Layout();
        inited = 1;
        return;
    }

    /* 计算前一列索引(用于绘制连接线段)，处理循环回绕 */
    uint16_t prev = (idx == 0) ? GRAPH_WIDTH - 1 : idx - 1;

    /* 将当前采样值存入循环缓冲区 */
    hist_angle[idx]  = current;
    hist_target[idx] = target;
    hist_output[idx] = output;

    /* 计算当前列和前一列的X像素坐标 */
    uint16_t x  = GRAPH_X + idx;   /* 当前列X坐标 */
    uint16_t px = GRAPH_X + prev;  /* 前一列X坐标 */

    /* 值→像素Y坐标转换宏：
     * CALC_Y(v, scale): 将值v按量程scale映射到像素Y坐标
     *   v/scale归一化到[-1,1]，乘以半高得到像素偏移，加上半高得到中心偏移
     *   用GRAPH_Y减去偏移(因为屏幕Y轴向下，数值向上)
     * CLAMP(y): 将Y坐标限制在图表区域内 */
    #define CALC_Y(v, scale) (GRAPH_Y - (int)((v) / (scale) * (GRAPH_HEIGHT / 2) + GRAPH_HEIGHT / 2))
    #define CLAMP(y) do { if (y < GRAPH_Y - GRAPH_HEIGHT + 1) y = GRAPH_Y - GRAPH_HEIGHT + 1; if (y > GRAPH_Y - 1) y = GRAPH_Y - 1; } while(0)

    /* 计算当前列和前一列各波形的Y坐标，并限幅 */
    int yc = CALC_Y(hist_angle[idx], 45.0f);   CLAMP(yc);   /* 当前角度Y(量程±45°) */
    int yt = CALC_Y(hist_target[idx], 45.0f);  CLAMP(yt);   /* 当前目标Y(量程±45°) */
    int yo = CALC_Y(hist_output[idx], 100.0f); CLAMP(yo);   /* 当前输出Y(量程±100%) */
    int pyc = CALC_Y(hist_angle[prev], 45.0f);   CLAMP(pyc); /* 前一列角度Y */
    int pyt = CALC_Y(hist_target[prev], 45.0f);  CLAMP(pyt); /* 前一列目标Y */
    int pyo = CALC_Y(hist_output[prev], 100.0f); CLAMP(pyo); /* 前一列输出Y */

    /* 擦除当前列：用白色填充清除旧波形(宽度10像素，留出绘制余量) */
    lcd_fill(x, GRAPH_Y - GRAPH_HEIGHT - 1, x + 10, GRAPH_Y - 1, WHITE);

    /* 重绘被擦除的网格线：每隔40像素画一条垂直灰色网格线 */
    if (idx > 0 && idx % 40 == 0)
        lcd_draw_line(x, GRAPH_Y - GRAPH_HEIGHT + 1, x, GRAPH_Y - 1, GRAY);
    /* 重绘零线上的网格点 */
    lcd_draw_point(x, GRAPH_Y - GRAPH_HEIGHT / 2, GRAY);

    /* 绘制从(前一列, 前值)到(当前列, 当前值)的线段，形成连续波形 */
    if (idx > 0) {
        lcd_draw_line(px, pyc, x, yc, RED);    /* 红色：当前角度波形 */
        lcd_draw_line(px, pyt, x, yt, BLUE);   /* 蓝色：目标角度波形 */
        lcd_draw_line(px, pyo, x, yo, GREEN);  /* 绿色：PID输出波形 */
    }

    /* 索引递增并取模，实现循环滚动 */
    idx = (idx + 1) % GRAPH_WIDTH;
}

/**
 * Update_Info() — 条件刷新LCD信息显示区
 * @current: 当前角度
 * @target:  目标角度
 * @output:  PID输出
 *
 * 优化策略：只有当数值变化超过阈值时才重绘对应区域
 *   - 角度/目标：变化 > 0.1° 才刷新(避免微小抖动频繁重绘)
 *   - 输出：变化 > 1% 才刷新
 *   - PID参数：任一参数变化超过阈值才刷新整行
 *
 * 这种"脏矩形"策略显著减少LCD全屏重绘带来的闪烁，
 * 同时降低CPU占用(300ms周期调用，但实际重绘频率远低于此)
 *
 * 显示布局：
 *   第一行：Ang:xx.x | Tgt:xx.x | Out:xxx% (颜色分别对应红/蓝/白)
 *   第二行：P:x.x I:x.xxx D:x.x (灰色，PID参数)
 */
static void Update_Info(float current, float target, float output)
{
    /* 上一次显示的值，初始化为不可能的值以确保首次必定刷新 */
    static float lc = -999, lt = -999, lo = -999, lkp = -1, lki = -1, lkd = -1;
    char buf[40];
    uint32_t old_bg = g_back_color;
    g_back_color = WGT_CLR_BG;  /* 设置背景色为小组件背景色 */

    /* 第一行：角度 | 目标 | 输出 */

    /* 角度显示：变化超过0.1°才重绘(红色) */
    if (fabsf(current - lc) > 0.1f) {
        snprintf(buf, sizeof(buf), "Ang:%.1f", current);
        lcd_fill(14, INFO_AREA_Y, 120, INFO_AREA_Y + 16, WGT_CLR_BG);  /* 先清除旧文字 */
        lcd_show_string(14, INFO_AREA_Y, 106, 16, 16, buf, WGT_CLR_KEY_NEG); /* 红色显示 */
        lc = current;  /* 记录已显示的值 */
    }
    /* 目标角度显示：变化超过0.1°才重绘(蓝色) */
    if (fabsf(target - lt) > 0.1f) {
        snprintf(buf, sizeof(buf), "Tgt:%.1f", target);
        lcd_fill(130, INFO_AREA_Y, 240, INFO_AREA_Y + 16, WGT_CLR_BG);
        lcd_show_string(130, INFO_AREA_Y, 110, 16, 16, buf, WGT_CLR_ACCENT); /* 蓝色显示 */
        lt = target;
    }
    /* 输出显示：变化超过1%才重绘(白色/主色) */
    if (fabsf(output - lo) > 1.0f) {
        snprintf(buf, sizeof(buf), "Out:%.0f%%", output);
        lcd_fill(250, INFO_AREA_Y, 360, INFO_AREA_Y + 16, WGT_CLR_BG);
        lcd_show_string(250, INFO_AREA_Y, 110, 16, 16, buf, WGT_CLR_TEXT_PRI); /* 主色显示 */
        lo = output;
    }

    /* 第二行：PID参数(任一参数变化超过阈值则整行重绘) */
    if (fabsf(pid.kp - lkp) > 0.01f || fabsf(pid.ki - lki) > 0.001f || fabsf(pid.kd - lkd) > 0.01f) {
        snprintf(buf, sizeof(buf), "P:%.1f I:%.3f D:%.1f", pid.kp, pid.ki, pid.kd);
        lcd_fill(14, INFO_AREA_Y + 20, 360, INFO_AREA_Y + 36, WGT_CLR_BG);
        lcd_show_string(14, INFO_AREA_Y + 20, 340, 16, 16, buf, WGT_CLR_TEXT_SEC); /* 次色显示 */
        lkp = pid.kp; lki = pid.ki; lkd = pid.kd;  /* 记录已显示的PID参数 */
    }

    g_back_color = old_bg;  /* 恢复原始背景色 */
}

/* ======================== 键盘输入处理 ======================== */
/**
 * Handle_Key() — 处理键盘按键输入，调整PID参数和目标角度
 * @key: 按键字符(来自detect_key_press())
 *
 * 按键映射表：
 *   按键  功能            步进值      范围限制
 *   '1'   Kp 增大         +0.5        无上限
 *   '4'   Kp 减小         -0.5        >= 0
 *   '2'   Ki 增大         +0.1        无上限
 *   '5'   Ki 减小         -0.1        >= 0
 *   '3'   Kd 增大         +0.2        无上限
 *   '6'   Kd 减小         -0.2        >= 0
 *   '7'   目标角度增大     +5°         <= +45°
 *   '9'   目标角度减小     -5°         >= -45°
 *   '8'   重置所有参数     —           目标=0，清积分，停电机
 *   'n'   退出当前模块(返回菜单)
 *
 * 调参顺序建议(屏幕提示)：1/4调P，2/5调I，3/6调D，7/9调目标角度，8重置
 * 每次按键后立即刷新信息显示区
 */
static void Handle_Key(char key)
{
    switch (key) {
    /* --- PID增益系数调整 --- */
    case '1': pid.kp += 0.5f; break;                                    /* Kp+0.5 */
    case '4': pid.kp -= 0.5f; if (pid.kp < 0) pid.kp = 0; break;       /* Kp-0.5，不低于0 */
    case '2': pid.ki += 0.1f; break;                                    /* Ki+0.1 */
    case '5': pid.ki -= 0.1f; if (pid.ki < 0) pid.ki = 0; break;       /* Ki-0.1，不低于0 */
    case '3': pid.kd += 0.2f; break;                                    /* Kd+0.2 */
    case '6': pid.kd -= 0.2f; if (pid.kd < 0) pid.kd = 0; break;       /* Kd-0.2，不低于0 */

    /* --- 目标角度调整(限幅±45°) --- */
    case '7': target_angle += 5.0f; if (target_angle > 45) target_angle = 45; pid.setpoint = target_angle; break; /* 目标+5° */
    case '9': target_angle -= 5.0f; if (target_angle < -45) target_angle = -45; pid.setpoint = target_angle; break; /* 目标-5° */

    /* --- 重置：清零所有状态，停止电机 --- */
    case '8':
        target_angle = 0; pid.setpoint = 0;    /* 目标角度归零 */
        pid.integral = 0; pid.prev_input = 0;   /* 清除积分和前次输入 */
        L9110_SetMotor(0);                       /* 停止电机 */
        break;
    /* --- 退出模块 --- */
    case 'n':
        module_exit_current();                   /* 通知状态机退出当前模块 */
        return;                                  /* 直接返回，不刷新显示 */
    default: break;
    }
    /* 按键处理后立即刷新信息显示(不等待300ms定时) */
    Update_Info(current_angle, target_angle, motor_output);
}

/* ======================== 模块接口 ======================== */
/**
 * mpu_pid_enter() — 模块初始化入口(进入MPU PID控制模式时调用)
 *
 * 初始化流程：
 *   1. 初始化MPU6050传感器(通过I2C)，失败则显示错误信息并返回
 *   2. 初始化L9110电机驱动(PWM定时器配置)
 *   3. 初始化PID控制器，默认参数：
 *      Kp=1.5(比例增益)、Ki=0.08(积分增益)、Kd=2.0(微分增益)
 *   4. 绘制LCD界面：标题栏、操作提示、波形图框架、信息显示区
 *   5. 记录初始时间戳，用于后续定时控制
 *
 * 默认PID参数说明：
 *   Kp=1.5  —— 中等比例响应，不会过于激进
 *   Ki=0.08 —— 较小积分增益，缓慢消除稳态误差，避免积分饱和
 *   Kd=2.0  —— 较大微分增益，提供较强阻尼，抑制振荡
 */
static void mpu_pid_enter(void)
{
    /* 第一步：初始化MPU6050传感器 */
    if (MPU_Init()) {
        /* 初始化失败(可能是I2C通信错误或传感器未连接) */
        lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WGT_CLR_BG);
        lcd_wgt_header("MPU PID CONTROL", 0, 0);
        lcd_wgt_status_label(14, 80, "MPU6050 Error!", 2);  /* 红色错误提示 */
        return;  /* 初始化失败，不继续后续操作 */
    }

    /* 第二步：初始化L9110电机驱动硬件 */
    L9110_Init_Local();

    /* 第三步：初始化PID控制器，默认参数 */
    PID_Init_Local(1.5f, 0.08f, 2.0f);  /* Kp=1.5, Ki=0.08, Kd=2.0 */
    target_angle = 0;  /* 目标角度初始为0(水平) */

    /* 第四步：绘制LCD界面 */
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WGT_CLR_BG);  /* 清屏 */
    lcd_wgt_header("MPU PID CONTROL", 0, 0);  /* 绘制标题栏 */
    /* 显示操作提示(按键功能说明) */
    lcd_show_string(14, MENU_AREA_HEIGHT - 24, 400, 16, 16, "1/4:P 2/5:I 3/6:D 7/9:Angle 8:Reset", WGT_CLR_KEY_ACT);

    /* 绘制波形图框架和初始信息 */
    Draw_Graph_Layout();
    Update_Info(current_angle, target_angle, motor_output);

    /* 第五步：记录初始时间戳(用于PID控制周期和显示刷新定时) */
    last_pid_tick  = HAL_GetTick();
    last_disp_tick = HAL_GetTick();
}

/**
 * mpu_pid_tick() — 模块主循环(由状态机周期性调用)
 *
 * 执行逻辑：
 *   1. 检测键盘输入，有按键则调用Handle_Key处理
 *   2. 每16ms(~60Hz)执行一次PID控制循环：
 *      a. 读取MPU6050当前角度(互补滤波)
 *      b. PID计算得到电机输出
 *      c. 设置L9110电机PWM
 *      d. 更新波形图
 *   3. 每300ms刷新一次信息显示区(角度/目标/PID参数)
 *
 * 两个定时器独立运行：
 *   PID控制周期16ms(60Hz) —— 保证控制响应速度和波形平滑
 *   显示刷新周期300ms(~3Hz) —— 降低LCD刷新频率，减少闪烁和CPU占用
 */
static void mpu_pid_tick(void)
{
    /* 第一步：检测并处理键盘输入 */
    int key = detect_key_press();
    if (key != EVT_NONE)
        Handle_Key((char)key);  /* 有按键时处理参数调整或退出 */

    /* 第二步：PID控制循环(16ms周期，约60Hz) */
    uint32_t now = HAL_GetTick();
    if (now - last_pid_tick >= 16) {
        float dt = (now - last_pid_tick) / 1000.0f;  /* 计算实际时间间隔(秒) */
        last_pid_tick = now;                          /* 更新上次PID时间戳 */

        /* a. 读取MPU6050传感器并进行互补滤波融合，得到当前倾斜角度 */
        current_angle = Get_MPU_Angle();

        /* b. PID计算：根据当前角度和目标角度，计算电机控制输出 */
        motor_output  = PID_Compute(current_angle, dt);

        /* c. 将PID输出映射为PWM信号驱动电机 */
        L9110_SetMotor(motor_output);

        /* d. 更新波形图(角度/目标/输出三条曲线) */
        Update_Graph(current_angle, target_angle, motor_output);
    }

    /* 第三步：信息显示刷新(300ms周期，约3Hz) */
    if (now - last_disp_tick >= 300) {
        Update_Info(current_angle, target_angle, motor_output);  /* 条件刷新LCD信息 */
        last_disp_tick = now;                                    /* 更新上次刷新时间戳 */
    }
}

/**
 * mpu_pid_exit() — 模块清理退出(离开MPU PID控制模式时调用)
 *
 * 清理流程：
 *   1. 立即停止电机(安全第一)
 *   2. 停止PWM定时器输出
 *   3. 释放GPIO引脚和定时器资源(供其他模块使用)
 *   4. 清除硬件初始化标志
 *
 * 资源释放顺序：先停PWM → 再释放引脚/定时器
 * 确保不会出现"定时器还在输出PWM但引脚已被释放"的危险状态
 */
static void mpu_pid_exit(void)
{
    /* 第一步：立即停止电机(安全措施) */
    L9110_SetMotor(0);

    /* 第二步：停止两个定时器的PWM输出 */
    HAL_TIM_PWM_Stop(&htim_motor_a, TIM_CHANNEL_1);  /* 停止TIM3 CH1(PA6) */
    HAL_TIM_PWM_Stop(&htim_motor_b, TIM_CHANNEL_1);  /* 停止TIM2 CH1(PA0) */

    /* 第三步：释放GPIO引脚和定时器资源到资源管理器 */
    pin_release(GPIOA, GPIO_PIN_6, "MPU_PID");  /* 释放PA6 */
    pin_release(GPIOA, GPIO_PIN_0, "MPU_PID");  /* 释放PA0 */
    tim_release(TIM3, "MPU_PID");                /* 释放TIM3 */
    tim_release(TIM2, "MPU_PID");                /* 释放TIM2 */

    /* 第四步：清除硬件初始化标志，允许下次进入时重新初始化 */
    hw_inited = 0;
}

/* ======================== 模块导出 ======================== */
/**
 * 模块接口导出结构体
 * 通过ModuleInterface标准接口，状态机可以统一管理本模块的生命周期：
 *   enter → 初始化硬件、绘制UI
 *   tick  → 周期性执行PID控制和显示更新
 *   exit  → 停止电机、释放资源
 */
const ModuleInterface mpu_pid_module = {
    .name  = "MPU_PID",         /* 模块名称标识 */
    .enter = mpu_pid_enter,     /* 进入模块回调 */
    .tick  = mpu_pid_tick,      /* 主循环回调(周期调用) */
    .exit  = mpu_pid_exit,      /* 退出模块回调 */
};
