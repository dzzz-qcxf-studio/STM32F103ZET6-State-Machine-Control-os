/**
 * @file    servo.c
 * @brief   双舵机控制模块 (SG90, TIM3 CH1/CH2, PA6/PA7)
 *
 * 模块化改造：提供 enter/tick/exit 接口，非阻塞运行。
 */

#include "./OVER/servo.h"
#include "./OVER/module.h"
#include "./OVER/resource.h"
#include "./OVER/menu.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/lcd_widgets.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>

/* ======================== 硬件句柄 ======================== */
/* TIM3 定时器句柄，用于输出两路 PWM 信号驱动 SG90 舵机 */
static TIM_HandleTypeDef htim_servo;

/* ======================== 模块状态 ======================== */
/* 舵机1当前角度（0~180），初始居中 90 度，对应 TIM3 CH1 (PA6) */
static uint8_t servo1_angle = 90;
/* 舵机2当前角度（0~180），初始居中 90 度，对应 TIM3 CH2 (PA7) */
static uint8_t servo2_angle = 90;
/* 每次按键的角度步进值，默认 5 度，可通过 +/- 键在 5~40 之间调节 */
static uint8_t step = 5;
/* 硬件初始化标志：0=未初始化，1=已初始化；用于防止重复初始化和判断是否需要释放资源 */
static uint8_t hw_inited = 0;
/* 记录模块进入时的系统 tick 值（毫秒），用于实现启动保护延时 */
static uint32_t enter_tick = 0;

/* ======================== PWM初始化 ======================== */
/**
 * @brief  初始化 TIM3 的两路 PWM 输出，用于驱动 SG90 舵机
 *
 * 硬件连接：
 *   - 舵机1 → PA6 (TIM3_CH1)
 *   - 舵机2 → PA7 (TIM3_CH2)
 *
 * PWM 频率计算：
 *   STM32F103 主频 72MHz，经预分频 144 分频后定时器时钟 = 72MHz / 144 = 500kHz
 *   自动重装载值 10000，故 PWM 频率 = 500kHz / 10000 = 50Hz（周期 20ms）
 *   这正是 SG90 舵机所需的 PWM 频率
 *
 * 脉宽范围：
 *   500kHz 下每个计数 = 2us
 *   最小脉宽 = 250 × 2us = 0.5ms（对应 0 度）
 *   最大脉宽 = 1250 × 2us = 2.5ms（对应 180 度）
 */
static void Servo_PWM_Init(void)
{
    /* 若硬件已初始化则直接返回，避免重复配置 */
    if (hw_inited) return;

    /* ---------- 申请硬件资源（带冲突检测） ---------- */
    /* 向资源管理器注册 PA6 引脚占用，模块名 "Servo"，若引脚已被其他模块占用会报错 */
    pin_request(GPIOA, GPIO_PIN_6, "Servo");
    /* 注册 PA7 引脚占用 */
    pin_request(GPIOA, GPIO_PIN_7, "Servo");
    /* 注册 TIM3 定时器占用 */
    tim_request(TIM3, "Servo");

    /* ---------- 使能外设时钟 ---------- */
    /* 开启 GPIOA 端口时钟，PA6/PA7 挂在 GPIOA 上 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* 开启 TIM3 定时器时钟 */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* ---------- 配置 GPIO 为复用推挽输出 ---------- */
    /* SG90 舵机信号线由定时器 PWM 硬件驱动，故需配置为复用推挽模式 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_6 | GPIO_PIN_7;   /* 同时配置 PA6 和 PA7 */
    gpio.Mode  = GPIO_MODE_AF_PP;            /* 复用推挽输出（Alternate Function Push-Pull） */
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;       /* 高速模式，确保 PWM 波形边沿陡峭 */
    HAL_GPIO_Init(GPIOA, &gpio);

    /* ---------- 配置 TIM3 基本参数 ---------- */
    htim_servo.Instance = TIM3;                          /* 使用 TIM3 定时器 */
    htim_servo.Init.Prescaler         = 143;             /* 预分频 143 → 实际分频 144，72MHz/144=500kHz */
    htim_servo.Init.CounterMode       = TIM_COUNTERMODE_UP; /* 向上计数模式 */
    htim_servo.Init.Period            = 10000 - 1;       /* 自动重装载值 9999，计数 0~9999 共 10000 个计数 */
    htim_servo.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; /* 不分频 */
    htim_servo.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; /* 禁用预装载，立即生效 */
    HAL_TIM_PWM_Init(&htim_servo);

    /* ---------- 配置 PWM 输出通道 ---------- */
    /* 两路通道共用相同的初始配置：750 计数 = 1.5ms 脉宽 = 舵机居中（90 度） */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;         /* PWM 模式 1：CNT < CCR 时输出高电平 */
    oc.Pulse      = 750;                      /* 初始占空比 = 750 计数 = 1.5ms（舵机居中） */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;      /* 有效极性为高电平 */
    oc.OCFastMode = TIM_OCFAST_DISABLE;       /* 禁用快速模式 */
    HAL_TIM_PWM_ConfigChannel(&htim_servo, &oc, TIM_CHANNEL_1); /* 通道 1 → PA6 → 舵机1 */
    HAL_TIM_PWM_ConfigChannel(&htim_servo, &oc, TIM_CHANNEL_2); /* 通道 2 → PA7 → 舵机2 */

    /* ---------- 启动 PWM 输出 ---------- */
    /* 使能两个通道的 PWM 输出，此时舵机开始接收控制信号 */
    HAL_TIM_PWM_Start(&htim_servo, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim_servo, TIM_CHANNEL_2);

    /* 标记硬件已初始化，后续 servo_enter() 不会重复初始化 */
    hw_inited = 1;
}

/* ======================== 角度设置 ======================== */
/**
 * @brief  设置指定舵机的目标角度
 * @param  ch    舵机通道号：1 = TIM3_CH1 (PA6)，2 = TIM3_CH2 (PA7)
 * @param  angle 目标角度，范围 0~180 度
 *
 * 角度到脉宽的线性映射：
 *   0 度 → 250 计数 → 0.5ms 脉宽（SG90 最小脉宽）
 *   90 度 → 750 计数 → 1.5ms 脉宽（舵机居中）
 *   180 度 → 1250 计数 → 2.5ms 脉宽（SG90 最大脉宽）
 *   公式：pulse = 250 + angle * 1000 / 180
 */
static void Servo_SetAngle(uint8_t ch, uint8_t angle)
{
    /* 角度限幅，防止越界导致脉宽超出舵机有效范围 */
    if (angle > 180) angle = 180;
    /* 将角度线性映射为 PWM 计数值（250~1250 对应 0.5ms~2.5ms） */
    uint16_t pulse = 250 + (angle * 1000 / 180);

    /* 根据通道号写入对应的比较寄存器，立即改变 PWM 占空比 */
    if (ch == 1)
        __HAL_TIM_SET_COMPARE(&htim_servo, TIM_CHANNEL_1, pulse); /* 更新 CH1 (PA6) 的脉宽 */
    else
        __HAL_TIM_SET_COMPARE(&htim_servo, TIM_CHANNEL_2, pulse); /* 更新 CH2 (PA7) 的脉宽 */
}

/* ======================== 刷新显示 ======================== */
/**
 * @brief  在 LCD 上刷新显示两个舵机的当前角度和步进值
 *
 * 显示布局（Y 坐标对应 lcd_wgt_header 绘制的界面）：
 *   Y=60  → 舵机1角度（3位数字，高亮色）
 *   Y=110 → 舵机2角度（3位数字，高亮色）
 *   Y=160 → 步进值（2位数字，高亮色）
 *
 * 采用"先清后画"方式：先用背景色填充数字区域（避免残留），再重新绘制数字
 */
static void Servo_UpdateDisplay(void)
{
    /* 用背景色清除三个数值显示区域，防止旧数字残留造成重影 */
    lcd_fill(210, 60, 300, 84, WGT_CLR_BG);    /* 清除舵机1角度区域 */
    lcd_fill(210, 110, 300, 134, WGT_CLR_BG);  /* 清除舵机2角度区域 */
    lcd_fill(210, 160, 300, 184, WGT_CLR_BG);  /* 清除步进值区域 */
    /* 重新绘制三个数值，使用强调色（高亮）显示 */
    lcd_show_num(210, 60,  servo1_angle, 3, 24, WGT_CLR_ACCENT);  /* 显示舵机1角度，3位，24号字体 */
    lcd_show_num(210, 110, servo2_angle, 3, 24, WGT_CLR_ACCENT);  /* 显示舵机2角度，3位，24号字体 */
    lcd_show_num(210, 160, step, 2, 24, WGT_CLR_ACCENT);          /* 显示步进值，2位，24号字体 */
}

/* ======================== 模块接口 ======================== */
/**
 * @brief  模块进入函数 —— 由菜单系统在切换到舵机模块时调用
 *
 * 职责：
 *   1. 初始化 PWM 硬件（仅首次，后续通过 hw_inited 跳过）
 *   2. 将两个舵机角度重置为 90 度居中位置，步进值重置为 5
 *   3. 绘制 LCD 界面：标题栏 + 标签文字 + 数值显示
 *   4. 记录进入时间戳，用于 servo_tick() 中的启动保护
 */
static void servo_enter(void)
{
    /* 初始化 PWM 外设（GPIO + 定时器 + 通道），已初始化则跳过 */
    Servo_PWM_Init();

    /* 重置模块状态：两舵机居中，步进值默认 5 度 */
    servo1_angle = 90;
    servo2_angle = 90;
    step = 5;

    /* 将重置后的角度写入 PWM 比较寄存器，舵机物理转动到居中位置 */
    Servo_SetAngle(1, servo1_angle);
    Servo_SetAngle(2, servo2_angle);

    /* ---------- 绘制 LCD 用户界面 ---------- */
    /* 用背景色清除整个菜单显示区域 */
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WGT_CLR_BG);
    /* 在顶部绘制标题栏："SERVO CONTROL" */
    lcd_wgt_header("SERVO CONTROL", 0, 0);
    /* 绘制三行标签文字，说明各数值的含义和对应引脚 */
    lcd_show_string(50, 60,  200, 24, 24, "Servo1 (PA6):", WGT_CLR_TEXT_PRI); /* 主色：舵机1标签 */
    lcd_show_string(50, 110, 200, 24, 24, "Servo2 (PA7):", WGT_CLR_TEXT_PRI); /* 主色：舵机2标签 */
    lcd_show_string(50, 160, 200, 24, 24, "Step size:",    WGT_CLR_TEXT_SEC); /* 副色：步进值标签 */
    /* 在标签右侧绘制初始数值 */
    Servo_UpdateDisplay();
    /* 记录进入时的系统 tick（毫秒），用于后续 300ms 启动保护判断 */
    enter_tick = HAL_GetTick();
}

/**
 * @brief  模块主循环函数 —— 由系统调度器周期性调用（非阻塞）
 *
 * 处理逻辑：
 *   1. 启动保护：进入模块后 300ms 内忽略所有按键，避免切换瞬间的误触
 *   2. 检测按键输入，根据按键执行对应操作
 *   3. 按键映射：
 *      '4' → 舵机1 减小角度（左转）
 *      '6' → 舵机1 增大角度（右转）
 *      '2' → 舵机2 减小角度（下转）
 *      '8' → 舵机2 增大角度（上转）
 *      '+' → 增大步进值（5→10→15→...→40）
 *      '-' → 减小步进值（40→35→30→...→5）
 *      'n' → 退出模块，返回菜单
 *   4. 角度变化后刷新 LCD 显示
 */
static void servo_tick(void)
{
    /* 启动保护：进入模块后 300ms 内忽略键盘输入，防止菜单切换时残留按键触发误操作 */
    if (HAL_GetTick() - enter_tick < 300) return;

    /* 读取按键事件，若无按键按下则直接返回 */
    int key = detect_key_press();
    if (key == EVT_NONE) return;

    switch (key) {
    case '4':  /* 舵机1 左转：减小角度，步进 step 度，下限为 0 度 */
        servo1_angle = (servo1_angle >= step) ? servo1_angle - step : 0;
        Servo_SetAngle(1, servo1_angle);
        break;
    case '6':  /* 舵机1 右转：增大角度，步进 step 度，上限为 180 度 */
        servo1_angle = (servo1_angle <= 180 - step) ? servo1_angle + step : 180;
        Servo_SetAngle(1, servo1_angle);
        break;
    case '2':  /* 舵机2 下转：减小角度，步进 step 度，下限为 0 度 */
        servo2_angle = (servo2_angle >= step) ? servo2_angle - step : 0;
        Servo_SetAngle(2, servo2_angle);
        break;
    case '8':  /* 舵机2 上转：增大角度，步进 step 度，上限为 110 度（机械限位保护） */
        servo2_angle = (servo2_angle <= 110 - step) ? servo2_angle + step : 110;
        Servo_SetAngle(2, servo2_angle);
        break;
    case '+':  /* 增大步进值：每次 +5，最大 40 度 */
        step = (step < 40) ? step + 5 : 40;
        break;
    case '-':  /* 减小步进值：每次 -5，最小 5 度 */
        step = (step > 5) ? step - 5 : 5;
        break;
    case 'n':  /* 按下 'n' 键：退出舵机模块，返回上一级菜单 */
        module_exit_current();
        return;  /* 直接返回，不需要刷新显示 */
    default:    /* 其他按键：忽略，不做任何处理 */
        return;
    }

    /* 有角度或步进值变化时，刷新 LCD 上的数值显示 */
    Servo_UpdateDisplay();
}

/**
 * @brief  模块退出函数 —— 由菜单系统在离开舵机模块时调用
 *
 * 职责：
 *   1. 停止两路 PWM 输出（舵机停止响应，保持当前位置）
 *   2. 释放 GPIO 引脚和定时器资源，供其他模块使用
 *   3. 清除硬件初始化标志，下次进入时需重新初始化
 */
static void servo_exit(void)
{
    /* ---------- 停止 PWM 输出 ---------- */
    /* 关闭 TIM3 通道 1 和通道 2 的 PWM 输出，PA6/PA7 恢复为默认状态 */
    HAL_TIM_PWM_Stop(&htim_servo, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim_servo, TIM_CHANNEL_2);

    /* ---------- 释放硬件资源 ---------- */
    /* 向资源管理器归还 PA6、PA7 引脚和 TIM3 定时器，其他模块可重新申请 */
    pin_release(GPIOA, GPIO_PIN_6, "Servo");
    pin_release(GPIOA, GPIO_PIN_7, "Servo");
    tim_release(TIM3, "Servo");

    /* 清除初始化标志，下次 servo_enter() 时将重新执行完整的硬件初始化流程 */
    hw_inited = 0;
}

/* ======================== 模块导出 ======================== */
/**
 * @brief  舵机控制模块接口结构体
 *
 * 通过 ModuleInterface 统一接口，菜单系统通过函数指针调用：
 *   - .name  → 模块名称，用于菜单显示
 *   - .enter → 进入模块时调用（初始化硬件 + 绘制界面）
 *   - .tick  → 周期性调用（处理按键输入 + 更新舵机角度）
 *   - .exit  → 退出模块时调用（停止 PWM + 释放资源）
 */
const ModuleInterface servo_module = {
    .name  = "Servo",          /* 模块名称，显示在主菜单中 */
    .enter = servo_enter,      /* 进入回调：初始化硬件、绘制 UI */
    .tick  = servo_tick,       /* 主循环回调：按键处理、角度更新 */
    .exit  = servo_exit,       /* 退出回调：停止 PWM、释放资源 */
};
