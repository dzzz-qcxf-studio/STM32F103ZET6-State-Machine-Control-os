/**
 * @file    vdraw.c
 * @brief   PWM波形分析器模块 (TIM2 输入捕获, PA0)
 *
 * 测量PWM信号的频率和占空比，并在LCD上绘制波形。
 * 模块化改造：提供 enter/tick/exit 接口，非阻塞运行。
 */

#include "./OVER/vdraw.h"
#include "./OVER/module.h"
#include "./OVER/resource.h"
#include "./OVER/menu.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/lcd_widgets.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>

/* ======================== 波形绘制参数 ======================== */
/* GRAPH_WIDTH: 波形显示区域的宽度（像素），即横轴总长度 */
#define GRAPH_WIDTH  400
/* GRAPH_HEIGHT: 波形显示区域的高度（像素），即纵轴总长度，用于表示高/低电平幅值 */
#define GRAPH_HEIGHT 200
/* GRAPH_X: 波形绘制区域左上角的X坐标（像素），相对于LCD屏幕左边缘的偏移 */
#define GRAPH_X      40
/* GRAPH_Y: 波形绘制区域底边的Y坐标（像素），同时也是时间轴（横轴）所在位置 */
#define GRAPH_Y      280

/* ======================== 硬件句柄 ======================== */
/* TIM2定时器句柄，用于PWM输入捕获配置，由HAL库驱动 */
static TIM_HandleTypeDef htim_pwm_capture;

/* ======================== 测量数据（中断更新） ======================== */
/*
 * 以下四个变量均在TIM2中断回调函数中被修改，因此必须声明为volatile，
 * 防止编译器优化导致主循环读取到缓存的旧值。
 */

/* pwm_high_time: PWM高电平持续时间，单位为定时器计数值（1计数 = 1us，因预分频72-1） */
static volatile uint32_t pwm_high_time = 0;
/* pwm_period: PWM信号完整周期，单位同上（us）。由连续两次上升沿时间差计算得出 */
static volatile uint32_t pwm_period    = 0;
/* pwm_updated: 数据更新标志，由中断置1，主循环读取后清0，用于通知主循环有新数据可刷新 */
static volatile uint8_t  pwm_updated   = 0;
/* last_rise_time: 最近一次上升沿捕获时刻的计数器值，用于计算高电平时间（下降沿-上升沿） */
static volatile uint32_t last_rise_time = 0;

/* ======================== 模块状态 ======================== */
/* last_update_tick: 上次刷新波形的系统节拍值（ms），用于实现200ms定时刷新 */
static uint32_t last_update_tick = 0;
/* hw_inited: 硬件初始化标志，防止重复初始化GPIO和定时器资源，1=已初始化 */
static uint8_t  hw_inited = 0;

/* ======================== 中断处理 ======================== */

/*
 * TIM2中断服务函数（ISR）
 * STM32启动文件中weak定义的中断入口，此处覆盖实现。
 * 不做任何业务逻辑，直接转交HAL库的统一中断分发函数，
 * 由HAL库根据具体的中断标志位（捕获/更新等）调用对应的回调函数。
 */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim_pwm_capture);
}

/*
 * HAL定时器输入捕获回调函数
 * 由HAL_TIM_IRQHandler()在检测到捕获事件后自动调用。
 * 利用TIM2的双通道实现PWM测量：
 *   - CH1（PA0）配置为上升沿捕获 → 计算信号周期
 *   - CH2（同一引脚PA0，间接映射）配置为下降沿捕获 → 计算高电平时间
 * 这种方式称为"TI1+TI2"或"TI1FP1+TI1FP2"的PWM输入模式。
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* rise_time: 静态局部变量，保存上一次上升沿的捕获值，跨调用保持 */
    static uint32_t rise_time = 0;

    /* 安全检查：只处理TIM2的捕获事件，忽略其他定时器 */
    if (htim->Instance != TIM2) return;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        /* ===== CH1 上升沿事件 ===== */
        /* 读取CH1捕获寄存器当前值，即本次上升沿发生的定时器计数值 */
        uint32_t current_rise = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        /* 如果已有上一次上升沿记录，则计算周期（两次上升沿之差） */
        if (rise_time != 0) {
            /* 处理计数器溢出回绕的情况：
             * 若current_rise > rise_time → 直接相减
             * 否则说明计数器从0xFFFF翻转回0 → 需要分段计算：
             *   先算从rise_time到0xFFFF的距离，再加上current_rise+1 */
            pwm_period = (current_rise > rise_time)
                         ? current_rise - rise_time
                         : (0xFFFF - rise_time) + current_rise + 1;
        }
        /* 更新上升沿记录，供下次周期计算和高电平时间计算使用 */
        rise_time = current_rise;
        last_rise_time = current_rise;
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        /* ===== CH2 下降沿事件 ===== */
        /* 读取CH2捕获寄存器当前值，即下降沿发生的定时器计数值 */
        uint32_t fall_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

        /* 高电平时间 = 下降沿时刻 - 上升沿时刻
         * 同样需要处理计数器溢出回绕的情况 */
        pwm_high_time = (fall_time >= last_rise_time)
                        ? fall_time - last_rise_time
                        : (0xFFFF - last_rise_time) + fall_time + 1;

        /* 置位更新标志，通知主循环有新的测量数据可读取 */
        pwm_updated = 1;
    }
}

/* ======================== 硬件初始化 ======================== */

/*
 * PWM_Measure_Init() - PWM测量硬件初始化
 * 配置TIM2的双通道输入捕获功能，实现PWM信号的频率和占空比测量。
 * 硬件连接：待测PWM信号接入PA0引脚（TIM2_CH1）
 *
 * 工作原理：
 *   TIM2_CH1 捕获上升沿 → 两次上升沿之差 = 周期
 *   TIM2_CH2 捕获下降沿 → 下降沿与上升沿之差 = 高电平时间
 *   占空比 = 高电平时间 / 周期
 */
static void PWM_Measure_Init(void)
{
    /* 防止重复初始化：如果硬件已初始化则直接返回 */
    if (hw_inited) return;

    /* 向资源管理器申请GPIOA PIN_0和TIM2的使用权，标签为"PWM_Analyzer" */
    pin_request(GPIOA, GPIO_PIN_0, "PWM_Analyzer");
    tim_request(TIM2, "PWM_Analyzer");

    /* 使能GPIOA和TIM2的外设时钟，不使能则寄存器无法访问 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* 配置PA0为复用功能输入模式（AF_INPUT）：
     *   Mode = GPIO_MODE_AF_INPUT → 引脚由片上外设（TIM2_CH1）控制
     *   Pull = GPIO_NOPULL → 不使用内部上拉/下拉，信号由外部驱动
     *   Speed = GPIO_SPEED_FREQ_HIGH → 高速模式，适合高频信号捕获 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_0;
    gpio.Mode  = GPIO_MODE_AF_INPUT;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* ===== TIM2基本参数配置 ===== */
    htim_pwm_capture.Instance = TIM2;
    /* 预分频器 = 72-1 = 71：将72MHz系统时钟分频为1MHz（即1us计数一次）
     * 这样捕获寄存器的差值直接对应微秒数，便于后续频率计算 */
    htim_pwm_capture.Init.Prescaler         = 72 - 1;
    /* 向上计数模式：计数器从0递增到Period后溢出归零 */
    htim_pwm_capture.Init.CounterMode       = TIM_COUNTERMODE_UP;
    /* 自动重装载值 = 0xFFFF（65535）：16位计数器最大值
     * 配合1MHz时钟，最大可测量65.535ms周期（约15Hz以上信号） */
    htim_pwm_capture.Init.Period            = 0xFFFF;
    /* 时钟不分频（保持1MHz计数频率） */
    htim_pwm_capture.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    /* 禁用预装载，写入ARR寄存器立即生效 */
    htim_pwm_capture.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_IC_Init(&htim_pwm_capture);

    /* ===== CH1输入捕获配置：上升沿捕获，用于测量信号周期 ===== */
    TIM_IC_InitTypeDef ic = {0};
    /* 上升沿触发：信号从低变高时锁存计数器值 */
    ic.ICPolarity  = TIM_ICPOLARITY_RISING;
    /* 直接映射（DIRECTTI）：CH1输入通道直接连接到IC1，即TI1→IC1 */
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    /* 输入捕获不分频：每次有效边沿都触发捕获 */
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    /* 输入滤波器=0：不滤波，适用于干净的数字信号；若有噪声可增大此值 */
    ic.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&htim_pwm_capture, &ic, TIM_CHANNEL_1);

    /* ===== CH2输入捕获配置：下降沿捕获，用于测量高电平时间 ===== */
    /* 下降沿触发：信号从高变低时锁存计数器值 */
    ic.ICPolarity  = TIM_ICPOLARITY_FALLING;
    /* 间接映射（INDIRECTTI）：CH2输入通道连接到IC1的输入（即TI1→IC2），
     * 这样CH1和CH2都监听同一个引脚PA0，分别响应上升沿和下降沿 */
    ic.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&htim_pwm_capture, &ic, TIM_CHANNEL_2);

    /* 启动CH1和CH2的输入捕获，并使能对应的捕获中断 */
    HAL_TIM_IC_Start_IT(&htim_pwm_capture, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim_pwm_capture, TIM_CHANNEL_2);

    /* 配置TIM2中断优先级：
     * 抢占优先级=6（较低），子优先级=0
     * 使用较低优先级避免影响其他关键中断（如串口、按键等） */
    HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
    /* 使能TIM2在NVIC中的中断请求 */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    /* 标记硬件已初始化完成 */
    hw_inited = 1;
}

/* ======================== 波形绘制 ======================== */

/*
 * Draw_PWM_Waveform() - 在LCD上绘制PWM波形和测量信息
 * @high_time: 高电平持续时间（单位：us，即定时器计数值）
 * @period:    PWM信号完整周期（单位：us）
 *
 * 绘制内容：
 *   1. 清除绘图区域
 *   2. 绘制坐标轴和刻度
 *   3. 根据占空比绘制PWM方波（红色线条）
 *   4. 在波形下方显示频率、占空比、周期信息
 */
static void Draw_PWM_Waveform(uint32_t high_time, uint32_t period)
{
    /* 用白色填充整个绘图区域（含下方信息区），清除上一帧波形 */
    lcd_fill(GRAPH_X, GRAPH_Y - GRAPH_HEIGHT, GRAPH_X + GRAPH_WIDTH, GRAPH_Y + 40, WHITE);

    /* 绘制坐标轴：水平时间轴 + 垂直幅值轴，黑色线条 */
    lcd_draw_line(GRAPH_X, GRAPH_Y, GRAPH_X + GRAPH_WIDTH, GRAPH_Y, BLACK);
    lcd_draw_line(GRAPH_X, GRAPH_Y, GRAPH_X, GRAPH_Y - GRAPH_HEIGHT, BLACK);

    /* 绘制横轴刻度线：每隔100像素画一条5像素高的短线，便于观察时间分布 */
    for (int i = 100; i < GRAPH_WIDTH; i += 100)
        lcd_draw_line(GRAPH_X + i, GRAPH_Y, GRAPH_X + i, GRAPH_Y + 5, BLACK);

    /* 计算占空比 = 高电平时间 / 周期，无信号时为0 */
    float duty = (period > 0) ? (float)high_time / period : 0;
    /* 限幅到[0, 1]，防止异常数据导致显示错误 */
    if (duty > 1.0f) duty = 1.0f;

    /* 计算波形绘制参数：
     * high_width: 高电平部分在屏幕上的宽度（像素），与占空比成正比
     * high_level: 高电平所在的Y坐标（屏幕上方80%处）
     * low_level:  低电平所在的Y坐标（屏幕下方20%处）
     * 注意：LCD坐标系Y轴向下，所以用GRAPH_Y减去偏移量 */
    uint16_t high_width = (uint16_t)(GRAPH_WIDTH * duty);
    uint16_t high_level = GRAPH_Y - GRAPH_HEIGHT * 8 / 10;
    uint16_t low_level  = GRAPH_Y - GRAPH_HEIGHT * 2 / 10;

    /* ===== 绘制PWM方波（红色） ===== */
    /* 第一段：高电平水平线，从起点到高电平结束位置 */
    lcd_draw_line(GRAPH_X, high_level, GRAPH_X + high_width, high_level, RED);
    if (high_width < GRAPH_WIDTH) {
        /* 第二段：下降沿垂直线，从高电平跳变到低电平 */
        lcd_draw_line(GRAPH_X + high_width, high_level, GRAPH_X + high_width, low_level, RED);
        /* 第三段：低电平水平线，从下降沿到波形区域右边界 */
        lcd_draw_line(GRAPH_X + high_width, low_level, GRAPH_X + GRAPH_WIDTH, low_level, RED);
    }

    /* ===== 显示测量信息 ===== */
    char info[80];
    if (period > 0) {
        /* 有信号时：计算并显示频率、占空比、周期
         * 频率 = 1,000,000 / 周期(us) → Hz
         * 周期转换为ms显示 */
        float freq = 1000000.0f / period;
        snprintf(info, sizeof(info), "Freq:%.1fHz  Duty:%.1f%%  Period:%.2fms",
                 freq, duty * 100, period / 1000.0f);
    } else {
        /* 无信号时显示提示文字 */
        snprintf(info, sizeof(info), "No Signal!");
    }
    /* 在波形下方（GRAPH_Y + 20）显示信息，使用强调色 */
    lcd_show_string(GRAPH_X, GRAPH_Y + 20, 400, 16, 16, info, WGT_CLR_ACCENT);
}

/* ======================== 模块接口 ======================== */

/*
 * vdraw_enter() - 模块进入函数
 * 用户从菜单选择进入PWM分析器时被调用，负责：
 *   1. 清零所有测量数据（避免显示上一次的残留数据）
 *   2. 初始化PWM捕获硬件
 *   3. 绘制UI界面（标题栏、状态标签、初始波形）
 */
static void vdraw_enter(void)
{
    /* 清零测量变量，确保初始状态干净 */
    pwm_high_time = 0;
    pwm_period    = 0;
    pwm_updated   = 0;

    /* 初始化TIM2输入捕获硬件（含GPIO配置） */
    PWM_Measure_Init();

    /* 清屏并绘制UI框架：
     * 用背景色填充整个显示区域（菜单区除外） */
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WGT_CLR_BG);
    /* 绘制模块标题栏 "PWM ANALYZER" */
    lcd_wgt_header("PWM ANALYZER", 0, 0);
    /* 显示输入通道信息标签 "Input: PA0" */
    lcd_wgt_status_label(14, WGT_HEADER_H + 6, "Input: PA0", 0);
    /* 绘制初始空波形（高电平=0, 周期=0 → 显示"No Signal!"） */
    Draw_PWM_Waveform(0, 0);

    /* 记录当前系统节拍，作为定时刷新的起始时间 */
    last_update_tick = HAL_GetTick();
}

/*
 * vdraw_tick() - 模块主循环函数
 * 由系统调度器周期性调用（通常每10~20ms一次），负责：
 *   1. 检测用户按键，按'n'退出模块
 *   2. 每200ms检查一次是否有新的测量数据
 *   3. 用临界区保护读取volatile变量（防止读取过程中被中断修改导致数据不一致）
 *   4. 有新数据时重绘波形
 */
static void vdraw_tick(void)
{
    /* ===== 按键检测：按'n'键退出当前模块，返回上一级菜单 ===== */
    int key = detect_key_press();
    if (key == 'n') {
        module_exit_current();
        return;
    }

    /* ===== 定时刷新逻辑（每200ms执行一次） ===== */
    uint32_t now = HAL_GetTick();
    if (now - last_update_tick >= 200) {
        /* 检查中断是否设置了数据更新标志 */
        if (pwm_updated) {
            /*
             * 临界区保护：读取volatile变量时必须关中断
             * 原因：pwm_high_time和pwm_period是32位变量，
             * 在16位MCU上读取需要多条指令。如果读取过程中
             * 被中断打断并修改了值，会导致读到"半新半旧"的数据。
             *
             * 操作步骤：
             *   1. 保存当前PRIMASK（中断使能状态）
             *   2. 关闭全局中断（__disable_irq）
             *   3. 读取两个volatile变量的值到局部变量
             *   4. 恢复PRIMASK（重新使能中断）
             *
             * 注意：使用PRIMASK保存/恢复而非简单__enable_irq()，
             * 是为了不破坏嵌套关中断的场景（如果上层也关了中断）。
             */
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            uint32_t ht = pwm_high_time;
            uint32_t pd = pwm_period;
            __set_PRIMASK(primask);

            /* 清除更新标志，表示数据已被消费 */
            pwm_updated = 0;

            /* 用局部变量副本绘制波形，避免绘图过程中再次被中断改值 */
            Draw_PWM_Waveform(ht, pd);
        }
        /* 无论是否有新数据，都更新上次刷新时间戳 */
        last_update_tick = now;
    }
}

/*
 * vdraw_exit() - 模块退出函数
 * 用户按'n'退出PWM分析器时被调用，负责：
 *   1. 停止输入捕获中断
 *   2. 关闭TIM2中断
 *   3. 释放GPIO和定时器资源给资源管理器
 *   4. 重置硬件初始化标志
 *
 * 退出后其他模块可重新申请使用PA0和TIM2。
 */
static void vdraw_exit(void)
{
    /* 停止CH1和CH2的输入捕获中断，不再产生捕获事件 */
    HAL_TIM_IC_Stop_IT(&htim_pwm_capture, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop_IT(&htim_pwm_capture, TIM_CHANNEL_2);

    /* 在NVIC中禁用TIM2中断，彻底阻断该中断源 */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);

    /* 向资源管理器归还GPIO和定时器的使用权
     * 其他模块调用pin_request/tim_request时可重新分配 */
    pin_release(GPIOA, GPIO_PIN_0, "PWM_Analyzer");
    tim_release(TIM2, "PWM_Analyzer");

    /* 重置硬件初始化标志，下次进入模块时会重新初始化 */
    hw_inited = 0;
}

/* ======================== 模块导出 ======================== */

/*
 * 模块接口结构体，供菜单系统和模块调度器使用
 * - name:  模块名称标识符，用于调试和资源管理
 * - enter: 进入模块时调用（初始化+首屏绘制）
 * - tick:  主循环轮询函数（按键检测+定时刷新波形）
 * - exit:  退出模块时调用（停止中断+释放资源）
 */
const ModuleInterface vdraw_module = {
    .name  = "PWM_Analyzer",
    .enter = vdraw_enter,
    .tick  = vdraw_tick,
    .exit  = vdraw_exit,
};
