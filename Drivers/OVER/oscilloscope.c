/**
 * @file    oscilloscope.c
 * @brief   简单示波器模块 — ADC1_CH1 (PA1) 轮询采样, LCD 波形显示
 *
 * 采样策略：在 tick 回调中调用 adc_read_once() 进行逐次逼近式采样，
 * 每次 tick 批量采 32 个点，缓冲区满 256 个点后触发一次波形绘制。
 * 全程无中断、无定时器，采用纯轮询方式，避免 HAL 底层兼容性问题。
 *
 * 显示流程：采样完成 → 寻找上升沿触发电平 → 计算 Vpp/Vavg/Freq →
 *           绘制带网格的波形图 → 显示测量信息卡片 → 重新开始采样。
 *
 * 硬件资源：ADC1 通道 1，对应 PA1 引脚（模拟输入）。
 * 参考电压：3.3V，12 位 ADC 分辨率（0~4095）。
 */

#include "./OVER/oscilloscope.h"
#include "./OVER/module.h"
#include "./OVER/resource.h"
#include "./OVER/menu.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/lcd_widgets.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

/* ======================== 布局常量 ======================== */
#define GFX_X           20      /* 波形绘图区域左上角 X 坐标（像素） */
#define GFX_Y           40      /* 波形绘图区域左上角 Y 坐标（像素） */
#define GFX_W           440     /* 波形绘图区域宽度（像素） */
#define GFX_H           320     /* 波形绘图区域高度（像素） */
#define SAMPLE_COUNT    256     /* 采样缓冲区大小，共 256 个采样点 */
#define ADC_MAX         4095    /* 12 位 ADC 最大值（2^12 - 1 = 4095） */
#define VREF            3.3f    /* ADC 参考电压 3.3V，用于将 ADC 值转换为实际电压 */

/* ======================== 硬件 ======================== */
static ADC_HandleTypeDef hadc1;  /* ADC1 外设句柄，由 HAL 库管理 */
static uint8_t hw_inited = 0;    /* 硬件初始化标志：0=未初始化，1=已初始化（防止重复初始化） */

/* ======================== 采样缓冲 ======================== */
static uint16_t sample_buf[SAMPLE_COUNT]; /* ADC 采样数据缓冲区，存储 12 位 ADC 原始值（0~4095） */
static uint16_t sample_idx = 0;           /* 当前采样索引，指向缓冲区中下一个待写入位置 */
static uint8_t  buf_ready = 0;            /* 缓冲区就绪标志：0=采样中，1=缓冲区已满，可进行显示 */

/* ======================== 单次 ADC 采样 (直接寄存器操作) ======================== */
/**
 * @brief  通过直接操作 ADC 寄存器完成一次单通道采样
 *
 * 直接访问寄存器比调用 HAL_ADC_PollForConversion() 更快，
 * 减少了函数调用开销和 HAL 层的额外判断逻辑，适合高频轮询采样场景。
 *
 * 工作流程：
 *   1. 置位 CR2 寄存器的 SWSTART 位，触发 ADC 开始转换
 *   2. 轮询 SR 寄存器的 EOC（End Of Conversion）标志位，等待转换完成
 *   3. 读取 DR（Data Register）寄存器获取 12 位转换结果
 */
static inline uint16_t adc_read_once(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;          /* 置位 SWSTART，启动一次 ADC 转换 */
    while (!(ADC1->SR & ADC_SR_EOC)) ;     /* 轮询等待 EOC 标志，转换完成时硬件自动置 1 */
    return (uint16_t)ADC1->DR;              /* 读取数据寄存器，返回 12 位 ADC 原始值（0~4095） */
}

/* ======================== 硬件初始化 ======================== */
/**
 * @brief  初始化示波器所需的硬件资源：PA1 引脚 + ADC1 外设
 *
 * 初始化流程：
 *   1. 通过 pin_request() 申请 PA1 引脚的独占使用权（资源管理）
 *   2. 配置 PA1 为模拟输入模式（GPIO_MODE_ANALOG），直接接入 ADC 采样通道
 *   3. 使能 ADC1 时钟，配置 ADC1 为单次转换、软件触发、右对齐、单通道模式
 *   4. 配置 ADC 通道 1，采样时间为 7.5 个时钟周期（较短，采样速度更快）
 *   5. 执行 ADC 校准（提高精度），然后启动 ADC（等待后续软件触发转换）
 *
 * 使用 hw_inited 标志防止重复初始化，保护硬件状态。
 */
static void scope_hw_init(void)
{
    if (hw_inited) return;  /* 已初始化则直接返回，避免重复配置 */

    pin_request(GPIOA, GPIO_PIN_1, "Oscilloscope");  /* 申请 PA1 引脚资源，标签为 "Oscilloscope" */

    /* GPIO: PA1 模拟输入 — ADC 通道 1 对应 PA1，需配置为模拟模式以禁用数字输入缓冲 */
    __HAL_RCC_GPIOA_CLK_ENABLE();          /* 使能 GPIOA 端口时钟 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_1;                /* 选择 PA1 引脚 */
    gpio.Mode = GPIO_MODE_ANALOG;          /* 模拟输入模式，直接连接到 ADC 内部采样电路 */
    HAL_GPIO_Init(GPIOA, &gpio);           /* 应用 GPIO 配置 */

    /* ADC1 — 单次转换模式（非连续），由软件触发每次转换 */
    __HAL_RCC_ADC1_CLK_ENABLE();           /* 使能 ADC1 外设时钟 */
    hadc1.Instance                   = ADC1;                    /* 选择 ADC1 外设 */
    hadc1.Init.ScanConvMode          = DISABLE;                 /* 禁用扫描模式（仅单通道，无需扫描） */
    hadc1.Init.ContinuousConvMode    = DISABLE;                 /* 禁用连续转换（由软件逐次触发） */
    hadc1.Init.DiscontinuousConvMode = DISABLE;                 /* 禁用间断模式 */
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;      /* 软件触发转换（非外部硬件触发） */
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;     /* 数据右对齐，12 位结果占据低 12 位 */
    hadc1.Init.NbrOfConversion       = 1;                       /* 规则组仅 1 个转换通道 */
    HAL_ADC_Init(&hadc1);                                       /* 应用 ADC 配置 */

    /* 配置 ADC 通道 — 短采样时间换取更快的采样速率 */
    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = ADC_CHANNEL_1;            /* 选择 ADC 通道 1（对应 PA1） */
    ch.Rank         = ADC_REGULAR_RANK_1;       /* 规则组中排第 1（唯一通道） */
    ch.SamplingTime = ADC_SAMPLETIME_7CYCLES_5; /* 采样时间 7.5 个 ADC 时钟周期，约 1us，兼顾速度与精度 */
    HAL_ADC_ConfigChannel(&hadc1, &ch);         /* 应用通道配置 */

    /* ADC 校准后启动 — 校准可消除内部电容充放电偏差，提高转换精度 */
    HAL_ADCEx_Calibration_Start(&hadc1);  /* 执行 ADC 自动校准（F1 系列特有） */
    HAL_ADC_Start(&hadc1);                /* 启动 ADC，进入待转换状态，等待 SWSTART 触发 */

    hw_inited = 1;  /* 标记硬件已初始化完成 */
}

/* ======================== 触发查找 ======================== */
/**
 * @brief  在采样缓冲区中寻找上升沿触发点
 *
 * 触发策略：上升沿中点触发
 *   1. 遍历缓冲区，找出信号的最小值 vmin 和最大值 vmax
 *   2. 噪声门限：若峰峰值（vmax - vmin）< 50 ADC 码值，认为无有效信号，返回 0
 *   3. 计算中间电平 mid = (vmin + vmax) / 2
 *   4. 从第 2 个采样点开始搜索，找到第一个"从低于 mid 上升到 >= mid"的位置
 *      即 sample_buf[i-1] < mid && sample_buf[i] >= mid，即上升沿穿越中点
 *
 * 搜索范围 [2, SAMPLE_COUNT-10) 留有余量，避免在缓冲区尾部触发导致后续绘制越界。
 *
 * @return 触发点索引（在缓冲区中的位置），未找到则返回 0
 */
static uint16_t find_trigger(void)
{
    uint16_t vmin = 4095, vmax = 0;
    /* 第一遍遍历：求信号的最小值和最大值 */
    for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
        if (sample_buf[i] < vmin) vmin = sample_buf[i];
        if (sample_buf[i] > vmax) vmax = sample_buf[i];
    }
    if (vmax - vmin < 50) return 0;   /* 噪声门限：峰峰值太小，认为是噪声或无信号，不触发 */

    /* 计算中间电平（直流偏置 + 交流幅度的中心值） */
    uint16_t mid = (vmin + vmax) / 2;

    /* 第二遍遍历：寻找上升沿穿越中间电平的时刻 */
    for (uint16_t i = 2; i < SAMPLE_COUNT - 10; i++) {
        if (sample_buf[i - 1] < mid && sample_buf[i] >= mid) {
            return i;  /* 找到上升沿触发点，返回该采样点索引 */
        }
    }
    return 0;  /* 未找到有效触发点（信号可能为直流或下降沿），返回缓冲区起始位置 */
}

/* ======================== 测量 ======================== */
/**
 * @brief  计算波形的三个核心参数：峰峰值电压、平均电压、频率估算
 *
 * @param[in]  trig_idx  触发点索引（由 find_trigger() 返回）
 * @param[out] freq      频率估算结果（Hz），无法测量时输出 0
 * @param[out] vpp       峰峰值电压（V），即信号最大值与最小值之差
 * @param[out] vavg      平均电压（V），即信号的直流偏置分量
 *
 * 电压计算公式：V = ADC_value * VREF / ADC_MAX
 *   - ADC_value: 12 位 ADC 原始值（0~4095）
 *   - VREF: 参考电压 3.3V
 *   - ADC_MAX: 4095
 *
 * 频率估算原理：
 *   从触发点之后再次找到上升沿中点，两个上升沿之间即为一个完整周期。
 *   采样率估算：每个 tick 采 32 个点，系统 tick 频率约 100Hz（10ms），
 *   因此等效采样率 = 32 * 100 = 3200 Hz。
 *   频率 = 采样率 / 一个周期内的采样点数 = 3200 / (第二个上升沿索引 - 触发点索引)
 */
static void calc_measurements(uint16_t trig_idx,
                              float *freq, float *vpp, float *vavg)
{
    uint16_t vmin = 4095, vmax = 0;
    uint32_t vsum = 0;

    /* 遍历整个缓冲区，统计最小值、最大值和累加和 */
    for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
        uint16_t v = sample_buf[i];
        if (v < vmin) vmin = v;   /* 更新最小值 */
        if (v > vmax) vmax = v;   /* 更新最大值 */
        vsum += v;                /* 累加所有采样值，用于计算平均值 */
    }

    /* ADC 原始值 → 实际电压：V = raw * 3.3V / 4095 */
    *vpp  = (float)(vmax - vmin) * VREF / ADC_MAX;          /* 峰峰值 = (最大值 - 最小值) 转换为电压 */
    *vavg = (float)vsum / SAMPLE_COUNT * VREF / ADC_MAX;    /* 平均值 = 总和 / 采样数 转换为电压 */
    *freq = 0;                                               /* 默认频率为 0，表示无法测量 */

    if (vmax - vmin < 50) return;  /* 噪声门限：信号幅值太小，无法可靠测频 */

    /* 频率估算：从触发点之后寻找下一个上升沿，两上升沿间距 = 一个信号周期 */
    uint16_t mid = (vmin + vmax) / 2;  /* 中间电平（触发电平） */
    for (uint16_t i = trig_idx + 5; i < SAMPLE_COUNT - 1; i++) {
        if (sample_buf[i - 1] < mid && sample_buf[i] >= mid) {
            /* 找到下一个上升沿，计算频率：
             * 等效采样率 ≈ 3200 Hz（32 samples/tick × 100 ticks/sec）
             * 频率 = 采样率 / 两个上升沿之间的采样点数 */
            *freq = 3200.0f / (i - trig_idx);
            break;
        }
    }
}

/* ======================== 绘制波形 ======================== */
/**
 * @brief  在 LCD 上绘制示波器波形界面
 *
 * 绘制内容（从下到上）：
 *   1. 黑色背景填充整个绘图区域
 *   2. 暗绿色网格线：3 条水平线（4 等分）+ 7 条竖直线（8 等分），模拟示波器屏幕网格
 *   3. 青色波形线：从触发点开始，逐点连线绘制 ADC 采样波形
 *   4. 灰色矩形边框：围绕绘图区域，增强视觉边界感
 *
 * Y 坐标映射：ADC 值越大，Y 坐标越小（屏幕顶部对应高电压）。
 *   y = GFX_Y + GFX_H - 1 - (adc_value * GFX_H) / ADC_MAX
 *
 * @param trig_idx 触发点索引，波形从该采样点开始绘制
 */
static void draw_waveform(uint16_t trig_idx)
{
    /* 1. 黑色背景 — RGB565 黑色 = 0x0000 */
    lcd_fill(GFX_X, GFX_Y, GFX_X + GFX_W - 1, GFX_Y + GFX_H - 1, 0x0000);

    /* 2. 网格线 — 暗绿色 (RGB565: 0x03E0)，将绘图区分为 4×8 格 */
    uint16_t grid = 0x03E0;  /* 暗绿色，RGB565 格式：R=0, G=15, B=0（低亮度绿色） */
    /* 水平网格线：3 条，将 320 像素高度均分为 4 格，每格 80 像素 */
    for (uint8_t i = 1; i < 4; i++) {
        lcd_draw_hline(GFX_X, GFX_Y + i * (GFX_H / 4), GFX_W, grid);
    }
    /* 竖直网格线：7 条，将 440 像素宽度均分为 8 格，每格 55 像素 */
    for (uint8_t i = 1; i < 8; i++) {
        uint16_t gx = GFX_X + i * (GFX_W / 8);
        lcd_draw_line(gx, GFX_Y, gx, GFX_Y + GFX_H - 1, grid);
    }

    /* 3. 波形绘制 — 青色 (RGB565: 0x07FF)，从触发点开始逐点连线 */
    /* 计算触发点对应的第一个 Y 坐标（Y 轴翻转：ADC 值越大，Y 坐标越小） */
    uint16_t prev_y = GFX_Y + GFX_H - 1
                      - (sample_buf[trig_idx] * GFX_H) / ADC_MAX;
    /* 逐像素绘制：每个 X 像素对应一个采样点，用直线段连接相邻采样点 */
    for (uint16_t i = 1; i < GFX_W && trig_idx + i < SAMPLE_COUNT; i++) {
        /* 将 ADC 原始值映射为 Y 坐标：y = 底部 - (值/满量程) × 高度 */
        uint16_t cur_y = GFX_Y + GFX_H - 1
                         - (sample_buf[trig_idx + i] * GFX_H) / ADC_MAX;
        lcd_draw_line(GFX_X + i - 1, prev_y, GFX_X + i, cur_y, 0x07FF);  /* 青色线段连接 */
        prev_y = cur_y;  /* 更新前一个点的 Y 坐标 */
    }

    /* 4. 边框 — 灰色 (RGB565: 0x7BEF)，围绕绘图区域绘制矩形边框 */
    lcd_draw_rectangle(GFX_X, GFX_Y, GFX_X + GFX_W - 1, GFX_Y + GFX_H - 1, 0x7BEF);
}

/* ======================== 测量信息卡片 ======================== */

#define INFO_Y  (GFX_Y + GFX_H + 6)   /* 信息卡片 Y 起始位置：波形区域底部下方 6 像素 */
#define INFO_H  34                       /* 信息卡片高度（像素）：容纳 12px 标签 + 16px 数值 */
#define INFO_GAP 4                       /* 相邻卡片之间的水平间距（像素） */

/**
 * @brief  绘制一个带圆角的信息卡片（标签 + 数值）
 *
 * 卡片结构：
 *   ┌──────────────────────┐
 *   │  label (12px, 次要色) │  ← 第一行：参数名称（如 "Vpp"、"Freq"）
 *   │  value (16px, 强调色) │  ← 第二行：测量结果数值（如 "3.30 V"、"1000.0 Hz"）
 *   └──────────────────────┘
 *
 * 绘制流程：
 *   1. 保存当前背景色 → 绘制圆角矩形填充背景
 *   2. 绘制圆角矩形边框
 *   3. 设置卡片背景色，绘制标签文字（小号、次要色）和数值文字（大号、指定颜色）
 *   4. 恢复原始背景色
 *
 * @param x          卡片左上角 X 坐标
 * @param y          卡片左上角 Y 坐标
 * @param w          卡片宽度（像素）
 * @param label      标签文本（如 "Vpp"、"Vavg"、"Freq"）
 * @param value      数值文本（如 "3.30 V"、"1000.0 Hz"）
 * @param val_color  数值文字颜色（RGB565），用于区分不同测量参数的视觉风格
 */
static void draw_info_card(uint16_t x, uint16_t y, uint16_t w,
                           const char *label, const char *value, uint16_t val_color)
{
    uint32_t old_bg = g_back_color;                                /* 保存当前全局背景色 */
    lcd_draw_rounded_rect(x, y, x + w - 1, y + INFO_H - 1, 6, WGT_CLR_CARD_BG);   /* 填充圆角矩形背景 */
    lcd_draw_rounded_rect_border(x, y, x + w - 1, y + INFO_H - 1, 6, WGT_CLR_CARD_BRD); /* 绘制圆角边框 */
    g_back_color = WGT_CLR_CARD_BG;                                /* 临时切换背景色为卡片背景，避免文字区域出现异色 */
    lcd_show_string(x + 8, y + 3, w - 16, 12, 12, (char *)label, WGT_CLR_TEXT_SEC);  /* 第一行：12px 标签，次要文字色 */
    lcd_show_string(x + 8, y + 17, w - 16, 16, 16, (char *)value, val_color);        /* 第二行：16px 数值，指定强调色 */
    g_back_color = old_bg;                                         /* 恢复原始背景色 */
}

/**
 * @brief  在波形区域下方绘制三张测量信息卡片：Vpp、Vavg、Freq
 *
 * 三张卡片水平等宽排列，总宽度 = 屏幕宽 - 左右边距 - 卡片间距。
 * 卡片排列示意：
 *   ┌─────────┐  ┌─────────┐  ┌─────────┐
 *   │  Vpp    │  │  Vavg   │  │  Freq   │
 *   │ 3.30 V  │  │ 1.65 V  │  │1000.0 Hz│
 *   └─────────┘  └─────────┘  └─────────┘
 */
static void draw_info(float freq, float vpp, float vavg)
{
    char buf[20];
    /* 计算每张卡片的宽度：(屏幕宽 - 左右边距各20 - 两个间距) / 3 */
    uint16_t card_w = (SCREEN_WIDTH - GFX_X * 2 - INFO_GAP * 2) / 3;

    /* 第 1 张卡片：峰峰值电压 Vpp — 使用强调色（WGT_CLR_ACCENT）突出显示 */
    snprintf(buf, sizeof(buf), "%.2f V", vpp);
    draw_info_card(GFX_X, INFO_Y, card_w, "Vpp", buf, WGT_CLR_ACCENT);

    /* 第 2 张卡片：平均电压 Vavg — 使用主要文字色（WGT_CLR_TEXT_PRI）常规显示 */
    snprintf(buf, sizeof(buf), "%.2f V", vavg);
    draw_info_card(GFX_X + card_w + INFO_GAP, INFO_Y, card_w, "Vavg", buf, WGT_CLR_TEXT_PRI);

    /* 第 3 张卡片：频率 Freq — 使用强调色；频率过小（<0.1Hz）时显示 "---" 表示无法测量 */
    if (freq > 0.1f) {
        snprintf(buf, sizeof(buf), "%.1f Hz", freq);
    } else {
        snprintf(buf, sizeof(buf), "---");   /* 频率不可测，显示占位符 */
    }
    draw_info_card(GFX_X + (card_w + INFO_GAP) * 2, INFO_Y, card_w, "Freq", buf, WGT_CLR_ACCENT);
}

/* ======================== 模块接口 ======================== */

/**
 * @brief  示波器模块进入回调 — 初始化硬件、绘制初始界面
 *
 * 调用时机：当用户从菜单选择"Oscilloscope"时，由模块管理框架调用。
 * 执行流程：
 *   1. 初始化 ADC 硬件（PA1 模拟输入 + ADC1 配置）
 *   2. 清屏并绘制顶部标题栏 "OSCILLOSCOPE"
 *   3. 在波形区域中央显示 "Sampling..." 状态提示，告知用户正在采样
 *   4. 重置采样索引和缓冲区就绪标志，准备开始新一轮采样
 */
static void oscilloscope_enter(void)
{
    scope_hw_init();   /* 初始化 ADC 硬件：PA1 模拟输入 + ADC1 单次转换模式 */

    /* 清除整个屏幕（标题栏以下区域），填充模块背景色 */
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WGT_CLR_BG);
    lcd_wgt_header("OSCILLOSCOPE", 0, 0);  /* 绘制顶部标题栏，显示 "OSCILLOSCOPE" */

    /* 在波形区域中央显示采样状态提示（垂直居中：区域高度/2 - 文字高度/2） */
    lcd_wgt_status_label(GFX_X, GFX_Y + GFX_H / 2 - 8, "Sampling...", 0);

    sample_idx = 0;    /* 重置采样索引，从缓冲区起始位置开始写入 */
    buf_ready  = 0;    /* 清除缓冲区就绪标志，进入采样阶段 */
}

/**
 * @brief  示波器模块主循环回调 — 采样 + 显示的核心状态机
 *
 * 调用时机：由模块管理框架在主循环中周期性调用（约每 10ms 一次）。
 *
 * 状态机分为两个阶段：
 *   【采样阶段】(buf_ready == 0)
 *     - 每次 tick 批量采样 32 个点，填入 sample_buf 缓冲区
 *     - 当缓冲区满 256 个点后，设置 buf_ready = 1，转入显示阶段
 *   【显示阶段】(buf_ready == 1)
 *     - 寻找触发点 → 计算测量值 → 绘制波形 → 绘制信息卡片
 *     - 绘制完成后重置采样状态，进入下一轮采样
 *
 * 按键 'n' 可在任意时刻退出示波器模块，返回上一级菜单。
 *
 * 采样率估算：32 samples/tick × ~100 ticks/sec ≈ 3200 Hz（等效采样率）
 */
static void oscilloscope_tick(void)
{
    /* ---- 退出检测：按下 'n' 键退出示波器模块 ---- */
    int key = detect_key_press();
    if (key == 'n') {
        module_exit_current();   /* 通知模块管理框架退出当前模块，返回上级菜单 */
        return;
    }

    /* ---- 采样阶段：缓冲区未满时继续采样 ---- */
    if (!buf_ready) {
        /* 每次 tick 批量采 32 个点，平衡采样速度与系统响应性 */
        for (uint8_t i = 0; i < 32 && sample_idx < SAMPLE_COUNT; i++) {
            sample_buf[sample_idx++] = adc_read_once();  /* 单次 ADC 采样，直接寄存器操作 */
        }
        if (sample_idx >= SAMPLE_COUNT) {
            buf_ready = 1;   /* 缓冲区已满，标记就绪，下次 tick 进入显示阶段 */
        }
        return;  /* 采样未完成，本 tick 不执行显示，直接返回 */
    }

    /* ---- 显示阶段：缓冲区已满，执行信号处理和波形绘制 ---- */

    /* 1. 信号处理：寻找触发点并计算测量参数 */
    uint16_t trig = find_trigger();                  /* 在缓冲区中查找上升沿触发点 */
    float freq, vpp, vavg;
    calc_measurements(trig, &freq, &vpp, &vavg);    /* 计算 Vpp、Vavg、频率 */

    /* 2. 界面绘制：重绘标题栏 → 绘制波形 → 绘制测量信息卡片 */
    lcd_wgt_header("OSCILLOSCOPE", 0, 0);           /* 重绘标题栏（覆盖之前的状态提示） */
    draw_waveform(trig);                              /* 绘制带网格的波形图 */
    draw_info(freq, vpp, vavg);                       /* 绘制 Vpp/Vavg/Freq 三张信息卡片 */

    /* 3. 重新采样：重置状态，开始下一轮采样 → 显示循环 */
    sample_idx = 0;    /* 采样索引归零 */
    buf_ready  = 0;    /* 清除就绪标志，回到采样阶段 */
}

/**
 * @brief  示波器模块退出回调 — 释放硬件资源，重置初始化标志
 *
 * 调用时机：当用户按下 'n' 键退出示波器时，由模块管理框架调用。
 * 执行流程：
 *   1. 释放 PA1 引脚（通过 pin_release 归还资源管理器）
 *   2. 清除 hw_inited 标志，下次进入时重新初始化硬件
 */
static void oscilloscope_exit(void)
{
    pin_release(GPIOA, GPIO_PIN_1, "Oscilloscope");  /* 释放 PA1 引脚，允许其他模块使用 */
    hw_inited = 0;   /* 重置硬件初始化标志，确保下次进入时重新配置 ADC */
}

/* ======================== 模块导出 ======================== */
/**
 * @brief  示波器模块接口结构体 — 向模块管理框架提供模块名称和生命周期回调
 *
 * ModuleInterface 是所有功能模块的统一接口：
 *   - name:  模块名称字符串，用于菜单显示和日志标识
 *   - enter: 进入模块时调用（初始化硬件和界面）
 *   - tick:  主循环回调（采样和显示的核心逻辑）
 *   - exit:  退出模块时调用（释放硬件资源）
 */
const ModuleInterface oscilloscope_module = {
    .name  = "Oscilloscope",       /* 模块名称 */
    .enter = oscilloscope_enter,   /* 进入回调：初始化 ADC、清屏、显示采样提示 */
    .tick  = oscilloscope_tick,    /* 主循环回调：批量采样 + 波形绘制 + 测量显示 */
    .exit  = oscilloscope_exit,    /* 退出回调：释放 PA1 引脚、重置初始化状态 */
};
