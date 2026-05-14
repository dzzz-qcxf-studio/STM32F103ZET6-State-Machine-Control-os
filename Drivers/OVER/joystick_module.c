/**
 * @file    joystick_module.c
 * @brief   摇杆显示模块 — 包装 BSP 层 Joystick 驱动为 ModuleInterface
 *
 * 周期性读取双摇杆ADC值，在LCD上显示位置和数值。
 * 该模块将两个模拟摇杆的ADC原始数据可视化为LCD上的圆点，
 * 同时显示归一化后的浮点数值和按键状态。
 */

#include "./OVER/jystick_module.h"
#include "./OVER/module.h"
#include "./OVER/menu.h"
#include "./BSP/JOYSTICK/Joystick.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/lcd_widgets.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <math.h>

/* ======================== 布局常量 ======================== */
/* 信息显示区域的Y坐标起始位置 */
#define INFO_Y       5
/* 摇杆显示圆的半径（像素） */
#define JOY_RADIUS   70
/* 左摇杆圆心坐标 (屏幕左侧) */
#define JOY1_X       120
#define JOY1_Y       280
/* 右摇杆圆心坐标 (屏幕右侧) */
#define JOY2_X       360
#define JOY2_Y       280
/* 摇杆当前位置指示圆点的半径 */
#define DOT_RADIUS   5
/* 死区内圈半径，摇杆在此范围内视为居中 */
#define INNER_RADIUS 20

/* ======================== 模块状态 ======================== */
/* 上次刷新的时间戳（毫秒），用于控制刷新频率 */
static uint32_t last_update_tick = 0;
/* 上一帧两个摇杆指示点的屏幕坐标，用于擦除旧位置 */
static uint16_t last_x1, last_y1, last_x2, last_y2;
/* 首次运行标志，为1时跳过擦除旧点（因为没有旧点需要擦除） */
static uint8_t  first_run = 1;

/* ======================== 绘制摇杆布局 ======================== */
/**
 * DrawLayout() — 绘制完整的摇杆显示界面
 *
 * 包含以下元素：
 *   - 顶部标题栏 "JOYSTICK"
 *   - 左摇杆区域：外圆、死区内圈、十字准线、半径参考圆
 *   - 右摇杆区域：同上，使用不同颜色区分
 *   - 左/右标签文字
 *   - ADC引脚映射信息（PC0~PC5对应各轴和按键）
 */
static void DrawLayout(void)
{
    /* 用背景色填充整个模块区域，清除旧画面 */
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WGT_CLR_BG);
    /* 绘制顶部标题栏 */
    lcd_wgt_header("JOYSTICK", 0, 0);

    /* ---- 左摇杆 ---- */
    /* 外圆：摇杆最大活动范围的边界 */
    lcd_draw_circle(JOY1_X, JOY1_Y, JOY_RADIUS, WGT_CLR_KEY_NEG);
    /* 死区内圈：摇杆在此范围内不产生有效输出 */
    lcd_draw_circle(JOY1_X, JOY1_Y, INNER_RADIUS, WGT_CLR_CARD_BRD);
    /* 水平十字准线 */
    lcd_draw_line(JOY1_X - JOY_RADIUS, JOY1_Y, JOY1_X + JOY_RADIUS, JOY1_Y, WGT_CLR_TEXT_SEC);
    /* 垂直十字准线 */
    lcd_draw_line(JOY1_X, JOY1_Y - JOY_RADIUS, JOY1_X, JOY1_Y + JOY_RADIUS, WGT_CLR_TEXT_SEC);
    /* 半径参考圆（50%行程位置），辅助观察摇杆偏移程度 */
    lcd_draw_circle(JOY1_X, JOY1_Y, JOY_RADIUS / 2, WGT_CLR_CARD_BRD);

    /* ---- 右摇杆 ---- */
    /* 外圆、死区、十字准线、半径参考圆，结构与左摇杆相同 */
    lcd_draw_circle(JOY2_X, JOY2_Y, JOY_RADIUS, WGT_CLR_ACCENT);
    lcd_draw_circle(JOY2_X, JOY2_Y, INNER_RADIUS, WGT_CLR_CARD_BRD);
    lcd_draw_line(JOY2_X - JOY_RADIUS, JOY2_Y, JOY2_X + JOY_RADIUS, JOY2_Y, WGT_CLR_TEXT_SEC);
    lcd_draw_line(JOY2_X, JOY2_Y - JOY_RADIUS, JOY2_X, JOY2_Y + JOY_RADIUS, WGT_CLR_TEXT_SEC);
    lcd_draw_circle(JOY2_X, JOY2_Y, JOY_RADIUS / 2, WGT_CLR_CARD_BRD);

    /* ---- 文字标签 ---- */
    /* 临时切换背景色为模块背景色，避免文字背景与卡片背景不一致 */
    uint32_t old_bg = g_back_color;
    g_back_color = WGT_CLR_BG;
    /* 左/右摇杆名称标签，显示在对应圆的下方 */
    lcd_show_string(JOY1_X - 30, JOY1_Y + JOY_RADIUS + 15, 60, 16, 16, "Left", WGT_CLR_KEY_NEG);
    lcd_show_string(JOY2_X - 30, JOY2_Y + JOY_RADIUS + 15, 60, 16, 16, "Right", WGT_CLR_ACCENT);
    /* ADC引脚映射说明：PC2=左X轴, PC0=左Y轴, PC4=左按键 */
    lcd_show_string(14, JOY1_Y - JOY_RADIUS - 30, 200, 12, 12, "PC2(X1) PC0(Y1) PC4(K1)", WGT_CLR_TEXT_SEC);
    /* PC1=右X轴, PC3=右Y轴, PC5=右按键 */
    lcd_show_string(300, JOY2_Y - JOY_RADIUS - 30, 200, 12, 12, "PC1(X2) PC3(Y2) PC5(K2)", WGT_CLR_TEXT_SEC);
    /* 恢复原始背景色 */
    g_back_color = old_bg;
}

/* ======================== 重绘被覆盖的背景元素 ======================== */
/**
 * RedrawElements() — 修复被指示圆点覆盖的背景线条和圆圈
 *
 * 当旧的指示圆点被白色填充擦除后，十字准线和参考圆也会被一并擦除。
 * 此函数检测圆点是否靠近某条线或某个圆，并重新绘制被破坏的部分。
 *
 * @param dx, dy  旧指示圆点的屏幕坐标
 * @param cx, cy  对应摇杆的圆心坐标
 * @param color   外圆边框颜色（左红右蓝）
 */
static void RedrawElements(int dx, int dy, int cx, int cy, uint16_t color)
{
    /* 如果旧点靠近垂直十字准线（X方向偏差小），则重绘垂直线 */
    if (abs(dx - cx) <= DOT_RADIUS + 1)
        lcd_draw_line(cx, cy - JOY_RADIUS, cx, cy + JOY_RADIUS, GRAY);
    /* 如果旧点靠近水平十字准线（Y方向偏差小），则重绘水平线 */
    if (abs(dy - cy) <= DOT_RADIUS + 1)
        lcd_draw_line(cx - JOY_RADIUS, cy, cx + JOY_RADIUS, cy, GRAY);

    /* 计算旧点到圆心的距离 */
    int dist = sqrt((dx - cx) * (dx - cx) + (dy - cy) * (dy - cy));
    /* 如果旧点靠近死区内圈边界，重绘死区圆 */
    if (abs(dist - INNER_RADIUS) <= DOT_RADIUS + 1)
        lcd_draw_circle(cx, cy, INNER_RADIUS, LGRAY);
    /* 如果旧点靠近半径参考圆，重绘半径参考圆 */
    if (abs(dist - JOY_RADIUS / 2) <= DOT_RADIUS + 1)
        lcd_draw_circle(cx, cy, JOY_RADIUS / 2, GRAY);
    /* 如果旧点靠近外圆边界，重绘外圆 */
    if (abs(dist - JOY_RADIUS) <= DOT_RADIUS + 1)
        lcd_draw_circle(cx, cy, JOY_RADIUS, color);
}

/* ======================== 更新摇杆图形 ======================== */
/**
 * UpdateGraph() — 更新双摇杆在LCD上的指示圆点位置
 *
 * 处理流程：
 *   1. 将ADC原始值（0~4095）归一化到 -1.0 ~ +1.0 范围
 *      - 2048为ADC中点（12位ADC满量程4096的一半）
 *      - Y轴取反是因为LCD坐标Y轴向下增长，而摇杆向上为正
 *   2. 如果归一化后的向量长度超过1.0（超出圆形范围），则等比缩放到圆边界
 *   3. 将归一化坐标映射为LCD像素坐标（圆心 + 偏移量 * 有效半径）
 *   4. 用白色填充擦除旧圆点，重绘被覆盖的背景元素
 *   5. 在新位置绘制彩色指示圆点
 *
 * @param js  包含双摇杆ADC原始值的结构体指针
 */
static void UpdateGraph(JoystickState *js)
{
    /* ---- 左摇杆坐标归一化 ---- */
    /* 减去2048使中点为0，除以2048得到 -1.0~+1.0 范围 */
    float x1n = (float)(js->x1 - 2048) / 2048.0f;
    /* Y轴取反：摇杆向上推时y1值增大，但LCD需要向上偏移为负值 */
    float y1n = -(float)(js->y1 - 2048) / 2048.0f;
    /* 计算向量长度（距中心的距离） */
    float r1 = sqrtf(x1n * x1n + y1n * y1n);
    /* 如果超出圆形显示范围，等比缩放到单位圆边界 */
    if (r1 > 1.0f) { x1n /= r1; y1n /= r1; }

    /* 将归一化坐标转换为LCD像素坐标 */
    /* 有效半径 = 外圆半径 - 圆点半径，防止圆点画到圆外 */
    int x1p = JOY1_X + (int)(x1n * (JOY_RADIUS - DOT_RADIUS));
    int y1p = JOY1_Y - (int)(y1n * (JOY_RADIUS - DOT_RADIUS));

    /* ---- 右摇杆坐标归一化（流程相同）---- */
    float x2n = (float)(js->x2 - 2048) / 2048.0f;
    float y2n = (float)(js->y2 - 2048) / 2048.0f;
    float r2 = sqrtf(x2n * x2n + y2n * y2n);
    if (r2 > 1.0f) { x2n /= r2; y2n /= r2; }

    int x2p = JOY2_X + (int)(x2n * (JOY_RADIUS - DOT_RADIUS));
    int y2p = JOY2_Y - (int)(y2n * (JOY_RADIUS - DOT_RADIUS));

    /* ---- 擦除旧圆点并修复背景 ---- */
    /* 首次运行时没有旧圆点需要擦除，跳过此步 */
    if (!first_run) {
        /* 用白色填充擦除上一帧的圆点 */
        lcd_fill_circle(last_x1, last_y1, DOT_RADIUS, WHITE);
        lcd_fill_circle(last_x2, last_y2, DOT_RADIUS, WHITE);
        /* 重绘被白色填充破坏的十字准线和参考圆 */
        RedrawElements(last_x1, last_y1, JOY1_X, JOY1_Y, RED);
        RedrawElements(last_x2, last_y2, JOY2_X, JOY2_Y, BLUE);
    }

    /* ---- 绘制新圆点 ---- */
    /* 左摇杆用洋红色，右摇杆用青色，便于区分 */
    lcd_fill_circle(x1p, y1p, DOT_RADIUS, MAGENTA);
    lcd_fill_circle(x2p, y2p, DOT_RADIUS, CYAN);

    /* 保存当前圆点坐标，供下次擦除使用 */
    last_x1 = x1p; last_y1 = y1p;
    last_x2 = x2p; last_y2 = y2p;
    /* 首次运行完成，后续帧都需要擦除旧圆点 */
    first_run = 0;
}

/* ======================== 显示数值 ======================== */
/**
 * DisplayValues() — 在LCD上显示摇杆的ADC原始值、归一化值和按键状态
 *
 * 显示内容：
 *   - 左摇杆X/Y轴：ADC原始值 + 归一化浮点值
 *   - 右摇杆X/Y轴：ADC原始值 + 归一化浮点值
 *   - 两个按键状态：[ON] 表示按下，[OFF] 表示释放
 *
 * @param js  包含双摇杆ADC原始值和按键状态的结构体指针
 */
static void DisplayValues(JoystickState *js)
{
    char buf[50];
    /* 将ADC原始值归一化到 -1.0 ~ +1.0（此处仅用于显示，不取反Y轴） */
    float x1n = (float)(js->x1 - 2048) / 2048.0f;
    float y1n = (float)(js->y1 - 2048) / 2048.0f;
    float x2n = (float)(js->x2 - 2048) / 2048.0f;
    float y2n = (float)(js->y2 - 2048) / 2048.0f;

    /* 临时切换背景色，确保文字区域背景与模块背景一致 */
    uint32_t old_bg = g_back_color;
    g_back_color = WGT_CLR_BG;

    /* 左摇杆X轴：显示ADC原始值和归一化值，格式如 "L-X:2048(+0.00)" */
    snprintf(buf, sizeof(buf), "L-X:%4d(%+.2f)", js->x1, x1n);
    lcd_show_string(14, INFO_Y + 30, 200, 16, 16, buf, WGT_CLR_KEY_NEG);
    /* 左摇杆Y轴 */
    snprintf(buf, sizeof(buf), "L-Y:%4d(%+.2f)", js->y1, y1n);
    lcd_show_string(14, INFO_Y + 50, 200, 16, 16, buf, WGT_CLR_KEY_NEG);

    /* 右摇杆X轴，显示在屏幕右侧 */
    snprintf(buf, sizeof(buf), "R-X:%4d(%+.2f)", js->x2, x2n);
    lcd_show_string(250, INFO_Y + 30, 200, 16, 16, buf, WGT_CLR_ACCENT);
    /* 右摇杆Y轴 */
    snprintf(buf, sizeof(buf), "R-Y:%4d(%+.2f)", js->y2, y2n);
    lcd_show_string(250, INFO_Y + 50, 200, 16, 16, buf, WGT_CLR_ACCENT);

    /* 按键状态显示区域 */
    lcd_show_string(14, INFO_Y + 75, 80, 16, 16, "Keys:", WGT_CLR_TEXT_PRI);
    /* 左按键：k1为真时显示[ON]并用高亮色，否则显示[OFF]用灰色 */
    lcd_show_string(80,  INFO_Y + 75, 40, 16, 16, js->k1 ? "[ON]" : "[OFF]", js->k1 ? WGT_CLR_KEY_NEG : WGT_CLR_TEXT_SEC);
    /* 右按键：k2同理 */
    lcd_show_string(130, INFO_Y + 75, 40, 16, 16, js->k2 ? "[ON]" : "[OFF]", js->k2 ? WGT_CLR_ACCENT : WGT_CLR_TEXT_SEC);

    /* 恢复原始背景色 */
    g_back_color = old_bg;
}

/* ======================== 模块接口 ======================== */
/**
 * joystick_enter() — 模块进入函数
 *
 * 重置所有状态变量，绘制初始界面，记录初始时间戳。
 * 由菜单系统在切换到摇杆模块时调用。
 */
static void joystick_enter(void)
{
    first_run = 1;                              /* 标记首次运行，跳过擦除旧圆点 */
    last_x1 = last_y1 = last_x2 = last_y2 = 0; /* 清零旧坐标 */
    DrawLayout();                               /* 绘制摇杆界面布局 */
    last_update_tick = HAL_GetTick();           /* 记录初始时间戳 */
}

/**
 * joystick_tick() — 模块周期调度函数
 *
 * 每次被主循环调用时执行以下操作：
 *   1. 检测触摸键盘 'n' 键，按下则退出模块
 *   2. 检查是否达到5ms刷新间隔（200Hz刷新率）
 *   3. 读取摇杆ADC数据
 *   4. 更新数值显示和图形显示
 */
static void joystick_tick(void)
{
    /* 检测触摸键盘的退出按键 */
    int key = detect_key_press();
    if (key == 'n') {
        module_exit_current();  /* 通知模块系统退出当前模块 */
        return;
    }

    /* 节流控制：每5ms刷新一次（约200Hz），避免过度刷新导致画面闪烁 */
    uint32_t now = HAL_GetTick();
    if (now - last_update_tick < 5) return;  /* 未到刷新间隔，直接返回 */

    /* 通过BSP层读取双摇杆的ADC值和按键状态 */
    JoystickState js;
    Joystick_Read(&js);
    /* 在LCD上更新数值文本 */
    DisplayValues(&js);
    /* 在LCD上更新图形圆点 */
    UpdateGraph(&js);
    /* 记录本次刷新时间 */
    last_update_tick = now;
}

/**
 * joystick_exit() — 模块退出函数
 *
 * 当前无需特殊清理（LCD画面会被下一个模块的enter覆盖）。
 */
static void joystick_exit(void)
{
    /* 无需清理资源，画面会被后续模块覆盖 */
}

/* ======================== 模块导出 ======================== */
/* 模块接口结构体，供菜单系统注册和调用 */
const ModuleInterface joystick_module = {
    .name  = "Joystick",         /* 模块名称，显示在菜单中 */
    .enter = joystick_enter,     /* 进入模块时调用 */
    .tick  = joystick_tick,      /* 主循环周期调用 */
    .exit  = joystick_exit,      /* 退出模块时调用 */
};
