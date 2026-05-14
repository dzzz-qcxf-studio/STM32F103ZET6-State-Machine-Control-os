/**
 * @file    lcd_widgets.c
 * @brief   LCD UI 组件库实现
 *
 * 本文件提供嵌入式触摸屏项目中常用的 UI 绘图基元和组件，
 * 包括圆角矩形、按钮、卡片、滚动条、键盘按键、状态标签等。
 * 所有组件均基于底层 lcd 驱动（lcd_fill / lcd_draw_point / lcd_show_string 等）实现，
 * 支持选中高亮、按下闪烁等交互视觉反馈。
 */

#include "./BSP/LCD/lcd_widgets.h"
#include "./BSP/LCD/lcd.h"
#include <stdio.h>
#include <string.h>

/* ======================== 绘图基元 ======================== */

/**
 * @brief 填充圆角矩形（实心）
 *
 * 算法思路：
 *   将圆角矩形分解为 1 个中央矩形 + 左右两个窄条 + 四个圆角区域，
 *   分步填充以避免重复绘制。
 *
 *   1. 圆角半径 r == 0 时退化为普通矩形，直接 lcd_fill 填满。
 *   2. 中间主体：x 从 (x1+r) 到 (x2-r)，覆盖全部 y 范围。
 *   3. 左侧条带：x 从 x1 到 (x1+r-1)，y 从 (y1+r) 到 (y2-r)，
 *      避开上下圆角区域。
 *   4. 右侧条带：x 从 (x2-r+1) 到 x2，y 范围同上。
 *   5. 四个圆角：逐行扫描 dy = 0..r-1，利用圆的方程 r^2 = dx^2 + dy^2
 *      计算当前行在圆角内需要填充的水平宽度 dx。
 *      - left = r^2 - (r-dy)^2 是当前行对应的 dx^2 最大值
 *      - 通过 while 循环求 dx = floor(sqrt(left))
 *      - 然后在四个对称角落各画一条水平短线段
 *
 * @param x1, y1   左上角坐标
 * @param x2, y2   右下角坐标
 * @param r        圆角半径（像素）
 * @param color    填充颜色（RGB565）
 */
void lcd_draw_rounded_rect(uint16_t x1, uint16_t y1,
                           uint16_t x2, uint16_t y2,
                           uint16_t r, uint16_t color)
{
    /* 圆角半径为 0 时，直接填充完整矩形即可 */
    if (r == 0) {
        lcd_fill(x1, y1, x2, y2, color);
        return;
    }

    /* 步骤1：填充中央水平矩形（左右各留出 r 像素给圆角） */
    lcd_fill(x1 + r, y1, x2 - r, y2, color);

    /* 步骤2：填充左侧竖直条带（上下各留出 r 像素给圆角） */
    lcd_fill(x1, y1 + r, x1 + r - 1, y2 - r, color);

    /* 步骤3：填充右侧竖直条带（上下各留出 r 像素给圆角） */
    lcd_fill(x2 - r + 1, y1 + r, x2, y2 - r, color);

    /* 步骤4：逐行计算并填充四个圆角区域 */
    for (uint16_t dy = 0; dy < r; dy++) {
        /*
         * 圆角是半径为 r 的四分之一圆弧。
         * 圆心在 (x1+r, y1+r) 等位置，dy 是当前行到圆心的垂直距离。
         * 利用圆方程: r^2 = dx^2 + (r-dy)^2
         * 求出当前行圆弧内侧的最大水平偏移 dx。
         */
        uint16_t left = r * r - (r - dy) * (r - dy);

        /* 用逐步递增法求 dx = floor(sqrt(left))，避免引入 math.h */
        uint16_t dx = 0;
        while ((dx + 1) * (dx + 1) <= left) dx++;

        if (dx > 0) {
            /* 左上角圆弧：从圆心向左填充 dx 个像素 */
            lcd_fill(x1 + r - dx, y1 + dy, x1 + r - 1, y1 + dy, color);
            /* 右上角圆弧：从圆心向右填充 dx 个像素 */
            lcd_fill(x2 - r + 1, y1 + dy, x2 - r + dx, y1 + dy, color);
            /* 左下角圆弧：与左上对称，y 从底部向上 */
            lcd_fill(x1 + r - dx, y2 - dy, x1 + r - 1, y2 - dy, color);
            /* 右下角圆弧：与右上对称，y 从底部向上 */
            lcd_fill(x2 - r + 1, y2 - dy, x2 - r + dx, y2 - dy, color);
        }
    }
}

/**
 * @brief 绘制圆角矩形边框（空心，仅轮廓线）
 *
 * 算法思路：
 *   与 lcd_draw_rounded_rect 类似，但只绘制轮廓而非填充整个区域。
 *   分为 4 条直线边 + 4 段圆弧：
 *   - 上水平边：从 (x1+r, y1) 到 (x2-r, y1)，单行填充
 *   - 下水平边：从 (x1+r, y2) 到 (x2-r, y2)
 *   - 左垂直边：从 (x1, y1+r) 到 (x1, y2-r)，单列填充
 *   - 右垂直边：从 (x2, y1+r) 到 (x2, y2-r)
 *   - 四段圆弧：逐行扫描 dy，用圆方程计算 dx，每行画 2 个像素点
 *     （主点 + 相邻补点）以确保弧线连续无断点
 *
 * @param x1, y1   左上角坐标
 * @param x2, y2   右下角坐标
 * @param r        圆角半径（像素）
 * @param color    边框颜色（RGB565）
 */
void lcd_draw_rounded_rect_border(uint16_t x1, uint16_t y1,
                                  uint16_t x2, uint16_t y2,
                                  uint16_t r, uint16_t color)
{
    /* 圆角半径为 0 时，退化为普通矩形边框 */
    if (r == 0) {
        lcd_draw_rectangle(x1, y1, x2, y2, color);
        return;
    }

    /* 水平边：上边和下边，x 范围跳过左右圆角区域 */
    lcd_fill(x1 + r, y1, x2 - r, y1, color);  /* 上边 */
    lcd_fill(x1 + r, y2, x2 - r, y2, color);  /* 下边 */

    /* 垂直边：左边和右边，y 范围跳过上下圆角区域 */
    lcd_fill(x1, y1 + r, x1, y2 - r, color);  /* 左边 */
    lcd_fill(x2, y1 + r, x2, y2 - r, color);  /* 右边 */

    /* 四角弧线：逐行扫描，用圆方程计算弧线上的像素位置 */
    for (uint16_t dy = 0; dy < r; dy++) {
        /*
         * 与填充圆角矩形相同的圆方程计算：
         * left = r^2 - (r-dy)^2, 求 dx = floor(sqrt(left))
         * dx 表示当前行从圆心水平方向需要偏移的像素数
         */
        uint16_t left = r * r - (r - dy) * (r - dy);
        uint16_t dx = 0;
        while ((dx + 1) * (dx + 1) <= left) dx++;

        if (dx > 0) {
            /*
             * 每个角画 2 个相邻像素点（主点 + 补点），
             * 目的是填补可能的像素间隙，保证弧线视觉上连续平滑。
             * 主点在弧线精确位置，补点在其右侧/左侧相邻位置。
             */

            /* 左上角弧线：主点 + 右侧补点 */
            lcd_draw_point(x1 + r - dx, y1 + dy, color);
            lcd_draw_point(x1 + r - dx + 1, y1 + dy, color);
            /* 右上角弧线：主点 + 左侧补点 */
            lcd_draw_point(x2 - r + dx, y1 + dy, color);
            lcd_draw_point(x2 - r + dx - 1, y1 + dy, color);
            /* 左下角弧线：与左上对称（y 镜像） */
            lcd_draw_point(x1 + r - dx, y2 - dy, color);
            lcd_draw_point(x1 + r - dx + 1, y2 - dy, color);
            /* 右下角弧线：与右上对称（y 镜像） */
            lcd_draw_point(x2 - r + dx, y2 - dy, color);
            lcd_draw_point(x2 - r + dx - 1, y2 - dy, color);
        }
    }
}

/**
 * @brief 绘制向右的实心小三角箭头（形状如 "▸"）
 *
 * 算法思路：
 *   逐行绘制，每行是一个竖直像素段。i 从 0 递增到 sz-1，
 *   第 i 行的 x 坐标为 (cx + i)，即每行向右移动 1 像素；
 *   y 方向从 (cy - i) 到 (cy + i)，即高度逐行扩展 2 像素。
 *   最终形成一个底边在左、尖端在右的等腰三角形。
 *
 *   例如 sz=4 时：
 *     i=0: 垂直线段，高 1 像素（仅 cy）
 *     i=1: 垂直线段，高 3 像素（cy-1 到 cy+1）
 *     i=2: 垂直线段，高 5 像素（cy-2 到 cy+2）
 *     i=3: 垂直线段，高 7 像素（cy-3 到 cy+3）
 *
 * @param cx    箭头尖端的 x 坐标（三角形左顶点）
 * @param cy    箭头垂直中心 y 坐标
 * @param sz    箭头大小（即三角形的半宽/行数）
 * @param color 箭头颜色（RGB565）
 */
void lcd_draw_arrow(uint16_t cx, uint16_t cy, uint16_t sz, uint16_t color)
{
    for (uint16_t i = 0; i < sz; i++) {
        /* 第 i 行：x = cx+i，y 范围从 (cy-i) 到 (cy+i)，形成逐渐变宽的竖线 */
        lcd_fill(cx + i, cy - i, cx + i, cy + i, color);
    }
}

/* ======================== 标题栏 ======================== */

/**
 * @brief 绘制页面顶部的深色标题栏
 *
 * 布局结构：
 *   ┌─────────────────────────────────────────┐
 *   │  [标题文字]                    [N/总数]  │  <- 高度 WGT_HEADER_H
 *   └─────────────────────────────────────────┘
 *
 * 实现细节：
 *   1. 用圆角半径 0 的圆角矩形（即纯矩形）填充深色背景
 *   2. 左侧 x=16 处绘制白色标题文字，垂直居中（(H-16)/2）
 *   3. 右侧绘制 "当前/总数" 计数器（仅当 total 和 current 均 > 0 时显示），
 *      使用 12px 字体，垂直居中
 *   4. 绘制前设置 g_back_color 为标题栏背景色，绘制后恢复原背景色
 *      （因为 lcd_show_string 需要 g_back_color 来绘制文字背景）
 *
 * @param title    标题字符串
 * @param current  当前页/项序号（1-based），0 则不显示计数
 * @param total    总页/项数，0 则不显示计数
 */
void lcd_wgt_header(const char *title, uint8_t current, uint8_t total)
{
    uint16_t w = lcddev.width;
    uint32_t old_bg = g_back_color;

    /* 步骤1：用深色填充整个标题栏区域（圆角半径0 = 纯矩形） */
    lcd_draw_rounded_rect(0, 0, w - 1, WGT_HEADER_H - 1, 0, WGT_CLR_HEADER_BG);

    /* 步骤2：左侧绘制白色标题文字（x=16，垂直居中） */
    g_back_color = WGT_CLR_HEADER_BG;
    lcd_show_string(16, (WGT_HEADER_H - 16) / 2, 300, 16, 16, (char *)title, WGT_CLR_TEXT_WHT);

    /* 步骤3：右侧绘制 "当前/总数" 计数器（12px 小字体，仅在有效范围内显示） */
    if (total > 0 && current > 0) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d/%d", current, total);
        lcd_show_string(w - 64, (WGT_HEADER_H - 12) / 2, 56, 12, 12, buf, WGT_CLR_TEXT_WHT);
    }

    /* 恢复调用者原来的背景色，避免影响后续绘图 */
    g_back_color = old_bg;
}

/* ======================== 卡片 ======================== */

/**
 * @brief 绘制菜单卡片组件（带序号 + 文字）
 *
 * 视觉结构（选中状态）：
 *   ┌──────────────────────────────────┐
 *   │▌ [序号]  文本内容             ▸  │  蓝色背景 + 左侧白色竖条 + 右侧三角箭头
 *   └──────────────────────────────────┘
 *
 * 视觉结构（未选中状态）：
 *   ┌──────────────────────────────────┐
 *   │   [序号]  文本内容               │  白色卡片 + 灰色边框
 *   └──────────────────────────────────┘
 *
 * 两种状态的区别：
 *   - 选中：蓝色实心背景、白色文字、左侧白色指示条、右侧向右箭头
 *   - 未选中：白色卡片（用灰色边框画外框，内缩1px画白色填充）、
 *     深色文字、序号用次要灰色、无箭头
 *
 * @param x1, y1     卡片左上角坐标
 * @param x2, y2     卡片右下角坐标
 * @param r          圆角半径
 * @param index_num  序号（1-based），0 则不显示；显示时取个位数 (% 10)
 * @param text       卡片显示的文字
 * @param selected   1=选中状态，0=未选中状态
 */
void lcd_wgt_card(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                  uint16_t r, uint8_t index_num, const char *text,
                  uint8_t selected)
{
    uint16_t h = y2 - y1;       /* 卡片高度，用于计算文字垂直位置 */
    uint32_t old_bg = g_back_color;

    if (selected) {
        /* ====== 选中状态 ====== */

        /* 用蓝色填充整个圆角矩形作为选中背景 */
        lcd_draw_rounded_rect(x1, y1, x2, y2, r, WGT_CLR_CARD_SEL);

        /* 左侧白色竖条指示器：表示当前选中项，上下各留 3px 边距 */
        lcd_fill(x1, y1 + 3, x1 + WGT_CARD_BAR_W - 1, y2 - 3, WGT_CLR_TEXT_WHT);

        /* 设置背景色为选中蓝色，供 lcd_show_string 绘制文字背景 */
        g_back_color = WGT_CLR_CARD_SEL;

        /* 左侧序号：白色文字，x=12 偏移留出左侧条宽度 */
        if (index_num > 0) {
            char num[4];
            snprintf(num, sizeof(num), "%d", index_num % 10);
            lcd_show_string(x1 + 12, y1 + 2, 20, h - 4, 16, num, WGT_CLR_TEXT_WHT);
        }

        /* 卡片文本：白色文字，x=32 为序号区域之后，宽度留出右侧箭头空间 */
        lcd_show_string(x1 + 32, y1 + 2, (x2 - x1) - 56, h - 4, 16,
                        (char *)text, WGT_CLR_TEXT_WHT);

        /* 右侧向右小三角箭头，垂直居中，表示"可进入" */
        lcd_draw_arrow(x2 - 18, (y1 + y2) / 2, 4, WGT_CLR_TEXT_WHT);
    } else {
        /* ====== 未选中状态 ====== */

        /*
         * 画法：先用灰色画完整圆角矩形（作为边框），
         * 再内缩 1 像素用白色画稍小的圆角矩形（作为填充），
         * 形成 1px 灰色边框 + 白色内部的视觉效果。
         * 内部圆角半径 r-1 以保持弧度一致。
         */
        lcd_draw_rounded_rect(x1, y1, x2, y2, r, WGT_CLR_CARD_BRD);       /* 外框：灰色边框 */
        lcd_draw_rounded_rect(x1 + 1, y1 + 1, x2 - 1, y2 - 1, r - 1, WGT_CLR_CARD_BG);  /* 内部：白色填充 */

        /* 设置背景色为白色，供文字绘制 */
        g_back_color = WGT_CLR_CARD_BG;

        /* 序号：次要灰色文字 */
        if (index_num > 0) {
            char num[4];
            snprintf(num, sizeof(num), "%d", index_num % 10);
            lcd_show_string(x1 + 12, y1 + 2, 20, h - 4, 16, num, WGT_CLR_TEXT_SEC);
        }

        /* 文本：主要深色文字 */
        lcd_show_string(x1 + 32, y1 + 2, (x2 - x1) - 56, h - 4, 16,
                        (char *)text, WGT_CLR_TEXT_PRI);
        /* 未选中状态不绘制右侧箭头 */
    }

    /* 恢复调用者原来的背景色 */
    g_back_color = old_bg;
}

/**
 * @brief 绘制带图标的菜单卡片（图标 + 文字）
 *
 * 视觉结构（选中状态）：
 *   ┌──────────────────────────────────┐
 *   │▌ [ICON]  文本内容             ▸  │  蓝色背景 + 白色图标 + 白色文字 + 箭头
 *   └──────────────────────────────────┘
 *
 * 视觉结构（未选中状态）：
 *   ┌──────────────────────────────────┐
 *   │   [ICON]  文本内容               │  白色卡片 + 灰边框 + 强调色图标 + 深色文字
 *   └──────────────────────────────────┘
 *
 * 与 lcd_wgt_card() 的区别：
 *   - 用 icon 字符串替代序号（通常是一个 Unicode/ASCII 图标字符）
 *   - icon 区域 x 偏移 12，宽度 24；文本起始 x 偏移 40（比序号卡片的 32 更靠右）
 *   - 未选中时 icon 使用强调色（WGT_CLR_ACCENT）而非灰色
 *
 * @param x1, y1   卡片左上角坐标
 * @param x2, y2   卡片右下角坐标
 * @param r        圆角半径
 * @param icon     图标字符串（如 Unicode 图标或单字符符号）
 * @param text     卡片显示的文字
 * @param selected 1=选中，0=未选中
 */
void lcd_wgt_card_icon(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                       uint16_t r, const char *icon, const char *text,
                       uint8_t selected)
{
    uint32_t old_bg = g_back_color;

    if (selected) {
        /* ====== 选中状态：蓝色背景 ====== */

        /* 蓝色填充整个圆角矩形 */
        lcd_draw_rounded_rect(x1, y1, x2, y2, r, WGT_CLR_CARD_SEL);

        /* 左侧白色竖条指示器 */
        lcd_fill(x1, y1 + 3, x1 + WGT_CARD_BAR_W - 1, y2 - 3, WGT_CLR_TEXT_WHT);

        g_back_color = WGT_CLR_CARD_SEL;

        /* 图标：白色，x 偏移 12，宽度 24px */
        lcd_show_string(x1 + 12, y1 + 2, 24, y2 - y1 - 4, 16, (char *)icon, WGT_CLR_TEXT_WHT);

        /* 文本：白色，x 偏移 40（在图标右侧），宽度留出右侧箭头空间 */
        lcd_show_string(x1 + 40, y1 + 2, (x2 - x1) - 64, y2 - y1 - 4, 16,
                        (char *)text, WGT_CLR_TEXT_WHT);

        /* 右侧向右小三角箭头 */
        lcd_draw_arrow(x2 - 18, (y1 + y2) / 2, 4, WGT_CLR_TEXT_WHT);
    } else {
        /* ====== 未选中状态：白色卡片 + 灰色边框 ====== */

        /* 灰色边框 + 白色内部（与 lcd_wgt_card 相同的双层画法） */
        lcd_draw_rounded_rect(x1, y1, x2, y2, r, WGT_CLR_CARD_BRD);
        lcd_draw_rounded_rect(x1 + 1, y1 + 1, x2 - 1, y2 - 1, r - 1, WGT_CLR_CARD_BG);

        g_back_color = WGT_CLR_CARD_BG;

        /* 图标：使用强调色（如蓝色），让未选中卡片也有视觉亮点 */
        lcd_show_string(x1 + 12, y1 + 2, 24, y2 - y1 - 4, 16, (char *)icon, WGT_CLR_ACCENT);

        /* 文本：主要深色文字 */
        lcd_show_string(x1 + 40, y1 + 2, (x2 - x1) - 64, y2 - y1 - 4, 16,
                        (char *)text, WGT_CLR_TEXT_PRI);
    }

    /* 恢复调用者原来的背景色 */
    g_back_color = old_bg;
}

/* ======================== 按钮 ======================== */

/**
 * @brief 根据按钮风格编号获取背景颜色
 *
 * 风格映射：
 *   style=0（默认）：WGT_CLR_KEY_BG   — 普通灰色按钮
 *   style=1         ：WGT_CLR_KEY_ACT  — 主要操作按钮（蓝色）
 *   style=2         ：WGT_CLR_KEY_NEG  — 危险/否定操作按钮（红色）
 *
 * @param style  风格编号（0/1/2）
 * @return 对应的 RGB565 颜色值
 */
static uint16_t get_button_color(uint8_t style)
{
    switch (style) {
    case 1:  return WGT_CLR_KEY_ACT;   /* 主要：蓝色，用于确认/提交等正向操作 */
    case 2:  return WGT_CLR_KEY_NEG;   /* 危险：红色，用于删除/取消等破坏性操作 */
    default: return WGT_CLR_KEY_BG;    /* 普通：灰色，用于一般按钮 */
    }
}

/**
 * @brief 绘制按钮组件（圆角矩形 + 边框 + 居中文字）
 *
 * 绘制流程：
 *   1. 根据 style 调用 get_button_color() 获取背景色
 *   2. 用背景色填充圆角矩形作为按钮主体
 *   3. 再画一层圆角矩形边框（与主体同尺寸叠加，形成轮廓线）
 *   4. 计算文字居中位置：
 *      - 16px 字体每字符约 8px 宽（半角 ASCII），text_w = len * 8
 *      - 水平居中：tx = x1 + (按钮宽度 - 文字宽度) / 2
 *      - 垂直居中：ty = y1 + (按钮高度 - 16) / 2
 *   5. 设置 g_back_color 为按钮背景色后绘制白色文字
 *
 * @param x1, y1   按钮左上角坐标
 * @param x2, y2   按钮右下角坐标
 * @param r        圆角半径
 * @param label    按钮文字标签
 * @param style    按钮风格（0=普通灰, 1=主要蓝, 2=危险红）
 */
void lcd_wgt_button(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                    uint16_t r, const char *label, uint8_t style)
{
    /* 根据风格获取背景色 */
    uint16_t bg = get_button_color(style);
    uint32_t old_bg = g_back_color;

    /* 用风格色填充按钮主体（圆角矩形） */
    lcd_draw_rounded_rect(x1, y1, x2, y2, r, bg);

    /* 叠加绘制边框轮廓线（与主体同尺寸，边框像素覆盖在主体之上） */
    lcd_draw_rounded_rect_border(x1, y1, x2, y2, r, WGT_CLR_KEY_BRD);

    /* 计算文字水平居中位置：假设 16px 字体每个字符宽约 8px */
    g_back_color = bg;
    uint8_t len = strlen(label);
    uint16_t text_w = len * 8;  /* 16px 字体，每字符约 8px 宽（半角） */
    uint16_t tx = x1 + ((x2 - x1) - text_w) / 2;  /* 水平居中 */
    uint16_t ty = y1 + ((y2 - y1) - 16) / 2;       /* 垂直居中 */

    /* 绘制白色文字（+8 余量避免截断） */
    lcd_show_string(tx, ty, text_w + 8, 16, 16, (char *)label, WGT_CLR_TEXT_WHT);

    /* 恢复调用者原来的背景色 */
    g_back_color = old_bg;
}

/**
 * @brief 按钮按下闪烁效果（视觉反馈）
 *
 * 实现原理：
 *   触摸事件触发时调用此函数，产生短暂的白色闪烁，
 *   让用户感知到"按下了按钮"。具体步骤：
 *   1. 立即用白色填充按钮区域（整个圆角矩形变白）
 *   2. 延时 30ms（肉眼可感知的短暂闪烁）
 *   3. 调用 lcd_wgt_button() 重新绘制正常状态的按钮
 *
 * 整个过程耗时约 30ms，在嵌入式触摸屏上提供即时的按压视觉反馈。
 *
 * @param x1, y1   按钮左上角坐标
 * @param x2, y2   按钮右下角坐标
 * @param r        圆角半径
 * @param label    按钮文字标签
 * @param style    按钮风格（0/1/2）
 */
void lcd_wgt_button_press(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                          uint16_t r, const char *label, uint8_t style)
{
    /* 阶段1：用白色覆盖按钮区域，模拟"闪亮"效果 */
    lcd_draw_rounded_rect(x1, y1, x2, y2, r, WGT_CLR_TEXT_WHT);

    /* 阶段2：保持白色 30ms，让用户肉眼能感知到闪烁 */
    HAL_Delay(30);

    /* 阶段3：恢复绘制正常状态的按钮（包含背景、边框、文字） */
    lcd_wgt_button(x1, y1, x2, y2, r, label, style);
}

/* ======================== 滚动条 ======================== */

/**
 * @brief 绘制垂直滚动条（轨道 + 比例滑块）
 *
 * 视觉结构：
 *   ┌──┐  x 坐标
 *   │  │  浅色轨道（宽度 4px）
 *   │██│  深色滑块（高度按内容比例缩放）
 *   │  │
 *   │  │
 *   └──┘  y_start + track_len
 *
 * 滑块高度计算：
 *   thumb_h = (可见数量 / 总数量) * 轨道长度
 *   最小值限制为 16px，避免滑块太小无法看清
 *
 * 滑块位置计算：
 *   可滚动范围 = total_count - visible_count（即 max_scroll）
 *   滑块在轨道内的偏移 = scroll_pos / max_scroll * (轨道长 - 滑块高)
 *   这样 scroll_pos=0 时滑块在最顶部，scroll_pos=max_scroll 时在最底部
 *
 * 提前返回条件：
 *   当总数量 <= 可见数量时，不需要滚动条，直接返回不绘制
 *
 * @param x             滚动条 x 坐标（左侧）
 * @param y_start       轨道起始 y 坐标
 * @param track_len     轨道总长度（像素）
 * @param visible_count 屏幕可显示的项目数
 * @param total_count   总项目数
 * @param scroll_pos    当前滚动位置（0 = 最顶部）
 */
void lcd_wgt_scrollbar(uint16_t x, uint16_t y_start, uint16_t track_len,
                       uint8_t visible_count, uint8_t total_count,
                       uint8_t scroll_pos)
{
    /* 总项目数不超过可见数量时无需滚动条 */
    if (total_count <= visible_count) return;

    /* 绘制浅色轨道背景：4px 宽的竖直矩形 */
    lcd_fill(x, y_start, x + 3, y_start + track_len, WGT_CLR_SCROLL_TRK);

    /* 计算滑块高度：按可见比例缩放，最小 16px */
    uint16_t thumb_h = (visible_count * track_len) / total_count;
    if (thumb_h < 16) thumb_h = 16;

    /* 计算滑块在轨道内的 y 位置（线性映射 scroll_pos 到像素偏移） */
    uint16_t max_scroll = total_count - visible_count;
    uint16_t thumb_y = y_start + (scroll_pos * (track_len - thumb_h)) / max_scroll;

    /* 绘制深色滑块 */
    lcd_fill(x, thumb_y, x + 3, thumb_y + thumb_h, WGT_CLR_SCROLL_THM);
}

/* ======================== 键盘 ======================== */

/**
 * @brief 填充键盘区域背景
 *
 * 将从 y_start 到屏幕底部的整个区域填充为键盘背景色。
 * 通常在绘制键盘按键之前调用，先清除该区域的残留内容。
 *
 * @param y_start 键盘区域的起始 y 坐标
 */
void lcd_wgt_keyboard_bg(uint16_t y_start)
{
    /* 从 y_start 到屏幕底部，全宽填充键盘背景色 */
    lcd_fill(0, y_start, lcddev.width, lcddev.height - 1, WGT_CLR_KBD_BG);
}

/**
 * @brief 绘制单个键盘按键（圆角矩形 + 居中字符）
 *
 * 颜色编码规则：
 *   - 'y'（确认键）：蓝色背景（WGT_CLR_KEY_ACT），视觉上突出为正向操作
 *   - 'n'（取消键）：红色背景（WGT_CLR_KEY_NEG），视觉上警示为否定操作
 *   - 其他字符      ：灰色背景（WGT_CLR_KEY_BG），普通按键
 *
 * 绘制流程：
 *   1. 根据字符确定背景色
 *   2. 填充圆角矩形作为按键主体
 *   3. 叠加绘制边框轮廓
 *   4. 将单字符转为字符串（ch + '\0'）
 *   5. 计算字符居中位置（假设 16px 字体宽 16px、高 16px）
 *   6. 设置 g_back_color 后绘制白色字符
 *
 * @param x1, y1   按键左上角坐标
 * @param x2, y2   按键右下角坐标
 * @param r        圆角半径
 * @param ch       按键字符（'y'=确认, 'n'=取消, 其他=普通）
 */
void lcd_wgt_key(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                 uint16_t r, char ch)
{
    /* 根据字符类型选择背景色：y=蓝色确认, n=红色取消, 其他=灰色 */
    uint16_t bg = WGT_CLR_KEY_BG;
    if (ch == 'y') bg = WGT_CLR_KEY_ACT;      /* 确认键：蓝色 */
    else if (ch == 'n') bg = WGT_CLR_KEY_NEG;  /* 取消键：红色 */

    /* 填充按键主体 + 边框 */
    lcd_draw_rounded_rect(x1, y1, x2, y2, r, bg);
    lcd_draw_rounded_rect_border(x1, y1, x2, y2, r, WGT_CLR_KEY_BRD);

    /* 将单字符转为以 null 结尾的字符串，供 lcd_show_string 使用 */
    char str[2] = { ch, '\0' };

    /* 计算字符居中位置：假设 16px 字体每个字符宽 16px */
    uint16_t tx = x1 + ((x2 - x1) - 16) / 2;  /* 水平居中 */
    uint16_t ty = y1 + ((y2 - y1) - 16) / 2;   /* 垂直居中 */

    /* 临时切换背景色为按键底色，绘制白色字符后恢复 */
    uint32_t old_bg = g_back_color;
    g_back_color = bg;
    lcd_show_string(tx, ty, 32, 20, 16, str, WGT_CLR_TEXT_WHT);
    g_back_color = old_bg;
}

/**
 * @brief 键盘按键按下闪烁效果（与 lcd_wgt_button_press 原理相同）
 *
 * 实现原理：
 *   1. 白色覆盖按键区域（30ms）模拟按压闪光
 *   2. 延时后调用 lcd_wgt_key() 恢复按键原始外观
 *   提供即时的触摸反馈，增强用户体验
 *
 * @param x1, y1   按键左上角坐标
 * @param x2, y2   按键右下角坐标
 * @param r        圆角半径
 * @param ch       按键字符（用于恢复时确定颜色）
 */
void lcd_wgt_key_press(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                       uint16_t r, char ch)
{
    /* 阶段1：白色覆盖，模拟按下高亮 */
    lcd_draw_rounded_rect(x1, y1, x2, y2, r, WGT_CLR_TEXT_WHT);

    /* 阶段2：保持 30ms 让用户感知到闪烁 */
    HAL_Delay(30);

    /* 阶段3：恢复按键正常外观 */
    lcd_wgt_key(x1, y1, x2, y2, r, ch);
}

/* ======================== 状态标签 ======================== */

/**
 * @brief 绘制状态指示标签（彩色圆点 + 文字说明）
 *
 * 视觉结构：
 *   ● 文字说明
 *   ^
 *   彩色圆点（半径 4px），三种状态颜色：
 *     active=0：灰色（WGT_CLR_TEXT_SEC）— 离线/未激活/未知
 *     active=1：绿色（0x07E0, RGB565）  — 正常/在线/运行中
 *     active=2：红色（WGT_CLR_KEY_NEG） — 异常/错误/告警
 *
 * 注意事项：
 *   - lcd_fill_circle 绘制实心圆点
 *   - 文字背景色由调用者事先设置 g_back_color，本函数不修改它
 *   - 圆点中心在 (x+4, y+8)，文字从 (x+12) 开始，两者水平对齐
 *
 * @param x, y   标签左上角坐标
 * @param text   状态文字描述
 * @param active 状态值（0=灰色离线, 1=绿色正常, 2=红色异常）
 */
void lcd_wgt_status_label(uint16_t x, uint16_t y, const char *text, uint8_t active)
{
    /* 根据状态值选择圆点颜色 */
    uint16_t dot_color = WGT_CLR_TEXT_SEC;      /* 默认灰色（未激活/未知） */
    if (active == 1) dot_color = 0x07E0;        /* 绿色：正常运行 (RGB565 纯绿) */
    else if (active == 2) dot_color = WGT_CLR_KEY_NEG;  /* 红色：异常/告警 */

    /* 绘制半径 4px 的实心彩色圆点，圆心在 (x+4, y+8) */
    lcd_fill_circle(x + 4, y + 8, 4, dot_color);

    /* 在圆点右侧绘制状态文字（背景色由调用者保证已正确设置） */
    lcd_show_string(x + 12, y, 200, 16, 16, (char *)text, WGT_CLR_TEXT_PRI);
}
