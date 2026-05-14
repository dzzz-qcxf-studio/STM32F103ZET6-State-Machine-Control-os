/**
 * @file    usart_ui.c
 * @brief   串口通信终端模块 (USART1, PA9/PA10, 115200)
 *
 * 接收区域显示串口数据，发送区域通过键盘输入。
 * 模块化改造：提供 enter/tick/exit 接口，非阻塞运行。
 * 界面分为上下两部分：上方为RX接收显示区，下方为TX发送输入区。
 * 支持通过触摸键盘输入内容并发送，支持切换波特率。
 */

#include "./OVER/usart_ui.h"
#include "./OVER/module.h"
#include "./OVER/menu.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/lcd_widgets.h"
#include "./SYSTEM/usart/usart.h"
#include <stdio.h>
#include <string.h>

/* ======================== 布局常量 ======================== */
/* RX接收区域起始Y坐标（标题栏下方留4像素间距） */
#define RX_START_Y       (WGT_HEADER_H + 4)
/* RX接收区域高度（像素） */
#define RX_HEIGHT        240
/* TX发送区域起始Y坐标（RX区域下方留6像素间距） */
#define TX_START_Y       (RX_START_Y + RX_HEIGHT + 6)
/* TX发送区域高度（像素） */
#define TX_HEIGHT        110
/* 每行文本的高度（像素） */
#define LINE_HEIGHT      20
/* 每行最大显示字符数（超过则自动换行） */
#define MAX_LINE_CHARS   53
/* RX区域能显示的最大行数 */
#define MAX_LINES        11

/* ======================== 缓冲区 ======================== */
/* TX发送缓冲区大小（字节） */
#define TX_BUF_SIZE  100

/* RX接收缓冲区：存储从串口环形缓冲区读取的文本数据 */
static char    g_rx_buf[MAX_LINES * MAX_LINE_CHARS + 1];
/* RX缓冲区当前有效数据长度 */
static uint16_t g_rx_len = 0;
/* TX发送缓冲区：存储用户通过触摸键盘输入的待发送内容 */
static char    g_tx_buf[TX_BUF_SIZE];
/* TX缓冲区当前输入长度 */
static uint8_t g_tx_len = 0;

/* ======================== 波特率 ======================== */
/* 支持的波特率查找表，从低到高排列 */
static const uint32_t baud_table[] = { 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };
/* 波特率表项数量 */
#define BAUD_COUNT  (sizeof(baud_table) / sizeof(baud_table[0]))
/* 当前选中的波特率索引，4对应115200（最常用） */
static uint8_t baud_idx = 4;  /* 默认 115200 */

/* ======================== 模块状态 ======================== */
/* 模块激活标志，为0时tick函数直接返回（防止模块未初始化时被调用） */
static uint8_t module_active = 0;

/* ======================== 绘制界面 ======================== */
/**
 * DrawBaudInfo() — 在标题栏右侧绘制当前波特率数值
 *
 * 先用背景色填充该区域清除旧数字，再显示新波特率。
 * 波特率切换时调用此函数刷新显示。
 */
static void DrawBaudInfo(void)
{
    char buf[24];
    uint32_t old_bg = g_back_color;
    g_back_color = WGT_CLR_HEADER_BG;
    /* 将当前波特率数值转为字符串 */
    snprintf(buf, sizeof(buf), "%ld", (long)baud_table[baud_idx]);
    /* 清除标题栏右侧旧的波特率文字区域 */
    lcd_fill(lcddev.width - 120, (WGT_HEADER_H - 12) / 2, lcddev.width - 8, (WGT_HEADER_H + 12) / 2, WGT_CLR_HEADER_BG);
    /* 在标题栏右侧居中显示波特率数字 */
    lcd_show_string(lcddev.width - 120, (WGT_HEADER_H - 12) / 2, 112, 12, 12, buf, WGT_CLR_TEXT_WHT);
    g_back_color = old_bg;
}

/**
 * DrawInterface() — 绘制串口终端的完整界面
 *
 * 界面布局（从上到下）：
 *   1. 顶部标题栏 "USART1  PA9/10" + 右侧波特率显示
 *   2. RX接收区域（圆角矩形卡片，标注 "RX"）
 *   3. TX发送区域（圆角矩形卡片，标注 "TX"）
 *   4. 底部操作提示文字
 */
static void DrawInterface(void)
{
    /* 用背景色清除整个模块显示区域 */
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WGT_CLR_BG);
    /* 绘制顶部标题栏，显示USART1使用的引脚 */
    lcd_wgt_header("USART1  PA9/10", 0, 0);
    /* 在标题栏右侧显示当前波特率 */
    DrawBaudInfo();

    /* ---- RX接收区域 ---- */
    /* 绘制RX卡片背景（圆角矩形填充） */
    lcd_draw_rounded_rect(6, RX_START_Y, lcddev.width - 8, RX_START_Y + RX_HEIGHT, 6, WGT_CLR_CARD_BG);
    /* 绘制RX卡片边框（圆角矩形描边） */
    lcd_draw_rounded_rect_border(6, RX_START_Y, lcddev.width - 8, RX_START_Y + RX_HEIGHT, 6, WGT_CLR_CARD_BRD);
    /* "RX" 标签，用高亮色标注这是接收区域 */
    lcd_show_string(14, RX_START_Y + 4, 40, 16, 12, "RX", WGT_CLR_ACCENT);

    /* ---- TX发送区域 ---- */
    /* 绘制TX卡片背景 */
    lcd_draw_rounded_rect(6, TX_START_Y, lcddev.width - 8, TX_START_Y + TX_HEIGHT, 6, WGT_CLR_CARD_BG);
    /* 绘制TX卡片边框 */
    lcd_draw_rounded_rect_border(6, TX_START_Y, lcddev.width - 8, TX_START_Y + TX_HEIGHT, 6, WGT_CLR_CARD_BRD);
    /* "TX" 标签，用绿色(0x07E0)标注这是发送区域 */
    lcd_show_string(14, TX_START_Y + 4, 40, 16, 12, "TX", 0x07E0);

    /* ---- 底部操作提示 ---- */
    /* 显示按键功能说明：'='发送、'C'清空接收、'7/9'切换波特率、'n'返回 */
    lcd_show_string(14, TX_START_Y + TX_HEIGHT - 22, 450, 16, 12,
                    "'='send 'C'clear '7/9'baud 'n'back", WGT_CLR_TEXT_SEC);
}

/* ======================== 刷新接收显示 ======================== */
/**
 * RefreshRX() — 从串口环形缓冲区读取新数据并刷新RX显示区域
 *
 * 处理流程：
 *   1. 检查环形缓冲区是否有新数据（与上次可用字节数比较）
 *   2. 逐字节读取新数据到g_rx_buf，换行符(\r\n)统一转为'\n'
 *   3. 缓冲区接近满时，裁剪旧数据保留最后2行（防止溢出）
 *   4. 清空RX显示区域，重新计算可见行范围并逐行渲染
 *
 * 行数超过MAX_LINES时，自动滚动只显示最新的内容。
 */
static void RefreshRX(void)
{
    /* 获取环形缓冲区中可读取的字节数 */
    uint16_t avail = rx_ringbuf_available();
    /* 静态变量记录上次的可用字节数，用于判断是否有新数据到达 */
    static uint16_t last_avail = 0;
    /* 没有新数据则直接返回，避免不必要的屏幕刷新 */
    if (avail == last_avail) return;

    /* ---- 从环形缓冲区读取新数据到显示缓冲区 ---- */
    uint8_t ch;
    while (rx_ringbuf_get(&ch) && g_rx_len < sizeof(g_rx_buf) - 1) {
        if (ch == '\r' || ch == '\n') {
            /* 遇到换行符：如果上一个字符不是'\n'，则存入一个'\n' */
            /* 这样可以将 \r\n 统一为单个 \n，避免多余空行 */
            if (g_rx_len > 0 && g_rx_buf[g_rx_len - 1] != '\n')
                g_rx_buf[g_rx_len++] = '\n';
        } else {
            /* 普通字符直接存入缓冲区 */
            g_rx_buf[g_rx_len++] = ch;
        }
    }

    /* ---- 缓冲区溢出保护 ---- */
    /* 当剩余空间不足一行时，裁剪掉前面的旧数据，保留最后2行 */
    if (g_rx_len >= sizeof(g_rx_buf) - MAX_LINE_CHARS) {
        /* 计算裁剪位置：跳过前面的内容，保留最后2行的空间 */
        uint16_t cut = g_rx_len - MAX_LINE_CHARS * 2;
        /* 向前搜索到最近的换行符，确保从完整行的开头开始裁剪 */
        while (cut > 0 && g_rx_buf[cut - 1] != '\n') cut--;
        if (cut > 0) {
            /* 将后面的数据前移，覆盖已裁剪的旧数据 */
            g_rx_len -= cut;
            memmove(g_rx_buf, g_rx_buf + cut, g_rx_len);
        }
    }

    /* ---- 重绘RX显示区域 ---- */
    /* 用卡片背景色清除RX区域内的文本显示部分（保留标签和边框） */
    lcd_fill(10, RX_START_Y + 20, lcddev.width - 12, RX_START_Y + RX_HEIGHT - 4, WGT_CLR_CARD_BG);

    if (g_rx_len > 0) {
        /* ---- 计算可见行的起始位置 ---- */
        /* 当总行数超过MAX_LINES时，需要跳过前面的行，只显示最新的 */
        uint16_t disp_start = 0;  /* 可见区域起始的字节偏移 */
        uint16_t line_cnt = 0;    /* 已遍历的行数计数 */
        for (uint16_t i = 0; i < g_rx_len; i++) {
            /* 遇到换行符或行长度达到上限时，行计数+1 */
            if (g_rx_buf[i] == '\n' || (i - disp_start + 1) >= MAX_LINE_CHARS) {
                line_cnt++;
                if (line_cnt > MAX_LINES) {
                    /* 行数超出显示能力，跳过最早的一行 */
                    disp_start = i + 1;
                    line_cnt--;
                }
            }
        }

        /* ---- 逐行渲染文本 ---- */
        uint16_t y = RX_START_Y + 22;  /* 当前行的Y坐标 */
        uint16_t ls = disp_start;       /* 当前行的起始字节偏移 */
        uint16_t li = 0;                /* 已绘制的行数 */
        for (uint16_t i = disp_start; i <= g_rx_len && li < MAX_LINES; i++) {
            /* 遇到换行符、行满或到达缓冲区末尾时，绘制一行 */
            if (i == g_rx_len || g_rx_buf[i] == '\n' || (i - ls) >= MAX_LINE_CHARS) {
                char line[MAX_LINE_CHARS + 1];
                uint16_t llen = i - ls;
                if (llen > MAX_LINE_CHARS) llen = MAX_LINE_CHARS;
                /* 将该行数据拷贝到临时缓冲区并添加字符串结束符 */
                memcpy(line, g_rx_buf + ls, llen);
                line[llen] = '\0';
                /* 在LCD上渲染该行文本 */
                lcd_show_string(8, y, lcddev.width - 16, LINE_HEIGHT, 16, line, WGT_CLR_TEXT_PRI);
                y += LINE_HEIGHT;  /* 下移一行 */
                /* 跳过换行符，开始下一行 */
                if (i < g_rx_len && g_rx_buf[i] == '\n') i++;
                ls = i;  /* 更新下一行起始位置 */
                li++;    /* 已绘制行数+1 */
            }
        }
    }

    /* 记录本次可用字节数，供下次比较 */
    last_avail = avail;
}

/* ======================== 刷新发送显示 ======================== */
/**
 * RefreshTX() — 刷新TX发送区域的输入内容显示
 *
 * 仅在输入长度发生变化时才重绘，避免不必要的屏幕刷新。
 * 在TX卡片内显示用户当前正在输入的字符串。
 */
static void RefreshTX(void)
{
    /* 上次的输入长度，用于检测内容是否变化 */
    static uint8_t last_len = 0xFF;
    /* 长度未变化则跳过刷新 */
    if (g_tx_len == last_len) return;

    /* 清除TX区域的文字显示部分 */
    lcd_fill(50, TX_START_Y + 4, lcddev.width - 12, TX_START_Y + 26, WGT_CLR_CARD_BG);
    if (g_tx_len > 0) {
        /* 添加字符串结束符，将缓冲区转为合法C字符串 */
        g_tx_buf[g_tx_len] = '\0';
        /* 显示当前输入内容 */
        lcd_show_string(50, TX_START_Y + 6, lcddev.width - 64, 16, 16, g_tx_buf, WGT_CLR_TEXT_PRI);
    }
    /* 更新上次长度记录 */
    last_len = g_tx_len;
}

/* ======================== 模块接口 ======================== */
/**
 * usart_ui_enter() — 模块进入函数
 *
 * 初始化串口（默认115200波特率），清空所有缓冲区，
 * 绘制完整的终端界面，并激活模块。
 */
static void usart_ui_enter(void)
{
    /* 初始化USART1，设置默认波特率115200 */
    usart_init(115200);

    /* 清空RX和TX缓冲区 */
    memset(g_rx_buf, 0, sizeof(g_rx_buf));
    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    g_rx_len = 0;
    g_tx_len = 0;
    /* 清空底层串口硬件环形接收缓冲区 */
    rx_ringbuf_clear();

    /* 绘制终端界面（标题栏、RX/TX卡片、提示文字） */
    DrawInterface();
    /* 标记模块为激活状态 */
    module_active = 1;
}

/**
 * usart_ui_tick() — 模块周期调度函数
 *
 * 每次被主循环调用时执行：
 *   1. 刷新RX接收显示（检查新数据）
 *   2. 刷新TX发送显示（检查输入变化）
 *   3. 处理触摸键盘输入事件
 *
 * 键盘功能映射：
 *   '=' 或 'y' — 发送TX缓冲区内容
 *   'C'        — 清空RX接收区
 *   '<'        — 退格删除TX输入的最后一个字符
 *   '7'        — 切换到更低的波特率
 *   '9'        — 切换到更高的波特率
 *   'n'        — 退出模块返回菜单
 *   其他可打印字符 — 追加到TX输入缓冲区
 */
static void usart_ui_tick(void)
{
    /* 模块未激活时直接返回 */
    if (!module_active) return;

    /* 刷新RX接收区和TX发送区的显示 */
    RefreshRX();
    RefreshTX();

    /* 读取触摸键盘按键事件 */
    int key = detect_key_press();
    /* 无按键事件则返回 */
    if (key == EVT_NONE) return;

    /* 根据按键类型执行对应操作 */
    switch (key) {
    case '=':
    case 'y':
        /* ---- 发送功能 ---- */
        /* 将TX缓冲区内容通过串口发送出去 */
        if (g_tx_len > 0) {
            g_tx_buf[g_tx_len] = '\0';  /* 确保字符串正确终止 */
            printf("%s\r\n", g_tx_buf);  /* 通过printf发送（重定向到USART1） */
            g_tx_len = 0;               /* 清空发送缓冲区 */
            memset(g_tx_buf, 0, TX_BUF_SIZE);
        }
        break;

    case 'C':
        /* ---- 清空接收区 ---- */
        /* 清空底层环形缓冲区和显示缓冲区 */
        rx_ringbuf_clear();
        memset(g_rx_buf, 0, sizeof(g_rx_buf));
        g_rx_len = 0;
        /* 清除RX区域的屏幕显示内容 */
        lcd_fill(10, RX_START_Y + 20, lcddev.width - 12, RX_START_Y + RX_HEIGHT - 4, WGT_CLR_CARD_BG);
        break;

    case '<':
        /* ---- 退格功能 ---- */
        /* 删除TX缓冲区最后一个字符 */
        if (g_tx_len > 0) g_tx_buf[--g_tx_len] = '\0';
        break;

    case '7':
        /* ---- 降低波特率 ---- */
        /* 切换到波特率表中上一档（更低速率） */
        if (baud_idx > 0) {
            baud_idx--;
            /* 重新初始化串口，应用新波特率 */
            usart_init(baud_table[baud_idx]);
            /* 刷新标题栏中的波特率显示 */
            DrawBaudInfo();
        }
        break;

    case '9':
        /* ---- 提高波特率 ---- */
        /* 切换到波特率表中下一档（更高速率） */
        if (baud_idx < BAUD_COUNT - 1) {
            baud_idx++;
            usart_init(baud_table[baud_idx]);
            DrawBaudInfo();
        }
        break;

    case 'n':
        /* ---- 退出模块 ---- */
        module_exit_current();
        return;

    default:
        /* ---- 输入可打印字符 ---- */
        /* ASCII 0x20('~')~0x7E(' ')为可打印字符范围 */
        /* 预留3字节空间用于发送时添加 '\r\n' 和 '\0' */
        if (key >= ' ' && key <= '~' && g_tx_len < TX_BUF_SIZE - 3) {
            g_tx_buf[g_tx_len++] = (char)key;
        }
        break;
    }
}

/**
 * usart_ui_exit() — 模块退出函数
 *
 * 将模块状态标记为非激活，使tick函数不再处理事件。
 * 串口硬件本身不需要关闭，因为可能被其他模块使用。
 */
static void usart_ui_exit(void)
{
    module_active = 0;
}

/* ======================== 模块导出 ======================== */
/* 模块接口结构体，供菜单系统注册和调用 */
const ModuleInterface usart_ui_module = {
    .name  = "USART_Terminal",   /* 模块名称，显示在菜单中 */
    .enter = usart_ui_enter,     /* 进入模块时调用 */
    .tick  = usart_ui_tick,      /* 主循环周期调用 */
    .exit  = usart_ui_exit,      /* 退出模块时调用 */
};
