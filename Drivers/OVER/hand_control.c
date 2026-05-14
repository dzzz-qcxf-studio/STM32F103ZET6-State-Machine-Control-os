/**
 * @file    hand_control.c
 * @brief   坐标发送模块 (USART3, PB10/PB11, 115200)
 *
 * 功能概述：
 *   通过触摸屏虚拟键盘输入X/Y坐标值，以二进制协议帧通过USART3串口发送给上位机或下位机。
 *
 * 通信参数：
 *   - 串口：USART3，引脚 PB10(TX) / PB11(RX)
 *   - 波特率：115200，8位数据位，无校验，1停止位（8N1）
 *
 * 二进制协议帧格式（共12字节）：
 *   帧头(0xAA) + X标签(0x01) + X值(4字节小端序) + Y标签(0x02) + Y值(4字节小端序) + 帧尾(0x55)
 *
 * 用户交互：
 *   - '=' 键切换X/Y编辑模式
 *   - 数字键、小数点、负号输入坐标值
 *   - '<' 退格删除，'C' 清空当前字段
 *   - 'y' 发送坐标，'n' 退出模块
 *
 * 模块化设计：提供 enter/tick/exit 三段式接口，由模块管理器调度，非阻塞运行。
 */

#include "./OVER/hand_control.h"
#include "./OVER/module.h"
#include "./OVER/menu.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/lcd_widgets.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ======================== 硬件 ======================== */
/* USART3外设句柄，HAL库通过此句柄管理串口3的所有操作 */
static UART_HandleTypeDef huart3;
/* USART3初始化标志：0=未初始化，1=已初始化；防止重复初始化 */
static uint8_t uart3_inited = 0;

/* ======================== 输入状态 ======================== */
/* X坐标输入缓冲区，最多容纳9个字符 + 1个'\0'结束符（如 "-1234.56"） */
static char x_input[10];
/* Y坐标输入缓冲区，格式同上 */
static char y_input[10];
/* X缓冲区当前写入位置（下一个字符写入的索引），同时代表已输入字符数 */
static uint8_t x_idx = 0;
/* Y缓冲区当前写入位置 */
static uint8_t y_idx = 0;
/* 当前编辑模式：0=正在编辑X坐标，1=正在编辑Y坐标 */
static int input_mode = 0;

/* ======================== USART3初始化 ======================== */
/**
 * @brief  初始化USART3外设（PB10=TX, PB11=RX, 115200-8N1）
 *
 * 采用懒加载模式：首次调用时执行硬件初始化，后续调用直接返回。
 * 初始化步骤：开时钟 -> 配GPIO -> 配UART参数 -> 标记已初始化。
 */
static void USART3_Init(void)
{
    /* 如果已经初始化过，直接返回，避免重复配置 */
    if (uart3_inited) return;

    /* 使能USART3和GPIOB的外设时钟，否则寄存器无法访问 */
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 配置PB10(TX)和PB11(RX)为复用推挽输出模式 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_10 | GPIO_PIN_11;  /* PB10=USART3_TX, PB11=USART3_RX */
    gpio.Mode  = GPIO_MODE_AF_PP;             /* 复用推挽输出（TX需要推挽驱动能力） */
    gpio.Pull  = GPIO_NOPULL;                 /* 不使用上下拉，由外部电路决定 */
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;        /* 高速模式，满足115200波特率要求 */
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 配置USART3通信参数 */
    huart3.Instance          = USART3;              /* 指定USART3外设 */
    huart3.Init.BaudRate     = 115200;              /* 波特率115200bps */
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;  /* 数据位长度8位 */
    huart3.Init.StopBits     = UART_STOPBITS_1;     /* 1个停止位 */
    huart3.Init.Parity       = UART_PARITY_NONE;    /* 无校验位 */
    huart3.Init.Mode         = UART_MODE_TX_RX;     /* 收发双向模式 */
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE; /* 无硬件流控（RTS/CTS） */
    huart3.Init.OverSampling = UART_OVERSAMPLING_16; /* 16倍过采样，抗噪性好 */
    HAL_UART_Init(&huart3);

    /* 标记USART3已完成初始化，后续调用将跳过 */
    uart3_inited = 1;
}

/* ======================== 发送坐标 ======================== */
/**
 * @brief  构建二进制协议帧并通过USART3发送坐标
 * @param  x  X坐标值（有符号32位整数）
 * @param  y  Y坐标值（有符号32位整数）
 *
 * 协议帧格式（共12字节）：
 *   [0]    0xAA       - 帧头标识
 *   [1]    0x01       - X坐标标签
 *   [2..5] X的4字节   - 小端序（低字节在前）
 *   [6]    0x02       - Y坐标标签
 *   [7..10] Y的4字节  - 小端序（低字节在前）
 *   [11]   0x55       - 帧尾标识
 *
 * 发送后在LCD上显示十六进制原始数据和可读坐标值，方便调试。
 */
static void Hand_SendCoordinate(int32_t x, int32_t y)
{
    uint8_t buf[12];
    /* 帧头：固定标识0xAA，接收方据此判断一帧的开始 */
    buf[0]  = 0xAA;
    /* X坐标标签：0x01表示接下来4字节是X值 */
    buf[1]  = 0x01;
    /* X坐标值拆分为4字节，小端序排列（最低有效字节在前） */
    buf[2]  = (uint8_t)(x & 0xFF);         /* X的最低字节 [7:0]  */
    buf[3]  = (uint8_t)((x >> 8)  & 0xFF); /* X的次低字节 [15:8] */
    buf[4]  = (uint8_t)((x >> 16) & 0xFF); /* X的次高字节 [23:16]*/
    buf[5]  = (uint8_t)((x >> 24) & 0xFF); /* X的最高字节 [31:24]*/
    /* Y坐标标签：0x02表示接下来4字节是Y值 */
    buf[6]  = 0x02;
    /* Y坐标值拆分为4字节，小端序排列 */
    buf[7]  = (uint8_t)(y & 0xFF);         /* Y的最低字节 [7:0]  */
    buf[8]  = (uint8_t)((y >> 8)  & 0xFF); /* Y的次低字节 [15:8] */
    buf[9]  = (uint8_t)((y >> 16) & 0xFF); /* Y的次高字节 [23:16]*/
    buf[10] = (uint8_t)((y >> 24) & 0xFF); /* Y的最高字节 [31:24]*/
    /* 帧尾：固定标识0x55，接收方据此判断一帧的结束 */
    buf[11] = 0x55;

    /* 通过USART3发送全部12字节，超时时间100ms（阻塞式发送） */
    HAL_UART_Transmit(&huart3, buf, sizeof(buf), 100);

    /* ---- 在LCD上显示发送结果，方便调试 ---- */

    /* 将12字节协议帧转为十六进制字符串显示（如 "AA 01 64 00 00 00 02 C8 00 00 00 55 "） */
    char hex[40] = {0};
    for (int i = 0; i < 12; i++)
        snprintf(hex + i * 3, 4, "%02X ", buf[i]);

    /* 清除LCD底部区域，显示十六进制原始数据（小字体，辅助色） */
    lcd_fill(14, 260, 460, 296, WGT_CLR_BG);
    lcd_show_string(14, 260, 440, 16, 12, hex, WGT_CLR_TEXT_SEC);

    /* 显示可读的坐标数值（如 "X:100 Y:200"），用醒目颜色 */
    char coord[30];
    snprintf(coord, sizeof(coord), "X:%ld Y:%ld", (long)x, (long)y);
    lcd_show_string(14, 278, 200, 16, 16, coord, WGT_CLR_ACCENT);
}

/* ======================== 刷新输入显示 ======================== */
/**
 * @brief  刷新LCD上X/Y输入框和模式指示器的显示
 *
 * 显示逻辑：
 *   - 先清除X和Y输入框区域的旧内容（用背景色填充）
 *   - 当前正在编辑的字段用红色(WGT_CLR_KEY_NEG)高亮显示
 *   - 非活动字段用常规文本色(WGT_CLR_TEXT_PRI)显示
 *   - 底部显示当前编辑模式提示（"Editing X" 或 "Editing Y"）
 */
static void UpdateInputDisplay(void)
{
    /* 清除X输入框区域的旧内容（坐标100,56到400,80） */
    lcd_fill(100, 56, 400, 80, WGT_CLR_BG);
    /* 清除Y输入框区域的旧内容（坐标100,106到400,130） */
    lcd_fill(100, 106, 400, 130, WGT_CLR_BG);
    /* 显示X缓冲区内容：编辑模式=0时为红色高亮，否则为普通文本色 */
    lcd_show_string(100, 56,  200, 24, 24, x_input, (input_mode == 0) ? WGT_CLR_KEY_NEG : WGT_CLR_TEXT_PRI);
    /* 显示Y缓冲区内容：编辑模式=1时为红色高亮，否则为普通文本色 */
    lcd_show_string(100, 106, 200, 24, 24, y_input, (input_mode == 1) ? WGT_CLR_KEY_NEG : WGT_CLR_TEXT_PRI);

    /* 模式指示：清除旧文字后重绘，提示用户当前正在编辑哪个坐标轴 */
    lcd_fill(14, 150, 400, 174, WGT_CLR_BG);
    lcd_show_string(14, 150, 300, 16, 16,
                    input_mode == 0 ? "Editing X  (= to switch)" : "Editing Y  (= to switch)",
                    WGT_CLR_ACCENT);
}

/* ======================== 模块接口 ======================== */
/**
 * @brief  模块进入函数 —— 初始化硬件、清空输入状态、绘制UI界面
 *
 * 由模块管理器在切换到本模块时调用（仅调用一次）。
 * 执行顺序：初始化串口 -> 重置输入缓冲区 -> 绘制界面布局 -> 刷新显示。
 */
static void hand_control_enter(void)
{
    /* 初始化USART3串口（懒加载，已初始化则跳过） */
    USART3_Init();

    /* 清空X/Y输入缓冲区，重置写入指针和编辑模式 */
    memset(x_input, 0, sizeof(x_input));  /* X缓冲区全部清零 */
    memset(y_input, 0, sizeof(y_input));  /* Y缓冲区全部清零 */
    x_idx = 0;       /* X写入指针归零 */
    y_idx = 0;       /* Y写入指针归零 */
    input_mode = 0;  /* 默认进入X编辑模式 */

    /* ---- 绘制界面布局 ---- */

    /* 清除整个内容区域（菜单栏以上），用背景色填充 */
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WGT_CLR_BG);
    /* 顶部标题栏：显示 "COORD SENDER"（坐标发送器） */
    lcd_wgt_header("COORD SENDER", 0, 0);
    /* X输入标签（固定文字 "Enter X:"，位于Y=56行） */
    lcd_show_string(14, 56,  200, 24, 24, "Enter X:", WGT_CLR_TEXT_PRI);
    /* Y输入标签（固定文字 "Enter Y:"，位于Y=106行） */
    lcd_show_string(14, 106, 200, 24, 24, "Enter Y:", WGT_CLR_TEXT_PRI);
    /* 操作提示第一行：切换模式和发送 */
    lcd_show_string(14, 200, 200, 16, 16, "'=' switch  'y' send", WGT_CLR_KEY_ACT);
    /* 操作提示第二行：退出和清空 */
    lcd_show_string(14, 222, 200, 16, 16, "'n' exit  'C' clear", WGT_CLR_KEY_ACT);
    /* 刷新输入框显示（此时为空，高亮X编辑框） */
    UpdateInputDisplay();
}

/**
 * @brief  模块主循环函数 —— 每帧被模块管理器调用，处理按键输入
 *
 * 按键映射：
 *   '='  切换X/Y编辑模式
 *   '<'  退格删除当前字段最后一个字符
 *   'C'  清空当前编辑字段的全部内容
 *   'y'  将输入的字符串转为整数并通过串口发送坐标
 *   'n'  退出本模块，返回上一级菜单
 *   '0'-'9', '-', '.'  向当前活动缓冲区追加字符
 */
static void hand_control_tick(void)
{
    /* 读取触摸屏按键事件，无按键则直接返回（非阻塞） */
    int key = detect_key_press();
    if (key == EVT_NONE) return;  /* 无按键事件，不做任何处理 */

    switch (key) {
    case '=':
        /* '='键：切换X/Y编辑模式（0变1，1变0） */
        input_mode = !input_mode;
        UpdateInputDisplay();  /* 刷新显示，高亮切换到新的活动字段 */
        break;

    case '<':
        /* '<'键：退格删除（将写入指针前移一格，覆盖结束符） */
        if (input_mode == 0) {
            /* 当前编辑X：如果X缓冲区非空，将末尾字符置为'\0' */
            if (x_idx > 0) x_input[--x_idx] = '\0';
        } else {
            /* 当前编辑Y：如果Y缓冲区非空，将末尾字符置为'\0' */
            if (y_idx > 0) y_input[--y_idx] = '\0';
        }
        UpdateInputDisplay();
        break;

    case 'C':
        /* 'C'键：清空当前编辑字段的全部内容 */
        if (input_mode == 0) {
            /* 清空X缓冲区，写入指针归零 */
            memset(x_input, 0, sizeof(x_input));
            x_idx = 0;
        } else {
            /* 清空Y缓冲区，写入指针归零 */
            memset(y_input, 0, sizeof(y_input));
            y_idx = 0;
        }
        UpdateInputDisplay();
        break;

    case 'y': {
        /* 'y'键：发送坐标 —— 将输入字符串转为int32，构建协议帧并发送 */
        int32_t xv = atoi(x_input);  /* 将X输入字符串转为整数（atoi忽略小数部分） */
        int32_t yv = atoi(y_input);  /* 将Y输入字符串转为整数 */
        Hand_SendCoordinate(xv, yv); /* 构建12字节协议帧并通过USART3发送 */
        break;
    }

    case 'n':
        /* 'n'键：退出当前模块，由模块管理器执行清理并返回上级菜单 */
        module_exit_current();
        return;

    default:
        /* 数字键(0-9)、负号(-)、小数点(.)：追加字符到当前活动缓冲区 */
        if (key >= '0' && key <= '9' || key == '-' || key == '.') {
            if (input_mode == 0 && x_idx < 9) {
                /* 当前编辑X且缓冲区未满(最多9字符)：追加字符并更新结束符 */
                x_input[x_idx++] = (char)key;
                x_input[x_idx] = '\0';
            } else if (input_mode == 1 && y_idx < 9) {
                /* 当前编辑Y且缓冲区未满：追加字符并更新结束符 */
                y_input[y_idx++] = (char)key;
                y_input[y_idx] = '\0';
            }
            UpdateInputDisplay();  /* 刷新输入框显示 */
        }
        break;
    }
}

/**
 * @brief  模块退出函数 —— 本模块退出时的清理工作
 *
 * 注意：USART3外设在此不关闭、不反初始化。
 * 原因：USART3可能被其他模块共享使用（如通信模块、调试模块），
 * 如果此处关闭串口，会影响其他模块的正常通信。
 * 因此采用"只用不还"策略，让串口保持运行状态。
 */
static void hand_control_exit(void)
{
    /* USART3 保持初始化，不释放（共享资源，可能其他模块也需要） */
}

/* ======================== 模块导出 ======================== */
/**
 * @brief  模块接口实例 —— 供模块管理器注册和调度
 *
 * ModuleInterface结构体定义了模块的三段式生命周期：
 *   .enter  —— 进入模块时调用（初始化硬件、绘制界面）
 *   .tick   —— 主循环中每帧调用（处理用户输入、刷新显示）
 *   .exit   —— 退出模块时调用（释放资源，此处为空）
 *
 * 模块管理器通过 menu.c 中的模块列表找到此实例，
 * 根据用户选择动态调用对应的 enter/tick/exit 函数。
 */
const ModuleInterface hand_control_module = {
    .name  = "Hand_Control",        /* 模块名称，用于调试和日志 */
    .enter = hand_control_enter,    /* 进入回调：初始化串口+绘制UI */
    .tick  = hand_control_tick,     /* 主循环回调：处理按键+发送坐标 */
    .exit  = hand_control_exit,     /* 退出回调：保持串口不关闭 */
};
