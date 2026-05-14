/**
 * @file    bus_servo.c
 * @brief   总线舵机控制模块 — 串口指令控制
 *
 * ========== 串口协议说明 ==========
 * 本模块通过 USART3 串口向总线舵机发送 ASCII 字符串指令进行控制。
 * 协议格式: #<ID>P<PWM>T<TIME>!
 *   - '#'  : 指令起始标志
 *   - <ID> : 舵机编号, 固定3位数字, 不足补0 (如: 001, 015, 254)
 *   - 'P'  : PWM 关键字分隔符
 *   - <PWM>: PWM脉宽值, 固定4位数字, 单位微秒(μs), 范围500~2500
 *   - 'T'  : 时间关键字分隔符
 *   - <TIME>: 运行时间, 固定4位数字, 单位毫秒(ms), 范围0~9999
 *   - '!'  : 指令结束标志
 *
 * 特殊指令:
 *   - #<ID>PULK! : 释放舵机扭力 (舵机可自由转动)
 *   - #<ID>PULR! : 恢复舵机扭力 (舵机锁定在当前位置)
 *   - #<ID>PRAD! : 读取当前角度位置, 返回 #<ID>P<PWM>!
 *   - #<ID>PRTV! : 读取温度和电压, 返回 #<ID>T<temp>V<voltage>!
 *
 * 通信参数: USART3, PB10-TX / PB11-RX, 115200bps, 8数据位, 无校验, 1停止位
 *
 * 键盘操作:
 *   4/6   - PWM ±50  (左/右, 粗调)
 *   2/8   - Time ±100 (下/上)
 *   +/-   - PWM ±10  (细调)
 *   1     - 发送移动控制指令
 *   3     - 释放扭力 (PULK)
 *   9     - 恢复扭力 (PULR)
 *   0     - 读取位置 (PRAD)
 *   5     - 读取温度电压 (PRTV)
 *   y     - 切换舵机ID (000~005循环)
 *   n     - 退出模块
 */

/* ======================== 头文件包含 ======================== */
#include "./OVER/bus_servo.h"     /* 总线舵机模块头文件 (模块声明) */
#include "./OVER/module.h"        /* 模块系统接口 (ModuleInterface, module_exit_current) */
#include "./OVER/resource.h"      /* 全局资源定义 (主题色常量 WGT_CLR_xxx) */
#include "./OVER/menu.h"          /* 菜单系统接口 (detect_key_press, EVT_NONE) */
#include "./BSP/LCD/lcd.h"        /* LCD 底层驱动 (lcd_fill, lcd_show_string) */
#include "./BSP/LCD/lcd_widgets.h"/* LCD 高级控件 (lcd_wgt_header, lcd_wgt_status_label) */
#include "stm32f1xx_hal.h"        /* STM32F1 HAL 库 (UART, GPIO, RCC 等外设操作) */
#include <stdio.h>                /* 标准IO (snprintf 格式化字符串) */
#include <string.h>               /* 字符串操作 (strlen, strchr) */

/* ======================== USART3 配置 ======================== */
static UART_HandleTypeDef huart3;   /* USART3 句柄, 用于 HAL 库串口收发操作 */
static uint8_t hw_inited = 0;       /* 硬件初始化标志: 0=未初始化, 1=已初始化, 防止重复初始化 */

/* ======================== 舵机参数 ======================== */
/* 当前选中的舵机 ID, 范围 0~254, 默认 0 */
static uint8_t  servo_id   = 0;
/* PWM 脉宽值, 单位微秒(μs), 控制舵机转动角度, 范围 500~2500, 默认 1500 (中间位置) */
static uint16_t servo_pwm  = 1500;
/* 舵机运行到目标位置所需的时间, 单位毫秒(ms), 范围 0~9999, 默认 1000ms */
static uint16_t servo_time = 1000;
/* 舵机工作模式, 范围 1~8, 不同模式对应不同的控制特性 */
static uint8_t  servo_mode = 1;

/**
 * ======================== USART3 初始化 ========================
 * 初始化 USART3 外设及对应的 GPIO 引脚, 配置为 115200-8N1 模式。
 * 引脚分配: PB10 = TX (发送), PB11 = RX (接收)
 * 采用静态初始化保护, 只在首次调用时执行, 避免重复配置。
 */
static void BusServo_USART_Init(void)
{
    if (hw_inited) return;  /* 已初始化则直接返回, 防止重复配置 */

    /* 使能 USART3 和 GPIOB 外设时钟, 这是使用外设前的必要步骤 */
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 配置 PB10 为 USART3_TX: 复用推挽输出模式, 高速 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_10;               /* TX 引脚: PB10 */
    gpio.Mode  = GPIO_MODE_AF_PP;           /* 复用功能推挽输出, 串口TX必须用此模式 */
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;      /* 高速模式, 保证115200bps通信质量 */
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 配置 PB11 为 USART3_RX: 复用输入模式, 上拉, 空闲时保持高电平 */
    gpio.Pin   = GPIO_PIN_11;               /* RX 引脚: PB11 */
    gpio.Mode  = GPIO_MODE_AF_INPUT;        /* 复用功能输入, 串口RX必须用此模式 */
    gpio.Pull  = GPIO_PULLUP;               /* 内部上拉, 防止空闲时误触发 */
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 配置 USART3 参数: 115200波特率, 8数据位, 无校验, 1停止位 (即 115200-8N1) */
    huart3.Instance          = USART3;              /* 使用 USART3 外设 */
    huart3.Init.BaudRate     = 115200;              /* 波特率 115200 bps */
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;  /* 数据位: 8位 */
    huart3.Init.StopBits     = UART_STOPBITS_1;     /* 停止位: 1位 */
    huart3.Init.Parity       = UART_PARITY_NONE;    /* 校验位: 无 */
    huart3.Init.Mode         = UART_MODE_TX_RX;     /* 模式: 收发双向 */
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE; /* 硬件流控: 无 */
    huart3.Init.OverSampling = UART_OVERSAMPLING_16; /* 16倍过采样, 提高抗干扰能力 */
    HAL_UART_Init(&huart3);                          /* 调用 HAL 库完成初始化 */

    hw_inited = 1;  /* 标记硬件已初始化 */
}

/**
 * ======================== 发送指令 ========================
 * 通过 USART3 向舵机发送 ASCII 字符串指令。
 * @param cmd  要发送的指令字符串, 如 "#001P1500T1000!"
 *
 * 内部调用 HAL_UART_Transmit 阻塞发送, 超时 200ms。
 * 将 char* 强制转换为 uint8_t* 以匹配 HAL 库函数签名。
 */
static void servo_send_cmd(const char *cmd)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)cmd, strlen(cmd), 200);
}

/**
 * 接收舵机应答 (非阻塞轮询, 总超时 100ms)
 * 逐字节读取 USART3 数据, 直到收到结束符 '!' 或超时。
 * @param buf      接收缓冲区
 * @param max_len  缓冲区最大长度
 * @return         实际接收到的字节数
 *
 * 工作原理:
 *   1. 记录起始时间 t0
 *   2. 循环轮询, 总耗时不超过 100ms
 *   3. 每次尝试读取 1 字节, 单次超时 5ms (非阻塞)
 *   4. 读到数据则存入缓冲区, 遇到 '!' 则结束
 *   5. 最后添加 '\0' 字符串终止符
 */
static uint8_t servo_recv(char *buf, uint8_t max_len)
{
    uint8_t idx = 0;                    /* 当前缓冲区写入位置索引 */
    uint32_t t0 = HAL_GetTick();        /* 记录起始时间戳 (系统毫秒计数) */
    while (HAL_GetTick() - t0 < 100) {  /* 总等待时间不超过 100ms */
        uint8_t ch;
        /* 尝试读取 1 字节, 单次超时 5ms, 返回 HAL_OK 表示成功读到 */
        if (HAL_UART_Receive(&huart3, &ch, 1, 5) == HAL_OK) {
            if (idx < max_len - 1) {    /* 防止缓冲区溢出, 留 1 字节给 '\0' */
                buf[idx++] = ch;        /* 存入缓冲区并移动索引 */
            }
            if (ch == '!') break;       /* 收到指令结束符 '!', 停止接收 */
        }
    }
    buf[idx] = '\0';                    /* 添加字符串终止符 */
    return idx;                         /* 返回实际接收的字节数 */
}

/* ======================== 控制指令 ======================== */

/**
 * 发送舵机移动控制指令
 * 协议格式: #<ID>P<PWM>T<TIME>!
 *   - #%03d : 舵机ID, 3位数字, 不足补0 (如 001, 025)
 *   - P%04d : PWM脉宽, 4位数字, 不足补0 (如 0500, 1500, 2500)
 *   - T%04d : 运行时间, 4位数字, 不足补0 (如 0500, 1000)
 *   - '!'   : 指令结束标志
 *
 * @param id   舵机ID (0~254)
 * @param pwm  PWM脉宽 (500~2500μs)
 * @param time 运行时间 (0~9999ms)
 *
 * 示例: servo_move(1, 1500, 1000) 发送 "#001P1500T1000!"
 */
static void servo_move(uint8_t id, uint16_t pwm, uint16_t time)
{
    char cmd[24];   /* 指令缓冲区, 最长格式约16字节, 留足余量 */
    /* 按协议格式拼接指令字符串: #<ID>P<PWM>T<TIME>! */
    snprintf(cmd, sizeof(cmd), "#%03dP%04dT%04d!", id, pwm, time);
    servo_send_cmd(cmd);    /* 通过串口发送指令 */
}

/**
 * 释放舵机扭力 (舵机可自由转动, 不保持位置)
 * 协议格式: #<ID>PULK!
 *   - PULK : 释放扭力的固定命令关键字
 * @param id 舵机ID (0~254)
 * 示例: servo_release(1) 发送 "#001PULK!"
 */
static void servo_release(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPULK!", id);
    servo_send_cmd(cmd);
}

/**
 * 恢复舵机扭力 (舵机锁定在当前位置)
 * 协议格式: #<ID>PULR!
 *   - PULR : 恢复扭力的固定命令关键字
 * @param id 舵机ID (0~254)
 * 示例: servo_restore(1) 发送 "#001PULR!"
 */
static void servo_restore(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPULR!", id);
    servo_send_cmd(cmd);
}

/**
 * 读取舵机当前角度位置
 * 发送指令: #<ID>PRAD!  (PRAD = 读取角度)
 * 舵机应答: #<ID>P<PWM>!  其中 <PWM> 为当前位置的脉宽值
 *
 * 解析流程:
 *   1. 发送 #<ID>PRAD! 查询指令
 *   2. 等待接收舵机应答字符串
 *   3. 在应答中查找 'P' 字符, 提取其后的数字作为位置值
 *
 * @param id 舵机ID (0~254)
 * @return 当前 PWM 位置值 (500~2500), 解析失败返回 0
 */
static uint16_t servo_read_pos(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPRAD!", id);   /* 拼接读取位置指令 */
    servo_send_cmd(cmd);                             /* 发送指令 */

    char resp[32];
    servo_recv(resp, sizeof(resp));                  /* 接收舵机应答 */

    /* 解析应答: #<ID>P<PWM>! —— 找到 'P' 后面的数字即为位置值 */
    char *p = strchr(resp, 'P');     /* 定位 'P' 字符位置 */
    if (p) return (uint16_t)atoi(p + 1);  /* 将 'P' 后面的字符串转为整数 */
    return 0;    /* 解析失败, 返回 0 */
}

/**
 * 读取舵机温度和电压
 * 发送指令: #<ID>PRTV!  (PRTV = 读取温度电压)
 * 舵机应答: #<ID>T<temp>V<voltage>!
 *   - T<temp> : 温度值, 单位摄氏度
 *   - V<voltage> : 电压值, 实际值 = volt/10 V (如 volt=78 表示 7.8V)
 *
 * @param id   舵机ID (0~254)
 * @param temp 输出参数, 温度值 (摄氏度)
 * @param volt 输出参数, 电压原始值 (需除以10得到实际电压)
 */
static void servo_read_temp_volt(uint8_t id, uint8_t *temp, uint8_t *volt)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPRTV!", id);   /* 拼接读取温度电压指令 */
    servo_send_cmd(cmd);                             /* 发送指令 */

    char resp[32];
    servo_recv(resp, sizeof(resp));                  /* 接收舵机应答 */

    *temp = 0; *volt = 0;    /* 初始化输出参数为 0 */
    /* 解析应答: #<ID>T<temp>V<voltage>! */
    char *t = strchr(resp, 'T');     /* 定位 'T' 字符, 提取温度值 */
    char *v = strchr(resp, 'V');     /* 定位 'V' 字符, 提取电压值 */
    if (t) *temp = (uint8_t)atoi(t + 1);  /* 将 'T' 后面的字符串转为温度 */
    if (v) *volt = (uint8_t)atoi(v + 1);  /* 将 'V' 后面的字符串转为电压 */
}

/* ======================== UI 布局常量 ======================== */
#define PARAM_Y      52     /* 参数卡片区域起始 Y 坐标 (顶部留出标题栏空间) */
#define PARAM_H      64     /* 每张参数卡片的高度 (像素) */
#define PARAM_GAP    6      /* 相邻卡片之间的间距 (像素) */
#define PARAM_MARGIN 10     /* 卡片区域左右边距 (像素) */
#define HINT_Y       260    /* 操作提示文字区域起始 Y 坐标 */
#define STATUS_Y     360    /* 状态栏起始 Y 坐标 */

/**
 * 绘制参数卡片 (带圆角矩形背景的 参数展示单元)
 * 卡片布局: 上方小字标签 + 下方大字数值
 *
 * @param x1, y1       左上角坐标
 * @param x2, y2       右下角坐标
 * @param label        标签文字 (如 "PWM", "TIME"), 12px 字体显示
 * @param value        数值文字 (如 "1500", "1000 ms"), 24px 大字显示
 * @param val_color    数值文字颜色 (使用主题色常量, 如 WGT_CLR_ACCENT)
 *
 * 绘制流程:
 *   1. 保存当前背景色
 *   2. 绘制圆角矩形填充背景 + 边框
 *   3. 在卡片顶部绘制 12px 标签 (次要文字色)
 *   4. 在标签下方绘制 24px 大字数值 (可配置颜色)
 *   5. 恢复原背景色
 */
static void draw_param_card(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                            const char *label, const char *value, uint16_t val_color)
{
    uint32_t old_bg = g_back_color;   /* 保存当前全局背景色, 用完后恢复 */
    /* 绘制圆角矩形背景 (圆角半径 6px, 卡片背景色) */
    lcd_draw_rounded_rect(x1, y1, x2, y2, 6, WGT_CLR_CARD_BG);
    /* 绘制圆角矩形边框 (同尺寸, 边框颜色) */
    lcd_draw_rounded_rect_border(x1, y1, x2, y2, 6, WGT_CLR_CARD_BRD);

    g_back_color = WGT_CLR_CARD_BG;   /* 临时设置背景色为卡片色, 保证文字背景一致 */
    /* 绘制标签文字: 左上角偏移 (10,6), 12px 字体, 次要文字色 */
    lcd_show_string(x1 + 10, y1 + 6, 100, 12, 12, (char *)label, WGT_CLR_TEXT_SEC);
    /* 绘制数值文字: 偏移 (10,26), 24px 大字, 使用指定的强调色 */
    lcd_show_string(x1 + 10, y1 + 26, 200, 24, 24, (char *)value, val_color);
    g_back_color = old_bg;             /* 恢复原背景色 */
}

/**
 * 绘制完整的舵机控制 UI 界面
 * 布局结构 (从上到下):
 *   第一行: [SERVO ID 卡片] [MODE 卡片]  —— 左右并排, 各占半宽
 *   第二行: [PWM 卡片]                    —— 全宽大卡片
 *   第三行: [TIME 卡片]                   —— 全宽大卡片
 *   操作提示区: 显示各按键的功能说明
 *   状态栏: 显示当前操作状态
 *
 * 每次参数变化时调用此函数刷新整个界面。
 */
static void draw_ui(void)
{
    char buf[24];   /* 数值格式化缓冲区 */
    /* 计算半宽: 屏幕宽度减去左右边距和中间间距后平分 */
    uint16_t half_w = (SCREEN_WIDTH - PARAM_MARGIN * 2 - PARAM_GAP) / 2;
    uint16_t x1 = PARAM_MARGIN;                /* 左边界 */
    uint16_t x2 = SCREEN_WIDTH - PARAM_MARGIN; /* 右边界 */
    uint16_t x_mid = x1 + half_w;             /* 中间分割线 X 坐标 */

    /* 清除参数区域 (从 PARAM_Y 到 STATUS_Y+20), 用背景色填充 */
    lcd_fill(0, PARAM_Y, SCREEN_WIDTH, STATUS_Y + 20, WGT_CLR_BG);

    /* ── 第一行: ID + Mode (两张卡片左右并排) ── */
    /* 左侧: SERVO ID 卡片, 显示当前舵机ID (3位数字) */
    snprintf(buf, sizeof(buf), "%03d", servo_id);
    draw_param_card(x1, PARAM_Y, x_mid, PARAM_Y + PARAM_H, "SERVO ID", buf, WGT_CLR_ACCENT);
    /* 右侧: MODE 卡片, 显示当前工作模式 */
    snprintf(buf, sizeof(buf), "%d", servo_mode);
    draw_param_card(x_mid + PARAM_GAP, PARAM_Y, x2, PARAM_Y + PARAM_H, "MODE", buf, WGT_CLR_TEXT_PRI);

    /* ── 第二行: PWM 全宽大卡片, 显示 PWM 脉宽值 ── */
    uint16_t row2_y = PARAM_Y + PARAM_H + PARAM_GAP;   /* 第二行 Y 起始位置 */
    snprintf(buf, sizeof(buf), "%04d", servo_pwm);      /* 格式化为4位数字 */
    draw_param_card(x1, row2_y, x2, row2_y + PARAM_H, "PWM (500~2500)", buf, WGT_CLR_ACCENT);

    /* ── 第三行: TIME 全宽大卡片, 显示运行时间 ── */
    uint16_t row3_y = row2_y + PARAM_H + PARAM_GAP;    /* 第三行 Y 起始位置 */
    snprintf(buf, sizeof(buf), "%04d ms", servo_time);  /* 格式化为4位数字 + ms单位 */
    draw_param_card(x1, row3_y, x2, row3_y + PARAM_H, "TIME (0~9999)", buf, WGT_CLR_ACCENT);

    /* ── 操作提示区域: 显示各按键功能, 分三行两列布局 ── */
    uint32_t old_bg = g_back_color;    /* 保存背景色 */
    g_back_color = WGT_CLR_BG;         /* 设置背景色用于文字绘制 */
    /* 第 1 行: PWM 调节快捷键说明 (次要文字色) */
    lcd_show_string(14, HINT_Y,     220, 16, 16, "4/6: PWM+/-50",   WGT_CLR_TEXT_SEC);
    lcd_show_string(240, HINT_Y,    220, 16, 16, "+/-: PWM+/-10",   WGT_CLR_TEXT_SEC);
    /* 第 2 行: 时间调节和ID切换说明 */
    lcd_show_string(14, HINT_Y+22,  220, 16, 16, "2/8: Time+/-100", WGT_CLR_TEXT_SEC);
    lcd_show_string(240, HINT_Y+22, 220, 16, 16, "y: Switch ID",    WGT_CLR_TEXT_SEC);
    /* 第 3 行: 操作命令快捷键说明 (活动文字色, 更醒目) */
    lcd_show_string(14, HINT_Y+44,  220, 16, 16, "1:Send 3:Free",   WGT_CLR_KEY_ACT);
    lcd_show_string(240, HINT_Y+44, 220, 16, 16, "9:Lock 0:Read",   WGT_CLR_KEY_ACT);
    lcd_show_string(14, HINT_Y+66,  220, 16, 16, "5:Temp/Voltage",  WGT_CLR_KEY_ACT);
    g_back_color = old_bg;             /* 恢复背景色 */

    /* ── 状态栏: 底部显示 "Ready" 就绪状态, active=0 表示普通状态 ── */
    lcd_wgt_status_label(14, STATUS_Y, "Ready", 0);
}

/**
 * 更新状态栏显示内容
 * 清除旧的状态文字, 显示新消息和对应颜色的状态指示点。
 *
 * @param msg    要显示的状态消息文字 (如 "Sent #001P1500T1000", "Torque released")
 * @param active 状态指示点颜色: 0=灰色(默认), 1=绿色(操作成功), 2=橙色(警告/释放)
 */
static void show_status(const char *msg, uint8_t active)
{
    uint32_t old_bg = g_back_color;    /* 保存背景色 */
    g_back_color = WGT_CLR_BG;
    /* 先用背景色填充清除旧的状态文字区域 */
    lcd_fill(10, STATUS_Y, SCREEN_WIDTH - 10, STATUS_Y + 20, WGT_CLR_BG);
    /* 绘制新的状态标签 (带颜色指示点) */
    lcd_wgt_status_label(14, STATUS_Y, msg, active);
    g_back_color = old_bg;             /* 恢复背景色 */
}

/* ======================== 模块接口 ======================== */

/**
 * 模块进入函数 —— 初始化舵机控制界面
 * 当用户从主菜单选择 "Bus Servo" 时由系统调用。
 * 执行流程:
 *   1. 初始化 USART3 串口硬件 (首次调用时)
 *   2. 将所有舵机参数重置为默认值
 *   3. 清屏并绘制标题栏
 *   4. 绘制完整的参数控制 UI 界面
 */
static void bus_servo_enter(void)
{
    BusServo_USART_Init();   /* 初始化 USART3 (仅首次生效, 后续调用直接返回) */

    /* 重置舵机参数为默认值: ID=0, PWM=1500(中间), 时间=1000ms, 模式=1 */
    servo_id   = 0;
    servo_pwm  = 1500;       /* 中间位置 (500~2500 范围的中点) */
    servo_time = 1000;       /* 默认 1 秒运行时间 */
    servo_mode = 1;          /* 默认工作模式 1 */

    /* 清除整个显示区域并绘制模块标题栏 */
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WGT_CLR_BG);
    lcd_wgt_header("BUS SERVO", 0, 0);   /* 顶部标题 "BUS SERVO" */

    draw_ui();               /* 绘制完整的参数卡片和操作提示界面 */
}

/**
 * 模块主循环函数 —— 按键事件处理
 * 由系统主循环周期性调用, 检测按键输入并执行对应操作。
 * 无按键时直接返回, 不阻塞系统。
 *
 * 按键映射表:
 *   '4' / '6' : PWM 粗调 -50/+50 (左/右方向)
 *   '-' / '+' : PWM 细调 -10/+10
 *   '2' / '8' : 运行时间 -100/+100ms (下/上方向)
 *   '1'       : 发送移动指令 (#<ID>P<PWM>T<TIME>!)
 *   '3'       : 释放扭力 (PULK)
 *   '9'       : 恢复扭力 (PULR)
 *   '0'       : 读取当前位置 (PRAD)
 *   '5'       : 读取温度和电压 (PRTV)
 *   'y'       : 循环切换舵机 ID (0 -> 1 -> 2 -> ... -> 5 -> 0)
 *   'n'       : 退出模块, 返回上级菜单
 */
static void bus_servo_tick(void)
{
    int key = detect_key_press();       /* 检测是否有按键按下 */
    if (key == EVT_NONE) return;        /* 无按键, 直接返回不处理 */

    char status[48];                    /* 状态消息格式化缓冲区 */

    switch (key) {
    /* ── PWM 调节 (粗调: 每次 ±50μs) ── */
    case '4':   /* 左键: PWM 减少 50, 下限 500 */
        servo_pwm = (servo_pwm > 550) ? servo_pwm - 50 : 500;
        draw_ui();
        break;
    case '6':   /* 右键: PWM 增加 50, 上限 2500 */
        servo_pwm = (servo_pwm < 2450) ? servo_pwm + 50 : 2500;
        draw_ui();
        break;
    /* ── PWM 调节 (细调: 每次 ±10μs) ── */
    case '+':   /* 加号: PWM 增加 10, 上限 2500 */
        servo_pwm = (servo_pwm < 2490) ? servo_pwm + 10 : 2500;
        draw_ui();
        break;
    case '-':   /* 减号: PWM 减少 10, 下限 500 */
        servo_pwm = (servo_pwm > 510) ? servo_pwm - 10 : 500;
        draw_ui();
        break;

    /* ── 运行时间调节 (每次 ±100ms) ── */
    case '2':   /* 下键: 时间减少 100ms, 下限 0 */
        servo_time = (servo_time > 100) ? servo_time - 100 : 0;
        draw_ui();
        break;
    case '8':   /* 上键: 时间增加 100ms, 上限 9999ms */
        servo_time = (servo_time < 9899) ? servo_time + 100 : 9999;
        draw_ui();
        break;

    /* ── 发送移动控制指令 ── */
    case '1':   /* 按键1: 构建并发送 #<ID>P<PWM>T<TIME>! 指令 */
        servo_move(servo_id, servo_pwm, servo_time);
        /* 格式化已发送的指令内容用于状态栏显示 */
        snprintf(status, sizeof(status), "Sent #%03dP%04dT%04d",
                 servo_id, servo_pwm, servo_time);
        show_status(status, 1);         /* 显示发送成功状态 (绿色指示点) */
        break;

    /* ── 释放/恢复扭力 ── */
    case '3':   /* 按键3: 释放扭力, 舵机可自由转动 */
        servo_release(servo_id);
        show_status("Torque released", 2);  /* 橙色指示点, 表示警告状态 */
        break;
    case '9':   /* 按键9: 恢复扭力, 舵机锁定当前位置 */
        servo_restore(servo_id);
        show_status("Torque restored", 1);  /* 绿色指示点, 表示正常状态 */
        break;

    /* ── 读取当前位置 ── */
    case '0': { /* 按键0: 查询舵机当前 PWM 位置值 */
        uint16_t pos = servo_read_pos(servo_id);   /* 发送 PRAD 指令并解析返回值 */
        snprintf(status, sizeof(status), "Pos #%03dP%04d", servo_id, pos);
        show_status(status, 0);         /* 灰色指示点, 表示查询结果 */
        break;
    }

    /* ── 读取温度和电压 ── */
    case '5': { /* 按键5: 查询舵机内部温度和供电电压 */
        uint8_t temp, volt;
        servo_read_temp_volt(servo_id, &temp, &volt);  /* 发送 PRTV 指令并解析 */
        /* 电压值 volt 需除以 10 得到实际电压 (如 78 -> 7.8V) */
        snprintf(status, sizeof(status), "#%03d T:%dC V:%d.%dV",
                 servo_id, temp, volt / 10, volt % 10);
        show_status(status, 0);         /* 灰色指示点, 表示查询结果 */
        break;
    }

    /* ── 切换舵机 ID ── */
    case 'y':   /* 按键y: 在 ID 0~5 之间循环切换 */
        servo_id = (servo_id + 1) % 6;  /* 0->1->2->3->4->5->0 循环 */
        draw_ui();                      /* 刷新界面显示新 ID */
        break;

    /* ── 退出模块 ── */
    case 'n':   /* 按键n: 退出舵机控制模块, 返回上级菜单 */
        module_exit_current();
        return;                         /* 直接返回, 不再执行后续代码 */

    default:
        break;
    }
}

/**
 * 模块退出函数
 * 当用户按 'n' 键退出时由系统调用。
 * 注意: USART3 保持初始化状态不关闭, 这样再次进入模块时无需重新初始化,
 * 也避免影响可能共享 USART3 的其他模块。
 */
static void bus_servo_exit(void)
{
    /* USART3 保持初始化, 不关闭 —— 复用硬件资源, 提高响应速度 */
}

/* ======================== 模块导出 ======================== */
/**
 * 模块接口结构体 —— 向系统注册 "Bus_Servo" 模块
 * 系统通过此结构体的函数指针调用模块的生命周期方法:
 *   - enter: 进入模块时调用 (初始化硬件和UI)
 *   - tick:  主循环中周期调用 (处理按键事件)
 *   - exit:  退出模块时调用 (清理资源)
 *   - name:  模块名称, 用于菜单显示和调试
 */
const ModuleInterface bus_servo_module = {
    .name  = "Bus_Servo",         /* 模块名称, 显示在主菜单中 */
    .enter = bus_servo_enter,     /* 进入回调: 初始化 USART3, 重置参数, 绘制 UI */
    .tick  = bus_servo_tick,      /* 循环回调: 检测按键, 调节参数, 发送指令 */
    .exit  = bus_servo_exit,      /* 退出回调: 保持 USART3 运行, 无需清理 */
};
