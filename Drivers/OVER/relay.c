/**
 * @file    relay.c
 * @brief   继电器控制模块 (PA3, 低电平有效)
 *
 * 硬件说明：
 *   - 继电器连接在 STM32 的 PA3 引脚上
 *   - 低电平有效（Active-Low）：PA3 输出低电平时继电器吸合（ON），
 *     输出高电平时继电器断开（OFF）
 *
 * 模块化改造：提供 enter/tick/exit 接口，供状态机框架统一调度。
 */

#include "./OVER/relay.h"
#include "./OVER/module.h"
#include "./OVER/resource.h"
#include "./OVER/menu.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/lcd_widgets.h"
#include "stm32f1xx_hal.h"

/* ======================== 硬件状态 ======================== */
/* relay_on: 继电器当前状态标志。
 *   0 = 继电器关闭（OFF），PA3 输出高电平，继电器断开
 *   1 = 继电器开启（ON），PA3 输出低电平，继电器吸合
 * 初始值为 0（上电默认关闭）。
 */
static uint8_t relay_on = 0;        /* 0=OFF, 1=ON */

/* hw_inited: 硬件初始化标志。
 *   0 = GPIO 尚未初始化
 *   1 = GPIO 已完成初始化
 * 用于防止重复初始化，进入模块时置 1，退出模块时置 0。
 */
static uint8_t hw_inited = 0;

/* ======================== GPIO初始化 ======================== */
/**
 * @brief  初始化继电器所用的 GPIO 引脚（PA3）
 *
 * 初始化流程：
 *   1. 检查是否已经初始化过，避免重复配置
 *   2. 通过 pin_request() 向资源管理器申请 PA3 引脚的独占使用权
 *   3. 使能 GPIOA 的外设时钟（STM32 使用 GPIO 前必须先开时钟）
 *   4. 配置 PA3 为推挽输出模式（Push-Pull），无上下拉，高速
 *   5. 将 PA3 默认拉高（高电平 = 继电器 OFF，因为继电器低电平有效）
 *   6. 标记硬件已初始化完成
 */
static void Relay_GPIO_Init(void)
{
    /* 如果已经初始化过，直接返回，避免重复配置 */
    if (hw_inited) return;

    /* 向资源管理器申请 PA3 引脚，模块名为 "Relay"，
     * 防止其他模块同时使用该引脚导致冲突 */
    pin_request(GPIOA, GPIO_PIN_3, "Relay");

    /* 使能 GPIOA 端口的时钟，STM32 外设使用前必须先开启对应时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 配置 GPIO 初始化结构体 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_3;                /* 引脚：PA3 */
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;       /* 模式：推挽输出（可主动输出高/低电平） */
    gpio.Pull  = GPIO_NOPULL;               /* 上下拉：无（外部电路已处理） */
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;      /* 速度：高速，确保开关响应快 */
    HAL_GPIO_Init(GPIOA, &gpio);            /* 写入寄存器，完成 GPIO 配置 */

    /* 默认输出高电平（GPIO_PIN_SET = 高电平）。
     * 因为继电器是低电平有效（Active-Low），
     * 所以高电平 = 继电器断开（OFF），这是安全的上电初始状态。 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

    /* 更新软件状态：继电器标记为关闭，硬件标记为已初始化 */
    relay_on = 0;
    hw_inited = 1;
}

/* ======================== 继电器控制 ======================== */
/**
 * @brief  设置继电器的开关状态
 * @param  on  目标状态：0=关闭（OFF），非0=开启（ON）
 *
 * 工作原理（低电平有效 / Active-Low）：
 *   - on=1（开启）：PA3 输出 GPIO_PIN_RESET（低电平 0V），继电器吸合
 *   - on=0（关闭）：PA3 输出 GPIO_PIN_SET（高电平 3.3V），继电器断开
 *
 * 三目运算符解析：
 *   on ? GPIO_PIN_RESET : GPIO_PIN_SET
 *   即 "on 为真时输出低电平，否则输出高电平"
 */
static void Relay_Set(uint8_t on)
{
    /* 先更新软件状态记录，保持与硬件同步 */
    relay_on = on;

    /* 根据目标状态控制 PA3 引脚电平 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* ======================== 刷新显示 ======================== */
/**
 * @brief  在 LCD 上刷新继电器的当前状态显示
 *
 * 显示逻辑：
 *   - 继电器开启时：在坐标 (220, 90) 显示绿色 "ON" 文字（颜色值 0x07E0 = 绿色）
 *   - 继电器关闭时：在坐标 (220, 90) 显示红色 "OFF" 文字（WGT_CLR_KEY_NEG = 红色）
 *
 * 操作流程：
 *   1. 先用背景色填充状态区域（清除旧文字）
 *   2. 临时设置全局背景色，确保文字背景与界面一致
 *   3. 根据 relay_on 状态选择文字内容和颜色
 *   4. 恢复原始背景色，避免影响后续其他绘图操作
 */
static void Relay_UpdateDisplay(void)
{
    /* 用背景色填充状态显示区域，清除之前显示的文字 */
    lcd_fill(220, 90, 340, 114, WGT_CLR_BG);

    /* 保存当前全局背景色，稍后恢复，防止影响其他模块的绘图 */
    uint32_t old_bg = g_back_color;
    g_back_color = WGT_CLR_BG;

    /* 显示继电器状态文字：
     *   relay_on 为真 → 显示 "ON"，颜色为绿色（0x07E0，RGB565 格式的纯绿色）
     *   relay_on 为假 → 显示 "OFF"，颜色为红色（WGT_CLR_KEY_NEG） */
    lcd_show_string(220, 90, 120, 24, 24,
                    relay_on ? "ON" : "OFF",
                    relay_on ? 0x07E0 : WGT_CLR_KEY_NEG);

    /* 恢复原始背景色 */
    g_back_color = old_bg;
}

/* ======================== 模块接口 ======================== */
/**
 * @brief  继电器模块入口函数（由状态机框架在切换到本模块时调用）
 *
 * 执行流程：
 *   1. 初始化继电器 GPIO 硬件（如果尚未初始化）
 *   2. 清屏并绘制用户界面：
 *      - 顶部标题栏："RELAY CONTROL"
 *      - 状态行：显示 "Current State:" 标签
 *      - 操作提示：按 '1' 切换继电器状态
 *      - 退出提示：按 'n' 返回上级菜单
 *   3. 刷新状态显示，展示继电器当前的开关状态
 */
static void relay_enter(void)
{
    /* 初始化继电器的 GPIO 引脚（仅首次调用时执行，之后会被跳过） */
    Relay_GPIO_Init();

    /* 用背景色填充整个菜单区域以上的屏幕区域（清屏） */
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WGT_CLR_BG);

    /* 绘制顶部标题栏，标题为 "RELAY CONTROL" */
    lcd_wgt_header("RELAY CONTROL", 0, 0);

    /* 显示 "Current State:" 标签（主色调文字） */
    lcd_show_string(50, 90,  200, 24, 24, "Current State:",  WGT_CLR_TEXT_PRI);

    /* 显示操作提示："按 '1' 切换继电器"（强调色文字） */
    lcd_show_string(50, 140, 250, 24, 24, "Press '1' to Toggle", WGT_CLR_ACCENT);

    /* 显示退出提示："按 'n' 退出模块"（次要色文字） */
    lcd_show_string(50, 190, 250, 24, 24, "Press 'n' to Exit",   WGT_CLR_TEXT_SEC);

    /* 刷新状态显示，将当前继电器状态（ON/OFF）以对应颜色显示出来 */
    Relay_UpdateDisplay();
}

/**
 * @brief  继电器模块的周期性轮询函数（由状态机框架循环调用）
 *
 * 按键处理逻辑：
 *   - 无按键时直接返回，不做任何操作
 *   - 按下 '1'：切换继电器状态（ON→OFF 或 OFF→ON），并刷新屏幕显示
 *     使用 "!relay_on" 取反当前状态，实现"切换"效果
 *   - 按下 'n'：调用 module_exit_current() 退出当前模块，返回上级菜单
 */
static void relay_tick(void)
{
    /* 检测是否有按键按下，如果没有则直接返回 */
    int key = detect_key_press();
    if (key == EVT_NONE) return;

    if (key == '1') {
        /* 按下 '1'：切换继电器状态。
         * !relay_on 将当前状态取反（0→1 或 1→0），
         * 传给 Relay_Set() 后会同时更新 GPIO 电平和软件标志 */
        Relay_Set(!relay_on);
        /* 刷新屏幕上的状态显示（ON/OFF 及对应颜色） */
        Relay_UpdateDisplay();
    } else if (key == 'n') {
        /* 按下 'n'：退出继电器模块，返回上级菜单。
         * 框架会自动调用 relay_exit() 进行清理 */
        module_exit_current();
    }
}

/**
 * @brief  继电器模块退出函数（由状态机框架在离开本模块时调用）
 *
 * 清理流程：
 *   1. 关闭继电器（将 PA3 拉高为高电平，确保继电器断开）
 *   2. 释放 PA3 引脚的独占使用权，允许其他模块申请使用
 *   3. 将 hw_inited 标志清零，下次进入模块时会重新初始化 GPIO
 */
static void relay_exit(void)
{
    /* 关闭继电器：输出高电平（低电平有效下，高电平 = OFF） */
    Relay_Set(0);

    /* 释放 PA3 引脚资源，通知资源管理器该引脚不再被占用 */
    pin_release(GPIOA, GPIO_PIN_3, "Relay");

    /* 重置硬件初始化标志，下次进入模块时将重新配置 GPIO */
    hw_inited = 0;
}

/* ======================== 模块导出 ======================== */
/**
 * @brief  继电器模块的接口导出结构体
 *
 * 此结构体将模块的三个核心回调函数注册到框架中：
 *   - name  : 模块名称，用于调试和日志输出
 *   - enter : 模块入口函数，初始化硬件并绘制 UI
 *   - tick  : 周期轮询函数，处理按键输入并更新状态
 *   - exit  : 模块退出函数，关闭继电器并释放资源
 *
 * 状态机框架通过此结构体统一调度所有模块的生命周期。
 */
const ModuleInterface relay_module = {
    .name  = "Relay",           /* 模块名称标识 */
    .enter = relay_enter,       /* 进入模块时调用：初始化 + 绘制 UI */
    .tick  = relay_tick,        /* 循环调用：处理按键事件 */
    .exit  = relay_exit,        /* 退出模块时调用：关闭继电器 + 释放资源 */
};
