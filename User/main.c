/**
 * @file    main.c
 * @brief   STM32F103ZET6 状态机控制系统 v3.0.0
 *
 * 全面重构版本：
 * - 模块生命周期框架 (ModuleInterface: enter/tick/exit)
 * - 引脚/定时器资源管理器
 * - 菜单系统统一注册
 * - 主循环事件驱动 + 非阻塞模块调度
 *
 * 整体架构说明：
 * 本文件是整个嵌入式控制系统的入口，采用"菜单 + 模块"的双层架构。
 * 菜单系统负责界面导航和用户交互，模块系统负责具体功能的执行。
 * 主循环通过轮询方式运行，根据当前是否有活跃模块来决定行为：
 *   - 有活跃模块时：调用模块的 tick 函数进行非阻塞运行
 *   - 无活跃模块时：扫描按键事件并驱动菜单导航
 * 每个功能模块通过统一的 ModuleInterface 接口接入框架，
 * 实现 enter(进入)、tick(运行)、exit(退出) 三个生命周期回调。
 */

/* ======================== 系统底层驱动头文件 ======================== */
/* STM32 HAL 库核心头文件，提供 HAL_Init()、GPIO、RCC 等底层操作 */
#include "./SYSTEM/sys/sys.h"
/* 延时函数库，提供 delay_init() 和 HAL_Delay() 等精确延时功能 */
#include "./SYSTEM/delay/delay.h"
/* USART 串口驱动，提供 usart_init() 初始化串口通信（用于调试打印） */
#include "./SYSTEM/usart/usart.h"
/* LED 指示灯驱动，提供 led_init() 初始化板载 LED（PB5 等引脚） */
#include "./BSP/LED/led.h"
/* 按键扫描驱动，提供 key_init() 和 Key_Scan() 实现物理按键检测 */
#include "./BSP/KEY/key.h"
/* LCD 液晶屏驱动，提供 lcd_init() 初始化 2.8/3.5 寸 TFT 彩屏（FSMC 接口） */
#include "./BSP/LCD/lcd.h"
/* 电阻触摸屏驱动，提供 tp_dev.init() 初始化触摸屏，支持坐标读取 */
#include "./BSP/TOUCH/touch.h"
/* 摇杆（游戏手柄）驱动，提供 Joystick_Init() 初始化 ADC 采样的双轴摇杆 */
#include "./BSP/JOYSTICK/Joystick.h"

/* ======================== 框架层头文件 ======================== */
/* 模块管理器：定义 ModuleInterface 结构体，管理模块的 enter/tick/exit 生命周期 */
#include "./OVER/module.h"
/* 资源管理器：统一管理引脚、定时器等硬件资源的分配与冲突检测 */
#include "./OVER/resource.h"
/* 菜单系统：提供菜单注册 menu_register()、事件驱动导航、界面渲染等功能 */
#include "./OVER/menu.h"

/* ======================== 功能模块头文件 ======================== */
/* 以下头文件对应各个独立功能模块，每个模块内部定义并导出一个 ModuleInterface 结构体，
 * 包含模块名称、enter(进入回调)、tick(周期运行回调)、exit(退出回调) 等接口。
 * 新增功能模块时：1)编写模块 .c/.h  2)在此添加头文件  3)在下方声明 extern 接口  4)定义菜单页  5)注册菜单 */

/* 舵机控制模块：通过 PWM 信号驱动 SG90/MG996R 等舵机，支持角度设定与扫描 */
#include "./OVER/servo.h"
/* 继电器控制模块：通过 GPIO 高低电平控制继电器的通断，实现开关量输出 */
#include "./OVER/relay.h"
/* PWM 分析/波形绘制模块：在 LCD 上实时绘制 PWM 波形图，分析频率与占空比 */
#include "./OVER/vdraw.h"
/* 2.4G 无线遥控模块：通过 NRF24L01 等 2.4G 模块实现无线数据收发 */
#include "./OVER/wireless.h"
/* 手动坐标控制模块：通过串口或触摸屏发送坐标指令，控制外部设备（如机械臂） */
#include "./OVER/hand_control.h"
/* 串口终端 UI 模块：在 LCD 上实现简易串口终端界面，显示收发数据 */
#include "./OVER/usart_ui.h"
/* MPU + PID 控制模块：读取 MPU6050 陀螺仪数据，通过 PID 算法实现姿态闭环控制 */
#include "./OVER/mpu_pid.h"
/* 图片浏览器模块：从 SD 卡/Flash 加载并显示 BMP/JPG 图片 */
#include "./OVER/picall.h"
/* 示波器模块：利用 ADC 采样外部模拟信号，在 LCD 上实时显示波形（含光标测量） */
#include "./OVER/oscilloscope.h"
/* 总线舵机控制模块：通过串口总线协议（如飞特 STS/SCS 系列总线舵机协议）控制多个舵机 */
#include "./OVER/bus_servo.h"

/* ======================== 外部模块接口声明 ======================== */
/*
 * 每个功能模块在自己的 .c 文件中定义并导出一个 const ModuleInterface 结构体实例。
 * ModuleInterface 结构体统一定义了模块的生命周期接口，典型结构如下：
 *   typedef struct {
 *       const char *name;                    // 模块名称（显示在菜单/状态栏）
 *       void (*enter)(void);                 // 进入模块时调用（初始化硬件、绘制界面）
 *       void (*tick)(void);                  // 主循环中周期性调用（处理输入、更新状态）
 *       void (*exit)(void);                  // 退出模块时调用（释放资源、清理界面）
 *   } ModuleInterface;
 *
 * 通过 extern 声明，将各模块导出的接口结构体引入 main.c，
 * 供菜单项定义时绑定到具体的模块入口。
 */

/* 舵机控制模块接口：PWM 舵机角度控制 */
extern const ModuleInterface servo_module;
/* 继电器控制模块接口：GPIO 继电器开关控制 */
extern const ModuleInterface relay_module;
/* PWM 分析模块接口：波形绘制与分析 */
extern const ModuleInterface vdraw_module;
/* 2.4G 无线遥控模块接口：NRF24L01 收发控制 */
extern const ModuleInterface wireless_module;
/* 手动坐标控制模块接口：坐标指令发送 */
extern const ModuleInterface hand_control_module;
/* 串口终端 UI 模块接口：串口数据收发界面 */
extern const ModuleInterface usart_ui_module;
/* MPU PID 控制模块接口：陀螺仪姿态闭环控制 */
extern const ModuleInterface mpu_pid_module;
/* 摇杆模块接口：ADC 双轴摇杆状态显示（板载外设，非独立功能模块） */
extern const ModuleInterface joystick_module;
/* 图片浏览器模块接口：SD 卡/Flash 图片加载显示 */
extern const ModuleInterface picall_module;
/* 示波器模块接口：ADC 采样波形实时显示 */
extern const ModuleInterface oscilloscope_module;
/* 总线舵机模块接口：串口总线多舵机控制 */
extern const ModuleInterface bus_servo_module;

/* ======================== 菜单页定义 ======================== */
/*
 * 菜单页采用"静态数组 + 终止哨兵"的模式定义，每一页都是一个 MenuItem 数组。
 * MenuItem 结构体典型定义如下：
 *   typedef struct {
 *       const char    *text;       // 菜单项显示文本（如 "Servo Control"）
 *       MenuState      target;     // 选中后跳转的目标菜单状态枚举值
 *       const ModuleInterface *module;  // 绑定的功能模块指针（NULL 表示不启动模块）
 *   } MenuItem;
 *
 * 约定规则：
 *   - 数组最后一个元素的 text 字段为 NULL，作为数组遍历的终止哨兵
 *   - "Back" 类型菜单项的 module 字段为 NULL，仅用于菜单跳转
 *   - 功能菜单项同时绑定 target 状态和 module 指针，选中后会调用 module->enter()
 */

/* ---------- 主菜单：列出所有可用功能模块，是用户看到的第一个界面 ---------- */
/*  { 显示文本,          跳转目标状态,     绑定模块指针 }                              */
static const MenuItem menu_main[] = {
    { "Servo Control",    MENU_SERVO,    &servo_module     },  /* [0]  PWM舵机控制        */
    { "Relay Control",    MENU_RELAY,    &relay_module     },  /* [1]  继电器开关控制      */
    { "PWM Analyzer",     MENU_VDRAW,    &vdraw_module     },  /* [2]  PWM波形分析绘制     */
    { "2.4G Remote",      MENU_WIRELESS, &wireless_module  },  /* [3]  2.4G无线遥控        */
    { "Hand Control",     MENU_HAND,     &hand_control_module }, /* [4]  手动坐标/指令控制  */
    { "MPU PID Control",  MENU_MPU_PID,  &mpu_pid_module   },  /* [5]  MPU6050姿态PID控制  */
    { "Joystick Status",  MENU_JOYSTICK, &joystick_module   }, /* [6]  ADC摇杆状态显示     */
    { "USART Terminal",   MENU_USART,    &usart_ui_module   }, /* [7]  串口收发终端界面    */
    { "Picture Viewer",   MENU_PICALL,   &picall_module     }, /* [8]  SD卡图片浏览器      */
    { "Oscilloscope",     MENU_OSCOPE,   &oscilloscope_module }, /* [9]  ADC示波器        */
    { "Bus Servo",        MENU_BUS_SERVO, &bus_servo_module    }, /* [10] 总线舵机控制     */
    { NULL, MENU_MAIN, NULL }  /* [11] 终止哨兵：text==NULL 表示数组结束 */
};

/* ---------- 子菜单：舵机控制 ----------
 * 用户从主菜单选中 "Servo Control" 后跳转到此页。
 * "Run Servo" 项绑定 servo_module，选中后调用 servo_module.enter() 进入模块。
 * "Back" 项的 module 为 NULL，仅跳转回 MENU_MAIN，不启动任何模块。
 */
static const MenuItem menu_servo[] = {
    { "Run Servo",  MENU_SERVO, &servo_module },  /* 启动舵机控制模块 */
    { "Back",       MENU_MAIN,  NULL          },  /* 返回主菜单，不启动模块 */
    { NULL, MENU_MAIN, NULL }                      /* 终止哨兵 */
};

/* ---------- 子菜单：继电器控制 ----------
 * 提供继电器的开/关切换功能，绑定 relay_module。
 */
static const MenuItem menu_relay[] = {
    { "Toggle Relay", MENU_RELAY, &relay_module },  /* 启动继电器控制模块 */
    { "Back",         MENU_MAIN,  NULL          },  /* 返回主菜单 */
    { NULL, MENU_MAIN, NULL }
};

/* ---------- 子菜单：PWM 波形分析 ----------
 * 在 LCD 上绘制 PWM 波形，用于分析频率和占空比，绑定 vdraw_module。
 */
static const MenuItem menu_vdraw[] = {
    { "PWM Analyze", MENU_VDRAW, &vdraw_module },  /* 启动波形分析模块 */
    { "Back",        MENU_MAIN,  NULL          },  /* 返回主菜单 */
    { NULL, MENU_MAIN, NULL }
};

/* ---------- 子菜单：2.4G 无线遥控 ----------
 * 通过 NRF24L01 模块进行无线通信，绑定 wireless_module。
 */
static const MenuItem menu_wireless[] = {
    { "Start Remote", MENU_WIRELESS, &wireless_module },  /* 启动无线遥控模块 */
    { "Back",         MENU_MAIN,     NULL             },  /* 返回主菜单 */
    { NULL, MENU_MAIN, NULL }
};

/* ---------- 子菜单：手动坐标控制 ----------
 * 通过串口或触摸发送坐标/指令，控制外部设备，绑定 hand_control_module。
 */
static const MenuItem menu_hand[] = {
    { "Send Coords", MENU_HAND, &hand_control_module },  /* 启动手动控制模块 */
    { "Back",        MENU_MAIN, NULL                 },  /* 返回主菜单 */
    { NULL, MENU_MAIN, NULL }
};

/* ---------- 子菜单：MPU6050 PID 姿态控制 ----------
 * 读取陀螺仪数据，通过 PID 算法实现姿态闭环，绑定 mpu_pid_module。
 */
static const MenuItem menu_mpu_pid[] = {
    { "Run PID",  MENU_MPU_PID, &mpu_pid_module },  /* 启动 MPU PID 模块 */
    { "Back",     MENU_MAIN,    NULL            },  /* 返回主菜单 */
    { NULL, MENU_MAIN, NULL }
};

/* ---------- 子菜单：摇杆状态显示 ----------
 * 实时显示 ADC 双轴摇杆的 X/Y 坐标和按键状态，绑定 joystick_module。
 */
static const MenuItem menu_joystick[] = {
    { "Show Joystick", MENU_JOYSTICK, &joystick_module },  /* 启动摇杆显示模块 */
    { "Back",          MENU_MAIN,     NULL             },  /* 返回主菜单 */
    { NULL, MENU_MAIN, NULL }
};

/* ---------- 子菜单：串口终端 ----------
 * 在 LCD 上显示串口收发数据的终端界面，绑定 usart_ui_module。
 */
static const MenuItem menu_usart[] = {
    { "Open Terminal", MENU_USART, &usart_ui_module },  /* 启动串口终端模块 */
    { "Back",          MENU_MAIN,  NULL             },  /* 返回主菜单 */
    { NULL, MENU_MAIN, NULL }
};

/* ---------- 子菜单：图片浏览器 ----------
 * 从 SD 卡或 Flash 中加载并显示图片，绑定 picall_module。
 */
static const MenuItem menu_picall[] = {
    { "View Pictures", MENU_PICALL, &picall_module },  /* 启动图片浏览器模块 */
    { "Back",          MENU_MAIN,   NULL           },  /* 返回主菜单 */
    { NULL, MENU_MAIN, NULL }
};

/* ---------- 子菜单：示波器 ----------
 * ADC 采样外部模拟信号并实时显示波形，绑定 oscilloscope_module。
 */
static const MenuItem menu_oscope[] = {
    { "Run Scope", MENU_OSCOPE, &oscilloscope_module },  /* 启动示波器模块 */
    { "Back",      MENU_MAIN,   NULL                 },  /* 返回主菜单 */
    { NULL, MENU_MAIN, NULL }
};

/* ---------- 子菜单：总线舵机控制 ----------
 * 通过串口总线协议控制多个总线舵机，绑定 bus_servo_module。
 */
static const MenuItem menu_bus_servo[] = {
    { "Control Servo", MENU_BUS_SERVO, &bus_servo_module },  /* 启动总线舵机模块 */
    { "Back",          MENU_MAIN,      NULL              },  /* 返回主菜单 */
    { NULL, MENU_MAIN, NULL }
};

/* ======================== 菜单注册 ======================== */
/**
 * @brief  将所有菜单页注册到菜单系统
 *
 * menu_register() 将每个 MenuState 枚举值与对应的 MenuItem 数组关联起来，
 * 建立"状态 -> 菜单项列表"的映射表。菜单系统在渲染和导航时，
 * 根据 current_state 查找对应的 MenuItem 数组来显示和响应用户操作。
 *
 * 必须在 update_display() 之前调用，否则菜单系统找不到菜单数据。
 *
 * 参数说明：
 *   - 第1个参数：MenuState 枚举值（菜单页的唯一标识）
 *   - 第2个参数：指向该菜单页的 MenuItem 数组（static const，常驻 RAM）
 */
void menu_init_all(void)
{
    menu_register(MENU_MAIN,      menu_main);       /* 注册主菜单（根页面）       */
    menu_register(MENU_SERVO,     menu_servo);       /* 注册舵机子菜单             */
    menu_register(MENU_RELAY,     menu_relay);       /* 注册继电器子菜单           */
    menu_register(MENU_VDRAW,     menu_vdraw);       /* 注册 PWM 分析子菜单        */
    menu_register(MENU_WIRELESS,  menu_wireless);    /* 注册 2.4G 无线子菜单       */
    menu_register(MENU_HAND,      menu_hand);        /* 注册手动控制子菜单         */
    menu_register(MENU_MPU_PID,   menu_mpu_pid);     /* 注册 MPU PID 子菜单        */
    menu_register(MENU_JOYSTICK,  menu_joystick);    /* 注册摇杆子菜单             */
    menu_register(MENU_USART,     menu_usart);       /* 注册串口终端子菜单         */
    menu_register(MENU_PICALL,    menu_picall);      /* 注册图片浏览子菜单         */
    menu_register(MENU_OSCOPE,    menu_oscope);      /* 注册示波器子菜单           */
    menu_register(MENU_BUS_SERVO, menu_bus_servo);   /* 注册总线舵机子菜单         */
}

/* ======================== 主函数 ======================== */
/**
 * @brief  系统入口 —— 初始化所有硬件和软件框架，然后进入主循环
 *
 * 程序执行流程：
 *   1. 硬件初始化（HAL、时钟、外设）
 *   2. 软件框架初始化（资源管理器、模块管理器、菜单注册、键盘布局）
 *   3. 绘制初始菜单界面
 *   4. 进入无限主循环，轮询处理事件
 */
int main(void)
{
    /* =====================================================================
     * 第一阶段：系统底层硬件初始化
     * 按照"先时钟、再延迟、后外设"的顺序依次初始化
     * ===================================================================== */

    /* HAL 库初始化：配置 SysTick 中断（1ms 周期）、NVIC 分组等 */
    HAL_Init();

    /* 系统时钟配置：外部 8MHz 晶振 -> PLL 9 倍频 -> SYSCLK = 72MHz
     * RCC_PLL_MUL9 表示 PLL 倍频系数为 9，即 8MHz x 9 = 72MHz
     * 同时配置 AHB/APB1/APB2 总线时钟分频 */
    sys_stm32_clock_init(RCC_PLL_MUL9);

    /* 延时函数初始化：参数 72 表示系统主频 72MHz，用于计算精确的 us/ms 延时 */
    delay_init(72);

    /* USART1 串口初始化：波特率 115200，8N1，PA9(TX)/PA10(RX)
     * 用于 printf 重定向输出调试信息 */
    usart_init(115200);

    /* LED 初始化：配置 PB5 等 GPIO 为推挽输出，驱动板载 LED 指示灯 */
    led_init();

    /* 按键初始化：配置 KEY0/KEY1/KEY_UP 等 GPIO 为上拉/下拉输入，支持消抖检测 */
    key_init();

    /* LCD 初始化：通过 FSMC 接口驱动 TFT 彩屏，配置背光、像素格式、绘图区域等 */
    lcd_init();

    /* 触摸屏初始化：初始化电阻触摸屏控制器（如 XPT2046），校准触摸坐标映射 */
    tp_dev.init();

    /* 摇杆初始化：配置 ADC 通道采样双轴（X/Y）模拟电压，以及摇杆按键 GPIO */
    Joystick_Init();

    /* =====================================================================
     * 第二阶段：软件框架初始化
     * 初始化上层管理系统，为功能模块运行做好准备
     * ===================================================================== */

    /* 资源管理器初始化：建立引脚/定时器的分配表，后续模块申请资源时进行冲突检测 */
    resource_init();

    /* 模块管理器初始化：清除当前活跃模块指针，准备接收模块 enter/tick/exit 调用 */
    module_mgr_init();

    /* 菜单注册：将所有菜单页（主菜单 + 11 个子菜单）注册到菜单系统
     * 必须在 update_display() 之前调用 */
    menu_init_all();

    /* 键盘布局初始化：配置 LCD 屏幕底部虚拟键盘的按键位置和映射关系
     * 用于支持触摸屏输入（如示波器参数输入、坐标输入等场景） */
    init_keyboard_layout();

    /* =====================================================================
     * 第三阶段：绘制初始界面
     * 菜单注册完成后，首次渲染主菜单界面到 LCD
     * ===================================================================== */

    /* 绘制初始界面：根据 current_state（默认 MENU_MAIN）渲染对应的菜单页
     * 包括菜单项列表、高亮选中项、底部键盘（如果当前页需要）等 */
    update_display();

    /* =====================================================================
     * 第四阶段：主循环 —— 事件驱动 + 非阻塞模块调度
     *
     * 主循环采用"双模式"设计：
     *   模式A - 模块活跃模式：当前有功能模块正在运行
     *     -> 调用 module_run_tick() 执行模块的周期处理
     *     -> 模块内部自行处理按键/触摸输入，以及退出条件判断
     *     -> 模块退出后（module_get_current() 返回 NULL），自动回到菜单模式
     *
     *   模式B - 菜单导航模式：无活跃模块，等待用户操作
     *     -> Key_Scan() 扫描物理按键状态（含消抖）
     *     -> get_next_event() 将按键动作转换为菜单事件（上/下/确认/返回等）
     *     -> handle_event() 根据事件执行菜单高亮移动、页面跳转、模块启动等操作
     *
     * HAL_Delay(10) 提供 10ms 的循环周期：
     *   - 给按键消抖提供足够的时间窗口
     *   - 降低 CPU 占用率，减少功耗
     *   - 对用户操作来说 10ms 响应延迟足够流畅
     * ===================================================================== */
    while (1) {
        if (module_get_current() != NULL) {
            /*
             * 模式A：当前有活跃的功能模块
             * 调用模块的 tick 回调，由模块自行处理输入和状态更新
             * tick 函数必须是非阻塞的，快速返回以保持主循环流畅
             */
            module_run_tick();

            /* 检测模块是否刚刚退出（tick 内部可能调用了 module_exit()）
             * 如果已退出：重置 current_state 为 MENU_MAIN，并强制重绘菜单界面
             * update_display_force_kbd() 会同时刷新菜单和虚拟键盘 */
            if (module_get_current() == NULL) {
                current_state = MENU_MAIN;       /* 回到主菜单状态 */
                update_display_force_kbd();       /* 强制重绘菜单 + 虚拟键盘 */
            }
        } else {
            /*
             * 模式B：菜单导航模式（无活跃模块）
             * 扫描按键 -> 生成事件 -> 处理事件
             */
            Key_Scan();                           /* 扫描物理按键，更新按键状态 */
            MenuEvent evt = get_next_event();     /* 将按键动作转为菜单事件枚举 */
            handle_event(evt);                    /* 处理菜单事件（导航/选择/返回） */
        }

        /* 10ms 循环延时：
         * - 按键消抖：物理按键抖动通常在 5-20ms，10ms 间隔可有效过滤
         * - 功耗控制：避免 CPU 空转，降低整体功耗
         * - 模块调度：为各模块的 tick 提供相对稳定的时间基准 */
        HAL_Delay(10);
    }
}
