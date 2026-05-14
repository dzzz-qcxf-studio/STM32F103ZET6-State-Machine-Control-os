/**
 * @file    module.h
 * @brief   模块生命周期框架 — 统一的功能模块管理接口
 *
 * 框架概述：
 *   本头文件定义了嵌入式状态机控制系统中"功能模块"的统一管理接口。
 *   系统将每个独立功能（如主菜单、参数设置、运行监控、校准等）封装为一个"模块"，
 *   每个模块通过 ModuleInterface 结构体声明自己的生命周期回调函数。
 *
 * 生命周期回调：
 *   - enter(): 模块被激活时调用一次（初始化硬件、绘制UI首屏、启动采集等）
 *   - tick():  主循环每帧调用一次（非阻塞，处理按键输入、刷新显示、更新状态）
 *   - exit():  模块被切走时调用一次（关闭外设、保存参数、释放显示区域等）
 *
 * 使用方式：
 *   1. 在各模块的 .c 文件中定义一个 const ModuleInterface 实例（填充 name/enter/tick/exit）
 *   2. 在系统初始化阶段调用 module_register() 将模块注册到管理器
 *   3. 主循环中调用 module_run_tick() 驱动当前活跃模块
 *   4. 需要切换模块时调用 module_switch_to() 或 module_exit_current()
 *
 * 设计优势：
 *   - 模块间完全解耦：模块之间不直接调用，只通过管理器间接切换
 *   - 统一的生命周期管理：enter/tick/exit 模式适用于所有功能模块
 *   - 轻量级实现：纯指针数组 + 函数指针，无动态内存分配，适合嵌入式环境
 *   - 可扩展性强：新增模块只需定义 ModuleInterface 并注册即可
 */

#ifndef __MODULE_H
#define __MODULE_H

#include "stm32f1xx_hal.h"

/*
 * 最大可注册模块数：
 *   限制模块注册表 g_modules[] 的容量为16。
 *   本项目的功能模块通常不超过10个（菜单、设置、运行、调参、校准等），
 *   预留一定余量以便后续扩展。此值直接影响 g_modules[] 数组的 RAM 占用
 *   （16 * sizeof指针 = 16 * 4 = 64 字节，在STM32F103上完全可以接受）。
 *   如需更多模块，增大此值即可，无需修改其他代码。
 */
#define MODULE_MAX_COUNT  16

/*
 * 模块操作返回码：
 *   用于 module_register() 等函数的返回值，指示操作结果。
 *   使用 uint8_t 类型而非 enum，节省空间且便于与其他错误码区分。
 */
#define MODULE_OK         0   /* 操作成功 */
#define MODULE_ERR_FULL   1   /* 注册表已满（已注册模块数达到 MODULE_MAX_COUNT） */
#define MODULE_ERR_NULL   2   /* 传入的模块指针为 NULL */

/*
 * 模块状态枚举：
 *   描述单个模块的运行状态。虽然当前管理器主要通过 g_current_module 指针
 *   判断模块是否活跃，但此枚举可用于更精细的状态管理场景（如模块暂停/恢复）。
 *
 *   MODULE_STATE_IDLE   — 空闲态：模块已注册但未被激活，其 tick() 不会被调用
 *   MODULE_STATE_ACTIVE — 活跃态：模块正在运行，主循环每帧调用其 tick()
 *
 * 状态转换路径：
 *   IDLE → (module_switch_to 调用 enter) → ACTIVE
 *   ACTIVE → (module_exit_current 调用 exit) → IDLE
 *   ACTIVE → (module_switch_to 调用 exit + 另一个模块 enter) → IDLE（旧模块）
 */
typedef enum {
    MODULE_STATE_IDLE,       /* 未激活：模块已注册但未运行 */
    MODULE_STATE_ACTIVE,     /* 正在运行：模块处于活跃状态，每帧执行 tick() */
} ModuleState;

/*
 * 模块生命周期接口结构体：
 *   这是整个模块框架的核心抽象。每个功能模块通过定义一个此结构体的
 *   const 实例来声明自己的名称和生命周期回调函数。
 *
 *   使用示例（在模块的 .c 文件中）：
 *     static void menu_enter(void) { ... }
 *     static void menu_tick(void)  { ... }
 *     static void menu_exit(void)  { ... }
 *     const ModuleInterface g_menu_module = {
 *         .name  = "Menu",
 *         .enter = menu_enter,
 *         .tick  = menu_tick,
 *         .exit  = menu_exit,
 *     };
 *
 * 设计说明：
 *   - 所有回调函数指针都可以为 NULL，管理器在调用前会做空指针检查
 *   - 结构体声明为 const，存储在 Flash（.rodata段）中，不占用 RAM
 *   - 回调函数均为 void(void) 签名，简化接口，模块内部自行管理状态
 *
 * 字段说明见下方注释：
 */
typedef struct {
    /*
     * 模块名称（字符串常量）：
     *   用于 module_find() 按名称查找模块，也可用于调试日志输出。
     *   建议使用简短的英文名称（如 "Menu", "Settings", "Runtime"），
     *   指向 .rodata 段中的字符串字面量，不占用 RAM。
     */
    const char *name;

    /*
     * enter() — 进入模块回调：
     *   在 module_switch_to() 切换到本模块时被调用，且仅调用一次。
     *   典型操作：初始化外设（ADC/UART/PWM）、绘制UI首屏、
     *   启动数据采集、加载用户配置等。
     *   注意：此函数中不应有长时间阻塞操作，以免影响系统响应性。
     */
    void (*enter)(void);

    /*
     * tick() — 帧更新回调：
     *   由主循环通过 module_run_tick() 每帧调用一次。
     *   这是模块的"主循环体"，必须设计为非阻塞的。
     *   典型操作：扫描按键（非阻塞轮询）、更新状态机、
     *   刷新LCD显示、读取传感器、处理通信协议等。
     *   执行频率取决于主循环的循环周期（通常几ms到几十ms）。
     */
    void (*tick)(void);

    /*
     * exit() — 退出模块回调：
     *   在 module_switch_to() 或 module_exit_current() 退出本模块时调用，且仅调用一次。
     *   典型操作：关闭外设（停止ADC采集、关闭PWM输出）、
     *   保存运行参数到 Flash/EEPROM、清理显示区域、释放独占资源等。
     *   确保模块退出后不留"脏状态"，以便下次进入时能正常初始化。
     */
    void (*exit)(void);
} ModuleInterface;

/**
 * @brief  初始化模块管理器
 */
void module_mgr_init(void);

/**
 * @brief  注册一个模块
 * @param  mod: 模块接口指针
 * @retval MODULE_OK / MODULE_ERR_FULL / MODULE_ERR_NULL
 */
uint8_t module_register(const ModuleInterface *mod);

/**
 * @brief  切换到指定模块
 * @param  mod: 目标模块接口指针
 * @note   会先调用当前模块的 exit()，再调用目标模块的 enter()
 */
void module_switch_to(const ModuleInterface *mod);

/**
 * @brief  退出当前模块，回到空闲状态
 * @note   调用当前模块的 exit()，然后 current 置 NULL
 */
void module_exit_current(void);

/**
 * @brief  运行当前模块的 tick（主循环中调用）
 */
void module_run_tick(void);

/**
 * @brief  获取当前活跃模块
 * @retval 当前模块指针，无活跃模块时返回 NULL
 */
const ModuleInterface* module_get_current(void);

/**
 * @brief  按名称查找已注册模块
 * @param  name: 模块名称
 * @retval 模块指针，未找到返回 NULL
 */
const ModuleInterface* module_find(const char *name);

/**
 * @brief  获取已注册模块数量
 */
uint8_t module_get_count(void);

#endif /* __MODULE_H */
