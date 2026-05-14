/**
 * @file    module.c
 * @brief   模块生命周期管理器实现
 *
 * 设计思路：
 *   本文件实现了一个轻量级的模块生命周期管理框架，适用于嵌入式状态机控制系统。
 *   核心思想是将系统功能拆分为独立模块（如菜单、参数设置、运行监控等），
 *   每个模块通过 ModuleInterface 结构体注册自己的生命周期回调函数。
 *
 *   管理器维护一个全局模块注册表 g_modules[]，最多支持 MODULE_MAX_COUNT 个模块。
 *   同一时刻只有一个模块处于 ACTIVE 状态（由 g_current_module 指针标识），
 *   主循环通过 module_run_tick() 驱动当前活跃模块的 tick() 回调，
 *   实现非阻塞式的帧驱动调度。
 *
 *   模块切换遵循严格的 exit → switch → enter 三步序列，
 *   确保旧模块能正确释放资源，新模块能完整初始化。
 */

#include "./OVER/module.h"
#include <string.h>

/*
 * 已注册模块表：存储所有通过 module_register() 注册的模块接口指针。
 * 使用静态指针数组实现，避免动态内存分配（嵌入式系统中 malloc 不推荐使用）。
 * 数组大小由 MODULE_MAX_COUNT（module.h 中定义，默认16）限定，
 * 足够覆盖本项目的功能模块数量（菜单、设置、运行、调参等）。
 * 指针指向的是调用方（各模块.c文件）中定义的 const ModuleInterface 实例，
 * 因此模块数据本身存储在 Flash 中，此处仅保存指针，RAM 开销极小。
 */
static const ModuleInterface *g_modules[MODULE_MAX_COUNT];

/* 当前已注册模块数量，同时也是 g_modules[] 数组中下一个空闲槽位的索引 */
static uint8_t g_module_count = 0;

/*
 * 当前活跃模块指针：
 *   - 非NULL：指向正在运行的模块，主循环会驱动其 tick() 回调
 *   - NULL：系统处于菜单/空闲状态，无模块在运行
 * 切换模块时，管理器会自动调用旧模块的 exit() 和新模块的 enter()，
 * 确保生命周期回调的完整性和顺序性。
 */
static const ModuleInterface *g_current_module = NULL;

/**
 * @brief  初始化模块管理器
 *
 * 将管理器恢复到上电初始状态：
 *   1. 清零注册计数器（无任何模块注册）
 *   2. 将当前活跃模块置 NULL（系统处于空闲态）
 *   3. 用 memset 将整个模块指针表清零，确保无野指针
 *
 * 此函数应在系统 main() 中、外设初始化之后、主循环之前调用一次。
 * 调用后各功能模块才能通过 module_register() 逐一注册自身。
 */
void module_mgr_init(void)
{
    g_module_count = 0;                  /* 无已注册模块 */
    g_current_module = NULL;             /* 无活跃模块，系统空闲 */
    memset(g_modules, 0, sizeof(g_modules)); /* 清零指针表，防止野指针 */
}

/**
 * @brief  注册一个模块到管理器
 *
 * 注册流程：
 *   1. 防御性检查：传入指针不能为 NULL
 *   2. 容量检查：已注册数量不能超过 MODULE_MAX_COUNT（16）
 *   3. 将模块指针存入 g_modules[] 数组的下一个空闲槽位，
 *      同时 g_module_count 自增，指向新的空闲位置
 *
 * 注册顺序决定了模块在数组中的索引位置，但不影响运行时行为，
 * 因为模块切换是通过指针直接定位的，而非按索引查找。
 *
 * @param  mod: 模块接口指针（通常指向各模块 .c 文件中的 const 静态实例）
 * @retval MODULE_OK       注册成功
 * @retval MODULE_ERR_NULL 传入指针为 NULL
 * @retval MODULE_ERR_FULL 注册表已满（已注册16个模块）
 */
uint8_t module_register(const ModuleInterface *mod)
{
    /* 空指针检查：防止注册无效模块 */
    if (mod == NULL) return MODULE_ERR_NULL;

    /* 容量检查：防止数组越界写入 */
    if (g_module_count >= MODULE_MAX_COUNT) return MODULE_ERR_FULL;

    /* 将模块指针存入注册表，计数器后移一位 */
    g_modules[g_module_count++] = mod;
    return MODULE_OK;
}

/**
 * @brief  切换到指定模块（核心调度函数）
 *
 * 模块切换的三步序列（exit → switch → enter）：
 *
 *   第一步 — 退出旧模块：
 *     如果当前有活跃模块（g_current_module != NULL），
 *     且该模块注册了 exit() 回调，则调用 exit() 让旧模块清理资源
 *     （如关闭定时器、保存参数、释放显示区域等）。
 *
 *   第二步 — 切换指针：
 *     将 g_current_module 指向新的目标模块。
 *     此刻旧模块已退出，新模块尚未进入，系统处于短暂的"过渡态"。
 *
 *   第三步 — 进入新模块：
 *     如果新模块注册了 enter() 回调，则调用 enter() 让新模块初始化
 *     （如配置硬件、绘制首屏UI、启动采集等）。
 *
 * 设计决策：
 *   - 使用"先退出再进入"的顺序，避免新旧模块同时操作硬件导致冲突
 *   - 每个回调在调用前都检查函数指针是否为 NULL，允许模块省略某些回调
 *   - 此函数可在任意上下文中调用（中断中不推荐，因为回调可能耗时）
 *
 * @param  mod: 目标模块接口指针，不能为 NULL
 */
void module_switch_to(const ModuleInterface *mod)
{
    /* 防御：目标模块指针为空则不执行任何操作 */
    if (mod == NULL) return;

    /*
     * 第一步：退出当前模块
     * 双重检查：1) 确实有活跃模块  2) 该模块注册了 exit 回调
     * exit() 中通常执行：关闭外设、保存运行数据、清理显示等
     */
    if (g_current_module != NULL && g_current_module->exit != NULL) {
        g_current_module->exit();
    }

    /* 第二步：将活跃指针指向新模块（逻辑切换点） */
    g_current_module = mod;

    /*
     * 第三步：进入新模块
     * enter() 中通常执行：初始化外设、绘制UI首屏、启动数据采集等
     * 注意：此处使用 g_current_module->enter 而非 mod->enter，
     * 虽然两者等价，但保持与第一步的风格一致（都通过 g_current_module 访问）
     */
    if (g_current_module->enter != NULL) {
        g_current_module->enter();
    }
}

/**
 * @brief  退出当前模块，系统回到空闲状态
 *
 * 使用场景：
 *   - 用户在某个功能模块中按下"返回"键，需要回到主菜单
 *   - 模块运行完毕自动退出（如校准流程结束）
 *   - 系统异常需要强制回到空闲态
 *
 * 执行逻辑：
 *   1. 如果有活跃模块且注册了 exit()，则调用 exit() 清理资源
 *   2. 将 g_current_module 置 NULL，系统回到空闲态
 *      （此时主循环的 module_run_tick() 会因 NULL 检查而跳过，不做任何事）
 *
 * 与 module_switch_to() 的区别：
 *   - switch_to 是"退出旧模块 + 进入新模块"，系统仍有活跃模块
 *   - exit_current 只"退出旧模块"，系统进入无模块的空闲态
 */
void module_exit_current(void)
{
    /* 调用当前模块的 exit 回调进行资源清理 */
    if (g_current_module != NULL && g_current_module->exit != NULL) {
        g_current_module->exit();
    }
    /* 置 NULL：系统回到空闲态，不再有活跃模块 */
    g_current_module = NULL;
}

/**
 * @brief  运行当前活跃模块的 tick 回调（主循环每帧调用）
 *
 * 这是整个调度框架的"心跳"函数，由 main() 中的 while(1) 主循环调用。
 * 每次调用相当于一个"帧"，当前活跃模块的 tick() 在其中执行一次。
 *
 * 调用条件（双重防御）：
 *   1. g_current_module != NULL — 确实有活跃模块在运行
 *   2. g_current_module->tick != NULL — 该模块注册了 tick 回调
 *
 * 典型的 tick() 实现（非阻塞设计）：
 *   - 扫描按键输入（非阻塞轮询）
 *   - 更新内部状态机
 *   - 刷新显示内容（LCD/OLED）
 *   - 读取传感器数据
 *   - 通信协议收发处理
 *
 * 注意：tick() 必须是非阻塞的！如果某个模块的 tick() 执行时间过长，
 * 会阻塞其他模块的调度（虽然同一时刻只有一个活跃模块），
 * 并影响系统的整体响应性。
 */
void module_run_tick(void)
{
    /* 仅在有活跃模块且注册了 tick 回调时才执行 */
    if (g_current_module != NULL && g_current_module->tick != NULL) {
        g_current_module->tick();
    }
    /* 无活跃模块时（g_current_module == NULL），直接返回，不做任何事 */
}

/**
 * @brief  获取当前活跃模块的接口指针
 *
 * 供外部代码（如菜单系统、UI层）查询当前正在运行的模块。
 * 返回值可能为 NULL，表示系统处于空闲态（无模块在运行）。
 *
 * 典型用途：
 *   - 菜单模块根据当前活跃模块决定显示哪个高亮项
 *   - 按键处理逻辑判断是否需要转发按键给当前模块
 *   - 调试时打印当前模块名称
 *
 * @retval 当前活跃模块的接口指针，无活跃模块时返回 NULL
 */
const ModuleInterface* module_get_current(void)
{
    return g_current_module;
}

/**
 * @brief  按名称查找已注册模块
 *
 * 在 g_modules[] 注册表中线性搜索，通过 strcmp 精确匹配模块名称。
 * 模块名称是 ModuleInterface 结构体中的 name 字符串字段。
 *
 * 搜索算法：线性扫描（O(n)），对于最多16个模块的场景完全足够。
 * 每次比较前检查 g_modules[i] != NULL 防止空指针解引用，
 * 虽然正常情况下注册表中不会有 NULL，但这是防御性编程。
 *
 * 典型用途：
 *   - 根据配置文件中的模块名称字符串，动态定位并切换到对应模块
 *   - 菜单系统中通过名称引用模块（与具体模块解耦）
 *   - 调试/日志中按名称查找模块信息
 *
 * @param  name: 要查找的模块名称（C字符串，以 '\0' 结尾）
 * @retval 找到时返回模块接口指针，未找到或 name 为 NULL 时返回 NULL
 */
const ModuleInterface* module_find(const char *name)
{
    /* 防御：名称指针为空则直接返回 */
    if (name == NULL) return NULL;

    /* 线性扫描注册表，逐个比较名称字符串 */
    for (uint8_t i = 0; i < g_module_count; i++) {
        /* 双重检查：槽位非空 且 名称完全匹配 */
        if (g_modules[i] != NULL && strcmp(g_modules[i]->name, name) == 0) {
            return g_modules[i]; /* 找到目标模块，返回其接口指针 */
        }
    }

    /* 遍历完整个注册表未找到匹配的模块名称 */
    return NULL;
}

/**
 * @brief  获取当前已注册的模块数量
 *
 * 返回值范围：0 ~ MODULE_MAX_COUNT（16）。
 * 可用于：
 *   - 菜单系统动态生成模块列表（遍历 g_modules[0..count-1]）
 *   - 调试时确认所有模块是否都已正确注册
 *   - 注册前预判是否还有空闲槽位（虽然 module_register 内部会检查）
 *
 * @retval 已注册模块的数量
 */
uint8_t module_get_count(void)
{
    return g_module_count;
}
