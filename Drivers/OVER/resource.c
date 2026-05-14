/**
 * @file    resource.c
 * @brief   引脚/定时器资源管理器实现
 *
 * 本模块通过静态表格记录每个 GPIO 引脚和定时器的占用者（owner），
 * 在模块注册资源时自动检测冲突并输出警告，防止多个驱动重复配置同一硬件。
 * 所有 API 均可在系统启动后任意时刻调用，适用于多模块协作的嵌入式项目。
 *
 * 设计要点：
 *   - 引脚和定时器各自维护一张静态注册表，容量由 resource.h 中的宏定义决定。
 *   - 释放资源时采用"与末尾元素交换"策略，实现 O(1) 时间复杂度的删除操作。
 *   - 冲突检测时通过 printf 输出详细的端口字母（A~G）和引脚编号，方便串口调试。
 */

#include "./OVER/resource.h"
#include <string.h>
#include <stdio.h>

/**
 * @brief  将 GPIO 引脚掩码（GPIO_PIN_x）转换为引脚编号（0~15）
 *
 * STM32 HAL 库中 GPIO_PIN_0 = 0x0001, GPIO_PIN_1 = 0x0002, ...,
 * GPIO_PIN_15 = 0x8000，即 pin = (1 << 编号)。
 * 本函数通过循环右移的方式统计最低位 1 前面有多少个 0，从而得到引脚编号。
 *
 * 为什么不直接用 __builtin_ctz()？
 *   因为 ARM Compiler 5 (ARMCC) 不支持该 GCC 内建函数，
 *   为了兼容 Keil MDK 默认编译器，这里手动实现了等价的位扫描逻辑。
 *
 * @param  pin  GPIO 引脚掩码，例如 GPIO_PIN_5（值为 0x0020）
 * @return 引脚编号 0~15；当 pin 为 0 时返回 0（属于无效输入的容错处理）
 */
static uint8_t pin_to_number(uint16_t pin)
{
    uint8_t n = 0;
    /* 每右移一位，引脚掩码缩小一半，直到只剩最低位的 1（即 pin == 1），
       此时 n 即为原始掩码中 1 所在的位索引 */
    while (pin > 1) { pin >>= 1; n++; }
    return n;
}

/* ======================== 引脚资源表 ======================== */

/**
 * @brief  引脚占用记录结构体
 *
 * 每条记录代表一个已被某模块"认领"的 GPIO 引脚，包含：
 *   - port:  GPIO 端口基地址（如 GPIOA、GPIOB 等），用于唯一标识端口
 *   - pin:   引脚掩码（如 GPIO_PIN_5 = 0x0020），与 STM32 HAL 定义一致
 *   - owner: 指向模块名称字符串的指针（如 "OLED"、"KEY"），用于冲突提示
 */
typedef struct {
    GPIO_TypeDef *port;   /**< GPIO 端口指针，如 GPIOA ~ GPIOG */
    uint16_t      pin;    /**< 引脚掩码，如 GPIO_PIN_0 ~ GPIO_PIN_15 */
    const char   *owner;  /**< 占用者名称字符串，通常为模块名 */
} PinEntry;

/** 引脚资源注册表，容量由 resource.h 中 RESOURCE_MAX_PINS 定义 */
static PinEntry pin_table[RESOURCE_MAX_PINS];
/** 当前已注册的引脚数量，同时也是 pin_table 中下一个空闲槽位的索引 */
static uint8_t  pin_count = 0;

/* ======================== 定时器资源表 ======================== */

/**
 * @brief  定时器占用记录结构体
 *
 * 每条记录代表一个已被某模块"认领"的硬件定时器，包含：
 *   - tim:   定时器外设基地址（如 TIM1、TIM2 等），用于唯一标识定时器实例
 *   - owner: 指向模块名称字符串的指针（如 "PWM_MOTOR"），用于冲突提示
 */
typedef struct {
    TIM_TypeDef *tim;     /**< 定时器外设指针，如 TIM1 ~ TIM8 */
    const char  *owner;   /**< 占用者名称字符串，通常为模块名 */
} TimEntry;

/** 定时器资源注册表，容量由 resource.h 中 RESOURCE_MAX_TIMS 定义 */
static TimEntry tim_table[RESOURCE_MAX_TIMS];
/** 当前已注册的定时器数量，同时也是 tim_table 中下一个空闲槽位的索引 */
static uint8_t  tim_count = 0;

/* ======================== 初始化 ======================== */

/**
 * @brief  初始化资源管理器，清空所有引脚和定时器的注册记录
 *
 * 通常在系统启动早期（各驱动模块注册资源之前）调用一次。
 * 将两个计数器归零，并用 memset 将整张表格清零，
 * 确保所有 port/tim 指针为 NULL、owner 指针也为 NULL。
 */
void resource_init(void)
{
    pin_count = 0;                          /* 引脚计数归零 */
    tim_count = 0;                          /* 定时器计数归零 */
    memset(pin_table, 0, sizeof(pin_table));/* 清零引脚注册表 */
    memset(tim_table, 0, sizeof(tim_table));/* 清零定时器注册表 */
}

/* ======================== 引脚管理 ======================== */

/**
 * @brief  申请注册一个 GPIO 引脚
 *
 * 调用前会先扫描注册表，检查该引脚（port + pin 组合）是否已被其他模块占用。
 * 如果冲突，通过 printf 输出警告信息（包含端口字母、引脚编号和双方模块名），
 * 并返回 RES_ERR_BUSY；如果表格已满则返回 RES_ERR_FULL；
 * 注册成功返回 RES_OK。
 *
 * @param  port   GPIO 端口指针（如 GPIOA）
 * @param  pin    引脚掩码（如 GPIO_PIN_5）
 * @param  owner  占用者名称字符串（如 "OLED_SDA"）
 * @return RES_OK=注册成功, RES_ERR_BUSY=引脚冲突, RES_ERR_FULL=表格已满
 */
uint8_t pin_request(GPIO_TypeDef *port, uint16_t pin, const char *owner)
{
    /* ---------- 第一步：遍历注册表，检查是否已被占用 ---------- */
    for (uint8_t i = 0; i < pin_count; i++) {
        if (pin_table[i].port == port && pin_table[i].pin == pin) {
            /* 发现冲突：打印详细的冲突报告
               %s = 请求方模块名, %c = 端口字母（A~G）, %d = 引脚编号（0~15）, %s = 已占用方模块名 */
            printf("[RESOURCE] PIN conflict: %s wants P%c%d, already owned by %s\r\n",
                   owner,
                   /* 端口地址 → 字母映射：GPIOA→'A', GPIOB→'B', ..., GPIOF→'F', 其余→'G' */
                   (port == GPIOA) ? 'A' : (port == GPIOB) ? 'B' :
                   (port == GPIOC) ? 'C' : (port == GPIOD) ? 'D' :
                   (port == GPIOE) ? 'E' : (port == GPIOF) ? 'F' : 'G',
                   pin_to_number(pin),   /* 将掩码转换为 0~15 的编号 */
                   pin_table[i].owner);  /* 已占用该引脚的模块名 */
            return RES_ERR_BUSY;        /* 返回"资源忙"错误码 */
        }
    }

    /* ---------- 第二步：检查注册表是否已满 ---------- */
    if (pin_count >= RESOURCE_MAX_PINS) return RES_ERR_FULL;

    /* ---------- 第三步：在表尾追加新条目 ---------- */
    pin_table[pin_count].port  = port;   /* 记录端口 */
    pin_table[pin_count].pin   = pin;    /* 记录引脚掩码 */
    pin_table[pin_count].owner = owner;  /* 记录占用者名称 */
    pin_count++;                         /* 已注册计数加 1 */

    return RES_OK;                       /* 注册成功 */
}

/**
 * @brief  释放（注销）一个已注册的 GPIO 引脚
 *
 * 在注册表中查找匹配的 port + pin 条目，找到后将其删除。
 * 删除策略：将表尾最后一个元素复制到被删除的位置，然后计数减 1。
 * 这种"尾元素覆盖"方式避免了大量元素的移位操作，实现 O(1) 时间复杂度。
 * 注意：这会导致表中元素的相对顺序改变，但本模块不依赖顺序，所以没有影响。
 *
 * @param  port   GPIO 端口指针
 * @param  pin    引脚掩码
 * @param  owner  占用者名称（当前实现中未用于匹配，仅 port+pin 唯一标识）
 * @return RES_OK=释放成功, RES_ERR_NOTFOUND=未找到该引脚记录
 */
uint8_t pin_release(GPIO_TypeDef *port, uint16_t pin, const char *owner)
{
    for (uint8_t i = 0; i < pin_count; i++) {
        if (pin_table[i].port == port && pin_table[i].pin == pin) {
            /* O(1) 删除：用表尾元素覆盖当前位置，然后缩减计数 */
            pin_table[i] = pin_table[pin_count - 1];
            pin_count--;
            return RES_OK;
        }
    }
    return RES_ERR_NOTFOUND;  /* 未找到匹配的引脚记录 */
}

/**
 * @brief  查询指定 GPIO 引脚的当前占用者
 *
 * 遍历注册表，查找与给定 port + pin 匹配的条目。
 * 如果找到，返回占用者的名称字符串；否则返回 NULL 表示该引脚未被占用。
 * 典型用途：在配置引脚前检查其是否已被其他模块使用，或在调试时打印资源状态。
 *
 * @param  port  GPIO 端口指针
 * @param  pin   引脚掩码
 * @return 占用者名称字符串指针；未找到时返回 NULL
 */
const char* pin_query(GPIO_TypeDef *port, uint16_t pin)
{
    for (uint8_t i = 0; i < pin_count; i++) {
        if (pin_table[i].port == port && pin_table[i].pin == pin) {
            return pin_table[i].owner;  /* 找到匹配条目，返回占用者名称 */
        }
    }
    return NULL;  /* 未找到，该引脚未被任何模块占用 */
}

/* ======================== 定时器管理 ======================== */

/**
 * @brief  申请注册一个硬件定时器
 *
 * 与 pin_request() 逻辑类似：先扫描注册表检查定时器是否已被占用，
 * 冲突时输出警告（包含定时器编号和双方模块名），表格满则返回错误，
 * 注册成功返回 RES_OK。
 *
 * @param  tim    定时器外设指针（如 TIM1、TIM2 等）
 * @param  owner  占用者名称字符串（如 "PWM_MOTOR"）
 * @return RES_OK=注册成功, RES_ERR_BUSY=定时器冲突, RES_ERR_FULL=表格已满
 */
uint8_t tim_request(TIM_TypeDef *tim, const char *owner)
{
    /* ---------- 第一步：遍历注册表，检查定时器是否已被占用 ---------- */
    for (uint8_t i = 0; i < tim_count; i++) {
        if (tim_table[i].tim == tim) {
            /* 发现冲突：打印详细的冲突报告
               %s = 请求方模块名, %d = 定时器编号（1~8）, %s = 已占用方模块名 */
            printf("[RESOURCE] TIM conflict: %s wants TIM%d, already owned by %s\r\n",
                   owner,
                   /* 定时器外设地址 → 编号映射：TIM1→1, TIM2→2, ..., TIM8→8, 其余→0 */
                   (tim == TIM1) ? 1 : (tim == TIM2) ? 2 :
                   (tim == TIM3) ? 3 : (tim == TIM4) ? 4 :
                   (tim == TIM5) ? 5 : (tim == TIM6) ? 6 :
                   (tim == TIM7) ? 7 : (tim == TIM8) ? 8 : 0,
                   tim_table[i].owner);  /* 已占用该定时器的模块名 */
            return RES_ERR_BUSY;        /* 返回"资源忙"错误码 */
        }
    }

    /* ---------- 第二步：检查注册表是否已满 ---------- */
    if (tim_count >= RESOURCE_MAX_TIMS) return RES_ERR_FULL;

    /* ---------- 第三步：在表尾追加新条目 ---------- */
    tim_table[tim_count].tim   = tim;    /* 记录定时器外设指针 */
    tim_table[tim_count].owner = owner;  /* 记录占用者名称 */
    tim_count++;                         /* 已注册计数加 1 */

    return RES_OK;                       /* 注册成功 */
}

/**
 * @brief  释放（注销）一个已注册的硬件定时器
 *
 * 与 pin_release() 逻辑相同：找到匹配条目后，用表尾元素覆盖实现 O(1) 删除。
 *
 * @param  tim    定时器外设指针
 * @param  owner  占用者名称（当前实现中未用于匹配，仅 tim 唯一标识）
 * @return RES_OK=释放成功, RES_ERR_NOTFOUND=未找到该定时器记录
 */
uint8_t tim_release(TIM_TypeDef *tim, const char *owner)
{
    for (uint8_t i = 0; i < tim_count; i++) {
        if (tim_table[i].tim == tim) {
            /* O(1) 删除：用表尾元素覆盖当前位置，然后缩减计数 */
            tim_table[i] = tim_table[tim_count - 1];
            tim_count--;
            return RES_OK;
        }
    }
    return RES_ERR_NOTFOUND;  /* 未找到匹配的定时器记录 */
}

/**
 * @brief  查询指定硬件定时器的当前占用者
 *
 * 与 pin_query() 逻辑相同：遍历注册表查找匹配的定时器实例，
 * 找到则返回占用者名称，未找到返回 NULL。
 *
 * @param  tim  定时器外设指针
 * @return 占用者名称字符串指针；未找到时返回 NULL
 */
const char* tim_query(TIM_TypeDef *tim)
{
    for (uint8_t i = 0; i < tim_count; i++) {
        if (tim_table[i].tim == tim) {
            return tim_table[i].owner;  /* 找到匹配条目，返回占用者名称 */
        }
    }
    return NULL;  /* 未找到，该定时器未被任何模块占用 */
}
