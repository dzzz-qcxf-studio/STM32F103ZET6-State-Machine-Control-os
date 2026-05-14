/**
 * @file    resource.h
 * @brief   引脚/定时器资源管理器 — 防止多模块资源冲突
 */

#ifndef __RESOURCE_H
#define __RESOURCE_H

#include "stm32f1xx_hal.h"

#define RESOURCE_MAX_PINS  32
#define RESOURCE_MAX_TIMS   8

/* 资源操作结果 */
#define RES_OK           0
#define RES_ERR_BUSY     1  /* 资源已被占用 */
#define RES_ERR_FULL     2  /* 资源表已满 */
#define RES_ERR_NOTFOUND 3  /* 未找到占用记录 */

/**
 * @brief  初始化资源管理器
 */
void resource_init(void);

/**
 * @brief  申请引脚使用权
 * @param  port: GPIO 端口 (GPIOA, GPIOB, ...)
 * @param  pin:  引脚号 (GPIO_PIN_0, ...)
 * @param  owner: 申请者模块名
 * @retval RES_OK / RES_ERR_BUSY / RES_ERR_FULL
 */
uint8_t pin_request(GPIO_TypeDef *port, uint16_t pin, const char *owner);

/**
 * @brief  释放引脚使用权
 * @param  port: GPIO 端口
 * @param  pin:  引脚号
 * @param  owner: 释放者模块名（必须与申请者一致）
 * @retval RES_OK / RES_ERR_NOTFOUND
 */
uint8_t pin_release(GPIO_TypeDef *port, uint16_t pin, const char *owner);

/**
 * @brief  查询引脚当前占用者
 * @retval 占用者模块名，未占用返回 NULL
 */
const char* pin_query(GPIO_TypeDef *port, uint16_t pin);

/**
 * @brief  申请定时器使用权
 * @param  tim:   定时器实例 (TIM1, TIM2, ...)
 * @param  owner: 申请者模块名
 * @retval RES_OK / RES_ERR_BUSY / RES_ERR_FULL
 */
uint8_t tim_request(TIM_TypeDef *tim, const char *owner);

/**
 * @brief  释放定时器使用权
 * @param  tim:   定时器实例
 * @param  owner: 释放者模块名
 * @retval RES_OK / RES_ERR_NOTFOUND
 */
uint8_t tim_release(TIM_TypeDef *tim, const char *owner);

/**
 * @brief  查询定时器当前占用者
 * @retval 占用者模块名，未占用返回 NULL
 */
const char* tim_query(TIM_TypeDef *tim);

#endif /* __RESOURCE_H */
