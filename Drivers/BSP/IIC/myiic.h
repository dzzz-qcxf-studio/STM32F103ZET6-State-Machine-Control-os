/**
 ****************************************************************************************************
 * @file        myiic.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-06-30
 * @brief       IIC 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 M144Z-M3最小系统板STM32F103版
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 * 
 ****************************************************************************************************
 */
 
#ifndef __MYIIC_H
#define __MYIIC_H

#include "./SYSTEM/sys/sys.h"

/* 引脚定义 */
#define IIC_SCL_GPIO_PORT           GPIOB
#define IIC_SCL_GPIO_PIN            GPIO_PIN_6
#define IIC_SCL_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define IIC_SDA_GPIO_PORT           GPIOB
#define IIC_SDA_GPIO_PIN            GPIO_PIN_7
#define IIC_SDA_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

/* IO操作 */
#define IIC_SCL(x)                  do{ x ? \
                                        HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET) : \
                                        HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_RESET); \
                                    }while(0)

#define IIC_SDA(x)                  do{ x ? \
                                        HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_SET) : \
                                        HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_RESET); \
                                    }while(0)

#define IIC_READ_SDA                HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN)

/* 函数声明 */
void iic_init(void);                        /* 初始化IIC的IO口 */
void iic_start(void);                       /* 发送IIC开始信号 */
void iic_stop(void);                        /* 发送IIC停止信号 */
void iic_ack(void);                         /* IIC发送ACK信号 */
void iic_nack(void);                        /* IIC不发送ACK信号 */
uint8_t iic_wait_ack(void);                 /* IIC等待ACK信号 */
void iic_send_byte(uint8_t txd);            /* IIC发送一个字节 */
uint8_t iic_read_byte(unsigned char ack);   /* IIC读取一个字节 */

#endif

