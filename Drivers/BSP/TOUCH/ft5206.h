/**
 ****************************************************************************************************
 * @file        ft5206.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-06-30
 * @brief       7寸电容触摸屏-FT5206/FT5426/CST340 驱动代码
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
 
#ifndef __FT5206_H
#define __FT5206_H

#include "./SYSTEM/sys/sys.h"

/* 引脚定义 */
#define FT5206_RST_GPIO_PORT            GPIOF
#define FT5206_RST_GPIO_PIN             GPIO_PIN_11
#define FT5206_RST_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

#define FT5206_INT_GPIO_PORT            GPIOF
#define FT5206_INT_GPIO_PIN             GPIO_PIN_10
#define FT5206_INT_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

/* IO操作 */
#define FT5206_RST(x)                   do{ x ? \
                                            HAL_GPIO_WritePin(FT5206_RST_GPIO_PORT, FT5206_RST_GPIO_PIN, GPIO_PIN_SET) : \
                                            HAL_GPIO_WritePin(FT5206_RST_GPIO_PORT, FT5206_RST_GPIO_PIN, GPIO_PIN_RESET); \
                                        }while(0)

#define FT5206_INT                      HAL_GPIO_ReadPin(FT5206_INT_GPIO_PORT, FT5206_INT_GPIO_PIN)

/* IIC读写命令 */
#define FT5206_CMD_WR                   0X70                    /* 写命令(最低位为0) */
#define FT5206_CMD_RD                   0X71                    /* 读命令(最低位为1) */

/* FT5206 部分寄存器定义  */
#define FT5206_DEVIDE_MODE              0x00                    /* FT5206模式控制寄存器 */
#define FT5206_REG_NUM_FINGER           0x02                    /* 触摸状态寄存器 */
#define FT5206_TP1_REG                  0X03                    /* 第一个触摸点数据地址 */
#define FT5206_TP2_REG                  0X09                    /* 第二个触摸点数据地址 */
#define FT5206_TP3_REG                  0X0F                    /* 第三个触摸点数据地址 */
#define FT5206_TP4_REG                  0X15                    /* 第四个触摸点数据地址 */
#define FT5206_TP5_REG                  0X1B                    /* 第五个触摸点数据地址 */ 
#define	FT5206_ID_G_LIB_VERSION         0xA1                    /* 版本 */
#define FT5206_ID_G_MODE                0xA4                    /* FT5206中断模式控制寄存器 */
#define FT5206_ID_G_THGROUP             0x80                    /* 触摸有效值设置寄存器 */
#define FT5206_ID_G_PERIODACTIVE        0x88                    /* 激活状态周期设置寄存器 */

/* 函数声明 */
uint8_t ft5206_wr_reg(uint16_t reg,uint8_t *buf,uint8_t len);   /* 向FT5206写入一次数据 */
void ft5206_rd_reg(uint16_t reg,uint8_t *buf,uint8_t len);      /* 从FT5206读出一次数据 */
uint8_t ft5206_init(void);                                      /* 初始化FT5206触摸屏 */
uint8_t ft5206_scan(uint8_t mode);                              /* 扫描触摸屏(采用查询方式) */

#endif
