/**
 ****************************************************************************************************
 * @file        key.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-06-08
 * @brief       按键驱动代码
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

#include "./BSP/KEY/key.h"
#include "./SYSTEM/delay/delay.h"

/**
 * @brief   初始化按键
 * @param   无
 * @retval  无
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    
    /* 使能GPIO端口时钟 */
    WKUP_GPIO_CLK_ENABLE();
    KEY0_GPIO_CLK_ENABLE();
    
    /* 配置WKUP控制引脚 */
    gpio_init_struct.Pin = WKUP_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WKUP_GPIO_PORT, &gpio_init_struct);
    
    /* 配置KEY0控制引脚 */
    gpio_init_struct.Pin = KEY0_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);
}

/**
 * @brief   扫描按键
 * @note    按键响应具有优先级：WKUP > KEY0
 * @param   mode: 扫描模式
 * @arg     0: 不支持连续按
 * @arg     1: 支持连续按
 * @retval  按键键值
 * @arg     NONE_PRES: 没有按键按下
 * @arg     WKUP_PRES: WKUP按键按下
 * @arg     KEY0_PRES: KEY0按键按下
 */
uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_release = 1;
    uint8_t key_value = NONE_PRES;
    
    if (mode != 0)
    {
        key_release = 1;
    }
    
    if ((key_release == 1) && ((WKUP == 1) || (KEY0 == 1)))
    {
        delay_ms(10);
        key_release = 0;
        
        if (KEY0 == 1)
        {
            key_value = KEY0_PRES;
        }
        
        if (WKUP == 1)
        {
            key_value = WKUP_PRES;
        }
    }
    else if ((WKUP == 0) && (KEY0 == 0))
    {
        key_release = 1;
    }
    
    return key_value;
}
