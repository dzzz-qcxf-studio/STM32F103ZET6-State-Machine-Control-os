#ifndef __JOYSTICK_H
#define __JOYSTICK_H


#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

// 摇杆数据结构
typedef struct {
    int16_t x1;    // 左摇杆X
    int16_t y1;    // 左摇杆Y
    int16_t x2;    // 右摇杆X
    int16_t y2;    // 右摇杆Y
    uint8_t k1;    // 左按键
    uint8_t k2;    // 右按键
} JoystickState;

/* BSP层接口 */
void Joystick_Init(void);
void Joystick_Read(JoystickState *state);

#endif
