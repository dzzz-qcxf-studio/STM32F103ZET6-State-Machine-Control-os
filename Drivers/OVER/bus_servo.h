/**
 * @file    bus_servo.h
 * @brief   总线舵机控制模块 — 串口指令控制总线舵机
 *
 * 协议：#<ID>P<PWM>T<TIME>!  (ID 3位, PWM 4位, TIME 4位)
 * 通信：USART3 (PB10-TX, PB11-RX), 115200bps
 */

#ifndef __BUS_SERVO_H
#define __BUS_SERVO_H

#include "stm32f1xx_hal.h"

#endif /* __BUS_SERVO_H */
