#ifndef __WIRELESS_H
#define __WIRELESS_H


// 遥控器指令结构体 - 改进版
typedef struct {
    uint8_t header;            // 帧头 (0xAA)
    uint8_t command;           // 命令类型
    uint8_t speed;             // 速度值 (0-100)
    uint8_t direction;         // 方向值 (0-180)
    uint8_t steering;          // 转向值 (0-180, 90为中心)
    uint8_t function;          // 功能键 (灯光、喇叭等)
    uint8_t touch_points;      // 触摸点数量 (0-5)
    uint8_t checksum;          // 校验和
} RemoteControl_t;


// 函数声明
void Wireless_Init(void);
uint8_t CalculateChecksum(RemoteControl_t *data);
void Set_Nrf24_Tx(void);
void Draw_Control_Areas(void);
void Update_Throttle_Display(uint8_t speed);
void Update_Steering_Display(uint8_t steering);
void Update_Function_Display(void);
void Send_Remote_Command(void);
void Process_Touch_Input(void);
void Nrf24_Control(void);

#endif
