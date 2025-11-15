#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "./OVER/wireless.h"
#include "./OVER/menu.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/TOUCH/touch.h"
#include "./BSP/NRF24L01/nrf24l01.h"
#include "./BSP/NRF24L01/NRF24L01_define.h"
#include <stdio.h>
#include <math.h>

extern volatile uint8_t pressed;
uint16_t wireless_init_flag=0;
//static uint8_t connect_number=0;

// 命令类型定义
#define CMD_STOP          0x00    // 停止
#define CMD_FORWARD       0x01    // 前进
#define CMD_BACKWARD      0x02    // 后退
#define CMD_LEFT          0x03    // 左转
#define CMD_RIGHT         0x04    // 右转
#define CMD_FORWARD_LEFT  0x05    // 前进左转
#define CMD_FORWARD_RIGHT 0x06    // 前进右转
#define CMD_BACKWARD_LEFT 0x07    // 后退左转
#define CMD_BACKWARD_RIGHT 0x08   // 后退右转
#define CMD_CUSTOM        0x0F    // 自定义控制

// 功能键定义
#define FUNC_NONE         0x00    // 无功能
#define FUNC_LIGHT        0x01    // 灯光
#define FUNC_HORN         0x02    // 喇叭
#define FUNC_DEMO         0x04    // 演示模式
#define FUNC_TURBO        0x08    // 涡轮增压模式

// 控制区域定义
#define THROTTLE_AREA_X       50
#define THROTTLE_AREA_Y       150
#define THROTTLE_AREA_WIDTH   100
#define THROTTLE_AREA_HEIGHT  200

#define STEERING_AREA_X       250
#define STEERING_AREA_Y       150
#define STEERING_AREA_WIDTH   100
#define STEERING_AREA_HEIGHT  200

// 全局变量
RemoteControl_t remoteData;
uint8_t currentSpeed = 50;        // 默认速度50%
uint8_t currentDirection = 90;    // 默认方向居中90°
uint8_t currentSteering = 90;     // 默认转向居中90°
uint8_t currentFunction = 0;      // 默认无功能
uint8_t throttleActive = 0;       // 油门触控状态
uint8_t steeringActive = 0;       // 方向盘触控状态
uint8_t lastActiveTouchPoints = 0; // 上次活动触摸点数

// 油门和转向控制区域
typedef struct {
    uint16_t x;
    uint16_t y; 
    uint16_t width;
    uint16_t height;
    uint16_t centerX;
    uint16_t centerY;
} ControlArea_t;

ControlArea_t throttleArea;  // 油门区域
ControlArea_t steeringArea;  // 转向区域

void Wireless_Init(void){ 
    // 初始化 SPI 和 NRF24L01
	NRF24L01_Init();

	wireless_init_flag=1;
	
    // 清空菜单区域（不覆盖屏幕）
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WHITE);
	
}

// 计算校验和
uint8_t CalculateChecksum(RemoteControl_t *data) {
    uint8_t sum = data->header + data->command + data->speed + 
                  data->direction + data->steering + data->function + 
                  data->touch_points;
    return (uint8_t)(~sum + 1); // 取反加1
}

// 设置NRF24L01为发送模式
void Set_Nrf24_Tx(void){
    
    // 初始化遥控数据结构
    remoteData.header = 0xAA;
    remoteData.command = CMD_STOP;
    remoteData.speed = currentSpeed;
    remoteData.direction = currentDirection;
    remoteData.steering = currentSteering;
    remoteData.function = currentFunction;
    remoteData.touch_points = 0;
    remoteData.checksum = CalculateChecksum(&remoteData);
    
    delay_ms(200);
}


// 绘制控制区域
void Draw_Control_Areas() {
    // 绘制油门控制区域
    lcd_draw_rectangle(throttleArea.x, throttleArea.y, 
                       throttleArea.x + throttleArea.width, 
                       throttleArea.y + throttleArea.height, RED);
    
    // 绘制油门刻度
    for(int i=0; i<=10; i++) {
        uint16_t y = throttleArea.y + i * (throttleArea.height / 10);
        lcd_draw_line(throttleArea.x, y, throttleArea.x + 10, y, DARKBLUE);
        
        // 在右侧添加刻度值
        if(i % 2 == 0) {
            char value[4];
            sprintf(value, "%d", 100 - i*10);
            lcd_show_string(throttleArea.x + throttleArea.width + 5, y-8, 30, 16, 16, value, BLACK);
        }
    }
    
    // 绘制方向盘控制区域
    lcd_draw_rectangle(steeringArea.x, steeringArea.y, 
                       steeringArea.x + steeringArea.width, 
                       steeringArea.y + steeringArea.height, BLUE);
    
    // 绘制方向盘中心线
    lcd_draw_line(steeringArea.x, steeringArea.centerY, 
                 steeringArea.x + steeringArea.width, steeringArea.centerY, DARKBLUE);
    
    // 绘制方向盘刻度
    for(int i=0; i<=10; i++) {
        uint16_t x = steeringArea.x + i * (steeringArea.width / 10);
        lcd_draw_line(x, steeringArea.y + steeringArea.height - 10, 
                     x, steeringArea.y + steeringArea.height, DARKBLUE);
        
        // 在底部添加刻度值(只显示几个关键值)
        if(i == 0 || i == 5 || i == 10) {
            char value[4];
            sprintf(value, "%d", i*18);  // 0-180度
            lcd_show_string(x-8, steeringArea.y + steeringArea.height + 5, 30, 16, 16, value, BLACK);
        }
    }
    
    // 标签
    lcd_show_string(throttleArea.x, throttleArea.y - 30, 100, 24, 24, "WS",RED);
    lcd_show_string(steeringArea.x, steeringArea.y - 30, 100, 24, 24, "AD",BLUE);
}

// 更新油门控制器显示
void Update_Throttle_Display(uint8_t speed) {
    static uint8_t last_speed = 0xFF;
    
    if(last_speed == speed) return; // 避免重复绘制
    
    // 清除旧的指示器
    if(last_speed != 0xFF) {
        uint16_t last_y = throttleArea.y + throttleArea.height - (last_speed * throttleArea.height / 100);
        lcd_fill(throttleArea.x + 5, last_y - 10, 
                 throttleArea.x + throttleArea.width - 5, last_y + 10, WHITE);
    }
    
    // 绘制新的指示器
    uint16_t y = throttleArea.y + throttleArea.height - (speed * throttleArea.height / 100);
    lcd_fill(throttleArea.x + 5, y - 10, 
             throttleArea.x + throttleArea.width - 5, y + 10, RED);
    
    last_speed = speed;
}

// 更新方向盘控制器显示
void Update_Steering_Display(uint8_t steering) {
    static uint8_t last_steering = 0xFF;
    
    if(last_steering == steering) return; // 避免重复绘制
    
    // 清除旧的指示器
    if(last_steering != 0xFF) {
        uint16_t last_x = steeringArea.x + (last_steering * steeringArea.width / 180);
        lcd_fill(last_x - 10, steeringArea.centerY - 10, 
                 last_x + 10, steeringArea.centerY + 10, WHITE);
    }
    
    // 绘制新的指示器
    uint16_t x = steeringArea.x + (steering * steeringArea.width / 180);
    lcd_fill(x - 10, steeringArea.centerY - 10, 
             x + 10, steeringArea.centerY + 10, BLUE);
    
    last_steering = steering;
}

// 显示功能状态
void Update_Function_Display() {
    // 绘制功能按钮区域
    //lcd_fill(50, 385, 400, 430, WHITE);
    
    // 显示功能状态
    lcd_show_string(60, 385, 60, 16, 16, "LIGHT:", BLACK);
    lcd_show_string(110, 385, 30, 16, 16, (currentFunction & FUNC_LIGHT) ? " ON" : "OFF", //注意此处on前方有空格，实现对齐off
                   (currentFunction & FUNC_LIGHT) ? GREEN : RED);
    
    lcd_show_string(140, 385, 60, 16, 16, "HORN:", BLACK);
    lcd_show_string(180, 385, 30, 16, 16, (currentFunction & FUNC_HORN) ? " ON" : "OFF", 
                   (currentFunction & FUNC_HORN) ? GREEN : RED);
    
    lcd_show_string(220, 385, 60, 16, 16, "DEMO:", BLACK);
    lcd_show_string(260, 385, 30, 16, 16, (currentFunction & FUNC_DEMO) ? " ON" : "OFF", 
                   (currentFunction & FUNC_DEMO) ? GREEN : RED);
    
    lcd_show_string(300, 385, 60, 16, 16, "TURBO:", BLACK);
    lcd_show_string(350, 385, 30, 16, 16, (currentFunction & FUNC_TURBO) ? " ON" : "OFF", 
                   (currentFunction & FUNC_TURBO) ? GREEN : RED);
}

// 发送遥控命令
void Send_Remote_Command() {
    // 更新遥控数据
    remoteData.header = 0xAA;
    
    // 根据当前油门和转向值判断命令类型
    if(currentSpeed < 10) {
        remoteData.command = CMD_STOP;
    } else {
        if(currentSteering < 40) {
            remoteData.command = (currentSpeed > 0) ? CMD_FORWARD_LEFT : CMD_BACKWARD_LEFT;
        } else if(currentSteering > 140) {
            remoteData.command = (currentSpeed > 0) ? CMD_FORWARD_RIGHT : CMD_BACKWARD_RIGHT;
        } else if(abs(currentSteering - 90) <= 15) {
            remoteData.command = (currentSpeed > 0) ? CMD_FORWARD : CMD_BACKWARD;
        } else if(currentSteering < 90) {
            remoteData.command = (currentSpeed > 0) ? CMD_FORWARD_LEFT : CMD_BACKWARD_LEFT;
        } else {
            remoteData.command = (currentSpeed > 0) ? CMD_FORWARD_RIGHT : CMD_BACKWARD_RIGHT;
        }
    }
    
    remoteData.speed = currentSpeed;
    remoteData.direction = currentDirection;
    remoteData.steering = currentSteering;
    remoteData.function = currentFunction;
    remoteData.touch_points = lastActiveTouchPoints;
    remoteData.checksum = CalculateChecksum(&remoteData);
    
    // 发送数据  
	Send((uint8_t*)&remoteData);

//    if(R_Reg(R_REGISTER+STATUS)==RX_OK){
//		connect_number=0;
//		lcd_show_string(10, 100, 160, 16, 16, "conect", BLACK);
//	}else{
//		if(connect_number>15)lcd_show_string(10, 100, 160, 16, 16, "no conect", RED);
//		connect_number++;
//	} 
    
	// 显示命令与触点信息
    char cmdInfo[64];
    sprintf(cmdInfo, "command:%02X Speed:%d Steering:%d LASTPoints:%d", 
            remoteData.command, currentSpeed, currentSteering, lastActiveTouchPoints);
    lcd_show_string(50, 75, 350, 16, 16, cmdInfo, DARKBLUE);
}

// 处理多点触控输入
void Process_Touch_Input() {
    uint8_t touchPoints = 0;
    uint8_t throttleFound = 0;
    uint8_t steeringFound = 0;
    
    // 扫描触摸屏
    tp_dev.scan(0);
    
    // 如果有触摸
    if(tp_dev.sta & TP_PRES_DOWN) {
        // 计算有效触摸点数量
        for(uint8_t i = 0; i < CT_MAX_TOUCH; i++) {
            // 检查触点是否有效 (sta的低位表示每个触点的状态)
            if((tp_dev.sta & (1 << i)) && (tp_dev.x[i] > 0) && (tp_dev.y[i] > 0)) {
                touchPoints++;
                
                // 检查是否在油门控制区域内
                if(!throttleFound && 
                   tp_dev.x[i] >= throttleArea.x && 
                   tp_dev.x[i] <= throttleArea.x + throttleArea.width &&
                   tp_dev.y[i] >= throttleArea.y && 
                   tp_dev.y[i] <= throttleArea.y + throttleArea.height) {
                    
                    throttleFound = 1;
                    throttleActive = 1;
                    
                    // 计算速度百分比 (上方为最大，下方为最小)
                    uint16_t relativeY = tp_dev.y[i] - throttleArea.y;
                    currentSpeed = 100 - (relativeY * 100 / throttleArea.height);
                    
                    // 限制范围
                    if(currentSpeed > 100) currentSpeed = 100;
                    //if(currentSpeed < 0) currentSpeed = 0;
                    
                    // 更新油门显示
                    Update_Throttle_Display(currentSpeed);
                }
                
                // 检查是否在转向控制区域内
                if(!steeringFound && 
                   tp_dev.x[i] >= steeringArea.x && 
                   tp_dev.x[i] <= steeringArea.x + steeringArea.width &&
                   tp_dev.y[i] >= steeringArea.y && 
                   tp_dev.y[i] <= steeringArea.y + steeringArea.height) {
                    
                    steeringFound = 1;
                    steeringActive = 1;
                    
                    // 计算转向值 (左侧为0度，右侧为180度)
                    uint16_t relativeX = tp_dev.x[i] - steeringArea.x;
                    currentSteering = relativeX * 180 / steeringArea.width;
                    
                    // 限制范围
                    if(currentSteering > 180) currentSteering = 180;
                    //if(currentSteering < 0) currentSteering = 0;
                    
                    // 更新方向盘显示
                    Update_Steering_Display(currentSteering);
                }
            }
        }
    } else {
        // 无触摸时，如果之前有激活的控制，则逐渐恢复中性位置
        if(throttleActive) {
            throttleActive = 0;
            // 可以实现渐变回中性位置的代码
            currentSpeed = 0;
            Update_Throttle_Display(currentSpeed);
        }
        
        if(steeringActive) {
            steeringActive = 0;
            // 方向盘回中
            currentSteering = 90;
            Update_Steering_Display(currentSteering);
        }
    }
    
    // 记录本次触点数量
    lastActiveTouchPoints = touchPoints;
}

// 主控制函数
void Nrf24_Control(void){
    Wireless_Init();


    if(wireless_init_flag){

        
        // 清空屏幕并显示控制界面
        lcd_fill(0, 0, lcddev.width, lcddev.height/2-1, WHITE);
        lcd_show_string(130, 20, 200, 24, 24, "CAR CONTROL MODE", RED);
		
		if(NRF24L01_Check()==0){
			lcd_show_string(170, 100, 160, 16, 16, "check ok", BLACK);
		}else{
			lcd_show_num(170, 100, NRF24L01_Check(), 16, 16, RED);
		}        
		

		// 设置为发送模式
        Set_Nrf24_Tx();
        // 初始化控制区域
        throttleArea.x = THROTTLE_AREA_X;
        throttleArea.y = THROTTLE_AREA_Y;
        throttleArea.width = THROTTLE_AREA_WIDTH;
        throttleArea.height = THROTTLE_AREA_HEIGHT;
        throttleArea.centerX = throttleArea.x + throttleArea.width/2;
        throttleArea.centerY = throttleArea.y + throttleArea.height/2;
        
        steeringArea.x = STEERING_AREA_X;
        steeringArea.y = STEERING_AREA_Y;
        steeringArea.width = STEERING_AREA_WIDTH;
        steeringArea.height = STEERING_AREA_HEIGHT;
        steeringArea.centerX = steeringArea.x + steeringArea.width/2;
        steeringArea.centerY = steeringArea.y + steeringArea.height/2;
        
        // 绘制控制界面
        Draw_Control_Areas();
        
        // 初始化显示
        Update_Throttle_Display(currentSpeed);
        Update_Steering_Display(currentSteering);
        Update_Function_Display();
        
        // 发送初始停止命令
        Send_Remote_Command();
		

    }
    
    uint32_t lastUpdateTime = 0;
    
    while(1){
        // 处理触摸输入
        Process_Touch_Input();
        
        // 限制发送频率，避免RF模块过载
        uint32_t currentTime = HAL_GetTick();
        if(currentTime - lastUpdateTime >= 50) { // 每50ms发送一次
            Send_Remote_Command();
            lastUpdateTime = currentTime;
        }
        
        // 检测按键输入
        int key = detect_key_press();
        if(key == 'n') { // 退出
            // 发送停止命令
            currentSpeed = 0;
            currentSteering = 90;
            Send_Remote_Command();
            current_state = MENU_WIRELESS;
            return;
        }
		
		if (key == '1') {
			currentFunction ^= FUNC_LIGHT;// 灯光按钮
			Update_Function_Display();
			delay_ms(200); // 防抖
        }
		if (key == '2') {
			currentFunction ^= FUNC_HORN;// 喇叭按钮
			Update_Function_Display();
			delay_ms(200); // 防抖
        }
		if (key == '3') {
			currentFunction ^= FUNC_DEMO;// 演示按钮
			Update_Function_Display();
			delay_ms(200); // 防抖
        }
		if (key == '4') {
			currentFunction ^= FUNC_TURBO;// 涡轮按钮
			Update_Function_Display();
			delay_ms(200); // 防抖
        }
		
        // 短暂延时释放CPU
        delay_ms(10);
    }    
}
