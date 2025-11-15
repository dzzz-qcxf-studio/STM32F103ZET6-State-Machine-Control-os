/**
 ****************************************************************************************************
 * @file        main.c
 * @author      jzd	责任人23011
 * @version     V1.6
 * @date        2025-04-01
 * @brief       研发部产权原创 M144Z-M3机小系统：STM32F103版状态机控制系统
 ****************************************************************************************************
 * @attention
 * 
 * 该状态机系统提供了多级菜单，并且实现多功能各自一套的管理接口。
 * 该状态机系统已经实现了模块化功能，提供了	
 * 1.触控事件处理函数
 * 2.显示更新函数
 * 3.键盘绘制函数
 * 4.按键扫描函数
 * 5.菜单事件处理函数
 * 见：menu.c/menu.h
 * 
 * 功能更新
 * 1.添加menu_draw菜单实现波形pwm占空比、频率测量和波形的功能（2025.4.1）
 * 2.移植点图片显示
 * 3.添加2.4g模块的调试程序，发送端测试模块型号为nrf24l01，而终端测试模块为NF-02-PA
 * 在实验中使用该方案NF-02-PA的方案可以接收nrf24l01的数据，并可以发送出数据，XD我的简称谁的拖XD。
 * 4.增强了2.4测试程序功能，实现简单遥控程序的功能
 * 5.添加MPU6050传感器与PID控制L9110电机的功能，实现角度波形显示
 * 6.添加了对于摇杆类型模块的支持
 * 
 * 
 * 注意：
 * 1.对于字符显示的长度限制： update_display 中 char display_text[40] 大小
 * 是显示限制字符长度大小，但是真实的lcd可能超过显示文本大小
 * 2.请不要忘记在添加新菜单情况时在菜单映射中同步添加对应的菜单映射，否则会出错误，使用
 * 时候也要更新的菜单
 * 3.如果新增menu.c相关键盘触摸部分，键盘添加时需要注意同步更新对应的键盘键
 * 外部分的定义
 * 
 * 一些问题：
 * 1.当前的结构注定了无法实现控制和后台运行的功能
 * 2.缺少一些代码提示
 * 3.在不同项目编译器工具中都发现，如果提供的模板示例中不通过对标准库的列表写代码时系统会输出乱码情况
 ****************************************************************************************************
 */
 
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./OVER/servo.h"
#include "./OVER/menu.h"
#include "./OVER/relay.h"
#include "./OVER/hand_control.h"
#include "./OVER/vdraw.h"
#include "./OVER/picall.h"
#include "./OVER/wireless.h"
#include "./OVER/mpu_pid.h"
#include "./BSP/NRF24L01/nrf24l01.h"
#include "./BSP/JOYSTICK/Joystick.h"

// 菜单定义
MenuItem menu_main[] = {
	// 显示名称    菜单标识符   初始化函数
    {"Main menu", MENU_MAIN, NULL},
    {"Pwm spare", MENU_PWM, NULL},
    {"Servo Control", MENU_SERVO, NULL},
    {"Trace", MENU_TRACE, NULL},
    {"Settings", MENU_SETTINGS, picall},
    {"Hand Control", MENU_HAND, NULL},
    {"Relay Control", MENU_RELAY, NULL},
	{"PWM DRAW", MENU_VDRAW, NULL},	
	{"2.4G Control", MENU_WIRELESS, NULL},
    {"MPU PID Control", MENU_MPU_PID, NULL},
    {"Joystick Status", MENU_JOYSTICK, NULL},
    {"Temp2", MENU_Temp, NULL},	
    {NULL, MENU_MAIN, NULL}  // 结束标志
};

MenuItem menu_hand[] = {
    {"Hand_Control", MENU_HAND, Hand_Control},
	{"Back", MENU_EXIT, NULL},
    {NULL, MENU_HAND, NULL}
};

MenuItem menu_settings[] = {
    {"Relay_Control", MENU_SETTINGS, Relay_Init},
    {"Set Time", MENU_SET_TIME, NULL},
    {"Set Alarm", MENU_SET_ALARM, NULL},
    {"Back", MENU_EXIT, NULL},
    {NULL, MENU_SETTINGS, NULL}
};

MenuItem menu_servo[] = {
    {"motor control run!", MENU_SERVO, Servo_control},
    {"BACK to stop", MENU_MAIN, NULL},
    {NULL, MENU_SERVO, NULL}
};

MenuItem menu_pwm[] = {
    {"PWM 1000Hz run!", MENU_PWM, NULL},
    {"BACK to stop", MENU_MAIN, NULL},
    {NULL, MENU_PWM, NULL}
};

MenuItem menu_trace[] = {
    {"TRACE RUN!", MENU_TRACE, NULL},
    {"BACK to stop", MENU_MAIN, NULL},
    {NULL, MENU_TRACE, NULL}
};

MenuItem menu_relay[] = {
    {"Relay Control!", MENU_RELAY, Relay_Control_Interface},
    {"BACK to stop", MENU_MAIN, NULL},
    {NULL, MENU_RELAY, NULL}
};

MenuItem menu_draw[] = {
    {"PWM DRAW!", MENU_VDRAW, PWM_Analyzer_Run},
    {"BACK to stop", MENU_MAIN, NULL},
    {NULL, MENU_VDRAW, NULL}
};

MenuItem menu_wireless[] = {
    {"2.4G go!", MENU_WIRELESS, Nrf24_Control},
    {"BACK to stop", MENU_MAIN, NULL},
    {NULL, MENU_WIRELESS, NULL}
};

MenuItem menu_mpu_pid[] = {
    {"MPU PID Control!", MENU_MPU_PID, MPU_PID_Control_Interface},
    {"BACK to stop", MENU_MAIN, NULL},
    {NULL, MENU_MPU_PID, NULL}
};

MenuItem menu_joystick[] = {
    {"Joystick Status", MENU_JOYSTICK, Joystick_Display},
    {"BACK", MENU_MAIN, NULL},
    {NULL, MENU_JOYSTICK, NULL}
};

// 菜单映射表
volatile const MenuItem* menu_map[] = {
    [MENU_MAIN] = menu_main,
    [MENU_SETTINGS] = menu_settings,
    [MENU_PWM] = menu_pwm,
    [MENU_SERVO] = menu_servo,
    [MENU_TRACE] = menu_trace,
	[MENU_RELAY] = menu_relay,
	[MENU_HAND] = menu_hand,
	[MENU_VDRAW] = menu_draw,
	[MENU_WIRELESS] = menu_wireless,
    [MENU_MPU_PID] = menu_mpu_pid,
    [MENU_JOYSTICK] = menu_joystick
};


//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------
// 主函数
int main(void) {
    HAL_Init();                         /* 初始化HAL库 */
    sys_stm32_clock_init(RCC_PLL_MUL9); /* 设置时钟，72MHz */
    delay_init(72);                     /* 初始化延时 */
    usart_init(115200);                 /* 初始化串口 */
    led_init();                         /* 初始化LED */
    key_init();                         /* 初始化按键 */
    lcd_init();                         /* 初始化LCD */
    tp_dev.init();                      /* 初始化触摸屏 */
    Joystick_Init();                    /* 初始化摇杆 */
    init_keyboard_layout();				/* 初始化键盘布局 */
    update_display();					/* 主界面初始化 */

 	
    while (1) {
        Key_Scan();

        MenuEvent evt = get_next_event();
        handle_event(evt);

        HAL_Delay(10);
    }
}
//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------



