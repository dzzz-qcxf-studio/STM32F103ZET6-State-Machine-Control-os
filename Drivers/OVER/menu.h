/* menu.h */
#ifndef __MENU_H
#define __MENU_H

#include "stm32f1xx_hal.h"

// 菜单状态枚举
typedef enum {
    MENU_MAIN,
    MENU_SETTINGS,
    MENU_SET_TIME,
    MENU_SET_ALARM,
    MENU_EXIT,
    MENU_Temp,
    MENU_PWM,
    MENU_SERVO,
    MENU_TRACE,
	MENU_RELAY,
	MENU_VDRAW,
	MENU_WIRELESS,
	MENU_HAND,
    MENU_MPU_PID,
    MENU_JOYSTICK
} MenuState;

// 菜单事件枚举
typedef enum {
    EVT_UP,
    EVT_DOWN,
    EVT_OK,
    EVT_BACK,
    EVT_NONE,
	EVT_S//特定自定义事件
} MenuEvent;

// 菜单项结构体
typedef struct {
    const char* text;
    MenuState next_state;
    void (*action)(void);
} MenuItem;

// 屏幕尺寸设置
extern volatile int SCREEN_HEIGHT;       // 屏幕垂直分辨率
extern volatile int ROW_HEIGHT;           // 每行文本高度（像素）
extern volatile int TEXT_OFFSET;          // 文本起始横向位置

// 菜单区域和键盘区域设置
extern volatile int MENU_AREA_HEIGHT;  // 菜单区高度(上半区)
extern volatile int KEYBOARD_AREA_Y;  // 键盘区起始Y坐标

// 菜单行显示设置
extern volatile int MENU_VISIBLE_ROWS;  // 400/32=12行

// 键盘按键尺寸设置
extern volatile int KEY_SIZE_X;     // 按键宽度(480/4=120)
extern volatile int KEY_SIZE_Y;        // 按键高度(400/5=80)
extern volatile int KEY_GAP;         // 最小间隙

extern volatile MenuState current_state;

// 事件处理函数声明
void handle_event(MenuEvent evt);
MenuEvent get_next_event(void);
void update_display(void);
void post_event(MenuEvent evt);

int handle_keyboard_touch(uint16_t x, uint16_t y);
int detect_key_press(void);
void init_keyboard_layout(void);
void draw_keyboard(void);
void Key_Scan(void);

#endif
