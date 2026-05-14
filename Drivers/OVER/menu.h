/**
 * @file    menu.h
 * @brief   状态机菜单系统 — 事件驱动的多级菜单框架
 *
 * 菜单项通过 ModuleInterface 指针关联功能模块。
 * 主循环：Key_Scan → handle_event → module_run_tick
 */

#ifndef __MENU_H
#define __MENU_H

#include "stm32f1xx_hal.h"
#include "./OVER/module.h"

/* ======================== 菜单状态枚举 ======================== */
typedef enum {
    MENU_MAIN = 0,
    MENU_SERVO,
    MENU_RELAY,
    MENU_VDRAW,
    MENU_WIRELESS,
    MENU_HAND,
    MENU_MPU_PID,
    MENU_JOYSTICK,
    MENU_USART,
    MENU_PICALL,
    MENU_OSCOPE,
    MENU_BUS_SERVO,
    MENU_STATE_COUNT  /* 状态总数，放在最后 */
} MenuState;

/* ======================== 菜单事件枚举 ======================== */
typedef enum {
    EVT_NONE = 0,
    EVT_UP,
    EVT_DOWN,
    EVT_OK,
    EVT_BACK,
} MenuEvent;

/* ======================== 菜单项结构体 ======================== */
typedef struct {
    const char *text;               /* 显示文本 */
    MenuState next_state;           /* 选中后跳转的目标状态 */
    const ModuleInterface *module;  /* 关联的功能模块（NULL 表示纯菜单项） */
} MenuItem;

/* ======================== 屏幕布局常量 ======================== */
#define SCREEN_WIDTH       480
#define SCREEN_HEIGHT      800
#define MENU_AREA_HEIGHT   400
#define KEYBOARD_AREA_Y    400
#define ROW_HEIGHT         32
#define TEXT_OFFSET         16
#define MENU_VISIBLE_ROWS  12

/* 键盘按键尺寸 */
#define KEY_SIZE_X   120
#define KEY_SIZE_Y    80
#define KEY_GAP         2

/* ======================== 菜单注册接口 ======================== */

/**
 * @brief  注册一个菜单页
 * @param  state: 菜单状态
 * @param  items: 菜单项数组（以 {NULL, 0, NULL} 结尾）
 */
void menu_register(MenuState state, const MenuItem *items);

/**
 * @brief  注册所有菜单页（由 main.c 调用）
 */
void menu_init_all(void);

/* ======================== 事件系统 ======================== */
void post_event(MenuEvent evt);
MenuEvent get_next_event(void);
void handle_event(MenuEvent evt);

/* ======================== 显示更新 ======================== */
void update_display(void);
void update_display_force_kbd(void);

/* ======================== 输入扫描 ======================== */
void Key_Scan(void);
int detect_key_press(void);

/* ======================== 键盘初始化 ======================== */
void init_keyboard_layout(void);

/* ======================== 当前状态（只读） ======================== */
extern volatile MenuState current_state;

#endif /* __MENU_H */
