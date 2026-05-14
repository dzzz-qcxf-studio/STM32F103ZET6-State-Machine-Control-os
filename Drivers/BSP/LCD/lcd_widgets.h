/**
 * @file    lcd_widgets.h
 * @brief   LCD 组件库 — 圆角矩形、卡片、按钮、标题栏、滚动条
 *
 * 提供可复用的 UI 组件，统一绘制风格。
 * 所有坐标均为绝对屏幕坐标。
 */

#ifndef __LCD_WIDGETS_H
#define __LCD_WIDGETS_H

#include "stm32f1xx_hal.h"

/* ======================== 配色方案 (RGB565) ======================== */

/* 页面级 */
#define WGT_CLR_BG          0xF7BE      /* 主背景 #F0F0F0 */
#define WGT_CLR_HEADER_BG   0x2B6A      /* 标题栏 #2B3A4A */

/* 卡片 */
#define WGT_CLR_CARD_BG     0xFFFF      /* 卡片白 */
#define WGT_CLR_CARD_SEL    0x1E5F      /* 选中蓝 #1A73E8 */
#define WGT_CLR_CARD_BRD    0xDEDB      /* 卡片边框 #D8D8D8 */

/* 文字 */
#define WGT_CLR_TEXT_PRI    0x2124      /* 主文字 #212121 */
#define WGT_CLR_TEXT_SEC    0x7BEF      /* 次文字 #757575 */
#define WGT_CLR_TEXT_WHT    0xFFFF      /* 白色文字 */

/* 键盘 */
#define WGT_CLR_KBD_BG      0x4208      /* 键盘背景 #404850 */
#define WGT_CLR_KEY_BG      0x5B10      /* 按键底色 #586068 */
#define WGT_CLR_KEY_ACT     0x1E5F      /* 功能键蓝 */
#define WGT_CLR_KEY_NEG     0xC924      /* 功能键红 #C0392B */
#define WGT_CLR_KEY_BRD     0x6B4D      /* 按键边框 #687078 */

/* 通用 */
#define WGT_CLR_ACCENT      0x1E5F      /* 强调色蓝 */
#define WGT_CLR_SCROLL_TRK  0xDEDB      /* 滚动条轨道 */
#define WGT_CLR_SCROLL_THM  0x7BEF      /* 滚动条滑块 */

/* ======================== 布局默认值 ======================== */

#define WGT_HEADER_H        44
#define WGT_CARD_H          36
#define WGT_CARD_GAP        4
#define WGT_CARD_RADIUS     6
#define WGT_CARD_MARGIN     8
#define WGT_CARD_BAR_W      4
#define WGT_KEY_RADIUS      8

/* ======================== 绘图基元 ======================== */

/**
 * @brief  填充圆角矩形
 * @param  r: 圆角半径 (0 = 普通矩形)
 */
void lcd_draw_rounded_rect(uint16_t x1, uint16_t y1,
                           uint16_t x2, uint16_t y2,
                           uint16_t r, uint16_t color);

/**
 * @brief  圆角矩形边框 (仅边框，不填充)
 */
void lcd_draw_rounded_rect_border(uint16_t x1, uint16_t y1,
                                  uint16_t x2, uint16_t y2,
                                  uint16_t r, uint16_t color);

/**
 * @brief  小三角箭头 (▸)
 * @param  cx,cy: 中心点  sz: 半边长
 */
void lcd_draw_arrow(uint16_t cx, uint16_t cy, uint16_t sz, uint16_t color);

/* ======================== 标题栏组件 ======================== */

/**
 * @brief  绘制标题栏 (深色背景 + 标题 + 右侧计数)
 * @param  title: 标题文本
 * @param  current: 当前序号 (从1开始, 0=不显示)
 * @param  total: 总数 (0=不显示)
 */
void lcd_wgt_header(const char *title, uint8_t current, uint8_t total);

/* ======================== 卡片组件 ======================== */

/**
 * @brief  绘制菜单卡片
 * @param  x1,y1,x2,y2: 卡片区域
 * @param  r: 圆角半径
 * @param  index_num: 序号 (显示在左侧, 0=不显示)
 * @param  text: 主文本
 * @param  selected: 是否选中
 */
void lcd_wgt_card(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                  uint16_t r, uint8_t index_num, const char *text,
                  uint8_t selected);

/**
 * @brief  绘制带图标的卡片 (预留图标区域)
 */
void lcd_wgt_card_icon(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                       uint16_t r, const char *icon, const char *text,
                       uint8_t selected);

/* ======================== 按钮组件 ======================== */

/**
 * @brief  绘制按钮
 * @param  r: 圆角半径
 * @param  label: 按钮文字
 * @param  style: 0=普通, 1=主要(蓝), 2=危险(红)
 */
void lcd_wgt_button(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                    uint16_t r, const char *label, uint8_t style);

/**
 * @brief  按键按下效果 (高亮闪烁)
 */
void lcd_wgt_button_press(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                          uint16_t r, const char *label, uint8_t style);

/* ======================== 滚动条组件 ======================== */

/**
 * @brief  绘制滚动条
 * @param  x: 滚动条 x 坐标
 * @param  y_start: 起始 y
 * @param  track_len: 轨道总长度
 * @param  visible_count: 可见条目数
 * @param  total_count: 总条目数
 * @param  scroll_pos: 当前滚动位置
 */
void lcd_wgt_scrollbar(uint16_t x, uint16_t y_start, uint16_t track_len,
                       uint8_t visible_count, uint8_t total_count,
                       uint8_t scroll_pos);

/* ======================== 键盘组件 ======================== */

/**
 * @brief  绘制键盘区域背景 + 标题
 * @param  y_start: 键盘区域起始 y
 */
void lcd_wgt_keyboard_bg(uint16_t y_start);

/**
 * @brief  绘制单个键盘按键
 * @param  x1,y1,x2,y2: 按键区域
 * @param  r: 圆角半径
 * @param  ch: 按键字符
 */
void lcd_wgt_key(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                 uint16_t r, char ch);

/**
 * @brief  按键按下反馈
 */
void lcd_wgt_key_press(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                       uint16_t r, char ch);

/* ======================== 信息标签 ======================== */

/**
 * @brief  绘制状态标签 (小圆点 + 文本)
 * @param  active: 圆点颜色 (0=灰色, 1=绿色, 2=红色)
 */
void lcd_wgt_status_label(uint16_t x, uint16_t y, const char *text, uint8_t active);

#endif /* __LCD_WIDGETS_H */
