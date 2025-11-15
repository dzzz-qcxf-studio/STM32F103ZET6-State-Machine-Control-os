#include "./OVER/menu.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/TOUCH/touch.h"
#include "./BSP/JOYSTICK/Joystick.h"
#include <stdio.h>

// 屏幕参数定义
#define MAX_EVENTS 10            	  // 事件队列最大长度
volatile int SCREEN_HEIGHT=800;       // 屏幕垂直像素数
volatile int ROW_HEIGHT=32;           // 每行文本高度（像素）
volatile int TEXT_OFFSET=16;          // 文本起始像素位置
volatile uint8_t pressed=0;  // 标记当前是否处于按下状态

// 菜单区域和键盘区域参数定义
volatile int MENU_AREA_HEIGHT=400;  // 菜单区高度(上半屏)
volatile int KEYBOARD_AREA_Y=400;  // 键盘区起始Y坐标

// 菜单可显示行数
volatile int MENU_VISIBLE_ROWS=12;  // 400/32=12行

// 键盘按键参数定义
volatile int KEY_SIZE_X=120;     // 按键宽度(480/4=120)
volatile int KEY_SIZE_Y=80;        // 按键高度(400/5=80)
volatile int KEY_GAP=2;         // 缩小间距


// 全局变量声明
extern const MenuItem* menu_map[];
volatile MenuState current_state = MENU_MAIN;
volatile MenuState last_current_state=MENU_MAIN;
uint8_t cursor_pos = 0;
uint8_t start_idx = 0; // 当前显示起始索引
MenuEvent event_queue[MAX_EVENTS];
uint8_t head = 0;
uint8_t tail = 0;


int handle_keyboard_touch(uint16_t x, uint16_t y);
int detect_key_press(void);
void init_keyboard_layout(void);
void draw_keyboard(void);
void Key_Scan(void);


// 按键结构体
typedef struct {
    uint16_t x_start;
    uint16_t x_end;
    uint16_t y_start;
    uint16_t y_end;
    char value;
} Key_TypeDef;

Key_TypeDef keys[5][4];  // 存储所有按键坐标信息

// 键盘映射表
const char keymap[5][4] = {
    {'7', '8', '9', '/'},
    {'4', '5', '6', '*'},
    {'1', '2', '3', '-'},
    {'0', '.', '=', '+'},
    {'C', 'y', 'n', '<'}  // C:清除 <:退格
};

// 更新显示函数
void update_display(void) {
    // 清空菜单区域
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WHITE);

    const MenuItem* items = menu_map[current_state];
    uint8_t item_count = 0;
    // 计算菜单项数量
    while (items[item_count].text != NULL) item_count++;

    // 调整滚动逻辑
    if (item_count > MENU_VISIBLE_ROWS) {
        if (cursor_pos >= start_idx + MENU_VISIBLE_ROWS) {
            start_idx = cursor_pos - MENU_VISIBLE_ROWS + 1;
        } else if (cursor_pos < start_idx) {
            start_idx = cursor_pos;
        }
        if (start_idx > item_count - MENU_VISIBLE_ROWS) {
            start_idx = item_count - MENU_VISIBLE_ROWS;
        }
    } else {
        start_idx = 0;
    }

    // 绘制菜单项
    for (uint8_t i = 0; i < MENU_VISIBLE_ROWS; i++) {
        uint8_t item_idx = start_idx + i;
        if (item_idx >= item_count) break;

        uint16_t y_pos = i * ROW_HEIGHT;

        // 光标指示
        lcd_show_string(0, y_pos, 8, ROW_HEIGHT, 32,
                        (item_idx == cursor_pos) ? ">" : " ", RED);

        // 文本显示
        char display_text[40];
        snprintf(display_text, sizeof(display_text), "%-15s", items[item_idx].text);
        lcd_show_string(TEXT_OFFSET, y_pos, 480, ROW_HEIGHT, 32, display_text, BLACK);
    }

    // 绘制键盘
    draw_keyboard();
}

// 辅助函数：仅刷新光标显示区域
void update_cursor_indicator(uint8_t old_cursor, uint8_t new_cursor, uint8_t base_idx) {
    // 计算相对于当前显示区域的行坐标
    uint16_t old_y = (old_cursor - base_idx) * ROW_HEIGHT;
    uint16_t new_y = (new_cursor - base_idx) * ROW_HEIGHT;
    // 清除原来的光标（显示空格）
    lcd_show_string(0, old_y, 8, ROW_HEIGHT, 32, " ", RED);
    // 绘制新的光标指示符
    lcd_show_string(0, new_y, 8, ROW_HEIGHT, 32, ">", RED);
}


void handle_event(MenuEvent evt) {
    if (evt == EVT_NONE) return;
	last_current_state=current_state;

    static uint8_t prev_start_idx = 0;

    const MenuItem* items = menu_map[current_state];
    uint8_t item_count = 0;
    while (items[item_count].text != NULL) item_count++;

    // 保存处理前的光标位置
    uint8_t old_cursor = cursor_pos;

    // 根据事件更新状态
    switch (evt) {
        case EVT_UP:
            if (cursor_pos > 0)
                cursor_pos--;
            else { // 上移到首位时换到最后一项
                while (items[cursor_pos + 1].text != NULL)
                    cursor_pos++;
            }
            break;
        case EVT_DOWN:
            if (items[cursor_pos + 1].text != NULL)
                cursor_pos++;
            else
                cursor_pos = 0;
            break;
        case EVT_OK:
            if (items[cursor_pos].action != NULL) {
                items[cursor_pos].action();
            }
            if (items[cursor_pos].next_state != current_state) {
                current_state = items[cursor_pos].next_state;
                cursor_pos = 0;
            }
            break;
        case EVT_BACK:
            if (current_state != MENU_MAIN) {
				current_state=MENU_MAIN;
            }
            cursor_pos = 0;
            break;
        default:
            break;
    }

    // 根据新的光标位置重新计算菜单起始索引（start_idx）
    uint8_t new_start_idx = 0;
    if (item_count > MENU_VISIBLE_ROWS) {
        if (cursor_pos >= start_idx + MENU_VISIBLE_ROWS)
            new_start_idx = cursor_pos - MENU_VISIBLE_ROWS + 1;
        else if (cursor_pos < start_idx)
            new_start_idx = cursor_pos;
        else
            new_start_idx = start_idx;  // 无需滚动
        if (new_start_idx > item_count - MENU_VISIBLE_ROWS)
            new_start_idx = item_count - MENU_VISIBLE_ROWS;
    } else {
        new_start_idx = 0;
    }
    start_idx = new_start_idx;  // 更新全局显示起始索引

    // 判断是否只需更新光标列
    // 当事件为上下移动且没有发生菜单滚动时，直接只刷新光标列
    if ((evt == EVT_UP || evt == EVT_DOWN) && (new_start_idx == prev_start_idx)) {
        update_cursor_indicator(old_cursor, cursor_pos, new_start_idx);
    } else {
        // 其它情况（例如菜单切换或滚动）全屏刷新菜单区域
        update_display();
    }
}


// 发布菜单事件函数
void post_event(MenuEvent evt) {
    if ((tail + 1) % MAX_EVENTS != head) {
        event_queue[tail] = evt;
        tail = (tail + 1) % MAX_EVENTS;
    }
}

// 获取下一个菜单事件函数
MenuEvent get_next_event(void) {
    MenuEvent evt = EVT_NONE;
    if (head != tail) {
        evt = event_queue[head];
        head = (head + 1) % MAX_EVENTS;
    }
    return evt;
}

// 处理键盘触摸事件函数
int handle_keyboard_touch(uint16_t x, uint16_t y) {
    for (uint8_t row = 0; row < 5; row++) {
        for (uint8_t col = 0; col < 4; col++) {
            Key_TypeDef *key = &keys[row][col];
            if (x >= key->x_start && x <= key->x_end &&
                y >= key->y_start && y <= key->y_end) {

                // 按下效果
                lcd_fill(key->x_start + 2, KEYBOARD_AREA_Y + key->y_start + 2,
                         key->x_end - 2, KEYBOARD_AREA_Y + key->y_end - 2, GRAY);

                HAL_Delay(30);

                // 恢复颜色
                lcd_fill(key->x_start + 2, KEYBOARD_AREA_Y + key->y_start + 2,
                         key->x_end - 2, KEYBOARD_AREA_Y + key->y_end - 2, WHITE);
				
				// 计算文字居中位置
				uint16_t text_x = keys[row][col].x_start + (KEY_SIZE_X - 32) / 2;
				uint16_t text_y = KEYBOARD_AREA_Y + keys[row][col].y_start + (KEY_SIZE_Y - 32) / 2;

				// 显示按键字符
				char str[2] = {keymap[row][col], '\0'};
				lcd_show_string(text_x + 2, text_y , 32, 32, 32, str, RED); // 微调使显示内容正中与方框
                return key->value;
            }
        }
    }
    return EVT_NONE;
}

int detect_key_press(void) {
    static uint32_t last_press = 0;
    tp_dev.scan(0);

    // 如果触摸屏检测到按下
    if (tp_dev.sta & TP_PRES_DOWN) {
        // 仅当之前未处于按下状态且满足最小间隔时，注册一次点击
        if (!pressed && (HAL_GetTick() - last_press >= 200)) {
            pressed = 1;               // 标记按下状态
            last_press = HAL_GetTick();
            uint16_t touch_y = tp_dev.y[0];
            return handle_keyboard_touch(tp_dev.x[0], touch_y - KEYBOARD_AREA_Y);
        }
    } else {
        // 当检测不到按下时，重置按下状态
        pressed = 0;
    }
    return EVT_NONE;
}


// 初始化键盘布局函数
void init_keyboard_layout(void) {
    for (uint8_t row = 0; row < 5; row++) {
        for (uint8_t col = 0; col < 4; col++) {
            keys[row][col].x_start = col * (KEY_SIZE_X + KEY_GAP);
            keys[row][col].x_end = keys[row][col].x_start + KEY_SIZE_X;
            keys[row][col].y_start = row * (KEY_SIZE_Y + KEY_GAP);  // 相对键盘区域坐标
            keys[row][col].y_end = keys[row][col].y_start + KEY_SIZE_Y;
            keys[row][col].value = keymap[row][col];
        }
    }
}

// 绘制键盘界面函数
void draw_keyboard(void) {
    for (uint8_t row = 0; row < 5; row++) {
        for (uint8_t col = 0; col < 4; col++) {

            // 计算文字居中位置
            uint16_t text_x = keys[row][col].x_start + (KEY_SIZE_X - 32) / 2;
            uint16_t text_y = KEYBOARD_AREA_Y + keys[row][col].y_start + (KEY_SIZE_Y - 32) / 2;

            // 显示按键字符
            char str[2] = {keymap[row][col], '\0'};
            lcd_show_string(text_x + 2, text_y , 32, 32, 32, str, RED); // 微调使显示内容正中与方框
        }
    }
}

// 按键扫描函数
void Key_Scan(void) {
    static uint8_t last_key = 0;
    int current_key = detect_key_press();  // 返回值类型改为int

    if (current_key != EVT_NONE && current_key != last_key) {
        switch (current_key) {
            case '8': post_event(EVT_UP); break;  // 假设'8'为向上键
            case '2': post_event(EVT_DOWN); break;  // 假设'2'为向下键
            case 'n': post_event(EVT_BACK); break;  // 假设'n'为返回键
            case 'y': post_event(EVT_OK); break;  // 假设'y'为确认键
            default: break;
        }
    }
    last_key = current_key;
}
