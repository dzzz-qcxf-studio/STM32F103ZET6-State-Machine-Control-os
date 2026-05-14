/**
 * @file    menu.c
 * @brief   状态机菜单系统实现 — 基于 lcd_widgets 组件库
 *
 * 设计语言：卡片式菜单 · 圆角按键 · 柔和配色
 */

#include "./OVER/menu.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/lcd_widgets.h"
#include "./BSP/TOUCH/touch.h"
#include <stdio.h>
#include <string.h>

/* ======================== 布局常量 ======================== */
/* 菜单卡片区域的起始Y坐标（屏幕顶部留出标题栏空间） */
#define CARD_Y_START    40
/* 屏幕上同时可见的最大卡片数量，超出部分通过滚动查看 */
#define VISIBLE_CARDS   9
/* 虚拟键盘每个按键的高度（像素） */
#define KBD_KEY_H       68
/* 虚拟键盘每个按键的宽度（像素） */
#define KBD_KEY_W       116
/* 虚拟键盘按键之间的间距（像素） */
#define KBD_GAP         3

/* ======================== 事件队列（环形缓冲区） ======================== */
/*
 * 事件队列采用经典的环形缓冲区（Circular Buffer）设计：
 *   - event_queue[]: 固定大小的事件数组，存储待处理的菜单事件
 *   - evt_head: 队列头指针，指向下一个待读取的位置（消费者）
 *   - evt_tail: 队列尾指针，指向下一个可写入的位置（生产者）
 * 判空条件：head == tail  （头尾重合表示无待处理事件）
 * 判满条件：(tail + 1) % MAX == head  （尾指针的下一个位置是头，预留一个空位防止歧义）
 */
#define MAX_EVENTS 10

static MenuEvent event_queue[MAX_EVENTS]; /* 事件缓冲区，存储最多 MAX_EVENTS 个事件 */
static uint8_t evt_head = 0;  /* 读指针：get_next_event() 从这里取事件 */
static uint8_t evt_tail = 0;  /* 写指针：post_event() 往这里存事件 */

/**
 * @brief  向事件队列投递一个事件（生产者端）
 * @param  evt  要投递的事件类型
 * @note   当队列已满时（tail+1 追上 head），事件会被静默丢弃，不会阻塞
 */
void post_event(MenuEvent evt)
{
    /* 检查队列是否已满：如果尾指针的下一个位置等于头指针，说明满了 */
    if ((evt_tail + 1) % MAX_EVENTS != evt_head) {
        event_queue[evt_tail] = evt;                           /* 将事件写入尾部 */
        evt_tail = (evt_tail + 1) % MAX_EVENTS;               /* 尾指针前进，取模实现环形 */
    }
}

/**
 * @brief  从事件队列取出下一个事件（消费者端）
 * @return 队列中的下一个事件，队列为空时返回 EVT_NONE
 */
MenuEvent get_next_event(void)
{
    MenuEvent evt = EVT_NONE;
    /* 队列非空时（头尾不重合），取出头部事件并推进头指针 */
    if (evt_head != evt_tail) {
        evt = event_queue[evt_head];                           /* 读取头部事件 */
        evt_head = (evt_head + 1) % MAX_EVENTS;               /* 头指针前进，取模实现环形 */
    }
    return evt;
}

/* ======================== 菜单注册表 ======================== */
/*
 * g_menu_table[] 是一个全局查找表，将 MenuState 枚举值映射到对应的菜单项数组。
 * 每个 MenuState（如 MENU_MAIN、MENU_SERVO 等）对应一个 MenuItem 数组，
 * 数组以 text == NULL 的元素作为结束哨兵。
 * 通过 menu_register() 在系统初始化时注册各子菜单的内容。
 */
static const MenuItem *g_menu_table[MENU_STATE_COUNT];

/**
 * @brief  注册某个菜单状态对应的菜单项数组
 * @param  state  菜单状态枚举值（作为数组索引）
 * @param  items  指向该菜单状态的 MenuItem 数组（以 NULL text 结尾）
 * @note   越界检查防止数组越界写入
 */
void menu_register(MenuState state, const MenuItem *items)
{
    if (state < MENU_STATE_COUNT && items != NULL) {
        g_menu_table[state] = items;
    }
}

/* ======================== 菜单状态变量 ======================== */
/* current_state: 当前所在的菜单层级（主菜单/舵机/继电器等），volatile 因可能在中断中被修改 */
volatile MenuState current_state = MENU_MAIN;
/* cursor_pos: 当前光标（选中项）在完整菜单项列表中的绝对索引 */
static uint8_t cursor_pos = 0;
/* start_idx: 当前可视区域第一条菜单项的索引，用于实现滚动显示 */
static uint8_t start_idx = 0;

/* ======================== 虚拟键盘 ======================== */
/* key_pressed: 按键状态标志，1 表示当前有键被按下，0 表示已释放，用于去抖判断 */
static uint8_t key_pressed = 0;

/**
 * @brief  按键区域定义结构体
 *         每个按键记录其在屏幕上的矩形区域（像素坐标）和对应的字符值
 */
typedef struct {
    uint16_t x_start, x_end;  /* 按键左/右边界 X 坐标 */
    uint16_t y_start, y_end;  /* 按键上/下边界 Y 坐标（相对于键盘区域顶部的偏移） */
    char value;               /* 按键对应的字符值，如 '8'、'y'、'n' 等 */
} KeyTypeDef;

/* keys[5][4]: 5行4列的键盘布局，每个元素存储该按键的屏幕坐标和字符值 */
static KeyTypeDef keys[5][4];

/*
 * keymap[5][4]: 虚拟键盘的逻辑映射表（5行 x 4列）
 * 第1-4行为数字和运算符，第5行为功能键：
 *   'C' = 清除, 'y' = 确认(Yes), 'n' = 返回(No), '<' = 退格
 * 触摸扫描时通过坐标匹配到行列，再查此表得到对应字符
 */
static const char keymap[5][4] = {
    {'7', '8', '9', '/'},
    {'4', '5', '6', '*'},
    {'1', '2', '3', '-'},
    {'0', '.', '=', '+'},
    {'C', 'y', 'n', '<'}
};

/**
 * @brief  初始化键盘布局 — 根据屏幕宽度和按键尺寸计算每个按键的像素坐标
 * @note   键盘水平居中，从屏幕顶部(Y=0)开始排列
 *         计算公式：总宽 = 4个键宽 + 3个间距，偏移 = (屏宽 - 总宽) / 2
 */
void init_keyboard_layout(void)
{
    /* 计算键盘总宽度和水平居中偏移量 */
    uint16_t kbd_total_w = 4 * KBD_KEY_W + 3 * KBD_GAP;
    uint16_t kbd_offset_x = (SCREEN_WIDTH - kbd_total_w) / 2;

    /* 遍历 5x4 网格，为每个按键计算其像素坐标范围并填入字符值 */
    for (uint8_t row = 0; row < 5; row++) {
        for (uint8_t col = 0; col < 4; col++) {
            keys[row][col].x_start = kbd_offset_x + col * (KBD_KEY_W + KBD_GAP);  /* 左边界 */
            keys[row][col].x_end   = keys[row][col].x_start + KBD_KEY_W;          /* 右边界 */
            keys[row][col].y_start = row * (KBD_KEY_H + KBD_GAP);                 /* 上边界 */
            keys[row][col].y_end   = keys[row][col].y_start + KBD_KEY_H;          /* 下边界 */
            keys[row][col].value   = keymap[row][col];                             /* 字符映射 */
        }
    }
}

/* ======================== 键盘绘制 ======================== */

/**
 * @brief  获取键盘区域的 Y 基准偏移量
 * @return 键盘第一个按键的 Y 起始像素坐标（相对于屏幕顶部）
 * @note   KEYBOARD_AREA_Y 是键盘区域的顶部，+22 是为标题文字预留空间
 */
static uint16_t get_key_y_base(void)
{
    return KEYBOARD_AREA_Y + 22;
}

/**
 * @brief  绘制整个虚拟键盘
 * @note   先绘制键盘背景，再绘制标题文字，最后逐行逐列绘制每个按键
 *         按键坐标基于 keys[][] 中预计算的值，加上 Y 基准偏移得到屏幕绝对坐标
 */
static void draw_keyboard(void)
{
    /* 绘制键盘区域的背景矩形 */
    lcd_wgt_keyboard_bg(KEYBOARD_AREA_Y);

    /* 计算键盘水平居中偏移（与 init_keyboard_layout 保持一致） */
    uint16_t kbd_total_w = 4 * KBD_KEY_W + 3 * KBD_GAP;
    uint16_t kbd_offset_x = (SCREEN_WIDTH - kbd_total_w) / 2;
    uint16_t key_y_base = get_key_y_base();

    /* 绘制键盘区域标题 "CONTROLS" */
    lcd_show_string(kbd_offset_x, KEYBOARD_AREA_Y + 4, 200, 14, 12,
                    "CONTROLS", WGT_CLR_KEY_BRD);

    /* 遍历 5x4 网格，将每个按键绘制到屏幕上 */
    for (uint8_t row = 0; row < 5; row++) {
        for (uint8_t col = 0; col < 4; col++) {
            uint16_t kx1 = keys[row][col].x_start;
            uint16_t ky1 = key_y_base + keys[row][col].y_start;  /* 加上 Y 基准偏移 */
            uint16_t kx2 = keys[row][col].x_end;
            uint16_t ky2 = key_y_base + keys[row][col].y_end;
            lcd_wgt_key(kx1, ky1, kx2, ky2, WGT_KEY_RADIUS, keymap[row][col]);
        }
    }
}

/* ======================== 菜单显示 ======================== */

/**
 * @brief  统计菜单项数组中的有效条目数
 * @param  items  指向 MenuItem 数组的指针
 * @return 菜单项数量（不包含结尾的 NULL 哨兵）
 * @note   MenuItem 数组约定以 .text == NULL 的元素作为结束标记（哨兵模式），
 *         通过逐个扫描直到遇到 NULL 文本来计数
 */
static uint8_t get_item_count(const MenuItem *items)
{
    uint8_t count = 0;
    if (items == NULL) return 0;            /* 空指针保护 */
    while (items[count].text != NULL) count++;  /* 遇到 text 为 NULL 的哨兵元素停止 */
    return count;
}

/**
 * @brief  根据当前菜单状态返回对应的标题字符串
 * @return 指向标题字符串常量的指针，用于在屏幕顶部标题栏显示
 * @note   每个 MenuState 枚举值对应一个英文标题，显示在菜单头部
 */
static const char *get_menu_title(void)
{
    switch (current_state) {
    case MENU_MAIN:     return "MAIN MENU";      /* 主菜单 */
    case MENU_SERVO:    return "SERVO";           /* 舵机控制 */
    case MENU_RELAY:    return "RELAY";           /* 继电器控制 */
    case MENU_VDRAW:    return "PWM ANALYZER";    /* PWM 分析仪 */
    case MENU_WIRELESS: return "2.4G REMOTE";     /* 2.4G 遥控 */
    case MENU_HAND:     return "HAND CONTROL";    /* 手动控制 */
    case MENU_MPU_PID:  return "MPU PID";         /* MPU PID 控制 */
    case MENU_JOYSTICK: return "JOYSTICK";        /* 摇杆 */
    case MENU_USART:    return "USART";           /* 串口通信 */
    case MENU_PICALL:   return "PICTURES";        /* 图片显示 */
    case MENU_OSCOPE:    return "OSCILLOSCOPE";   /* 示波器 */
    case MENU_BUS_SERVO: return "BUS SERVO";      /* 总线舵机 */
    default:             return "MENU";           /* 未知状态的默认标题 */
    }
}

/**
 * @brief  绘制单张菜单卡片
 * @param  vis_idx  卡片在可视区域中的位置索引（0 ~ VISIBLE_CARDS-1）
 * @param  is_sel   是否为当前选中项（1=高亮显示，0=普通显示）
 * @note   通过 vis_idx 计算卡片在屏幕上的 Y 坐标，再根据 start_idx + vis_idx
 *         得到绝对菜单项索引，从 g_menu_table 中取出对应菜单项的文本进行绘制
 */
static void draw_card(uint8_t vis_idx, uint8_t is_sel)
{
    /* 根据可视索引计算卡片的屏幕矩形区域 */
    uint16_t y1 = CARD_Y_START + vis_idx * (WGT_CARD_H + WGT_CARD_GAP);
    uint16_t x1 = WGT_CARD_MARGIN;
    uint16_t x2 = SCREEN_WIDTH - WGT_CARD_MARGIN;
    uint16_t y2 = y1 + WGT_CARD_H - 1;
    const MenuItem *items = g_menu_table[current_state];
    uint8_t abs_idx = start_idx + vis_idx;  /* 可视索引 + 滚动偏移 = 绝对索引 */

    /* 调用 lcd_widgets 库绘制卡片，序号从 1 开始显示 */
    lcd_wgt_card(x1, y1, x2, y2, WGT_CARD_RADIUS,
                 (abs_idx + 1), items[abs_idx].text, is_sel);
}

/* kbd_drawn: 键盘绘制标志，0 表示需要全屏重绘（含键盘），1 表示仅刷新菜单区 */
static uint8_t kbd_drawn = 0;

/**
 * @brief  全屏刷新菜单显示
 * @note   包含以下绘制步骤：
 *         1. 清屏（首次全屏清 + 键盘绘制，后续仅清菜单区域以避免闪烁）
 *         2. 绘制顶部标题栏（显示当前菜单名、光标位置/总数）
 *         3. 滚动逻辑计算（确保光标始终在可视区域内）
 *         4. 绘制可视范围内的所有菜单卡片
 *         5. 绘制右侧滚动条指示当前位置
 */
void update_display(void)
{
    const MenuItem *items = g_menu_table[current_state];
    uint8_t item_count = get_item_count(items);

    if (!kbd_drawn) {
        /* 首次绘制或强制重绘键盘时：清除整个屏幕并重绘键盘 */
        lcd_fill(0, 0, lcddev.width, lcddev.height, WGT_CLR_BG);
        draw_keyboard();
        kbd_drawn = 1;
    } else {
        /* 后续刷新：只清除菜单区域（键盘区域保持不变，减少闪烁和开销） */
        lcd_fill(0, 0, lcddev.width, KEYBOARD_AREA_Y - 1, WGT_CLR_BG);
    }

    /* 绘制顶部标题栏：菜单名称 + 当前选中序号/总条目数 */
    lcd_wgt_header(get_menu_title(), cursor_pos + 1, item_count);

    /* 滚动逻辑：当菜单项总数超过可视卡片数时，调整 start_idx 使光标可见 */
    if (item_count > VISIBLE_CARDS) {
        /* 光标超出可视区域下边界：向下滚动，使光标出现在可视区最后一行 */
        if (cursor_pos >= start_idx + VISIBLE_CARDS)
            start_idx = cursor_pos - VISIBLE_CARDS + 1;
        /* 光标超出可视区域上边界：向上滚动，使光标出现在可视区第一行 */
        else if (cursor_pos < start_idx)
            start_idx = cursor_pos;
        /* 边界保护：start_idx 不能超出最大合法值 */
        if (start_idx > item_count - VISIBLE_CARDS)
            start_idx = item_count - VISIBLE_CARDS;
    } else {
        /* 菜单项总数不超过可视数，无需滚动 */
        start_idx = 0;
    }

    /* 绘制当前可视区域内的所有菜单卡片，光标所在项高亮 */
    for (uint8_t i = 0; i < VISIBLE_CARDS; i++) {
        if (start_idx + i >= item_count) break;  /* 超出菜单项总数时停止 */
        draw_card(i, (start_idx + i == cursor_pos));  /* 选中项 is_sel=1 */
    }

    /* 绘制右侧滚动条：指示当前可视区域在完整列表中的位置 */
    uint16_t bar_h = VISIBLE_CARDS * (WGT_CARD_H + WGT_CARD_GAP) - WGT_CARD_GAP;
    lcd_wgt_scrollbar(SCREEN_WIDTH - 5, CARD_Y_START, bar_h,
                      VISIBLE_CARDS, item_count, start_idx);
}

/**
 * @brief  强制全屏重绘（包括键盘区域）
 * @note   将 kbd_drawn 标志清零，下次 update_display() 时会重新绘制键盘
 *         适用于从子模块返回菜单等需要完整重绘的场景
 */
void update_display_force_kbd(void)
{
    kbd_drawn = 0;
    update_display();
}

/**
 * @brief  局部刷新：仅重绘光标移动涉及的两张卡片（优化性能）
 * @param  old_pos  光标移动前的绝对索引
 * @param  new_pos  光标移动后的绝对索引
 * @param  base     当前可视区域的起始索引（start_idx）
 * @note   仅在上下移动且未发生翻页时调用，避免全屏刷新带来的闪烁
 *         old_pos 处取消高亮（is_sel=0），new_pos 处设为高亮（is_sel=1）
 */
static void update_cursor(uint8_t old_pos, uint8_t new_pos, uint8_t base)
{
    draw_card(old_pos - base, 0);  /* 旧位置：取消选中高亮 */
    draw_card(new_pos - base, 1);  /* 新位置：设为选中高亮 */
}

/* ======================== 事件处理 ======================== */

/**
 * @brief  处理一个菜单事件（上移/下移/确认/返回）
 * @param  evt  待处理的事件类型
 * @note   核心状态机逻辑：
 *         - EVT_UP/DOWN: 移动光标（支持循环滚动），然后判断是否需要翻页
 *         - EVT_OK: 确认当前选中项——若该项指向子菜单则切换状态，若绑定模块则进入模块
 *         - EVT_BACK: 返回上级——退出当前模块（如有），返回主菜单，重置光标和滚动
 *         最后根据是否翻页决定局部刷新（仅两张卡片）还是全屏刷新
 */
void handle_event(MenuEvent evt)
{
    if (evt == EVT_NONE) return;

    const MenuItem *items = g_menu_table[current_state];
    uint8_t item_count = get_item_count(items);
    if (item_count == 0) return;

    /* 保存移动前的光标和滚动位置，用于后续判断是否翻页 */
    uint8_t old_cursor = cursor_pos;
    uint8_t old_start  = start_idx;

    switch (evt) {
    case EVT_UP:
        /* 上移：光标上移一位，到头则循环到末尾 */
        cursor_pos = (cursor_pos > 0) ? cursor_pos - 1 : item_count - 1;
        break;

    case EVT_DOWN:
        /* 下移：光标下移一位，到末尾则循环到开头 */
        cursor_pos = (cursor_pos < item_count - 1) ? cursor_pos + 1 : 0;
        break;

    case EVT_OK: {
        /* 确认：获取当前选中的菜单项 */
        const MenuItem *sel = &items[cursor_pos];

        if (sel->next_state != current_state) {
            /* 该菜单项指向不同的子菜单状态 → 切换到子菜单，光标和滚动归零 */
            current_state = sel->next_state;
            cursor_pos = 0;
            start_idx = 0;
        } else if (sel->module != NULL) {
            /* 该菜单项绑定了功能模块 → 切换到该模块运行，直接返回不刷新菜单 */
            module_switch_to(sel->module);
            return;
        }
        break;
    }

    case EVT_BACK:
        /* 返回：先退出当前运行的模块（如果有） */
        if (module_get_current() != NULL) {
            module_exit_current();
        }
        /* 返回主菜单状态，重置所有显示状态 */
        if (current_state != MENU_MAIN) {
            current_state = MENU_MAIN;
        }
        cursor_pos = 0;       /* 光标归零 */
        start_idx = 0;        /* 滚动位置归零 */
        kbd_drawn = 0;        /* 标记需要重绘键盘（因为从模块返回需要全屏刷新） */
        break;

    default:
        break;
    }

    /* 重新计算滚动位置，确保光标在可视区域内 */
    uint8_t new_start = 0;
    if (item_count > VISIBLE_CARDS) {
        /* 光标超出可视区下边界：滚动跟随 */
        if (cursor_pos >= start_idx + VISIBLE_CARDS)
            new_start = cursor_pos - VISIBLE_CARDS + 1;
        /* 光标超出可视区上边界：滚动跟随 */
        else if (cursor_pos < start_idx)
            new_start = cursor_pos;
        else
            new_start = start_idx;  /* 光标仍在可视区内，滚动位置不变 */
        /* 边界保护：滚动不能超出最大值 */
        if (new_start > item_count - VISIBLE_CARDS)
            new_start = item_count - VISIBLE_CARDS;
    }
    start_idx = new_start;

    /* 刷新策略：上下移动且未翻页时仅重绘两张卡片（高性能），否则全屏刷新 */
    if ((evt == EVT_UP || evt == EVT_DOWN) && new_start == old_start) {
        update_cursor(old_cursor, cursor_pos, new_start);
    } else {
        update_display();
    }
}

/* ======================== 触摸键盘扫描 ======================== */

/**
 * @brief  将触摸坐标映射到键盘按键
 * @param  x  触摸点的 X 像素坐标
 * @param  y  触摸点的 Y 像素坐标
 * @return 命中的按键字符值（如 '8'、'y' 等），未命中返回 EVT_NONE
 * @note   遍历 5x4 按键网格，将触摸点坐标与每个按键的矩形区域比对，
 *         命中时先绘制按键按压视觉反馈，再返回对应字符
 */
static int handle_keyboard_touch(uint16_t x, uint16_t y)
{
    uint16_t key_y_base = get_key_y_base();  /* 获取键盘区域的 Y 基准偏移 */

    for (uint8_t row = 0; row < 5; row++) {
        for (uint8_t col = 0; col < 4; col++) {
            KeyTypeDef *k = &keys[row][col];
            uint16_t ky1 = key_y_base + k->y_start;  /* 按键绝对 Y 上界 */
            uint16_t ky2 = key_y_base + k->y_end;    /* 按键绝对 Y 下界 */

            /* 判断触摸点是否落在当前按键的矩形区域内 */
            if (x >= k->x_start && x <= k->x_end &&
                y >= ky1 && y <= ky2) {
                /* 命中：绘制按键按压动画效果 */
                lcd_wgt_key_press(k->x_start, ky1, k->x_end, ky2,
                                  WGT_KEY_RADIUS, keymap[row][col]);
                return k->value;  /* 返回该按键对应的字符 */
            }
        }
    }
    return EVT_NONE;  /* 未命中任何按键 */
}

/**
 * @brief  触摸按键检测（含 200ms 硬件去抖）
 * @return 按下按键的字符值，无按键按下返回 EVT_NONE
 * @note   工作流程：
 *         1. 调用触摸屏驱动扫描当前触摸状态
 *         2. 检测按下事件（TP_PRES_DOWN）并进行 200ms 去抖
 *         3. 去抖通过后调用 handle_keyboard_touch() 做坐标到按键的映射
 *         4. 按键释放后清除 key_pressed 标志，允许检测下一次按下
 */
int detect_key_press(void)
{
    static uint32_t last_press = 0;  /* 上次有效按下的时间戳（毫秒） */

    /* 执行触摸屏硬件扫描，更新 tp_dev 中的坐标和状态 */
    tp_dev.scan(0);

    if (tp_dev.sta & TP_PRES_DOWN) {
        /* 触摸屏被按下：检查是否是新的按下事件（非长按重复）且去抖时间已过 */
        if (!key_pressed && (HAL_GetTick() - last_press >= 200)) {
            key_pressed = 1;                        /* 标记为已按下，防止长按重复触发 */
            last_press = HAL_GetTick();             /* 更新上次按下时间戳 */
            return handle_keyboard_touch(tp_dev.x[0], tp_dev.y[0]);  /* 映射坐标到按键 */
        }
    } else {
        /* 触摸屏已释放：清除按下标志，允许检测下一次按键 */
        key_pressed = 0;
    }
    return EVT_NONE;
}

/* ======================== 按键扫描（主循环调用） ======================== */

/**
 * @brief  主循环中的按键扫描入口 — 将键盘字符转换为菜单事件
 * @note   在主循环中周期性调用，工作流程：
 *         1. 调用 detect_key_press() 获取当前按下的键盘字符
 *         2. 与上次按键比较（last_key），防止同一按键被重复投递事件
 *         3. 将特定键盘字符映射为菜单事件并投递到事件队列：
 *            - '8'（向上键）→ EVT_UP    — 光标上移
 *            - '2'（向下键）→ EVT_DOWN  — 光标下移
 *            - 'y'（确认键）→ EVT_OK    — 进入选中项
 *            - 'n'（返回键）→ EVT_BACK  — 返回上级菜单
 *         4. 更新 last_key 用于去重
 */
void Key_Scan(void)
{
    static int last_key = 0;  /* 上次检测到的按键值，用于去重 */
    int key = detect_key_press();  /* 扫描触摸键盘，获取当前按键字符 */

    /* 仅当检测到有效按键且与上次不同时才投递事件（防止长按重复触发） */
    if (key != EVT_NONE && key != last_key) {
        switch (key) {
        case '8': post_event(EVT_UP);   break;  /* 数字键 8 → 上移光标 */
        case '2': post_event(EVT_DOWN); break;  /* 数字键 2 → 下移光标 */
        case 'y': post_event(EVT_OK);   break;  /* 确认键 y → 进入子菜单/模块 */
        case 'n': post_event(EVT_BACK); break;  /* 返回键 n → 返回上级菜单 */
        default: break;  /* 其他按键（数字、运算符等）在此处不生成菜单事件，可由子模块自行处理 */
        }
    }
    last_key = key;  /* 记录本次按键值，供下次去重比较 */
}
