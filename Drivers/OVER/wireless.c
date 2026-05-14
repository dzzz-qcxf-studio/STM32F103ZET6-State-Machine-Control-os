/**
 * @file    wireless.c
 * @brief   NRF24L01 2.4G无线遥控模块
 *
 * 功能概述：
 *   基于NRF24L01 2.4GHz无线模块实现的小车遥控器。
 *   左侧触摸区域控制油门（速度0~100），右侧触摸区域控制转向（角度0~180）。
 *   支持多点触控，可同时操作油门和转向。
 *   数据发送间隔50ms，采用模块化架构（enter/tick/exit），非阻塞运行。
 *
 * 通信协议：
 *   8字节数据包：帧头(0xAA) + 命令 + 速度 + 方向 + 转向 + 功能 + 触点数 + 校验和
 *   校验和采用补码校验（前7字节之和取反加1）。
 */

#include "./OVER/wireless.h"
#include "./OVER/module.h"
#include "./OVER/menu.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/lcd_widgets.h"
#include "./BSP/TOUCH/touch.h"
#include "./BSP/NRF24L01/nrf24l01.h"
#include "./BSP/NRF24L01/NRF24L01_define.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

/* ======================== 命令定义 ======================== */
/* 运动方向命令：定义小车8个运动方向 + 停止，对应协议包中的command字段 */
#define CMD_STOP          0x00  /* 停止：速度低于阈值时自动发送 */
#define CMD_FORWARD       0x01  /* 前进：转向居中且速度足够 */
#define CMD_BACKWARD      0x02  /* 后退（预留，当前逻辑未使用） */
#define CMD_LEFT          0x03  /* 左转（预留，当前逻辑未使用） */
#define CMD_RIGHT         0x04  /* 右转（预留，当前逻辑未使用） */
#define CMD_FORWARD_LEFT  0x05  /* 前进左转：转向角 < 40 时 */
#define CMD_FORWARD_RIGHT 0x06  /* 前进右转：转向角 > 140 时 */
#define CMD_BACKWARD_LEFT 0x07  /* 后退左转（预留） */
#define CMD_BACKWARD_RIGHT 0x08 /* 后退右转（预留） */

/* 功能开关标志位：采用位掩码方式，可组合使用，对应协议包中的function字段 */
#define FUNC_NONE   0x00  /* 无功能 */
#define FUNC_LIGHT  0x01  /* bit0：车灯开关，键盘'1'切换 */
#define FUNC_HORN   0x02  /* bit1：喇叭开关，键盘'2'切换 */
#define FUNC_DEMO   0x04  /* bit2：演示模式，键盘'3'切换 */
#define FUNC_TURBO  0x08  /* bit3：涡轮增压，键盘'4'切换 */

/* ======================== 遥控数据结构 ======================== */
/**
 * 遥控数据包结构体（共8字节）
 * 通过NRF24L01无线发送，接收端按相同结构解析。
 * 传输时直接将整个结构体强转为 uint8_t* 发送。
 */
typedef struct {
    uint8_t header;        /* 帧头标识：固定为 0xAA，用于接收端校验帧起始 */
    uint8_t command;       /* 运动命令：取值 CMD_STOP ~ CMD_BACKWARD_RIGHT */
    uint8_t speed;         /* 速度值：0~100，由油门触摸区域映射得到 */
    uint8_t direction;     /* 方向角：固定 90（预留字段，当前未使用） */
    uint8_t steering;      /* 转向角：0~180，90为居中，由转向触摸区域映射得到 */
    uint8_t function;      /* 功能标志位：FUNC_LIGHT|FUNC_HORN|FUNC_DEMO|FUNC_TURBO 组合 */
    uint8_t touch_points;  /* 当前触点数量：用于接收端判断操作状态 */
    uint8_t checksum;      /* 校验和：前7字节之和的补码，用于传输完整性校验 */
} RemoteControl_t;

/* ======================== 控制区域定义 ======================== */
/*
 * 控制区域布局说明：
 *   屏幕左侧为油门区域（垂直方向），右侧为转向区域（水平方向）。
 *   用户在油门区域内上下滑动控制速度，区域内左右滑动控制转向。
 *
 *   油门区域：X=50, Y=150, 宽100, 高200 像素
 *     - Y轴底部(speed=0) 到 顶部(speed=100)
 *   转向区域：X=250, Y=150, 宽100, 高200 像素
 *     - X轴左侧(steering=0) 到 右侧(steering=180)
 */

/* 油门（左侧垂直条）区域坐标和尺寸 */
#define THROTTLE_AREA_X       50    /* 油门区域左上角X坐标 */
#define THROTTLE_AREA_Y       150   /* 油门区域左上角Y坐标 */
#define THROTTLE_AREA_WIDTH   100   /* 油门区域宽度（像素） */
#define THROTTLE_AREA_HEIGHT  200   /* 油门区域高度（像素） */

/* 转向（右侧水平条）区域坐标和尺寸 */
#define STEERING_AREA_X       250   /* 转向区域左上角X坐标 */
#define STEERING_AREA_Y       150   /* 转向区域左上角Y坐标 */
#define STEERING_AREA_WIDTH   100   /* 转向区域宽度（像素） */
#define STEERING_AREA_HEIGHT  200   /* 转向区域高度（像素） */

/* ======================== 模块状态 ======================== */
/* 协议数据包实例：每次发送前填充各字段，然后整体发送 */
static RemoteControl_t remoteData;

/* 当前控制量 */
static uint8_t currentSpeed    = 0;   /* 当前油门速度：0~100，0为停止 */
static uint8_t currentSteering = 90;  /* 当前转向角度：0~180，90为正中 */
static uint8_t currentFunction = 0;   /* 当前功能位图：各bit对应不同功能开关 */

/* 触摸状态标志 */
static uint8_t throttleActive  = 0;   /* 油门区域是否有手指按下：1=按下，0=释放 */
static uint8_t steeringActive  = 0;   /* 转向区域是否有手指按下：1=按下，0=释放 */
static uint8_t lastTouchPoints = 0;   /* 上次扫描到的触点总数，写入协议包 */

/* 控制区域起始坐标（可动态调整，初始化时从宏定义复制） */
static uint16_t throttle_x, throttle_y;  /* 油门区域左上角坐标 */
static uint16_t steering_x, steering_y;  /* 转向区域左上角坐标 */

/* 定时发送相关 */
static uint32_t lastSendTick = 0;    /* 上次发送数据的时间戳（ms），用于50ms定时 */
static uint8_t  module_active = 0;   /* 模块运行标志：1=运行中，0=已退出 */

/* ======================== 校验和计算 ======================== */
/**
 * @brief  计算遥控数据包的校验和（补码校验）
 *
 * 算法：将数据包前7个字节（header ~ touch_points）求和，
 *       然后对和取反加1（即求补码），得到校验和。
 *       接收端将所有8字节（含校验和）相加，结果应为0。
 *
 * @param  d  指向遥控数据包的指针
 * @return 校验和值（uint8_t）
 */
static uint8_t CalcChecksum(RemoteControl_t *d)
{
    /* 累加前7个字段的值（不含checksum本身） */
    uint8_t sum = d->header + d->command + d->speed +
                  d->direction + d->steering + d->function +
                  d->touch_points;
    /* 取反加1 = 补码，使得 (sum + checksum) & 0xFF == 0 */
    return (uint8_t)(~sum + 1);
}

/* ======================== 发送遥控数据 ======================== */
/**
 * @brief  构建并发送一帧遥控数据包
 *
 * 流程：
 *   1. 将当前油门速度、转向角度、功能标志等填入数据包各字段
 *   2. 根据速度和转向角自动判断运动命令（停止/前进/左转/右转）
 *   3. 计算校验和并写入数据包末尾
 *   4. 通过NRF24L01无线发送整个8字节数据包
 *   5. 在LCD屏幕状态行显示当前发送的命令信息
 */
static void Send_Remote_Command(void)
{
    /* ---- 填充数据包各字段 ---- */
    remoteData.header       = 0xAA;            /* 固定帧头，接收端用于识别有效数据包 */
    remoteData.speed        = currentSpeed;     /* 油门速度：0~100 */
    remoteData.direction    = 90;               /* 方向角：固定90（预留字段） */
    remoteData.steering     = currentSteering;  /* 转向角：0~180，90=正中 */
    remoteData.function     = currentFunction;  /* 功能位图：灯/喇叭/演示/涡轮 */
    remoteData.touch_points = lastTouchPoints;  /* 当前触点数 */

    /* ---- 根据速度和转向角自动判定运动命令 ---- */
    if (currentSpeed < 10) {
        /* 速度低于10，视为停止（防止低速误触发） */
        remoteData.command = CMD_STOP;
    } else if (currentSteering < 40) {
        /* 转向角偏左（<40度），执行前进左转 */
        remoteData.command = CMD_FORWARD_LEFT;
    } else if (currentSteering > 140) {
        /* 转向角偏右（>140度），执行前进右转 */
        remoteData.command = CMD_FORWARD_RIGHT;
    } else {
        /* 转向角居中（40~140度），执行直行 */
        remoteData.command = CMD_FORWARD;
    }

    /* ---- 计算校验和并通过NRF24L01发送 ---- */
    remoteData.checksum = CalcChecksum(&remoteData);
    Send((uint8_t *)&remoteData);

    /* ---- 在LCD状态行显示发送信息，便于调试 ---- */
    char info[64];
    snprintf(info, sizeof(info), "CMD:%02X SPD:%d STR:%d PT:%d",
             remoteData.command, currentSpeed, currentSteering, lastTouchPoints);
    lcd_show_string(14, WGT_HEADER_H + 26, 400, 16, 12, info, WGT_CLR_TEXT_SEC);
}

/* ======================== 绘制控制区域 ======================== */
/**
 * @brief  绘制油门和转向两个触摸控制区域的UI界面
 *
 * 布局：
 *   左侧 - 油门区域（红色边框垂直条），上下10格刻度，顶部=100，底部=0
 *   右侧 - 转向区域（蓝色边框水平条），左右10格刻度，左=0，右=180
 *   区域上方显示操作提示标签："WS"=油门(W/S键)，"AD"=转向(A/D键)
 */
static void Draw_Control_Areas(void)
{
    /* ---- 绘制油门区域（左侧垂直条） ---- */
    /* 红色边框矩形，标识可触摸的油门控制范围 */
    lcd_draw_rectangle(throttle_x, throttle_y,
                       throttle_x + THROTTLE_AREA_WIDTH,
                       throttle_y + THROTTLE_AREA_HEIGHT, RED);
    /* 绘制11条水平刻度线（i=0为顶部100，i=10为底部0） */
    for (int i = 0; i <= 10; i++) {
        uint16_t y = throttle_y + i * (THROTTLE_AREA_HEIGHT / 10);
        /* 每格左侧画10px短刻度线 */
        lcd_draw_line(throttle_x, y, throttle_x + 10, y, DARKBLUE);
        /* 每隔一格在右侧标注数值：100, 80, 60, 40, 20, 0 */
        if (i % 2 == 0) {
            char val[4];
            snprintf(val, sizeof(val), "%d", 100 - i * 10);
            lcd_show_string(throttle_x + THROTTLE_AREA_WIDTH + 5, y - 8, 30, 16, 16, val, BLACK);
        }
    }

    /* ---- 绘制转向区域（右侧水平条） ---- */
    /* 蓝色边框矩形，标识可触摸的转向控制范围 */
    lcd_draw_rectangle(steering_x, steering_y,
                       steering_x + STEERING_AREA_WIDTH,
                       steering_y + STEERING_AREA_HEIGHT, BLUE);
    /* 在中间画一条水平参考线，表示转向居中位置（90度） */
    lcd_draw_line(steering_x, steering_y + STEERING_AREA_HEIGHT / 2,
                  steering_x + STEERING_AREA_WIDTH, steering_y + STEERING_AREA_HEIGHT / 2, DARKBLUE);
    /* 绘制11条垂直刻度线 */
    for (int i = 0; i <= 10; i++) {
        uint16_t x = steering_x + i * (STEERING_AREA_WIDTH / 10);
        /* 底部画10px短刻度线 */
        lcd_draw_line(x, steering_y + STEERING_AREA_HEIGHT - 10,
                      x, steering_y + STEERING_AREA_HEIGHT, DARKBLUE);
        /* 仅在0、5、10位置标注角度值：0, 90, 180 */
        if (i == 0 || i == 5 || i == 10) {
            char val[4];
            snprintf(val, sizeof(val), "%d", i * 18);
            lcd_show_string(x - 8, steering_y + STEERING_AREA_HEIGHT + 5, 30, 16, 16, val, BLACK);
        }
    }

    /* ---- 绘制区域标题标签 ---- */
    /* "WS"提示油门操作（对应键盘W=加速/S=减速，或上下触摸） */
    lcd_show_string(throttle_x, throttle_y - 30, 100, 24, 24, "WS", WGT_CLR_KEY_NEG);
    /* "AD"提示转向操作（对应键盘A=左转/D=右转，或左右触摸） */
    lcd_show_string(steering_x, steering_y - 30, 100, 24, 24, "AD", WGT_CLR_ACCENT);
}

/* ======================== 指示器位置更新 ======================== */
/**
 * @brief  更新油门指示器的显示位置
 *
 * 原理：先用白色填充擦除旧位置的红色指示块，再在新位置绘制红色指示块。
 *       Y轴方向：区域底部 = speed 0，区域顶部 = speed 100。
 *       采用静态变量last_speed记录上次位置，值相同则跳过重绘以减少闪烁。
 *
 * @param  speed  当前油门速度值（0~100）
 */
static void Update_Throttle(uint8_t speed)
{
    static uint8_t last_speed = 0xFF;  /* 记录上次速度值，0xFF表示尚未绘制过 */
    /* 速度未变化，无需重绘 */
    if (last_speed == speed) return;

    /* 擦除旧位置指示块（用背景色WHITE填充） */
    if (last_speed != 0xFF) {
        /* 计算旧速度对应的Y坐标：speed越大Y越小（越靠近区域顶部） */
        uint16_t ly = throttle_y + THROTTLE_AREA_HEIGHT - (last_speed * THROTTLE_AREA_HEIGHT / 100);
        lcd_fill(throttle_x + 5, ly - 10, throttle_x + THROTTLE_AREA_WIDTH - 5, ly + 10, WHITE);
    }
    /* 在新位置绘制红色指示块（20像素高的填充矩形） */
    uint16_t y = throttle_y + THROTTLE_AREA_HEIGHT - (speed * THROTTLE_AREA_HEIGHT / 100);
    lcd_fill(throttle_x + 5, y - 10, throttle_x + THROTTLE_AREA_WIDTH - 5, y + 10, RED);
    last_speed = speed;  /* 更新记录 */
}

/**
 * @brief  更新转向指示器的显示位置
 *
 * 原理：先用白色填充擦除旧位置的蓝色指示块，再在新位置绘制蓝色指示块。
 *       X轴方向：区域左侧 = steering 0，区域右侧 = steering 180。
 *       指示器绘制在转向区域的垂直中线上。
 *
 * @param  steering  当前转向角度（0~180，90为正中）
 */
static void Update_Steering(uint8_t steering)
{
    static uint8_t last_steering = 0xFF;  /* 记录上次转向值，0xFF表示尚未绘制过 */
    /* 转向角未变化，无需重绘 */
    if (last_steering == steering) return;

    /* 擦除旧位置指示块（用背景色WHITE填充） */
    if (last_steering != 0xFF) {
        /* 计算旧转向角对应的X坐标 */
        uint16_t lx = steering_x + (last_steering * STEERING_AREA_WIDTH / 180);
        lcd_fill(lx - 10, steering_y + STEERING_AREA_HEIGHT / 2 - 10, lx + 10, steering_y + STEERING_AREA_HEIGHT / 2 + 10, WHITE);
    }
    /* 在新位置绘制蓝色指示块（20x20像素的填充矩形） */
    uint16_t x = steering_x + (steering * STEERING_AREA_WIDTH / 180);
    lcd_fill(x - 10, steering_y + STEERING_AREA_HEIGHT / 2 - 10, x + 10, steering_y + STEERING_AREA_HEIGHT / 2 + 10, BLUE);
    last_steering = steering;  /* 更新记录 */
}

/**
 * @brief  更新功能开关状态的LCD显示
 *
 * 在屏幕底部区域（Y=385）水平排列显示4个功能的开关状态：
 *   LIGHT(车灯)、HORN(喇叭)、DEMO(演示)、TURBO(涡轮)
 * 每个功能显示标签 + "ON"(绿色0x07E0) 或 "OFF"(红色)。
 * 通过检查 currentFunction 对应的位标志来判断开关状态。
 */
static void Update_Function_Display(void)
{
    /* 车灯状态：bit0 = FUNC_LIGHT */
    lcd_show_string(60,  385, 60, 16, 16, "LIGHT:", WGT_CLR_TEXT_SEC);
    lcd_show_string(110, 385, 30, 16, 16, (currentFunction & FUNC_LIGHT) ? " ON" : "OFF",
                   (currentFunction & FUNC_LIGHT) ? 0x07E0 : WGT_CLR_KEY_NEG);

    /* 喇叭状态：bit1 = FUNC_HORN */
    lcd_show_string(140, 385, 60, 16, 16, "HORN:", WGT_CLR_TEXT_SEC);
    lcd_show_string(180, 385, 30, 16, 16, (currentFunction & FUNC_HORN) ? " ON" : "OFF",
                   (currentFunction & FUNC_HORN) ? 0x07E0 : WGT_CLR_KEY_NEG);

    /* 演示模式状态：bit2 = FUNC_DEMO */
    lcd_show_string(220, 385, 60, 16, 16, "DEMO:", WGT_CLR_TEXT_SEC);
    lcd_show_string(260, 385, 30, 16, 16, (currentFunction & FUNC_DEMO) ? " ON" : "OFF",
                   (currentFunction & FUNC_DEMO) ? 0x07E0 : WGT_CLR_KEY_NEG);

    /* 涡轮模式状态：bit3 = FUNC_TURBO */
    lcd_show_string(300, 385, 60, 16, 16, "TURBO:", WGT_CLR_TEXT_SEC);
    lcd_show_string(350, 385, 30, 16, 16, (currentFunction & FUNC_TURBO) ? " ON" : "OFF",
                   (currentFunction & FUNC_TURBO) ? 0x07E0 : WGT_CLR_KEY_NEG);
}

/* ======================== 触摸输入处理 ======================== */
/**
 * @brief  处理多点触控输入，映射触摸坐标到油门/转向值
 *
 * 处理流程：
 *   1. 调用触摸屏驱动扫描当前触点状态
 *   2. 遍历所有可能的触点（最多CT_MAX_TOUCH个）
 *   3. 判断触点是否落在油门区域或转向区域
 *   4. 将触点的Y坐标映射为速度值（0~100），X坐标映射为转向角（0~180）
 *   5. 若无触点（手指抬起），则重置油门为0、转向为90（居中）
 *
 * 多点触控支持：允许同时操作油门和转向（一个手指控制油门，另一个控制转向）
 */
static void Process_Touch_Input(void)
{
    uint8_t touchPoints = 0;     /* 本次扫描到的有效触点计数 */
    uint8_t throttleFound = 0;   /* 本次扫描中是否已在油门区域找到触点 */
    uint8_t steeringFound = 0;   /* 本次扫描中是否已在转向区域找到触点 */

    /* 触发触摸屏硬件扫描 */
    tp_dev.scan(0);

    if (tp_dev.sta & TP_PRES_DOWN) {
        /* ---- 有手指按下：遍历所有触点 ---- */
        for (uint8_t i = 0; i < CT_MAX_TOUCH; i++) {
            /* 检查第i个触点是否有效（位标志置位且坐标非零） */
            if ((tp_dev.sta & (1 << i)) && (tp_dev.x[i] > 0) && (tp_dev.y[i] > 0)) {
                touchPoints++;

                /* ---- 判断触点是否在油门区域（左侧垂直条） ---- */
                if (!throttleFound &&
                    tp_dev.x[i] >= throttle_x &&
                    tp_dev.x[i] <= throttle_x + THROTTLE_AREA_WIDTH &&
                    tp_dev.y[i] >= throttle_y &&
                    tp_dev.y[i] <= throttle_y + THROTTLE_AREA_HEIGHT) {

                    throttleFound = 1;
                    throttleActive = 1;  /* 标记油门区域处于按下状态 */
                    /* 将触摸Y坐标映射为速度：区域顶部(Y小)=100，区域底部(Y大)=0 */
                    uint16_t relY = tp_dev.y[i] - throttle_y;  /* 相对于区域顶部的偏移 */
                    currentSpeed = 100 - (relY * 100 / THROTTLE_AREA_HEIGHT);
                    if (currentSpeed > 100) currentSpeed = 100;  /* 防溢出保护 */
                    Update_Throttle(currentSpeed);  /* 刷新油门指示器位置 */
                }

                /* ---- 判断触点是否在转向区域（右侧水平条） ---- */
                if (!steeringFound &&
                    tp_dev.x[i] >= steering_x &&
                    tp_dev.x[i] <= steering_x + STEERING_AREA_WIDTH &&
                    tp_dev.y[i] >= steering_y &&
                    tp_dev.y[i] <= steering_y + STEERING_AREA_HEIGHT) {

                    steeringFound = 1;
                    steeringActive = 1;  /* 标记转向区域处于按下状态 */
                    /* 将触摸X坐标映射为转向角：区域左侧(X小)=0，区域右侧(X大)=180 */
                    uint16_t relX = tp_dev.x[i] - steering_x;  /* 相对于区域左侧的偏移 */
                    currentSteering = relX * 180 / STEERING_AREA_WIDTH;
                    if (currentSteering > 180) currentSteering = 180;  /* 防溢出保护 */
                    Update_Steering(currentSteering);  /* 刷新转向指示器位置 */
                }
            }
        }
    } else {
        /* ---- 无手指按下（释放）：重置为默认值 ---- */
        if (throttleActive) {
            /* 油门从按下变为释放，重置速度为0（停止） */
            throttleActive = 0;
            currentSpeed = 0;
            Update_Throttle(currentSpeed);
        }
        if (steeringActive) {
            /* 转向从按下变为释放，重置角度为90（正中/直行） */
            steeringActive = 0;
            currentSteering = 90;
            Update_Steering(currentSteering);
        }
    }

    /* 记录触点数，后续写入协议包的touch_points字段 */
    lastTouchPoints = touchPoints;
}

/* ======================== 模块接口 ======================== */
/**
 * @brief  无线遥控模块初始化入口
 *
 * 初始化流程：
 *   1. 初始化NRF24L01无线模块硬件（SPI、GPIO、配置寄存器）
 *   2. 重置所有控制状态变量为默认值（速度0、转向居中、无功能）
 *   3. 清屏并绘制标题栏 "CAR REMOTE"
 *   4. 检测NRF24L01是否连接成功，在状态行显示结果
 *   5. 初始化控制区域坐标并绘制油门/转向UI界面
 *   6. 发送一帧初始停止命令（确保接收端处于停止状态）
 *   7. 记录启动时间戳，设置模块运行标志
 */
static void wireless_enter(void)
{
    /* 初始化NRF24L01无线模块（SPI通信、配置发送/接收参数） */
    NRF24L01_Init();

    /* ---- 重置所有状态变量为默认值 ---- */
    currentSpeed    = 0;    /* 油门速度归零 */
    currentSteering = 90;   /* 转向居中（90度=正中） */
    currentFunction = 0;    /* 关闭所有功能（灯/喇叭/演示/涡轮） */
    throttleActive  = 0;    /* 油门触摸状态：未按下 */
    steeringActive  = 0;    /* 转向触摸状态：未按下 */
    lastTouchPoints = 0;    /* 触点数归零 */
    memset(&remoteData, 0, sizeof(remoteData));  /* 清空协议数据包 */

    /* ---- 清屏并绘制UI框架 ---- */
    lcd_fill(0, 0, lcddev.width, MENU_AREA_HEIGHT - 1, WGT_CLR_BG);  /* 用背景色填充屏幕 */
    lcd_wgt_header("CAR REMOTE", 0, 0);  /* 绘制顶部标题栏 */

    /* ---- 检测NRF24L01硬件连接状态 ---- */
    if (NRF24L01_Check() != 0) {
        /* SPI通信失败，未检测到NRF24L01模块（红色警告，错误码2） */
        lcd_wgt_status_label(14, WGT_HEADER_H + 6, "NRF24L01 NOT FOUND!", 2);
    } else {
        /* NRF24L01模块检测成功（绿色提示，状态码1） */
        lcd_wgt_status_label(14, WGT_HEADER_H + 6, "NRF24L01 OK", 1);
    }

    /* ---- 初始化控制区域坐标（从宏定义复制到运行时变量） ---- */
    throttle_x  = THROTTLE_AREA_X;   /* 油门区域左上角X */
    throttle_y  = THROTTLE_AREA_Y;   /* 油门区域左上角Y */

    steering_x  = STEERING_AREA_X;   /* 转向区域左上角X */
    steering_y  = STEERING_AREA_Y;   /* 转向区域左上角Y */

    /* ---- 绘制控制界面和初始状态 ---- */
    Draw_Control_Areas();             /* 绘制油门/转向区域边框和刻度 */
    Update_Throttle(currentSpeed);    /* 绘制油门指示器初始位置（速度=0） */
    Update_Steering(currentSteering); /* 绘制转向指示器初始位置（角度=90） */
    Update_Function_Display();        /* 显示功能开关状态（全部OFF） */
    Send_Remote_Command();            /* 发送初始停止命令，确保接收端同步 */

    /* 记录当前时间戳，用于后续50ms定时发送 */
    lastSendTick = HAL_GetTick();
    module_active = 1;  /* 标记模块已激活，tick函数开始工作 */
}

/**
 * @brief  无线遥控模块主循环函数（非阻塞，由系统主循环周期性调用）
 *
 * 每次调用执行以下任务：
 *   1. 扫描触摸屏，处理油门/转向输入（每次tick都执行，保证实时响应）
 *   2. 每50ms通过NRF24L01发送一帧遥控数据包（平衡实时性和无线带宽）
 *   3. 检测键盘按键，切换功能开关或退出模块
 */
static void wireless_tick(void)
{
    /* 模块未激活时直接返回（安全保护） */
    if (!module_active) return;

    /* ---- 任务1：处理触摸输入（每次tick都执行） ---- */
    /* 扫描触摸屏，将手指位置映射为油门速度和转向角度 */
    Process_Touch_Input();

    /* ---- 任务2：50ms定时发送遥控数据包 ---- */
    uint32_t now = HAL_GetTick();
    if (now - lastSendTick >= 50) {
        /* 距上次发送已过50ms，构建数据包并通过NRF24L01发送 */
        Send_Remote_Command();
        lastSendTick = now;  /* 更新发送时间戳 */
    }

    /* ---- 任务3：键盘功能切换 ---- */
    /* 检测是否有按键按下，用异或(^)操作切换对应功能的开关状态 */
    int key = detect_key_press();
    switch (key) {
    case '1':
        /* 按键'1'：切换车灯开关（bit0取反） */
        currentFunction ^= FUNC_LIGHT;
        Update_Function_Display();
        break;
    case '2':
        /* 按键'2'：切换喇叭开关（bit1取反） */
        currentFunction ^= FUNC_HORN;
        Update_Function_Display();
        break;
    case '3':
        /* 按键'3'：切换演示模式（bit2取反） */
        currentFunction ^= FUNC_DEMO;
        Update_Function_Display();
        break;
    case '4':
        /* 按键'4'：切换涡轮模式（bit3取反） */
        currentFunction ^= FUNC_TURBO;
        Update_Function_Display();
        break;
    case 'n':
        /* 按键'n'：退出无线遥控模块，返回上级菜单 */
        module_exit_current();
        return;
    default:
        break;
    }
}

/**
 * @brief  无线遥控模块退出清理函数
 *
 * 退出前执行安全措施：
 *   - 将油门速度归零、转向居中，发送一帧停止命令
 *   - 确保接收端小车停止运动，避免模块退出后小车失控
 *   - 清除模块运行标志，停止tick函数的执行
 */
static void wireless_exit(void)
{
    /* ---- 发送停止命令：确保小车安全停止 ---- */
    currentSpeed = 0;        /* 油门归零 */
    currentSteering = 90;    /* 转向居中 */
    Send_Remote_Command();   /* 发送最后一帧停止数据包 */

    /* 清除模块运行标志，后续tick调用将直接返回 */
    module_active = 0;
}

/* ======================== 模块导出 ======================== */
/**
 * 模块接口结构体：供菜单系统和模块管理器调用
 *   .name  - 模块名称，用于菜单显示和日志标识
 *   .enter - 初始化入口，进入模块时调用一次
 *   .tick  - 主循环函数，由系统主循环周期性调用（非阻塞）
 *   .exit  - 退出清理函数，离开模块时调用一次
 */
const ModuleInterface wireless_module = {
    .name  = "Wireless_Remote",   /* 模块名称 */
    .enter = wireless_enter,      /* 进入模块：初始化NRF24L01，绘制UI */
    .tick  = wireless_tick,       /* 主循环：触摸处理 + 50ms发送 + 键盘功能 */
    .exit  = wireless_exit,       /* 退出模块：发送停止命令，清理状态 */
};
