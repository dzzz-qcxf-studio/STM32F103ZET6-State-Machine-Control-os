/**
 * @file    picall.c
 * @brief   SD卡图片浏览器模块 (带超时保护)
 *
 * 全屏阻塞式图片浏览，使用物理按键导航。
 * 所有初始化循环均带超时，失败时自动返回菜单。
 *
 * 注意：这是一个阻塞式模块，enter()函数内部包含完整的交互循环，
 * tick()和exit()不会被调用。模块退出时通过goto跳转到清理代码
 * 释放所有已分配的内存。
 *
 * 功能流程：初始化FATFS -> 初始化字库 -> 打开PICTURE目录 ->
 *           统计图片数量 -> 分配内存 -> 建立偏移表 -> 循环显示图片
 */

#include "./OVER/picall.h"
#include "./OVER/module.h"
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./MALLOC/malloc.h"
#include "./FATFS/exfuns/exfuns.h"
#include "./TEXT/text.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/KEY/key.h"
#include "./BSP/SRAM/sram.h"
#include "./BSP/SDIO/sdio_sdcard.h"
#include "./BSP/NORFLASH/norflash.h"
#include "./PICTURE/piclib.h"
#include <string.h>

/**
 * WAIT_WITH_TIMEOUT — 带超时保护的等待宏
 *
 * 用法：WAIT_WITH_TIMEOUT(条件, 超时毫秒数, 循环体)
 * 功能：反复执行 body 直到 condition 为真；若超过 max_ms 毫秒仍不满足，
 *       则通过 goto _fail 跳转到失败处理代码。
 *
 * 该宏用于防止硬件初始化失败时程序死锁（如SD卡未插入、字库损坏等）。
 *
 * @param condition  退出等待的条件表达式
 * @param max_ms     最大等待时间（毫秒）
 * @param body       等待期间每轮执行的代码（如延时、LED闪烁等）
 */
#define WAIT_WITH_TIMEOUT(condition, max_ms, body) \
    do { \
        uint32_t _t0 = HAL_GetTick(); \
        while (!(condition)) { \
            if (HAL_GetTick() - _t0 >= (max_ms)) goto _fail; \
            body; \
        } \
    } while (0)

/* ======================== 获取图片文件数量 ======================== */
/**
 * pic_get_tnum() — 统计指定目录下的图片文件数量
 *
 * 遍历目录中的所有文件，通过 exfuns_file_type() 判断文件类型。
 * 类型码高4位为0x50表示图片文件（如BMP、JPG、GIF等）。
 *
 * @param path  目录路径，如 "0:/PICTURE"
 * @return      图片文件的数量
 */
static uint16_t pic_get_tnum(char *path)
{
    uint8_t res;
    uint16_t rval = 0;      /* 图片计数器 */
    DIR tdir;                /* 目录对象 */
    FILINFO *tfileinfo;      /* 文件信息结构体，用于读取文件名和属性 */

    /* 在内部SRAM中分配文件信息结构体 */
    tfileinfo = (FILINFO *)mymalloc(SRAMIN, sizeof(FILINFO));
    /* 打开指定目录 */
    res = (uint8_t)f_opendir(&tdir, (const TCHAR *)path);
    if ((res == 0) && tfileinfo) {
        /* 逐个读取目录中的文件 */
        while (1) {
            res = (uint8_t)f_readdir(&tdir, tfileinfo);
            /* 读取失败或已到目录末尾（文件名为空）时退出循环 */
            if ((res != 0) || (tfileinfo->fname[0] == 0)) break;
            /* 获取文件类型 */
            res = exfuns_file_type(tfileinfo->fname);
            /* 高4位为0x50表示图片文件类型 */
            if ((res & 0xF0) == 0x50) rval++;
        }
    }
    /* 释放文件信息结构体占用的内存 */
    myfree(SRAMIN, tfileinfo);
    return rval;
}

/* ======================== 显示错误信息并等待 ======================== */
/**
 * show_error_and_wait() — 在LCD上显示错误信息并延时后返回
 *
 * 用于初始化失败时向用户显示友好的错误提示。
 * 显示红色错误信息和蓝色"Returning to menu..."提示，
 * 等待指定时间后自动返回（调用者通常接着goto到退出清理代码）。
 *
 * @param msg          要显示的错误信息字符串
 * @param duration_ms  显示持续时间（毫秒）
 */
static void show_error_and_wait(const char *msg, uint16_t duration_ms)
{
    lcd_clear(WHITE);
    /* 用大号红色字体显示错误信息 */
    lcd_show_string(30, 200, 400, 32, 24, (char *)msg, RED);
    /* 用蓝色字体显示返回菜单提示 */
    lcd_show_string(30, 260, 400, 16, 16, "Returning to menu...", BLUE);
    /* 延时，让用户有时间看到错误信息 */
    HAL_Delay(duration_ms);
}

/* ======================== 模块接口 ======================== */

/**
 * picall_enter() — 图片浏览器模块进入函数（阻塞式）
 *
 * 这是整个图片浏览器的核心函数，包含完整的初始化和交互循环。
 * 由于是阻塞式模块，所有逻辑都在此函数内完成。
 *
 * 初始化阶段（每步都有超时保护）：
 *   1. 初始化内存分配器和FATFS文件系统
 *   2. 挂载SD卡和NOR Flash文件系统
 *   3. 初始化字库（10秒超时，失败则尝试更新字库）
 *   4. 打开 "0:/PICTURE" 目录（5秒超时）
 *   5. 统计图片文件数量（5秒超时）
 *   6. 分配文件信息、文件名、偏移表所需的内存
 *   7. 遍历目录建立图片偏移表（用于快速跳转）
 *
 * 显示阶段：
 *   - 全屏显示当前图片，左上角显示文件路径
 *   - KEY0：上一张图片（到头则循环到最后一张）
 *   - WKUP：下一张图片（到尾则循环到第一张）
 *   - 触摸键盘 'n'：退出浏览器
 *   - LED0每2秒闪烁一次，表示系统正在运行
 */
static void picall_enter(void)
{
    uint8_t t = 0;              /* LED闪烁计数器 */
    uint8_t key;                /* 物理按键值 */
    uint8_t res;                /* 函数返回值/文件类型 */
    DIR picdir;                 /* 图片目录对象 */
    uint16_t totpicnum;         /* 图片总数 */
    FILINFO *picfileinfo;       /* 文件信息结构体指针 */
    char *pname;                /* 完整文件路径缓冲区 */
    uint32_t *picoffsettbl;     /* 图片在目录中的偏移表，用于快速定位 */
    uint16_t curindex;          /* 当前显示的图片索引 */
    uint16_t temp;              /* 临时变量，保存目录读取位置 */
    uint32_t tick0;             /* 超时计时起始时间戳 */

    /* ---- 第一步：初始化内存和文件系统 ---- */
    /* 初始化内部SRAM内存分配器 */
    my_mem_init(SRAMIN);
    /* 初始化FATFS扩展功能（文件类型识别等） */
    exfuns_init();
    /* 挂载SD卡（驱动器0）和NOR Flash（驱动器1） */
    f_mount(fs[0], "0:", 1);
    f_mount(fs[1], "1:", 1);

    /* ---- 第二步：字库初始化（最多重试10秒）---- */
    /* 字库存储在SPI NOR Flash中，可能需要从SD卡更新 */
    tick0 = HAL_GetTick();
    while (fonts_init() != 0) {
        /* 10秒内字库初始化一直失败，则显示错误并退出 */
        if (HAL_GetTick() - tick0 >= 10000) {
            show_error_and_wait("Font init timeout!", 2000);
            goto pic_exit;
        }
        /* 字库初始化失败，尝试重新初始化SD卡（最多3秒） */
        uint32_t sd_tick = HAL_GetTick();
        while (sd_init() != 0) {
            /* SD卡3秒内无法初始化，显示错误并退出 */
            if (HAL_GetTick() - sd_tick >= 3000) {
                show_error_and_wait("SD Card not found!", 2000);
                goto pic_exit;
            }
            /* SD卡初始化中，交替显示错误提示（闪烁效果） */
            lcd_show_string(30, 30, 200, 16, 16, "SD Card Error!", RED);
            delay_ms(200);
            lcd_show_string(30, 30, 200, 16, 16, "Please Check! ", RED);
            delay_ms(200);
            LED0_TOGGLE();  /* LED闪烁提示用户检查SD卡 */
        }
        /* SD卡初始化成功，开始从SD卡更新字库到NOR Flash */
        lcd_show_string(30, 50, 200, 16, 16, "SD Card OK", RED);
        lcd_show_string(30, 70, 200, 16, 16, "Font Updating...", RED);
        /* 将字库文件从SD卡写入NOR Flash，坐标(30,90)显示进度 */
        res = fonts_update_font(30, 90, 16, (uint8_t *)"0:", RED);
        if (res != 0) {
            show_error_and_wait("Font Update Failed!", 2000);
            goto pic_exit;
        }
        lcd_show_string(30, 90, 200, 16, 16, "Font Update Success!", RED);
        delay_ms(1500);
        lcd_clear(WHITE);
    }

    /* ---- 第三步：打开图片目录（最多5秒）---- */
    /* 尝试打开SD卡上的PICTURE文件夹 */
    tick0 = HAL_GetTick();
    while (f_opendir(&picdir, "0:/PICTURE") != FR_OK) {
        if (HAL_GetTick() - tick0 >= 5000) {
            show_error_and_wait("PICTURE folder not found!", 2000);
            goto pic_exit;
        }
        /* 目录打开失败时闪烁提示 */
        text_show_string(30, 130, 200, 16, "PICTURE folder error!", 16, 0, RED);
        delay_ms(100);
        lcd_fill(30, 130, 200, 16, WHITE);
        delay_ms(100);
    }

    /* ---- 第四步：统计图片数量（最多5秒）---- */
    totpicnum = pic_get_tnum("0:/PICTURE");
    tick0 = HAL_GetTick();
    while (totpicnum == 0) {
        if (HAL_GetTick() - tick0 >= 5000) {
            show_error_and_wait("No pictures found!", 2000);
            goto pic_exit;
        }
        /* 没有找到图片时闪烁提示 */
        text_show_string(30, 130, 200, 16, "No pictures found!", 16, 0, RED);
        delay_ms(100);
        lcd_fill(30, 130, 200, 16, WHITE);
        delay_ms(100);
        /* 重新统计（等待SD卡就绪） */
        totpicnum = pic_get_tnum("0:/PICTURE");
    }

    /* ---- 第五步：分配内存 ---- */
    /* picfileinfo：用于存储单个文件的信息（文件名、大小、日期等） */
    picfileinfo  = (FILINFO *)mymalloc(SRAMIN, sizeof(FILINFO));
    /* pname：存储完整的文件路径（如 "0:/PICTURE/test.bmp"） */
    /* FF_MAX_LFN * 2 + 1 为长文件名的最大字节数 */
    pname        = (char *)mymalloc(SRAMIN, FF_MAX_LFN * 2 + 1);
    /* picoffsettbl：存储每张图片在目录中的偏移位置 */
    /* 每个偏移量占4字节（uint32_t），共totpicnum个 */
    picoffsettbl = (uint32_t *)mymalloc(SRAMIN, 4 * totpicnum);
    /* 任一内存分配失败则清理已分配的内存并退出 */
    if ((picfileinfo == NULL) || (pname == NULL) || (picoffsettbl == NULL)) {
        show_error_and_wait("Memory alloc failed!", 2000);
        if (picfileinfo)  myfree(SRAMIN, picfileinfo);
        if (pname)        myfree(SRAMIN, pname);
        if (picoffsettbl) myfree(SRAMIN, picoffsettbl);
        return;
    }

    /* ---- 第六步：建立图片偏移表 ---- */
    /* 遍历PICTURE目录，记录每张图片文件在目录中的读取位置(dptr) */
    /* 这样后续可以通过 dir_sdi() 直接跳转到任意图片，无需从头遍历 */
    res = (uint8_t)f_opendir(&picdir, "0:/PICTURE");
    if (res == 0) {
        curindex = 0;
        while (1) {
            /* 保存当前目录读取位置（即本文件的偏移量） */
            temp = picdir.dptr;
            res = (uint8_t)f_readdir(&picdir, picfileinfo);
            /* 读取失败或目录结束时退出 */
            if ((res != 0) || (picfileinfo->fname[0] == 0)) break;
            /* 判断是否为图片文件 */
            res = exfuns_file_type(picfileinfo->fname);
            if ((res & 0xF0) == 0x50) {
                /* 记录该图片在目录中的偏移位置 */
                picoffsettbl[curindex] = temp;
                curindex++;
            }
        }
    }

    /* ---- 第七步：开始图片浏览循环 ---- */
    lcd_clear(WHITE);
    lcd_show_string(30, 130, 200, 16, 16, "Loading...", RED);
    delay_ms(500);
    /* 初始化图片解码库（设置解码回调函数等） */
    piclib_init();
    curindex = 0;
    /* 重新打开目录，准备开始浏览 */
    res = (uint8_t)f_opendir(&picdir, "0:/PICTURE");

    while (res == 0) {
        /* 使用偏移表直接定位到当前图片在目录中的位置 */
        dir_sdi(&picdir, picoffsettbl[curindex]);
        /* 读取当前图片的文件信息 */
        res = (uint8_t)f_readdir(&picdir, picfileinfo);
        if ((res != 0) || (picfileinfo->fname[0] == 0)) break;

        /* 拼接完整文件路径：如 "0:/PICTURE/image1.bmp" */
        strcpy(pname, "0:/PICTURE/");
        strcat(pname, picfileinfo->fname);
        /* 全屏加载并显示图片，参数1表示启用缩放以适应屏幕 */
        piclib_ai_load_picfile(pname, 0, 0, lcddev.width, lcddev.height, 1);
        /* 在左上角显示当前图片的文件路径 */
        text_show_string(2, 2, lcddev.width, 16, pname, 16, 1, RED);

        /* ---- 等待用户按键切换图片 ---- */
        while (1) {
            /* 扫描物理按键 */
            key = key_scan(0);
            if (key == KEY0_PRES) {
                /* KEY0：上一张（索引减1，到头则循环到最后一张） */
                curindex = (curindex > 0) ? curindex - 1 : totpicnum - 1;
                break;
            } else if (key == WKUP_PRES) {
                /* WKUP：下一张（索引加1，到尾则循环到第一张） */
                curindex = (curindex < totpicnum - 1) ? curindex + 1 : 0;
                break;
            }

            /* 检测触摸键盘的 'n' 键退出 */
            extern int detect_key_press(void);
            int tk = detect_key_press();
            if (tk == 'n') {
                goto pic_exit;  /* 跳转到退出清理代码 */
            }

            /* LED0每200ms翻转一次（20次*10ms=200ms周期），指示系统运行中 */
            if (++t == 20) { t = 0; LED0_TOGGLE(); }
            delay_ms(10);  /* 每10ms扫描一次按键 */
        }
    }

pic_exit:
    /* ---- 退出清理：释放所有已分配的内存 ---- */
    /* 注意：即使某指针为NULL，myfree内部也会安全处理 */
    myfree(SRAMIN, picfileinfo);
    myfree(SRAMIN, pname);
    myfree(SRAMIN, picoffsettbl);
}

/**
 * picall_tick() — 模块周期调度函数（空实现）
 *
 * 由于图片浏览器是阻塞式模块，所有逻辑在enter()中以阻塞方式运行，
 * tick()不会被系统调用，因此为空函数。
 */
static void picall_tick(void)
{
    /* 阻塞式模块，tick 不会被调用 */
}

/**
 * picall_exit() — 模块退出函数（空实现）
 *
 * 所有资源（内存）已在enter()函数的pic_exit标签处释放，
 * 此处无需额外清理。
 */
static void picall_exit(void)
{
    /* 清理（内存已在 enter 中释放） */
}

/* ======================== 模块导出 ======================== */
/* 模块接口结构体，供菜单系统注册和调用 */
const ModuleInterface picall_module = {
    .name  = "Picture_Viewer",   /* 模块名称，显示在菜单中 */
    .enter = picall_enter,       /* 进入模块时调用（阻塞式） */
    .tick  = picall_tick,        /* 空实现（阻塞式模块不使用） */
    .exit  = picall_exit,        /* 空实现（资源已在enter中释放） */
};
