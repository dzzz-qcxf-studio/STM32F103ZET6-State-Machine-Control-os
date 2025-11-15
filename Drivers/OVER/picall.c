#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./USMART/usmart.h"
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
#include "string.h"
#include "math.h"

/**
 * @brief   获取指定路径下有效图片文件的数量
 * @param   path: 指定路径
 * @retval  有效图片文件的数量
 */
static uint16_t pic_get_tnum(char *path)
{
    uint8_t res;
    uint16_t rval = 0;
    DIR tdir;
    FILINFO *tfileinfo;
    
    /* 申请内存并打开目录 */
    tfileinfo = (FILINFO *)mymalloc(SRAMIN, sizeof(FILINFO));
    res = (uint8_t)f_opendir(&tdir, (const TCHAR *)path);
    if ((res == 0) && tfileinfo)
    {
        while (1)
        {
            res = (uint8_t)f_readdir(&tdir, tfileinfo);
            if ((res != 0) || (tfileinfo->fname[0] == 0))
            {
                break;
            }
            
            /* 判断是否为图片文件 */
            res = exfuns_file_type(tfileinfo->fname);
            if ((res & 0xF0) == 0x50)
            {
                rval++;
            }
        }
    }
    
    /* 释放内存 */
    myfree(SRAMIN, tfileinfo);
    
    return rval;
}

void picall(void)
{
    uint8_t t = 0;
    uint8_t key;
    uint8_t res;
    DIR picdir;
    uint16_t totpicnum;
    FILINFO *picfileinfo;
    char *pname;
    uint32_t *picoffsettbl;
    uint16_t curindex;
    uint16_t temp;
   
    my_mem_init(SRAMIN);                /* 初始化内部SRAM内存池 */
    exfuns_init();                      /* 为exfuns申请内存 */
    f_mount(fs[0], "0:", 1);            /* 挂载SD卡 */
    f_mount(fs[1], "1:", 1);            /* 挂载NOR Flash */
    
    /* 检查字库 */
    while (fonts_init() != 0)
    {
        
        /* 初始化SD卡 */
        while (sd_init() != 0)
        {
            lcd_show_string(30, 30, 200, 16, 16, "SD Card Error!", RED);
            delay_ms(500);
            lcd_show_string(30, 30, 200, 16, 16, "Please Check! ", RED);
            delay_ms(500);
            LED0_TOGGLE();
        }
        
        lcd_show_string(30, 50, 200, 16, 16, "SD Card OK", RED);
        lcd_show_string(30, 70, 200, 16, 16, "Font Updating...", RED);
        
        /* 更新字库 */
        res = fonts_update_font(30, 90, 16, (uint8_t *)"0:", RED);
        while (res != 0)
        {
            lcd_show_string(30, 90, 200, 16, 16, "Font Update Failed!", RED);
            delay_ms(200);
            lcd_show_string(30, 90, 200, 16, 16, "Please Check!      ", RED);
            delay_ms(200);
        }
        
        lcd_show_string(30, 90, 200, 16, 16, "Font Update Success!   ", RED);
        delay_ms(1500);
        lcd_clear(WHITE);
    }
    
    /* 打开图片文件夹 */
    while (f_opendir(&picdir, "0:/PICTURE") != FR_OK)
    {
        text_show_string(30, 130, 200, 16, "PICTURE文件夹错误!", 16, 0, RED);
        delay_ms(200);
        lcd_fill(30, 130, 200, 16, WHITE);
        delay_ms(200);
    }
    
    /* 获取有效图片文件数量 */
    totpicnum = pic_get_tnum("0:/PICTURE");
    while (totpicnum == 0)
    {
        text_show_string(30, 130, 200, 16, "没有图片文件!", 16, 0, RED);
        delay_ms(200);
        lcd_fill(30, 130, 200, 16, WHITE);
        delay_ms(200);
    }
    
    /* 申请内存 */
    picfileinfo = (FILINFO *)mymalloc(SRAMIN, sizeof(FILINFO));
    pname = (char *)mymalloc(SRAMIN, FF_MAX_LFN * 2 + 1);
    picoffsettbl = (uint32_t *)mymalloc(SRAMIN, 4 * totpicnum);
    while ((picfileinfo == NULL) || (pname == NULL) || (picoffsettbl == NULL))
    {
        text_show_string(30, 130, 200, 16, "内存分配失败!", 16, 0, RED);
        delay_ms(200);
        lcd_fill(30, 130, 200, 16, WHITE);
        delay_ms(200);
    }
    
    /* 打开目录并记录图片索引 */
    res = (uint8_t)f_opendir(&picdir, "0:/PICTURE");
    if (res == 0)
    {
        curindex = 0;
        while (1)
        {
            temp = picdir.dptr;
            res = (uint8_t)f_readdir(&picdir, picfileinfo);
            if ((res != 0) || (picfileinfo->fname[0] == 0))
            {
                break;
            }
            
            res = exfuns_file_type(picfileinfo->fname);
            if ((res & 0xF0) == 0x50)
            {
                picoffsettbl[curindex] = temp;
                curindex++;
            }
        }
    }
    
    /* 开始显示图片 */
    text_show_string(30, 130, 200, 16, "开始显示...", 16, 0, RED);
    delay_ms(1500);
    piclib_init();
    curindex = 0;
    res = (uint8_t)f_opendir(&picdir, (const TCHAR *)"0:/PICTURE");
    while (res == 0)
    {
        /* 获取下一个图片文件信息 */
        dir_sdi(&picdir, picoffsettbl[curindex]);
        res = (uint8_t)f_readdir(&picdir, picfileinfo);
        if ((res != 0) || (picfileinfo->fname[0] == 0))
        {
            break;
        }
        
        /* 根据图片路径显示图片 */
        strcpy((char *)pname, "0:/PICTURE/");
        strcat((char *)pname, (const char *)picfileinfo->fname);
        //lcd_clear(BLACK);
        piclib_ai_load_picfile(pname, 0, 0, lcddev.width, lcddev.height, 1);
        text_show_string(2, 2, lcddev.width, 16, (char *)pname, 16, 1, RED);
        
        while (1)
        {
            key = key_scan(0);
            if (key == KEY0_PRES)
            {
                /* 切换上一张 */
                if (curindex != 0)
                {
                    curindex--;
                }
                else
                {
                    curindex = totpicnum - 1;
                }
                break;
            }
            else if (key == WKUP_PRES)
            {
                /* 切换下一张 */
                curindex++;
                if (curindex >= totpicnum)
                {
                    curindex = 0;
                }
                break;
            }
            
            if (++t == 20)
            {
                t = 0;
                LED0_TOGGLE();
            }
            
            delay_ms(10);
        }
    }
    
    /* 释放内存 */
    myfree(SRAMIN, picfileinfo);
    myfree(SRAMIN, pname);
    myfree(SRAMIN, picoffsettbl);
    
    while (1);
}
