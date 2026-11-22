#ifndef __LCD_ST7789_H
#define __LCD_ST7789_H

#include "main.h"

#include <stdbool.h>

/* ========== 硬件引脚（与 CubeMX 配置保持一致） ========== */
#define LCD_CS_PORT     GPIOE
#define LCD_CS_PIN      GPIO_PIN_11     /* 片选，低有效，软件控制（NSS Soft） */
#define LCD_DC_PORT     GPIOE
#define LCD_DC_PIN      GPIO_PIN_15     /* 数据/命令选择：低=命令，高=数据 */
#define LCD_BL_PORT     GPIOD
#define LCD_BL_PIN      GPIO_PIN_15     /* 背光：高电平点亮 */

/* ========== 面板物理尺寸（1.69 寸 240x280，ST7789 控制器） ========== */
#define LCD_PANEL_WIDTH     240U
#define LCD_PANEL_HEIGHT    280U

/* RGB565 常用颜色 */
#define LCD_COLOR_BLACK     0x0000U
#define LCD_COLOR_WHITE     0xFFFFU
#define LCD_COLOR_GREEN     0x07E0U

/* ========== 显示缓冲（横屏可视区 280x240） ==========
 * 摄像头固定 320x240，居中裁掉左右各 20 像素后送屏 */
#define LCD_SRC_WIDTH       320U                                        /* 源帧宽（摄像头固定输出） */
#define LCD_SHOW_WIDTH      280U                                        /* 显示宽（与面板横屏可视区一致） */
#define LCD_SHOW_HEIGHT     240U                                        /* 显示高 */
#define LCD_SHOW_PIXELS     (LCD_SHOW_WIDTH * LCD_SHOW_HEIGHT)          /* 67200 */
#define LCD_SHOW_BYTES      (LCD_SHOW_PIXELS * 2U)                      /* 134400 */
#define LCD_CROP_OFFSET     ((LCD_SRC_WIDTH - LCD_SHOW_WIDTH) / 2U)     /* 20，居中裁剪左偏移 */

/* 显示方向（H=横屏 280x240，V=竖屏 240x280，_FLIP 为 180 度翻转） */
typedef enum {
    LCD_DIR_H = 0,      /* 横屏：与 280x240 帧缓冲同序，摄像头显示用 */
    LCD_DIR_V,          /* 竖屏：面板原生方向 */
    LCD_DIR_H_FLIP,     /* 横屏 180 度 */
    LCD_DIR_V_FLIP,     /* 竖屏 180 度 */
} LcdDirection_t;

/* 函数接口 */
void LCD_Init(void);                                                        /* 上电初始化：出休眠、开显示、横屏、清屏、开背光 */
void LCD_SetDirection(LcdDirection_t dir);                                  /* 设置显示方向（MADCTL + 坐标偏移） */
void LCD_Clear(uint16_t color);                                             /* 整屏填充单色 */

/* 显示缓冲 + SPI DMA 路径（摄像头帧专用，非阻塞） */
void LCD_CropToBuffer(const uint16_t *src, uint16_t src_width);              /* 居中裁切源帧到显示缓冲 */
void LCD_FillRectBuffer(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);  /* 在显示缓冲内填充 */
void LCD_FlushBuffer(void);                                                  /* Clean cache 后启动 SPI DMA 分段发送 */
bool LCD_IsFlushBusy(void);                                                  /* 上一帧是否仍在发送 */
void LCD_DrawStringBuffer(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg);  /* 显示缓冲内绘制 8x8 字符串 */
void LCD_BacklightOn(void);                                                 /* 开背光 */
void LCD_BacklightOff(void);                                                /* 关背光 */

#endif /* __LCD_ST7789_H */
