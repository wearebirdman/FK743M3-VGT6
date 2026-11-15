#ifndef __OV5640_H
#define __OV5640_H

#include "main.h"

/* ========== 硬件引脚（与 CubeMX GPIO 配置保持一致） ========== */
#define OV5640_PWDN_PORT        GPIOD
#define OV5640_PWDN_PIN         GPIO_PIN_14     /* 高有效掉电（模块无 RST 引脚，硬件 RC 复位） */

/* ========== SCCB 总线参数（I2C1 标准模式 100kHz） ========== */
#define OV5640_I2C_HANDLE       (&hi2c1)          /* SCCB 所用 I2C 句柄，移植时仅改此处（需已包含对应 i2c.h） */
#define OV5640_SCCB_ADDR        0x78U             /* 8 位写地址（HAL 读时自动 |0x01） */
#define OV5640_I2C_TIMEOUT      100U              /* 单次传输超时（ms） */

/* ========== 关键寄存器（16 位地址） ========== */
#define OV5640_REG_CHIPID_H     0x300AU         /* 芯片 ID 高字节，固定 0x56 */
#define OV5640_REG_CHIPID_L     0x300BU         /* 芯片 ID 低字节，固定 0x40 */
#define OV5640_CHIPID_VALUE     0x5640U

#define OV5640_REG_SYS_CTRL     0x3008U         /* 系统电源控制 */
#define OV5640_REG_CLK_SEL      0x3103U         /* 系统时钟选择 */
#define OV5640_REG_FMT_CTRL     0x4300U         /* 数据接口输出格式 */
#define OV5640_REG_FMT_MUX      0x501FU         /* ISP 输出格式 */
#define OV5640_REG_JPEG_MODE    0x4713U         /* JPEG 模式选择 */
#define OV5640_REG_GRP_ACCESS   0x3212U         /* 寄存器组访问（timing 提交） */
#define OV5640_REG_DVPHO_H      0x3808U         /* 输出水平尺寸高字节 */
#define OV5640_REG_DVPHO_L      0x3809U         /* 输出水平尺寸低字节 */
#define OV5640_REG_DVPVO_H      0x380AU         /* 输出垂直尺寸高字节 */
#define OV5640_REG_DVPVO_L      0x380BU         /* 输出垂直尺寸低字节 */
#define OV5640_REG_FLIP         0x3820U         /* 垂直翻转 Bit[2:1] */
#define OV5640_REG_MIRROR       0x3821U         /* 水平镜像 Bit[2:1]，Bit[5] JPEG 使能 */

#define OV5640_REG_SYS_RST      0x3000U         /* 系统复位控制：Bit[5] MCU 复位（写 AF 固件前置 1） */
#define OV5640_REG_AF_CMD       0x3022U         /* AF 命令：0x03 单次 / 0x04 持续 / 0x08 释放马达 */
#define OV5640_REG_AF_STATUS    0x3029U         /* AF 状态：0x7E 初始化中 / 0x70 就绪 / 0x10、0x20 对焦完成 */
#define OV5640_AF_MCU_BASE      0x8000U         /* 片内 MCU 程序区起始地址（4KB） */

/* ========== 时序参数（上电/复位流程） ========== */
#define OV5640_PWDN_HOLD_MS     20U             /* PWDN 拉高保持时间 */
#define OV5640_RC_RESET_MS      5U              /* PWDN 拉低后等待硬件 RC 复位（6~10ms） */
#define OV5640_SETTLE_MS        20U             /* 复位后稳定等待，之后才允许 SCCB 通信 */
#define OV5640_SOFT_RESET_MS    5U              /* 软复位后等待时间 */

/* 错误码定义 */
typedef enum {
    OV5640_OK = 0,        /* 成功 */
    OV5640_ERR_PARAM,     /* 参数错误 */
    OV5640_ERR_I2C,       /* SCCB 传输失败 */
    OV5640_ERR_ID,        /* 芯片 ID 不匹配 */
    OV5640_ERR_AF,        /* AF 固件下载失败（状态轮询超时） */
} Ov5640Err_t;

/* 输出像素格式（一期仅实现 RGB565，其余为后续拓展预留） */
typedef enum {
    OV5640_PIXFMT_RGB565 = 0,   /* 2 字节/像素，小端序即标准 RGB565 */
    OV5640_PIXFMT_YUV422,       /* 预留：1 字节/像素 */
    OV5640_PIXFMT_JPEG,         /* 预留：变长压缩数据 */
} Ov5640Pixfmt_t;

/* AF 固件镜像（ov5640_af.c 定义，由参考例程自动提取，勿手改） */
extern const uint8_t OV5640_AF_Firmware[];
extern const uint32_t OV5640_AF_Firmware_Size;

/* 函数接口 */
Ov5640Err_t OV5640_HwPowerOn(void);                                     /* PWDN 上电时序（RC 复位 + 稳定等待） */
Ov5640Err_t OV5640_SoftReset(void);                                     /* 软复位（0x3103=0x11 + 0x3008=0x82） */
Ov5640Err_t OV5640_CheckId(void);                                       /* 读芯片 ID 校验在位（0x5640） */
Ov5640Err_t OV5640_Init(Ov5640Pixfmt_t fmt);                            /* 下发初始化表并设置像素格式 */
Ov5640Err_t OV5640_SetFramesize(uint16_t width, uint16_t height);       /* 设置输出尺寸（须保持 4:3） */
Ov5640Err_t OV5640_SetPixfmt(Ov5640Pixfmt_t fmt);                       /* 切换输出像素格式 */
Ov5640Err_t OV5640_SetOrientation(uint8_t mirror, uint8_t flip);        /* 镜像/翻转（1 使能） */
Ov5640Err_t OV5640_AF_DownloadFirmware(void);                           /* 下载 AF 固件（每次上电一次） */
Ov5640Err_t OV5640_AF_TriggerContinuous(void);                          /* 触发持续自动对焦 */
Ov5640Err_t OV5640_AF_TriggerSingle(void);                              /* 触发单次自动对焦 */
Ov5640Err_t OV5640_AF_Release(void);                                    /* 释放马达（镜头回无穷远） */
Ov5640Err_t OV5640_AF_QueryStatus(uint8_t *done);                       /* 查询对焦状态（1=完成/空闲） */
Ov5640Err_t OV5640_WriteReg(uint16_t reg, uint8_t val);                 /* 写单个寄存器 */
Ov5640Err_t OV5640_ReadReg(uint16_t reg, uint8_t *val);                 /* 读单个寄存器 */

#endif /* __OV5640_H */
