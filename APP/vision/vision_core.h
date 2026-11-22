#ifndef __VISION_CORE_H
#define __VISION_CORE_H

#include "main.h"

/* ========== 处理分辨率：对 320x240 帧做 2 倍降采样 ========== */
#define VIS_SRC_WIDTH       320U                            /* 源帧宽（摄像头固定输出） */
#define VIS_SRC_HEIGHT      240U                            /* 源帧高 */
#define VIS_DECIM           2U                              /* 降采样倍数 */
#define VIS_WIDTH           (VIS_SRC_WIDTH / VIS_DECIM)     /* 160 */
#define VIS_HEIGHT          (VIS_SRC_HEIGHT / VIS_DECIM)    /* 120 */
#define VIS_PIXELS          (VIS_WIDTH * VIS_HEIGHT)        /* 19200 */

/* ========== 目标筛选 ========== */
#define VIS_BLOB_MAX        8U      /* 输出目标数上限（按面积降序） */
#define VIS_AREA_MIN        40U     /* 最小目标面积（降采样域像素数），滤除噪点 */

/* ========== 默认阈值：红色色块 ==========
 * H 归一化到 0~255（360 度映射 255）；h_min > h_max 表示跨 0 环绕 */
#define VIS_H_MIN_DEFAULT   245U
#define VIS_H_MAX_DEFAULT   10U
#define VIS_S_MIN_DEFAULT   90U
#define VIS_V_MIN_DEFAULT   50U

/* HSV 阈值区间 */
typedef struct {
    uint8_t h_min;      /* 色调下限 */
    uint8_t h_max;      /* 色调上限（可小于下限，表示跨 0 环绕） */
    uint8_t s_min;      /* 饱和度下限 */
    uint8_t v_min;      /* 亮度下限 */
} VisRange_t;

/* 单个目标特征（坐标为降采样域） */
typedef struct {
    uint16_t area;      /* 面积（像素数） */
    uint16_t cx;        /* 质心 x */
    uint16_t cy;        /* 质心 y */
    uint16_t x0;        /* 外接框左（含） */
    uint16_t y0;        /* 外接框上（含） */
    uint16_t x1;        /* 外接框右（含） */
    uint16_t y1;        /* 外接框下（含） */
} VisBlob_t;

/* 单帧识别结果 */
typedef struct {
    uint8_t  count;                 /* 有效目标数（按面积降序） */
    VisBlob_t blob[VIS_BLOB_MAX];   /* blob[0] 为面积最大目标 */
} VisResult_t;

/* 函数接口 */
void Vision_SetRange(const VisRange_t *range);      /* 设置 HSV 阈值（可在运行期调参） */
void Vision_GetRange(VisRange_t *range);            /* 读取当前 HSV 阈值 */
void Vision_BuildMask(const uint16_t *frame);       /* 降采样 + RGB565→HSV + 阈值，生成内部二值掩码 */
const uint8_t *Vision_GetMask(void);                /* 掩码首地址（VIS_WIDTH*VIS_HEIGHT，1=前景） */
void Vision_FindBlobs(VisResult_t *result);         /* 游程连通域标记，输出目标特征 */

#endif /* __VISION_CORE_H */
