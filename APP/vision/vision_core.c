#include "vision_core.h"

#include <string.h>

/* ========== 内部常量 ========== */
#define VIS_RUN_MAX         (1024U)                     /* 单帧游程总数上限，超出说明噪声过多，放弃本帧 */
#define VIS_ROW_RUN_MAX     (VIS_WIDTH / 2U + 1U)       /* 单行游程上限（前景/背景交替最坏情况） */
#define VIS_HUE_SPAN        (255L)                      /* H 归一化幅度，360 度映射 255 */
#define VIS_HUE_SECTOR_G    (85L)                       /* 绿色扇区起始（120 度） */
#define VIS_HUE_SECTOR_B    (170L)                      /* 蓝色扇区起始（240 度） */

/* ========== 内部类型 ========== */
/* 水平游程：连通域标记的基本单元 */
typedef struct {
    uint16_t x0;        /* 起始列（含） */
    uint16_t x1;        /* 结束列（含） */
    uint16_t y;         /* 所在行 */
    uint16_t parent;    /* 并查集父游程下标，根指向自身 */
} VisRun_t;

/* 游程统计量，按根游程下标聚合 */
typedef struct {
    uint32_t area;      /* 面积（像素数） */
    uint32_t sum_x;     /* 列坐标累加 */
    uint32_t sum_y;     /* 行坐标累加 */
    uint16_t x0;        /* 外接框左 */
    uint16_t y0;        /* 外接框上 */
    uint16_t x1;        /* 外接框右 */
    uint16_t y1;        /* 外接框下 */
} VisStat_t;

/* ========== 内部状态（DTCM，CPU 独占访问，不参与 DMA） ========== */
static uint8_t s_Mask[VIS_PIXELS];      /* 二值掩码：1=前景，0=背景 */
static VisRun_t s_Run[VIS_RUN_MAX];     /* 全帧游程表 */
static VisStat_t s_Stat[VIS_RUN_MAX];   /* 游程统计量 */
static uint16_t s_RowRun[VIS_ROW_RUN_MAX];  /* 当前行游程下标 */
static uint16_t s_PrevRun[VIS_ROW_RUN_MAX]; /* 上一行游程下标 */

static VisRange_t s_Range = {
    .h_min = VIS_H_MIN_DEFAULT,
    .h_max = VIS_H_MAX_DEFAULT,
    .s_min = VIS_S_MIN_DEFAULT,
    .v_min = VIS_V_MIN_DEFAULT,
};

/* ========== 阈值配置 ========== */

void Vision_SetRange(const VisRange_t *range)
{
    if (range != NULL)
        s_Range = *range;
}

void Vision_GetRange(VisRange_t *range)
{
    if (range != NULL)
        *range = s_Range;
}

/* ========== HSV 分割 ========== */

/* 色调是否落在阈值区间内，支持跨 0 环绕（如红色 245~10） */
static uint8_t Vis_HueInRange(uint32_t h, const VisRange_t *range)
{
    if (range->h_min <= range->h_max)
        return ((h >= range->h_min) && (h <= range->h_max)) ? 1U : 0U;

    return ((h >= range->h_min) || (h <= range->h_max)) ? 1U : 0U;
}

/* RGB565 像素 → HSV → 阈值判定：1=前景，0=背景
 * H/S/V 均为 8 位定点，H 按 360 度映射到 0~255，可省去浮点运算 */
static uint8_t Vis_IsForeground(uint16_t px, const VisRange_t *range)
{
    uint32_t r5 = (px >> 11) & 0x1FU;
    uint32_t g6 = (px >> 5) & 0x3FU;
    uint32_t b5 = px & 0x1FU;
    uint32_t r = (r5 << 3) | (r5 >> 2);     /* 5 位扩展到 8 位 */
    uint32_t g = (g6 << 2) | (g6 >> 4);     /* 6 位扩展到 8 位 */
    uint32_t b = (b5 << 3) | (b5 >> 2);
    uint32_t max = r;
    uint32_t min = r;
    uint32_t d;
    int32_t h;

    if (g > max) max = g;
    if (g < min) min = g;
    if (b > max) max = b;
    if (b < min) min = b;

    if ((max == 0U) || (max < range->v_min))
        return 0U;

    d = max - min;
    if (((d * (uint32_t)VIS_HUE_SPAN) / max) < (uint32_t)range->s_min)   /* 饱和度不足，先淘汰 */
        return 0U;

    if (d == 0U)
        return 0U;                                   /* 灰阶无色调，不参与色块识别 */

    if (max == r)
        h = (int32_t)(((int32_t)g - (int32_t)b) * VIS_HUE_SPAN) / (int32_t)(6U * d);
    else if (max == g)
        h = VIS_HUE_SECTOR_G + (int32_t)(((int32_t)b - (int32_t)r) * VIS_HUE_SPAN) / (int32_t)(6U * d);
    else
        h = VIS_HUE_SECTOR_B + (int32_t)(((int32_t)r - (int32_t)g) * VIS_HUE_SPAN) / (int32_t)(6U * d);

    if (h < 0)
        h += VIS_HUE_SPAN;

    return Vis_HueInRange((uint32_t)h, range);
}

/* 降采样（取每 2x2 块左上像素）+ 逐像素阈值，生成掩码 */
void Vision_BuildMask(const uint16_t *frame)
{
    if (frame == NULL)
        return;

    for (uint32_t y = 0U; y < VIS_HEIGHT; y++)
    {
        const uint16_t *src = &frame[y * VIS_DECIM * VIS_SRC_WIDTH];
        uint8_t *dst = &s_Mask[y * VIS_WIDTH];

        for (uint32_t x = 0U; x < VIS_WIDTH; x++)
            dst[x] = Vis_IsForeground(src[x * VIS_DECIM], &s_Range);
    }
}

const uint8_t *Vision_GetMask(void)
{
    return s_Mask;
}

/* ========== 连通域标记（游程 + 并查集） ========== */

static uint16_t Vis_Find(uint16_t idx)
{
    while (s_Run[idx].parent != idx)
    {
        s_Run[idx].parent = s_Run[s_Run[idx].parent].parent;   /* 路径减半压缩 */
        idx = s_Run[idx].parent;
    }
    return idx;
}

static void Vis_Union(uint16_t a, uint16_t b)
{
    uint16_t ra = Vis_Find(a);
    uint16_t rb = Vis_Find(b);

    if (ra != rb)
        s_Run[rb].parent = ra;
}

/* 按面积降序插入结果，超出上限即丢弃最小者 */
static void Vis_InsertBlob(VisResult_t *result, const VisStat_t *stat)
{
    VisBlob_t blob;
    uint32_t pos;

    blob.area = (uint16_t)stat->area;
    blob.cx = (uint16_t)(stat->sum_x / stat->area);
    blob.cy = (uint16_t)(stat->sum_y / stat->area);
    blob.x0 = stat->x0;
    blob.y0 = stat->y0;
    blob.x1 = stat->x1;
    blob.y1 = stat->y1;

    if (result->count < VIS_BLOB_MAX)
        result->count++;
    else if (blob.area <= result->blob[VIS_BLOB_MAX - 1U].area)
        return;     /* 比现有最小目标还小，直接丢弃 */

    pos = (uint32_t)result->count - 1U;
    while ((pos > 0U) && (result->blob[pos - 1U].area < blob.area))
    {
        result->blob[pos] = result->blob[pos - 1U];
        pos--;
    }
    result->blob[pos] = blob;
}

/* 逐行提取游程 → 与上一行重叠游程合并 → 按根聚合面积/质心/外接框 */
void Vision_FindBlobs(VisResult_t *result)
{
    uint32_t run_cnt = 0U;
    uint32_t prev_cnt = 0U;
    uint8_t overflow = 0U;

    if (result == NULL)
        return;

    result->count = 0U;

    for (uint32_t y = 0U; (y < VIS_HEIGHT) && (overflow == 0U); y++)
    {
        const uint8_t *row = &s_Mask[y * VIS_WIDTH];
        uint32_t row_cnt = 0U;
        uint32_t x = 0U;

        while (x < VIS_WIDTH)
        {
            uint32_t x0 = x;

            if (row[x] == 0U)
            {
                x++;
                continue;
            }

            while ((x < VIS_WIDTH) && (row[x] != 0U))
                x++;

            if (run_cnt >= VIS_RUN_MAX)
            {
                overflow = 1U;
                break;
            }

            s_Run[run_cnt].x0 = (uint16_t)x0;
            s_Run[run_cnt].x1 = (uint16_t)(x - 1U);
            s_Run[run_cnt].y = (uint16_t)y;
            s_Run[run_cnt].parent = (uint16_t)run_cnt;
            s_RowRun[row_cnt++] = (uint16_t)run_cnt;

            /* 与上一行所有水平重叠的游程合并，同属一个连通域 */
            for (uint32_t p = 0U; p < prev_cnt; p++)
            {
                const VisRun_t *prev = &s_Run[s_PrevRun[p]];

                if ((prev->x0 <= s_Run[run_cnt].x1) && (prev->x1 >= s_Run[run_cnt].x0))
                    Vis_Union(s_PrevRun[p], (uint16_t)run_cnt);
            }

            run_cnt++;
        }

        (void)memcpy(s_PrevRun, s_RowRun, row_cnt * sizeof(uint16_t));
        prev_cnt = row_cnt;
    }

    if (overflow != 0U)
        return;     /* 游程表溢出，本帧结果不可信，直接放弃 */

    (void)memset(s_Stat, 0, run_cnt * sizeof(VisStat_t));

    /* 统计量全部累加到根游程，非根位置保持为 0 */
    for (uint32_t i = 0U; i < run_cnt; i++)
    {
        const VisRun_t *run = &s_Run[i];
        VisStat_t *stat = &s_Stat[Vis_Find((uint16_t)i)];
        uint32_t len = (uint32_t)run->x1 - (uint32_t)run->x0 + 1U;

        if (stat->area == 0U)
        {
            stat->x0 = run->x0;
            stat->y0 = run->y;
            stat->x1 = run->x1;
            stat->y1 = run->y;
        }
        else
        {
            if (run->x0 < stat->x0) stat->x0 = run->x0;
            if (run->x1 > stat->x1) stat->x1 = run->x1;
            if (run->y < stat->y0) stat->y0 = run->y;
            if (run->y > stat->y1) stat->y1 = run->y;
        }

        stat->area += len;
        stat->sum_x += ((uint32_t)run->x0 + (uint32_t)run->x1) * len / 2U;
        stat->sum_y += (uint32_t)run->y * len;
    }

    /* 提取所有根游程中面积达标的目标 */
    for (uint32_t i = 0U; i < run_cnt; i++)
    {
        if (Vis_Find((uint16_t)i) != i)
            continue;
        if (s_Stat[i].area < VIS_AREA_MIN)
            continue;

        Vis_InsertBlob(result, &s_Stat[i]);
    }
}
