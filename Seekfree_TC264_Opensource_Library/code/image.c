/**
 * @file image.c
 * @brief TC264 摄像头图像巡线处理模块。
 *
 * 主要流程：
 * 1. 统计灰度直方图并使用大津法计算二值化阈值；
 * 2. 从图像底部选择左右边线种子，执行八邻域爬线；
 * 3. 对断线、急弯、十字和环岛场景进行边线补齐；
 * 4. 根据左右边线生成并平滑中线，供控制层计算转向误差。
 *
 * 阈值统计变量：
 * hist 为灰度直方图，P 为灰度概率，PK 为累计概率，
 * MK 为累计均值，img_threshold 为最终二值化阈值。
 */

#include "zf_common_headfile.h"
#include "image.h"
// 参数 ENABLE_ONE_SIDE_FILL_NORMAL：普通单边补线关闭。

#define ENABLE_ONE_SIDE_FILL_NORMAL 0

// 参数 ONE_SIDE_FILL_TOP_MARGIN：单边补线顶部保留的行余量，当前值为 0。
#define ONE_SIDE_FILL_TOP_MARGIN   0
// 参数 RING_ENTER_FRAMES：环岛入口状态要求的基础连续帧数，当前值为 12。
#define RING_ENTER_FRAMES      12
// 参数 ONE_SIDE_FILL_LIMIT_TO_REAL_SPAN：单边补线是否限制在真实有效跨度内，当前值为 1。
#define ONE_SIDE_FILL_LIMIT_TO_REAL_SPAN   1
// 判断指定边是否为环岛短边；短边爬线时可适当放宽方向和向外跳变限制。
#define RING_RELAX_SIDE(side)   (ring_relax_trace && ((side) == ring_short_side))
// 判断指定边是否属于环岛长边；长边仍采用较严格的爬线限制。
#define RING_LONG_TIGHT(side)   (ring_relax_trace && ((side) != ring_short_side))
// 判断环岛短边当前是否允许使用较宽松的断点恢复范围。
#define RING_SHORT_TIGHT_RECOVER(side)   ((ring_state != 0) && ((side) == ring_short_side))
// 返回与环岛短边相对的另一侧边线，即环岛长边。
#define RING_LONG_SIDE()             ((ring_short_side == EDGE_RIGHT) ? EDGE_LEFT : EDGE_RIGHT)
// 判断环岛长边是否允许使用十字场景的跨白区爬线方式。
#define RING_TRACE_CROSS_SIDE(side)  (ring_relax_trace && ((side) == RING_LONG_SIDE()))
// 参数 MID_ANG_ENABLE：基于角度差切中线关闭。
#define MID_ANG_ENABLE     0
// 参数 MID_ANG_BLOCK：中线角度检测使用的分块行数，当前值为 6。
#define MID_ANG_BLOCK      6
// 参数 MID_ANG_STEP：中线角度检测时相邻分块的步进行数，当前值为 1。
#define MID_ANG_STEP       1
// 参数 MID_ANG_OUTLIER：中线角度统计时剔除离群点的阈值，当前值为 30。
#define MID_ANG_OUTLIER    30
// 参数 MID_ANG_DIFF：判定中线角度突变的阈值，当前值为 30。
#define MID_ANG_DIFF       30
// 参数 MID_ANG_MIN_ROWS：中线角度判断所需的最少有效行，当前值为 8。
#define MID_ANG_MIN_ROWS   8
// 参数 MID_OFF_ENABLE：基于中线偏移突变切中线开启。
#define MID_OFF_ENABLE    1
// 参数 MID_OFF_BLOCK：中线偏移突变检测使用的分块行数，当前值为 8。
#define MID_OFF_BLOCK     8
// 参数 MID_OFF_STEP：中线偏移检测时相邻分块的步进行数，当前值为 1。
#define MID_OFF_STEP      1
// 参数 MID_OFF_DIFF：普通道路中线偏移突变阈值，当前值为 11。
#define MID_OFF_DIFF      11
// 参数 MID_OFF_DIFF_RING：环岛场景中线偏移突变阈值，当前值为 15。
#define MID_OFF_DIFF_RING 15


// 参数 SINGLE_EDGE_TREND_TOL：单边边线趋势判断容差，当前值为 2。
 #define SINGLE_EDGE_TREND_TOL       2
// 参数 SINGLE_EDGE_MAX_VERIFY_ROWS：单边趋势验证允许使用的最大行数，当前值为 12。
#define SINGLE_EDGE_MAX_VERIFY_ROWS 12
// 参数 SINGLE_EDGE_MIN_OUTWARD_DX：单边持续向外跳变的最小累计量，当前值为 8。
#define SINGLE_EDGE_MIN_OUTWARD_DX  8
// 参数 IPM_CALIBRATED_LANE_WIDTH：逆透视鸟瞰图中的标定赛道宽度，当前值为 30.0f。
#define IPM_CALIBRATED_LANE_WIDTH 30.0f
// 参数 IPM_MAX_INTERPOLATE_GAP：IPM 中线允许线性插值的最大间隙，当前值为 24。
#define IPM_MAX_INTERPOLATE_GAP 24
// 参数 EDGE_MAX_BACKTRACK_ROWS：普通边线爬线允许回看的最大行数，当前值为 12。
#define EDGE_MAX_BACKTRACK_ROWS 12
// 参数 ZEBRA_Y_START：斑马线垂直扫描起始行，当前值为 45。
#define ZEBRA_Y_START              45
// 参数 ZEBRA_Y_END：斑马线垂直扫描结束行，当前值为 108。
#define ZEBRA_Y_END                108
// 参数 ZEBRA_X_START：斑马线水平扫描起始列，当前值为 40。
#define ZEBRA_X_START              40
// 参数 ZEBRA_X_END：斑马线水平扫描结束列，当前值为 148。
#define ZEBRA_X_END                148
// 参数 IPM_MIN_ACCEPT_ROWS：认可 IPM 补线结果所需的最少有效行数，当前值为 15。
#define IPM_MIN_ACCEPT_ROWS 15
// 参数 ZEBRA_DARK_PERCENT：斑马线行中黑色像素要求的最小占比，当前值为 35。
#define ZEBRA_DARK_PERCENT         35

// 参数 ZEBRA_MIN_BAND_ROWS：单条斑马线黑带要求的最少行数，当前值为 2。
#define ZEBRA_MIN_BAND_ROWS        2

// 参数 ZEBRA_MIN_BANDS：斑马线判定要求的最少黑带数，当前值为 3。
#define ZEBRA_MIN_BANDS            3

// 参数 ZEBRA_MIN_ACTIVE_ROWS：斑马线判定要求的最少有效行，当前值为 8。
#define ZEBRA_MIN_ACTIVE_ROWS      8
// 参数 CROSS_MIN_MISSING_ROWS：十字判定所需的最小连续缺线行数，当前值为 6。



#define CROSS_MIN_MISSING_ROWS              6
// 参数 CROSS_FIT_WINDOW：十字边线直线拟合使用的行窗口，当前值为 20。
#define CROSS_FIT_WINDOW                    20
// 参数 CROSS_BACK_ROWS：十字爬线允许回看的行数，当前值为 5。
#define CROSS_BACK_ROWS                     5
// 参数 CROSS_MAX_FIT_RMS_SQ：十字直线拟合允许的最大均方误差，当前值为 9.0f。
#define CROSS_MAX_FIT_RMS_SQ                9.0f
// 参数 CROSS_MIN_WIDTH：十字补线时允许的最小赛道宽度，当前值为 12。
#define CROSS_MIN_WIDTH                     12
// 参数 CROSS_MAX_SLOPE：十字补线时允许的最大直线斜率，当前值为 5.0f。
#define CROSS_MAX_SLOPE                     5.0f
// 参数 SINGLE_EDGE_JUMP_DX：单边检测允许的最大单次横向跳变，当前值为 5。
#define SINGLE_EDGE_JUMP_DX       5
// 参数 SINGLE_EDGE_OUTWARD_RUN：单边连续向外跳变判定次数，当前值为 4。
#define SINGLE_EDGE_OUTWARD_RUN   4
// 参数 CROSS_MIN_TRACE_POINTS：确认十字爬线结果所需的最少点数，当前值为 6。
#define CROSS_MIN_TRACE_POINTS              6
// 参数 CROSS_MIN_COMMON_ROWS_AROUND_GAP：十字缺口附近要求的最少共同有效行数，当前值为 0。
#define CROSS_MIN_COMMON_ROWS_AROUND_GAP    0

// 参数 CROSS_MAX_TOP_DIFF：十字场景左右线顶部允许的最大行差，当前值为 200。
#define CROSS_MAX_TOP_DIFF                  200

// 参数 MID_MIN_VALID_ROWS：中线输出所需的最少有效行数，当前值为 15。
#define MID_MIN_VALID_ROWS                  15
/* ===== 全局图像数据 ===== */
uint8 image[MT9V03X_H][MT9V03X_W];       /* 二值化图像缓冲区；白色像素表示可行驶道路。 */
uint8 base_image[MT9V03X_H][MT9V03X_W];  /* 摄像头原始灰度图，二值化前保留用于备用判断。 */
uint8 left_jidian;                       /* 左侧巡线起始点的横坐标。 */
uint8 right_jidian;                      /* 右侧巡线起始点的横坐标。 */
uint8 left_line_list[MT9V03X_H];         /* 左侧边线逐行缓存；每一行保存一个横坐标。 */
uint8 right_line_list[MT9V03X_H];        /* 右侧边线逐行缓存；每一行保存一个横坐标。 */
uint8 mid_line_list[MT9V03X_H];          /* 由左右边线计算得到的逐行中心线。 */
uint8 ipm_mid_line_valid[MT9V03X_H];

// 保存经过精确判断后确认的中线横坐标。
int16 mid_exact_x[MT9V03X_H];
uint8 mid_exact_ok[MT9V03X_H];

// 环岛场景下是否允许放宽边线爬取限制。
uint8 ring_relax_trace = 0;

/* ===== 图像处理参数配置 ===== */
#define GrayScale 256
// 灰度级数量的兼容别名，与 GrayScale 含义相同。
#define grayscale 256
uint16 hist[GrayScale] = {0};            /* 灰度直方图，统计每个灰度级出现的像素数量。 */
float P[GrayScale] = {0};                /* 各灰度级像素数量占总像素数的概率。 */
float PK[GrayScale] = {0};               /* 从最小灰度级累加得到的累计概率。 */
float MK[GrayScale] = {0};               /* 用于 Otsu 类间方差计算的累计灰度均值。 */
uint8 img_threshold;                     /* 当前帧由 Otsu 方法计算出的二值化阈值。 */
float imgsize;
// 逆透视鸟瞰图预览缓冲区。

uint8 ipm_preview[MT9V03X_H][MT9V03X_W];
/* ===== 图像处理参数配置 ===== */
#define SINGLE_EDGE_JUMP_DX       5
// 参数 SINGLE_EDGE_OUTWARD_RUN：单边连续向外跳变判定次数，当前值为 4。
#define SINGLE_EDGE_OUTWARD_RUN   4


/**
 * @brief 统计 256 级灰度直方图，计算累计概率，遍历阈值 5~244，选择类间方差最大的阈值。名字里的 Ostu 是拼写错误，实际是大津法 Otsu。
 */
uint8 Ostu(uint8 index[MT9V03X_H][MT9V03X_W])
{
    uint8 threshold = 0;
    imgsize = MT9V03X_W * MT9V03X_H;   /* 当前图像的像素总数。 */
    uint8 images_value_temp;           /* 当前像素的灰度值。 */

    float sumPK = 0;                   /* 累计概率总和，用于计算累计概率 PK。 */
    float sumMK = 0;                   /* 累计均值总和，用于计算累计均值 MK。 */
    float var = 0;                     /* 当前找到的最大类间方差。 */

    float vartmp = 0;                  /* 当前候选阈值对应的类间方差。 */

        // 循环遍历灰度直方图与二值化阈值，完成当前统计或筛选。
    for(uint16 i = 0; i < GrayScale; i++)
    {
        hist[i] = 0;
        P[i] = 0;
        PK[i] = 0;
        MK[i] = 0;
    }

        // 循环遍历统计 256 级灰度直方图，计算累计概率，遍历阈值 5~244，选择类间方差最大的阈值，完成当前统计或筛选。
    for(uint8 i = 0; i < MT9V03X_H; i++)
    {
        for(uint8 j = 0; j < MT9V03X_W; j++)
        {
            images_value_temp = index[i][j];
            hist[images_value_temp]++;
        }
    }

        // 循环遍历灰度直方图与二值化阈值，完成当前统计或筛选。
    for(uint16 i = 0; i < GrayScale; i++)
    {
        P[i] = (float) hist[i] / imgsize;   /* 灰度直方图，统计每个灰度级出现的像素数量。 */
        PK[i] = sumPK + P[i];               /* 各灰度级像素数量占总像素数的概率。 */
        sumPK = PK[i];
        MK[i] = sumMK + i * P[i];           /* 各灰度级像素数量占总像素数的概率。 */
        sumMK = MK[i];
    }

        // 循环遍历统计 256 级灰度直方图，计算累计概率，遍历阈值 5~244，选择类间方差最大的阈值，完成当前统计或筛选。
    for(uint8 i = 5; i < 245; i++)          /* 循环遍历统计 256 级灰度直方图，计算累计概率，遍历阈值 5~244，选择类间方差最大的阈值，完成当前统计或筛选。 */
    {
                // 从最小灰度级累加得到的累计概率。
        vartmp = ((MK[GrayScale - 1] * PK[i] - MK[i]) * (MK[GrayScale - 1] * PK[i] - MK[i])) / (PK[i] * (1 - PK[i]));

        if (vartmp > var)
        {
            var = vartmp;
            threshold = i; /* 当前找到的最佳二值化阈值。 */
        }
    }

    return threshold;
}

/* ===== 函数 set_image_twovalues ===== */
void set_image_twovalues(uint8 value)
{
    uint8 temp_value;
    uint8 i;
    uint8 j;
    uint8 lower_value;

    lower_value =
        (value > 10U)
        ? (uint8)(value - 10U)
        : 0U;
        // 循环遍历根据阈值生成黑白图，完成当前统计或筛选。
    for(i = 0; i < 40; i++)
    {
        for(j = 0; j < MT9V03X_W; j++)
        {
            temp_value = base_image[i][j];

            if(temp_value < value)
            {
                                // 二值化图像缓冲区；白色像素表示可行驶道路。
                image[i][j] = 0;
            }
            else
            {
                                // 二值化图像缓冲区；白色像素表示可行驶道路。
                image[i][j] = 255;
            }
        }
    }

        // 循环遍历根据阈值生成黑白图，完成当前统计或筛选。
    for(i = 40; i < MT9V03X_H; i++)
    {
        for(j = 0; j < MT9V03X_W; j++)
        {
            temp_value = base_image[i][j];

            if(temp_value < lower_value)
            {
                image[i][j] = 0;
            }
            else
            {
                image[i][j] = 255;
            }
        }
    }
}
/* ===== 函数 Limit_uint8 ===== */
    uint8 Limit_uint8(uint8 a, uint8 b, uint8 c)
    {
        if( (b >= a) && (b <= c) )
        {
            return b; /* 返回将 b 限制在 [a, c] 内，当前主视觉路径没有实际调用的判断或处理结果。 */
        }
        else if(b < a)
        {
            return a; /* 返回将 b 限制在 [a, c] 内，当前主视觉路径没有实际调用的判断或处理结果。 */
        }
        else if(b > c)
        {
            return c; /* 返回将 b 限制在 [a, c] 内，当前主视觉路径没有实际调用的判断或处理结果。 */
        }
        return 0;
    }
/* ===== 函数 find_jidian ===== */
void find_jidian(uint8 index[MT9V03X_H][MT9V03X_W])
{
        // 检查巡线种子点是否满足继续处理的条件。
    if (index[jidian_search_line - 1][MT9V03X_W/2] == 255 && index[jidian_search_line - 1][MT9V03X_W/2+1] == 255 && index[jidian_search_line - 1][MT9V03X_W/2-1] == 255) /* 检查巡线种子点是否满足继续处理的条件。 */
    {
                // 循环遍历旧版起点搜索，完成当前统计或筛选。
        for(uint8 j = MT9V03X_W/2; j > 0; j--)
        {
                        // 检查巡线种子点是否满足继续处理的条件。
            if(index[jidian_search_line - 1][j-1] == 0 && index[jidian_search_line - 1][j] == 255 && index[jidian_search_line - 1][j+1] == 255)
            {
                left_jidian = j;
                break;
            }
            if(j-1 == 1) /* 检查旧版起点搜索是否满足继续处理的条件。 */
            {
                left_jidian = 1;
                break;
            }
        }

                // 循环遍历旧版起点搜索，完成当前统计或筛选。
        for(uint8 j = MT9V03X_W/2; j < MT9V03X_W - 2; j++)
        {
                        // 检查巡线种子点是否满足继续处理的条件。
            if(index[jidian_search_line - 1][j-1] == 255 && index[jidian_search_line - 1][j] == 255 && index[jidian_search_line - 1][j+1] == 0)
            {
                right_jidian = j;
                break;
            }
            if(j+1 == MT9V03X_W - 2) /* 检查旧版起点搜索是否满足继续处理的条件。 */
            {
                right_jidian = MT9V03X_W - 2;
                break;
            }
        }
    }
        // 继续判断巡线种子点的其他情况。
    else if (index[jidian_search_line - 1][MT9V03X_W/4] == 255 && index[jidian_search_line - 1][MT9V03X_W/4+1] == 255 && index[jidian_search_line - 1][MT9V03X_W/4-1] == 255)
    {
                // 循环遍历旧版起点搜索，完成当前统计或筛选。
        for(uint8 j = MT9V03X_W/4; j > 0; j--)
        {
            if(index[jidian_search_line - 1][j-1] == 0 && index[jidian_search_line - 1][j] == 255 && index[jidian_search_line - 1][j+1] == 255)
            {
                left_jidian = j;
                break;
            }
            if(j-1 == 1)
            {
                left_jidian = 1;
                break;
            }
        }

                // 循环遍历旧版起点搜索，完成当前统计或筛选。
        for(uint8 j = MT9V03X_W/4; j < MT9V03X_W - 2; j++)
        {
            if(index[jidian_search_line - 1][j-1] == 255 && index[jidian_search_line - 1][j] == 255 && index[jidian_search_line - 1][j+1] == 0)
            {
                right_jidian = j;
                break;
            }
            if(j+1 == MT9V03X_W - 2)
            {
                right_jidian = MT9V03X_W - 2;
                break;
            }
        }
    }
        // 继续判断巡线种子点的其他情况。
    else if (index[jidian_search_line - 1][MT9V03X_W/4*3] == 255 && index[jidian_search_line - 1][MT9V03X_W/4*3+1] == 255 && index[jidian_search_line - 1][MT9V03X_W/4*3-1] == 255)
    {
                // 循环遍历旧版起点搜索，完成当前统计或筛选。
        for(uint8 j = MT9V03X_W/4*3; j > 0; j--)
        {
            if(index[jidian_search_line - 1][j-1] == 0 && index[jidian_search_line - 1][j] == 255 && index[jidian_search_line - 1][j+1] == 255)
            {
                left_jidian = j;
                break;
            }
            if(j-1 == 1)
            {
                left_jidian = 1;
                break;
            }
        }

                // 循环遍历旧版起点搜索，完成当前统计或筛选。
        for(uint8 j = MT9V03X_W/4*3; j < MT9V03X_W - 2; j++)
        {
            if(index[jidian_search_line - 1][j-1] == 255 && index[jidian_search_line - 1][j] == 255 && index[jidian_search_line - 1][j+1] == 0)
            {
                right_jidian = j;
                break;
            }
            if(j+1 == MT9V03X_W - 2)
            {
                right_jidian = MT9V03X_W - 2;
                break;
            }
        }
    }
}
/* ===== 全局图像数据 ===== */
uint8 search_end_line = 30;
uint8 road_mode = ROAD_MODE_NORMAL;

// 参数 CROSS_HOLD_FRAMES：十字锁定状态保持的帧数，当前值为 10U。
#define CROSS_HOLD_FRAMES 10U

// 是否处于十字锁定状态。
uint8 cross_lock_active = 0;
uint8 cross_hold_frames = 0;/* 十字锁定的剩余保持帧数。 */

// 参数 IPM_DISCARD_CROSS_HOLD_FRAMES：IPM 丢弃边线后抑制十字检测的保持帧数，当前值为 10U。
#define IPM_DISCARD_CROSS_HOLD_FRAMES   10U
// IPM 丢弃边线的保持帧数加一，用于安排保持状态的退出时刻。
#define IPM_DISCARD_CROSS_HOLD_ARM      (IPM_DISCARD_CROSS_HOLD_FRAMES + 1U)
// 环岛工作期间或 IPM 丢弃保持期间，暂时抑制十字检测。
#define RING_SUPPRESS_CROSS   ((ring_state != 0) || (ipm_discard_hold != 0))
// 定义crossdbghit变量，用于当前图像处理过程。
uint8 cross_dbg_hit = 0;
Point2D_u8 Left_Edge_Pts[MAX_EDGE_PTS];
uint16    Left_Edge_Count = 0;
Point2D_u8 Right_Edge_Pts[MAX_EDGE_PTS];
uint16    Right_Edge_Count = 0;

// 更新旧版起点搜索对应的数组状态。
static const int8 DIR_X[8] =
{
    0, 1, 1, 1, 0, -1, -1, -1
};

static const int8 DIR_Y[8] =
{
    -1, -1, 0, 1, 1, 1, 0, -1
};

/* ===== 图像处理参数配置 ===== */
#define EDGE_LEFT          0
// 参数 EDGE_RIGHT：用于配置边线右侧，当前值为 1。
#define EDGE_RIGHT         1
// 参数 EDGE_WHITE：用于配置边线白色，当前值为 255。
#define EDGE_WHITE         255
// 参数 EDGE_BLACK：用于配置边线黑色，当前值为 0。
#define EDGE_BLACK         0

uint8 ipm_discard_active = 0;    /* 是否暂时丢弃 IPM 补出的某一条边线。 */
uint8 ipm_discard_side   = 0;    /* 当前被 IPM 丢弃的边线方向。 */
uint8 ipm_discard_hold   = 0;    /* IPM 丢弃状态的剩余保持帧数。 */
#define EDGE_TOP_Y              12
// 参数 EDGE_MAX_NON_UP：普通爬线允许连续非向上移动的最大次数，当前值为 10。
#define EDGE_MAX_NON_UP         10
// 参数 EDGE_MIN_RUN：边线种子要求的最短连续白色段长度，当前值为 8。
#define EDGE_MIN_RUN             8
// 参数 EDGE_MAX_TOTAL_OUTWARD_DX：普通边线允许的累计向外偏移，当前值为 10。
#define EDGE_MAX_TOTAL_OUTWARD_DX            10
// 参数 EDGE_RECOVER_ROWS：边线断点恢复允许跨越的行数，当前值为 6。
#define EDGE_RECOVER_ROWS              6
// 参数 EDGE_RECOVER_RADIUS：边线断点恢复的横向搜索半径，当前值为 10。
#define EDGE_RECOVER_RADIUS            10
// 参数 EDGE_RECOVER_MAX_DX_TOTAL：边线断点恢复允许的累计横向偏移，当前值为 14。
#define EDGE_RECOVER_MAX_DX_TOTAL      14

// 参数 EDGE_MAX_OUTWARD_STREAK：普通边线允许连续向外移动的最大次数，当前值为 2。
#define EDGE_MAX_OUTWARD_STREAK        2

// 参数 RING_MAX_NON_UP：环岛爬线允许连续非向上移动的最大次数，当前值为 60。
#define RING_MAX_NON_UP             60
#define RING_RECOVER_ROWS           5    /* 参数 RING_RECOVER_ROWS：环岛边线断点恢复允许跨越的行数，当前值为 5。 */
#define RING_MAX_OUTWARD_STREAK     60
// 参数 CROSS_EDGE_MAX_OUTWARD_STREAK：十字边线允许连续向外跳变的最大次数，当前值为 4。
#define CROSS_EDGE_MAX_OUTWARD_STREAK        4
// 参数 CROSS_EDGE_RECOVER_ROWS：十字断线恢复允许跨越的最大行数，当前值为 24。
#define CROSS_EDGE_RECOVER_ROWS              24
// 参数 CROSS_EDGE_RECOVER_RADIUS：十字断线恢复时允许的横向搜索半径，当前值为 10。
#define CROSS_EDGE_RECOVER_RADIUS            10
// 参数 CROSS_EDGE_RECOVER_MAX_DX_TOTAL：十字断线恢复允许的累计横向偏移，当前值为 14。
#define CROSS_EDGE_RECOVER_MAX_DX_TOTAL      14
// 参数 CROSS_EDGE_MAX_TOTAL_OUTWARD_DX：十字边线允许的累计向外偏移，当前值为 20。
#define CROSS_EDGE_MAX_TOTAL_OUTWARD_DX      20

// 参数 EDGE_FIT_WINDOW：用于配置边线拟合窗口，当前值为 20。
#define EDGE_FIT_WINDOW                20
// 参数 EDGE_MIN_FIT_POINTS：边线拟合所需的最少点数，当前值为 6。
#define EDGE_MIN_FIT_POINTS            6
// 参数 EDGE_FILL_MAX_ROWS：旧版单边补线允许向上延伸的最大行数，当前值为 85。
#define EDGE_FILL_MAX_ROWS             85
// 参数 EDGE_MIN_TRACE_SPAN：边线拟合所需的最小纵向跨度，当前值为 10。
#define EDGE_MIN_TRACE_SPAN            10

// 参数 EDGE_MAX_FIT_RMS_SQ：边线直线拟合允许的最大均方误差，当前值为 9.0f。
#define EDGE_MAX_FIT_RMS_SQ            9.0f

// 参数 EDGE_START_BOTTOM_BAND：从底部选择起点时使用的搜索带高度，当前值为 24。
#define EDGE_START_BOTTOM_BAND  24
// 参数 SINGLE_SEED_MIN_Y：单边种子搜索允许的最低行，当前值为 80。
#define SINGLE_SEED_MIN_Y       80

// 参数 ZEBRA_SEED_BASE_Y：斑马线种子搜索的基准行，当前值为 70。
#define ZEBRA_SEED_BASE_Y       70
// 参数 SINGLE_SEED_MIN_X_GAP：单边种子与图像中心要求的最小横向距离，当前值为 8。
#define SINGLE_SEED_MIN_X_GAP        8

// 参数 SEED_MIN_WHITE_PERCENT：巡线种子区间要求的最小白色占比，当前值为 55。
#define SEED_MIN_WHITE_PERCENT       55

// 参数 BLACK_DOT_MIN_WHITE_NEIGHBORS：小黑洞填补所需的最少白色邻居数，当前值为 7。
#define BLACK_DOT_MIN_WHITE_NEIGHBORS  7

// 参数 IPM_MIN_KNOWN_POINTS：IPM 单边补线所需的最少已知边线点，当前值为 8。


#define IPM_MIN_KNOWN_POINTS    8
// 参数 IPM_MIN_EDGE_COUNT_DIFF：选择 IPM 基准边线时要求的最小点数差，当前值为 25U。
#define IPM_MIN_EDGE_COUNT_DIFF     25U
#define IPM_SINGLE_SIDE_FEW_ROWS_MAX   12U   /* 参数 IPM_SINGLE_SIDE_FEW_ROWS_MAX：IPM 单边补线时允许的少量有效行上限，当前值为 12U。 */
#define IPM_TRIGGER_MIN_MID_ROWS    35U
// 参数 IPM_KNOWN_NONE：用于配置逆透视已知NONE，当前值为 0U。
#define IPM_KNOWN_NONE              0U
// 参数 IPM_KNOWN_LEFT：用于配置逆透视已知左侧，当前值为 1U。
#define IPM_KNOWN_LEFT              1U
// 参数 IPM_KNOWN_RIGHT：用于配置逆透视已知右侧，当前值为 2U。
#define IPM_KNOWN_RIGHT             2U
// 参数 IPM_BASE_TO_OPPOSITE_TOP_GAP：IPM 基准边线到对侧顶部的最大允许间隔，当前值为 30U。
#define IPM_BASE_TO_OPPOSITE_TOP_GAP 30U

// 参数 IPM_COMPLETENESS_GUARD：IPM 补线完整性保护开关，当前值为 1。
#define IPM_COMPLETENESS_GUARD      1

// 参数 IPM_DISCARD_MAX_LOSS_ROWS：IPM 丢弃边线时允许连续丢失的最大行数，当前值为 0U。
#define IPM_DISCARD_MAX_LOSS_ROWS   0U
// 参数 IPM_MERGE_MAX_GAP_X：IPM 中线与普通中线合并时允许的最大横向差，当前值为 6。
#define IPM_MERGE_MAX_GAP_X         6
#define IPM_MERGE_NEAREST_MAX_ROWS   30   /* 参数 IPM_MERGE_NEAREST_MAX_ROWS：IPM 中线向附近有效中线合并的搜索行数，当前值为 30。 */
// 参数 IPM_CHECK_TWO_LINES_MERGED：检查左右线是否粘在一起。
#define IPM_CHECK_TWO_LINES_MERGED   1
// 参数 IPM_PAIR_MIN_GAP_X：IPM 左右边线之间要求的最小横向间隔，当前值为 3。
#define IPM_PAIR_MIN_GAP_X           3
/*
 * 生成 0,+1,-1,+2,-2,... 的搜索顺序，优先寻找最近横向位置。
 */
static int16 recover_offset(uint8 k)
{
    if(k == 0)
    {
        return 0;
    }

        // 检查旧版起点搜索是否满足继续处理的条件。
    if(k & 1)
    {
        return (int16)((k + 1) / 2);
    }

    return -(int16)(k / 2);
}

/**
 * @brief 判断白色道路像素是否贴着外侧黑色区域。左边检查 x-1，右边检查 x+1。
 */

static uint8 is_edge_pixel(
    uint8 img[MT9V03X_H][MT9V03X_W],
    int16 x,
    int16 y,
    uint8 side);

/**
 * @brief 检查候选点是否已经出现在爬线点数组中。
 */
static uint8 point_already_used(
    const Point2D_u8 *points,
    uint16 count,
    uint8 x,
    uint8 y);
/**
 * @brief 普通爬线断点恢复。默认最多跨 6 行，横向半径 10；环岛短边最多跨 5 行。
 */
static uint8 recover_edge_gap(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 *x,
    uint8 *y,
    uint8 side,
    const Point2D_u8 *points,
    uint16 point_count)
{
    uint8 row_step;
    uint8 k;

    int16 dx;
    int16 test_x;
    int16 test_y;

        // 逐行遍历普通爬线断点恢复。
        for(row_step = 1;
            row_step <= (RING_SHORT_TIGHT_RECOVER(side) ? RING_RECOVER_ROWS : EDGE_RECOVER_ROWS);
            row_step++)
    {
        test_y = (int16)(*y) - row_step;

        if(test_y <= EDGE_TOP_Y)
        {
            break;
        }

                // 循环遍历普通爬线断点恢复，完成当前统计或筛选。
        for(k = 0;
            k < (uint8)(2 * EDGE_RECOVER_RADIUS + 1);
            k++)
        {
            dx = recover_offset(k);

                        // 检查左右边线的有效性是否满足继续处理的条件。
            if(dx > EDGE_RECOVER_MAX_DX_TOTAL ||
               dx < -EDGE_RECOVER_MAX_DX_TOTAL)
            {
                continue;
            }

                        // 检查普通爬线断点恢复是否满足继续处理的条件。
            if(dx > (int16)(row_step * 2 + 2) ||
               dx < -(int16)(row_step * 2 + 2))
            {
                continue;
            }
            test_x = (int16)(*x) + dx;

                        // 检查普通爬线断点恢复是否满足继续处理的条件。
            if(test_x < 2 ||
               test_x >= MT9V03X_W - 2 ||
               test_y < 1 ||
               test_y >= MT9V03X_H - 1)
            {
                continue;
            }

                        // 检查左右边线的有效性是否可用；条件不满足时进入回退分支。
            if(!is_edge_pixel(
                    img,
                    test_x,
                    test_y,
                    side))
            {
                continue;
            }

                        // 检查普通爬线断点恢复是否满足继续处理的条件。
            if(point_already_used(
                    points,
                    point_count,
                    (uint8)test_x,
                    (uint8)test_y))
            {
                continue;
            }

                        /*
             * 更新普通爬线断点恢复相关变量。
             */
            *x = (uint8)test_x;
            *y = (uint8)test_y;

            return 1;
        }
    }

    return 0;
}



/**
 * @brief 十字爬线时限制横向偏移，防止从一条边突然跑到另一条边。
 */
static uint8 cross_candidate_within_seed_budget(
    uint8 side,
    uint8 seed_x,
    int16 candidate_x)
{
    if(side == EDGE_LEFT)
    {
        if(candidate_x <
           (int16)seed_x -
           CROSS_EDGE_MAX_TOTAL_OUTWARD_DX)
        {
            return 0;
        }
    }
    else
    {
        if(candidate_x >
           (int16)seed_x +
           CROSS_EDGE_MAX_TOTAL_OUTWARD_DX)
        {
            return 0;
        }
    }

    return 1;
}
/**
 * @brief 十字专用断点恢复。允许跨 24 行，但只能向内侧走，且总横向偏移受限。
 */
static uint8 recover_edge_gap_cross(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 *x,
    uint8 *y,
    uint8 side,
    uint8 seed_x,
    const Point2D_u8 *points,
    uint16 point_count)
{
    uint8 row_step;
    uint8 k;

    int16 dx;
    int16 test_x;
    int16 test_y;

    for(row_step = 1;
        row_step <= CROSS_EDGE_RECOVER_ROWS;
        row_step++)
    {
        test_y = (int16)(*y) - row_step;

        if(test_y <= EDGE_TOP_Y)
        {
            break;
        }

        for(k = 0;
            k < (uint8)(2 * CROSS_EDGE_RECOVER_RADIUS + 1);
            k++)
        {
            dx = recover_offset(k);

            if(dx > CROSS_EDGE_RECOVER_MAX_DX_TOTAL ||
               dx < -CROSS_EDGE_RECOVER_MAX_DX_TOTAL)
            {
                continue;
            }

            if(dx > (int16)(row_step * 2 + 2) ||
               dx < -(int16)(row_step * 2 + 2))
            {
                continue;
            }

            test_x = (int16)(*x) + dx;

            if(test_x < 2 ||
               test_x >= MT9V03X_W - 2 ||
               test_y < 1 ||
               test_y >= MT9V03X_H - 1)
            {
                continue;
            }

            if(!cross_candidate_within_seed_budget(
                    side,
                    seed_x,
                    test_x))
            {
                continue;
            }

                        // 检查左右边线的有效性是否满足继续处理的条件。
            if((side == EDGE_LEFT &&
                test_x < (int16)(*x)) ||
               (side == EDGE_RIGHT &&
                test_x > (int16)(*x)))
            {
                continue;
            }

            if(!is_edge_pixel(
                    img,
                    test_x,
                    test_y,
                    side))

            {
                continue;
            }

            if(point_already_used(
                    points,
                    point_count,
                    (uint8)test_x,
                    (uint8)test_y))
            {
                continue;
            }

            *x = (uint8)test_x;
            *y = (uint8)test_y;

            return 1;
        }
    }

    return 0;
}

uint8 edge_start_y = jidian_search_line - 1;

static const int8 left_order[8][2] = {
    { 0,-1}, { 1,-1}, {-1,-1}, { 1, 0},
    {-1, 0}, { 1, 1}, {-1, 1}, { 0, 1}
};

static const int8 right_order[8][2] = {
    { 0,-1}, {-1,-1}, { 1,-1}, {-1, 0},
    { 1, 0}, {-1, 1}, { 1, 1}, { 0, 1}
};

/**
 * @brief 判断白色道路像素是否贴着外侧黑色区域。左边检查 x-1，右边检查 x+1。
 */
static uint8 is_edge_pixel(
    uint8 img[MT9V03X_H][MT9V03X_W],
    int16 x,
    int16 y,
    uint8 side)
{
    uint8 black_count = 0;

    if(x < 2 ||
       x >= MT9V03X_W - 2 ||
       y < 1 ||
       y >= MT9V03X_H - 1)
    {
        return 0;
    }

    if(img[y][x] != EDGE_WHITE)
    {
        return 0;
    }

    if(side == EDGE_LEFT)
    {
        if(img[y][x - 1] == EDGE_BLACK)
        {
            black_count++;
        }

        if(img[y - 1][x - 1] == EDGE_BLACK)
        {
            black_count++;
        }

        if(img[y + 1][x - 1] == EDGE_BLACK)
        {
            black_count++;
        }
    }
    else
    {
        if(img[y][x + 1] == EDGE_BLACK)
        {
            black_count++;
        }

        if(img[y - 1][x + 1] == EDGE_BLACK)
        {
            black_count++;
        }

        if(img[y + 1][x + 1] == EDGE_BLACK)
        {
            black_count++;
        }
    }

        /*
     * 返回判断白色道路像素是否贴着外侧黑色区域的判断或处理结果。
     */
    return (black_count >= 1);
}

/**
 * @brief 在某行寻找连续白色区域，长度至少 8；跨越图像中心的区域会得到额外分数。
 */
static uint8 find_lane_run(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 y,
    uint8 *left,
    uint8 *right)
{
    uint16 best_score = 0;
    uint8 best_left = 0;
    uint8 best_right = 0;
    int16 x = 2;

    while(x < MT9V03X_W - 2)
    {
        while(x < MT9V03X_W - 2 && img[y][x] != EDGE_WHITE)
            x++;

        int16 begin = x;

        while(x < MT9V03X_W - 2 && img[y][x] == EDGE_WHITE)
            x++;

        int16 end = x - 1;
        int16 length = end - begin + 1;

        if(length >= EDGE_MIN_RUN &&
           begin > 2 &&
           end < MT9V03X_W - 3)
        {
            uint16 score = (uint16)length;

            if(begin <= MID_W && end >= MID_W)
                score += MT9V03X_W;

            if(score > best_score)
            {
                best_score = score;
                best_left = (uint8)begin;
                best_right = (uint8)end;
            }
        }
    }

    if(best_score == 0)
        return 0;

    *left = best_left;
    *right = best_right;
    return 1;
}
/**
 * @brief 检查候选白色区间内部至少 55% 为白色，防止把局部白点当成整条路。
 */
static uint8 seed_middle_is_road(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 y,
    uint8 left_x,
    uint8 right_x)
{
    uint16 x;
    uint16 road_count = 0;
    uint16 total_count = 0;

    if(img == 0 ||
       y >= MT9V03X_H ||
       left_x >= right_x)
    {
        return 0;
    }

        // 逐列遍历检查候选白色区间内部至少 55% 为白色，防止把局部白点当成整条路。
    for(x = (uint16)left_x + 2U;
        x + 2U < (uint16)right_x;
        x++)
    {
        total_count++;

        if(img[y][x] != EDGE_BLACK)
        {
            road_count++;
        }
    }

    if(total_count == 0)
    {
        return 0;
    }

    return
        ((road_count * 100U) / total_count >=
         SEED_MIN_WHITE_PERCENT);
}

/**
 * @brief 检查候选区间在连续 3 行中是否都足够白。
 */
static uint8 seed_has_vertical_support(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 y,
    uint8 left_x,
    uint8 right_x)
{
    int16 yy;
    uint16 x;
    uint16 white_count;
    uint16 total_count;

    uint8 checked_rows;
    uint8 good_rows;
    uint8 d;

    if(img == 0 ||
       left_x >= right_x ||
       y >= MT9V03X_H)
    {
        return 0;
    }

    checked_rows = 0;
    good_rows = 0;

        // 循环遍历检查候选区间在连续 3 行中是否都足够白，完成当前统计或筛选。
    for(d = 0; d < 3; d++)
    {
        yy = (int16)y - d;

        if(yy <= EDGE_TOP_Y)
        {
            continue;
        }

        white_count = 0;
        total_count = 0;

        for(x = (uint16)left_x + 2U;
            x + 2U < (uint16)right_x;
            x++)
        {
            total_count++;

            if(img[yy][x] == EDGE_WHITE)
            {
                white_count++;
            }
        }

        if(total_count == 0)
        {
            continue;
        }

        checked_rows++;

                // 检查检查候选区间在连续 3 行中是否都足够白是否满足继续处理的条件。
        if((white_count * 100U) /
           total_count >= 50U)
        {
            good_rows++;
        }
    }

        // 检查检查候选区间在连续 3 行中是否都足够白是否满足继续处理的条件。
    if(checked_rows >= 3 &&
       good_rows >= 3)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief 从底部向上寻找稳定、足够宽且尽量包含中心的白色通道，作为左右共同起点。
 */
static uint8 choose_start_row(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 min_y,
    uint8 max_y,
    uint8 *best_y,
    uint8 *best_left,
    uint8 *best_right)
{
    int16 candidate_y;

    uint16 best_score;
    uint16 candidate_width;

    uint8 candidate_left;
    uint8 candidate_right;

    if(img == 0 ||
       best_y == 0 ||
       best_left == 0 ||
       best_right == 0 ||
       min_y > max_y)
    {
        return 0;
    }

    best_score = 0;

    *best_y = 0;
    *best_left = 0;
    *best_right = 0;

    for(candidate_y = (int16)max_y;
        candidate_y >= (int16)min_y;
        candidate_y--)
    {
        if(!find_lane_run(
                img,
                (uint8)candidate_y,
                &candidate_left,
                &candidate_right))
        {
            continue;
        }

        if(candidate_right <= candidate_left)
        {
            continue;
        }

        candidate_width =
            (uint16)candidate_right -
            (uint16)candidate_left;

        if(candidate_width < EDGE_MIN_RUN)
        {
            continue;
        }
                // 检查巡线种子点是否可用；条件不满足时进入回退分支。
        if(!seed_has_vertical_support(
                img,
                (uint8)candidate_y,
                candidate_left,
                candidate_right))
        {
            continue;
        }
                // 检查巡线种子点是否可用；条件不满足时进入回退分支。
        if(!seed_middle_is_road(
                img,
                (uint8)candidate_y,
                candidate_left,
                candidate_right))
        {
            continue;
        }
                // 以下代码继续处理从底部向上寻找稳定、足够宽且尽量包含中心的白色通道，作为左右共同起点。
        {
            uint16 score;

            score = candidate_width;

            if(candidate_left <= MID_W &&
               candidate_right >= MID_W)
            {
                score += MT9V03X_W;
            }

                        /*
             * 执行从底部向上寻找稳定、足够宽且尽量包含中心的白色通道，作为左右共同起点相关操作。
             */
            score += (uint16)candidate_y;

            if(score > best_score)
            {
                best_score = score;

                *best_y = (uint8)candidate_y;
                *best_left = candidate_left;
                *best_right = candidate_right;
            }
        }
    }

    if(best_score == 0)
    {
        return 0;
    }

    return 1;
}

/**
 * @brief 从图像中心向指定一侧扫描，找到第一个边缘像素。
 */
static uint8 find_single_side_seed(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 y,
    uint8 side,
    uint8 *seed_x)
{
    int16 x;

    if(img == 0 ||
       seed_x == 0 ||
       y <= EDGE_TOP_Y ||
       y >= MT9V03X_H - 1)
    {
        return 0;
    }

        // 检查左右边线的有效性是否满足继续处理的条件。
    if(side == EDGE_LEFT)
    {
        for(x = MID_W; x >= 2; x--)
        {
            if(is_edge_pixel(img, x, y, side))
            {
                *seed_x = (uint8)x;
                return 1;
            }
        }
    }
        // 处理从图像中心向指定一侧扫描，找到第一个边缘像素不满足上述条件时的回退情况。
    else
    {
        for(x = MID_W; x < MT9V03X_W - 2; x++)
        {
            if(is_edge_pixel(img, x, y, side))
            {
                *seed_x = (uint8)x;
                return 1;
            }
        }
    }

    return 0;
}
// 参数 CROSS_WHITE_MIN_PERCENT：十字白区判定所需的最小白色占比，当前值为 75。
#define CROSS_WHITE_MIN_PERCENT       75
// 参数 CROSS_WHITE_RUN_MIN_PERCENT：十字白区连续段的最小白色占比，当前值为 55。
#define CROSS_WHITE_RUN_MIN_PERCENT   55
// 参数 CROSS_MIN_SUPPORT_ROWS：十字判定所需的最少支持行数，当前值为 0。
#define CROSS_MIN_SUPPORT_ROWS        0
// 参数 CROSS_MAX_SIDE_ROW_DIFF：十字场景左右边线允许的最大行数差，当前值为 45。
#define CROSS_MAX_SIDE_ROW_DIFF      45
// 参数 CROSS_MIN_MISSING_ROWS_RING：用于配置十字最小缺失行数环岛，当前值为 4。
#define CROSS_MIN_MISSING_ROWS_RING     4     /* 参数 CROSS_MIN_MISSING_ROWS_RING：用于配置十字最小缺失行数环岛，当前值为 4。 */
#define CROSS_WHITE_MIN_PERCENT_RING    70    /* 参数 CROSS_WHITE_MIN_PERCENT_RING：用于配置十字白色最小百分比环岛，当前值为 70。 */

// 定义crossmissingrowsth变量，用于当前图像处理过程。
static uint8 cross_missing_rows_th = CROSS_MIN_MISSING_ROWS;
static uint8 cross_white_pct_th    = CROSS_WHITE_MIN_PERCENT;
/**
 * @brief 在指定行从中心向左右寻找第一条边线。
 */
static uint8 find_side_edge_in_row(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 y,
    uint8 side,
    uint8 *edge_x)
{
    int16 x;

    if(img == 0 ||
       edge_x == 0 ||
       y <= EDGE_TOP_Y ||
       y >= MT9V03X_H - 1)
    {
        return 0;
    }

    if(side == EDGE_LEFT)
    {
        for(x = MID_W;
            x >= 2;
            x--)
        {
            if(is_edge_pixel(
                    img,
                    x,
                    y,
                    EDGE_LEFT))
            {
                *edge_x = (uint8)x;
                return 1;
            }
        }
    }
    else
    {
        for(x = MID_W;
            x < MT9V03X_W - 2;
            x++)
        {
            if(is_edge_pixel(
                    img,
                    x,
                    y,
                    EDGE_RIGHT))
            {
                *edge_x = (uint8)x;
                return 1;
            }
        }
    }

    return 0;
}
/**
 * @brief 统计一段 y 范围内某一侧能扫到边线的行数。
 */
static uint16 count_image_side_rows(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 side,
    uint8 top_y,
    uint8 bottom_y)
{
    int16 y;
    uint8 x;
    uint16 n = 0;

    for(y = (int16)top_y; y <= (int16)bottom_y; y++)
    {
        if(find_side_edge_in_row(img, (uint8)y, side, &x))
        {
            n++;
        }
    }

    return n;
}
/**
 * @brief 判断一行是否几乎全白，同时要求最长白色连续段达到一定比例。
 */
static uint8 row_is_nearly_all_white(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 y)
{
    uint16 x;
    uint16 white_count;
    uint16 run_count;
    uint16 longest_run;
    uint16 total_count;

    if(img == 0 ||
       y <= EDGE_TOP_Y ||
       y >= MT9V03X_H - 1)
    {
        return 0;
    }

    white_count = 0;
    run_count = 0;
    longest_run = 0;

    total_count = (uint16)MT9V03X_W - 4U;

    for(x = 2;
        x < MT9V03X_W - 2;
        x++)
    {
        if(img[y][x] == EDGE_WHITE)
        {
            white_count++;
            run_count++;

            if(run_count > longest_run)
            {
                longest_run = run_count;
            }
        }
        else
        {
            run_count = 0;
        }
    }

        // 检查判断一行是否几乎全白，同时要求最长白色连续段达到一定比例是否满足继续处理的条件。
    if((white_count * 100U) <
       (total_count * cross_white_pct_th))          /* 执行判断一行是否几乎全白，同时要求最长白色连续段达到一定比例相关操作。 */
    {
        return 0;
    }

    if((longest_run * 100U) <
       (total_count * CROSS_WHITE_RUN_MIN_PERCENT))
    {
        return 0;
    }

    return 1;
}
/**
 * @brief 包装单边种子查找；IPM 丢弃某一边时禁止重新使用该边。
 */
static uint8 cross_side_seed_ok(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 y,
    uint8 side,
    uint8 *x)
{
    if((ring_state == 0U) &&
       ((ipm_discard_active != 0U) ||
        (ipm_discard_hold != 0U)) &&
       (side == ipm_discard_side))
    {
        return 0;
    }

    return find_single_side_seed(img, y, side, x);
}
/**
 * @brief 十字白区检测。寻找连续“两侧都没有边线并且几乎全白”的行。
 */
static uint8 detect_cross_white_gap(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 top_y,
    uint8 bottom_y,
    uint8 *gap_top_y)
{
    int16 y;

    uint8 left_x;
    uint8 right_x;

    uint8 left_found;
    uint8 right_found;

    uint8 missing_streak;
    uint8 support_rows;

    if(img == 0 ||
       gap_top_y == 0 ||
       top_y >= bottom_y)
    {
        return 0;
    }

    missing_streak = 0;

    for(y = (int16)bottom_y;
        y >= (int16)top_y;
        y--)
    {
        if(ring_state != 0U)
        {
            left_found =
                find_side_edge_in_row(
                    img,
                    (uint8)y,
                    EDGE_LEFT,
                    &left_x);

            right_found =
                find_side_edge_in_row(
                    img,
                    (uint8)y,
                    EDGE_RIGHT,
                    &right_x);
        }
        else
        {
            left_found =
                cross_side_seed_ok(
                    img,
                    (uint8)y,
                    EDGE_LEFT,
                    &left_x);

            right_found =
                cross_side_seed_ok(
                    img,
                    (uint8)y,
                    EDGE_RIGHT,
                    &right_x);
        }

        if(!left_found &&
           !right_found &&
           row_is_nearly_all_white(
               img,
               (uint8)y))
        {
            if(missing_streak < 255)
            {
                missing_streak++;
            }

            if(missing_streak >=
               cross_missing_rows_th)
            {
                *gap_top_y = (uint8)y;
                break;
            }
        }
        else
        {
            missing_streak = 0;
        }
    }

    if(missing_streak <
       cross_missing_rows_th)
    {
        return 0;
    }

    support_rows = 0;

    for(y = (int16)bottom_y;
        y >= (int16)top_y;
        y--)
    {
        if(ring_state != 0U)
        {
            left_found =
                find_single_side_seed(
                    img,
                    (uint8)y,
                    EDGE_LEFT,
                    &left_x);

            right_found =
                find_single_side_seed(
                    img,
                    (uint8)y,
                    EDGE_RIGHT,
                    &right_x);
        }
        else
        {
            left_found =
                cross_side_seed_ok(
                    img,
                    (uint8)y,
                    EDGE_LEFT,
                    &left_x);

            right_found =
                cross_side_seed_ok(
                    img,
                    (uint8)y,
                    EDGE_RIGHT,
                    &right_x);
        }

        if(left_found &&
           right_found &&
           left_x < right_x)
        {
            if(support_rows < 255)
            {
                support_rows++;
            }
        }
    }

    if(support_rows < CROSS_MIN_SUPPORT_ROWS)
    {
        return 0;
    }

    return 1;
}
/**
 * @brief 检查单边种子在向上 3 行、左右 ±2 像素范围内是否有延续。
 */
static uint8 single_seed_has_upward_support(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 y,
    uint8 side,
    uint8 x)
{
    int16 yy;
    int16 xx;

    uint8 d;
    uint8 good_rows;
    uint8 row_good;

    if(img == 0)
    {
        return 0;
    }

    if(y <= EDGE_TOP_Y + 1)
    {
        return 0;
    }

    good_rows = 0;

        // 循环遍历检查单边种子在向上 3 行、左右 ±2 像素范围内是否有延续，完成当前统计或筛选。
    for(d = 0; d < 3; d++)
    {
        yy = (int16)y - (int16)d;

        if(yy <= EDGE_TOP_Y)
        {
            continue;
        }

        row_good = 0;

                // 逐列遍历检查单边种子在向上 3 行、左右 ±2 像素范围内是否有延续。
        for(xx = (int16)x - 2;
            xx <= (int16)x + 2;
            xx++)
        {
            if(xx < 1 ||
               xx >= MT9V03X_W - 1)
            {
                continue;
            }

            if(is_edge_pixel(
                    img,
                    (uint8)xx,
                    (uint8)yy,
                    side))
            {
                row_good = 1;
                break;
            }
        }

        if(row_good)
        {
            good_rows++;
        }
    }

        // 检查检查单边种子在向上 3 行、左右 ±2 像素范围内是否有延续是否满足继续处理的条件。
    if(good_rows >= 2)
    {
        return 1;
    }

    return 0;
}
/**
 * @brief 严格种子要求边线外侧连续两个黑像素。
 */
static uint8 find_side_seed_strict(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 y,
    uint8 side,
    uint8 *seed_x);

/**
 * @brief 从底部向上选择单边种子；环岛时使用更宽松条件。
 */
static uint8 choose_single_side_seed(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 min_y,
    uint8 max_y,
    uint8 side,
    uint8 *seed_y,
    uint8 *seed_x)
{
    int16 y;
    int16 min_y_used;
    uint8 x;
    uint8 found;

    if(img == 0 ||
       seed_y == 0 ||
       seed_x == 0 ||
       min_y > max_y)
    {
        return 0;
    }

    *seed_y = 0;
    *seed_x = 0;

        // 环岛状态机的当前状态。
    min_y_used = (ring_state != 0U)
               ? (int16)(EDGE_TOP_Y + 2)
               : (int16)min_y;

    if(min_y_used > (int16)max_y)
    {
        return 0;
    }

    for(y = (int16)max_y;
        y >= min_y_used;
        y--)
    {
        if(ring_state != 0U)
        {
            found = find_single_side_seed(
                        img,
                        (uint8)y,
                        side,
                        &x);
        }
        else
        {
            found = find_side_seed_strict(
                        img,
                        (uint8)y,
                        side,
                        &x);
        }

        if(found)
        {
            if(!single_seed_has_upward_support(
                    img,
                    (uint8)y,
                    side,
                    x))
            {
                continue;
            }

            *seed_y = (uint8)y;
            *seed_x = x;
            return 1;
        }
    }

    return 0;
}
/**
 * @brief 严格种子要求边线外侧连续两个黑像素。
 */
static uint8 find_side_seed_strict(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 y,
    uint8 side,
    uint8 *seed_x)
{
    int16 x;

    if(img == 0 ||
       seed_x == 0 ||
       y <= EDGE_TOP_Y ||
       y >= MT9V03X_H - 1)
    {
        return 0;
    }

        // 检查左右边线的有效性是否满足继续处理的条件。
    if(side == EDGE_LEFT)
    {
        for(x = MID_W; x >= 2; x--)
        {
            if((img[y][x]     == EDGE_WHITE) &&
               (img[y][x - 1] == EDGE_BLACK) &&
               (img[y][x - 2] == EDGE_BLACK))
            {
                *seed_x = (uint8)x;
                return 1;
            }
        }
    }
        // 处理严格种子要求边线外侧连续两个黑像素不满足上述条件时的回退情况。
    else
    {
        for(x = MID_W; x < MT9V03X_W - 2; x++)
        {
            if((img[y][x]     == EDGE_WHITE) &&
               (img[y][x + 1] == EDGE_BLACK) &&
               (img[y][x + 2] == EDGE_BLACK))
            {
                *seed_x = (uint8)x;
                return 1;
            }
        }
    }

    return 0;
}

/**
 * @brief 检查候选点是否已经出现在爬线点数组中。
 */
static uint8 point_already_used(
    const Point2D_u8 *points,
    uint16 count,
    uint8 x,
    uint8 y)
{
    uint16 i;

    for(i = 0; i < count; i++)
    {
        if(points[i].x == x && points[i].y == y)
            return 1;
    }

    return 0;
}

/**
 * @brief 普通八邻域爬线核心。
 */
static uint16 trace_edge(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 start_x,
    uint8 start_y,
    uint8 side,
    Point2D_u8 *points,
    uint8 *row_line,
    uint8 *top_y)
{int16 max_backtrack_y;
    uint8 x = start_x;
    uint8 y = start_y;

    uint8 non_up_count = 0;
    uint8 outward_streak = 0;

    uint16 count = 0;
        /*
     * 更新巡线种子点相关变量。
     */
    max_backtrack_y =
        (int16)jidian_search_line - 1;
    *top_y = start_y;

    while(count < MAX_EDGE_PTS)
    {
        uint8 found = 0;
        uint8 k;

        int16 nx;
        int16 ny;
        int16 dx_move;

        uint8 moving_outward;

                // 检查左右边线的有效性是否可用；条件不满足时进入回退分支。
        if(!is_edge_pixel(img, x, y, side))
        {
            uint8 recovered;

            recovered = recover_edge_gap(
                img,
                &x,
                &y,
                side,
                points,
                count);

            if(recovered)
            {
                                // 检查左右边线的有效性是否满足继续处理的条件。
                if(non_up_count <= EDGE_MAX_NON_UP - 2)
                {
                    non_up_count += 2;
                }
                else
                {
                    break;
                }

                continue;
            }

            break;
        }

                /*
         * 更新普通八邻域爬线核心相关变量。
         */
        points[count].x = x;
        points[count].y = y;
        count++;

                // 检查左右边线的有效性是否满足继续处理的条件。
        if(side == EDGE_LEFT)
        {
            if(row_line[y] == 0 ||
               x > row_line[y])
            {
                row_line[y] = x;
            }
        }
        else
        {
            if(row_line[y] == 0 ||
               x < row_line[y])
            {
                row_line[y] = x;
            }
        }

        if(y < *top_y)
        {
            *top_y = y;
        }

                // 循环遍历普通八邻域爬线核心，完成当前统计或筛选。
        for(k = 0; k < 8; k++)
        {
            if(side == EDGE_LEFT)
            {
                nx = (int16)x + left_order[k][0];
                ny = (int16)y + left_order[k][1];
            }
            else
            {
                nx = (int16)x + right_order[k][0];
                ny = (int16)y + right_order[k][1];
            }

                        // 检查普通八邻域爬线核心是否满足继续处理的条件。
            if(nx < 1 ||
               nx >= MT9V03X_W - 1 ||
               ny < 1 ||
               ny >= MT9V03X_H - 1)
            {
                continue;
            }

                        // 检查普通八邻域爬线核心是否满足继续处理的条件。
            if(ny > max_backtrack_y)
            {
                continue;
            }

            if(ny <= EDGE_TOP_Y)
            {
                continue;
            }

            if(!is_edge_pixel(img, nx, ny, side))
            {
                continue;
            }

            if(point_already_used(
                    points,
                    count,
                    (uint8)nx,
                    (uint8)ny))
            {
                continue;
            }
                        // 检查环岛状态与环岛边线是否可用；条件不满足时进入回退分支。
            if(!RING_RELAX_SIDE(side))
            {
                if((side == EDGE_LEFT  &&
                    nx < (int16)start_x - (int16)EDGE_MAX_TOTAL_OUTWARD_DX) ||
                   (side == EDGE_RIGHT &&
                    nx > (int16)start_x + (int16)EDGE_MAX_TOTAL_OUTWARD_DX))
                {
                    break;
                }
            }
            dx_move = nx - (int16)x;
            moving_outward = 0;

                        // 检查左右边线的有效性是否满足继续处理的条件。
            if(side == EDGE_LEFT && dx_move < 0)
            {
                moving_outward = 1;
            }

            if(side == EDGE_RIGHT && dx_move > 0)
            {
                moving_outward = 1;
            }

            if(ny < y)
            {
                                /*
                 * 更新普通八邻域爬线核心相关变量。
                 */
                non_up_count = 0;

                if(moving_outward)
                {
                    if(outward_streak < 255)
                    {
                        outward_streak++;
                    }
                }
                else if(outward_streak > 0)
                {
                    outward_streak--;
                }
            }
            else
            {
                                // 检查普通八邻域爬线核心是否满足继续处理的条件。
                if(non_up_count < 255)
                {
                    non_up_count++;
                }

                if(outward_streak > 0)
                {
                    outward_streak--;
                }
            }

            x = (uint8)nx;
            y = (uint8)ny;
            found = 1;

            break;
        }

                // 检查普通八邻域爬线核心是否可用；条件不满足时进入回退分支。
        if(!found)
        {
            uint8 recovered;

            recovered = recover_edge_gap(
                img,
                &x,
                &y,
                side,
                points,
                count);

            if(recovered)
            {
                if(non_up_count <= EDGE_MAX_NON_UP - 2)
                {
                    non_up_count += 2;
                }
                else
                {
                    break;
                }

                continue;
            }

            break;
        }

                // 检查环岛状态与环岛边线是否满足继续处理的条件。
        if(non_up_count >
           (RING_RELAX_SIDE(side) ? RING_MAX_NON_UP : EDGE_MAX_NON_UP))
        {
            break;
        }

                // 检查环岛状态与环岛边线是否满足继续处理的条件。
        if(outward_streak >=
           (RING_RELAX_SIDE(side) ? RING_MAX_OUTWARD_STREAK : EDGE_MAX_OUTWARD_STREAK))
        {
            break;
        }
    }

    return count;
}
/**
 * @brief 十字专用爬线，禁止向下走，允许跨更大的白区。
 */
static uint16 trace_edge_cross(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 start_x,
    uint8 start_y,
    uint8 side,
    Point2D_u8 *points,
    uint8 *row_line,
    uint8 *top_y)
{
    uint8 x = start_x;
    uint8 y = start_y;

    uint8 non_up_count = 0;
    uint8 outward_streak = 0;

    uint16 count = 0;

    *top_y = start_y;

    while(count < MAX_EDGE_PTS)
    {
        uint8 found = 0;
        uint8 k;

        int16 nx;
        int16 ny;
        int16 dx_move;

        uint8 moving_outward;

        if(!is_edge_pixel(img, x, y, side))
        {
            uint8 recovered;

            recovered =
                    recover_edge_gap_cross(
                        img,
                        &x,
                        &y,
                        side,
                        start_x,
                        points,
                        count);
            if(recovered)
            {
                if(non_up_count <= EDGE_MAX_NON_UP - 2)
                {
                    non_up_count += 2;
                }
                else
                {
                    break;
                }

                continue;
            }

            break;
        }

        points[count].x = x;
        points[count].y = y;
        count++;

        if(side == EDGE_LEFT)
        {
            if(row_line[y] == 0 ||
               x > row_line[y])
            {
                row_line[y] = x;
            }
        }
        else
        {
            if(row_line[y] == 0 ||
               x < row_line[y])
            {
                row_line[y] = x;
            }
        }

        if(y < *top_y)
        {
            *top_y = y;
        }

        for(k = 0; k < 8; k++)
        {
            if(side == EDGE_LEFT)
            {
                nx = (int16)x + left_order[k][0];
                ny = (int16)y + left_order[k][1];
            }
            else
            {
                nx = (int16)x + right_order[k][0];
                ny = (int16)y + right_order[k][1];
            }

            if(nx < 1 ||
               nx >= MT9V03X_W - 1 ||
               ny < 1 ||
               ny >= MT9V03X_H - 1)
            {
                continue;
            }
            if(!cross_candidate_within_seed_budget(
                    side,
                    start_x,
                    nx))
            {
                continue;
            }

            if((side == EDGE_LEFT &&
                nx < (int16)x) ||
               (side == EDGE_RIGHT &&
                nx > (int16)x))
            {
                continue;
            }

            if(ny > (int16)y)
            {
                continue;
            }

            if(ny <= EDGE_TOP_Y)
            {
                continue;
            }

            if(!is_edge_pixel(img, nx, ny, side))
            {
                continue;
            }

            if(point_already_used(
                    points,
                    count,
                    (uint8)nx,
                    (uint8)ny))
            {
                continue;
            }

            dx_move = nx - (int16)x;
            moving_outward = 0;

            if(side == EDGE_LEFT && dx_move < 0)
            {
                moving_outward = 1;
            }

            if(side == EDGE_RIGHT && dx_move > 0)
            {
                moving_outward = 1;
            }

            if(ny < y)
            {
                non_up_count = 0;

                if(moving_outward)
                {
                    if(outward_streak < 255)
                    {
                        outward_streak++;
                    }
                }
                else if(outward_streak > 0)
                {
                    outward_streak--;
                }
            }
            else
            {
                if(non_up_count < 255)
                {
                    non_up_count++;
                }

                if(outward_streak > 0)
                {
                    outward_streak--;
                }
            }

            x = (uint8)nx;
            y = (uint8)ny;
            found = 1;

            break;
        }

        if(!found)
        {
            uint8 recovered;

            recovered =
                    recover_edge_gap_cross(
                        img,
                        &x,
                        &y,
                        side,
                        start_x,
                        points,
                        count);
            if(recovered)
            {
                if(non_up_count <= EDGE_MAX_NON_UP - 2)
                {
                    non_up_count += 2;
                }
                else
                {
                    break;
                }

                continue;
            }

            break;
        }

        if(non_up_count > EDGE_MAX_NON_UP)
        {
            break;
        }

        if(outward_streak >= CROSS_EDGE_MAX_OUTWARD_STREAK)
        {
            break;
        }
    }

    return count;
}
/**
 * @brief 旧版单边缺口补线。拟合现有边线并向上预测最多 85 行，当前没有调用。
 */
static uint8 complete_edge_gap(
    uint8 *line,
    uint8 start_y,
    uint8 *top_y)
{
    int16 y;
    int16 fit_top;
    int16 fill_top;

    uint16 point_count = 0;
    uint16 error_count = 0;

    float sum_y = 0.0f;
    float sum_x = 0.0f;
    float sum_yy = 0.0f;
    float sum_yx = 0.0f;

    float denominator;
    float slope;
    float intercept;
    float predicted_x;
    float error;
    float error_sum = 0.0f;

    int16 predicted_pixel;

    fit_top = (int16)(*top_y);

        // 检查左右边线的有效性是否满足继续处理的条件。
    if(fit_top <= EDGE_TOP_Y + 1)
    {
        return 0;
    }

        // 逐行遍历边线斜率与直线拟合。
    for(y = fit_top;
        y <= (int16)start_y &&
        point_count < EDGE_FIT_WINDOW;
        y++)
    {
        if(line[y] != 0)
        {
            sum_y += (float)y;
            sum_x += (float)line[y];
            sum_yy += (float)y * (float)y;
            sum_yx += (float)y * (float)line[y];

            point_count++;
        }
    }

        // 检查左右边线的有效性是否满足继续处理的条件。
    if(point_count < EDGE_MIN_FIT_POINTS)
    {
        return 0;
    }

    denominator =
        (float)point_count * sum_yy -
        sum_y * sum_y;

    if(denominator > -0.001f &&
       denominator <  0.001f)
    {
        return 0;
    }

        /*
     * 更新边线斜率与直线拟合相关变量。
     */
    slope =
        ((float)point_count * sum_yx -
         sum_y * sum_x) / denominator;

    intercept =
        (sum_x - slope * sum_y) /
        (float)point_count;

        // 逐行遍历边线斜率与直线拟合。
    for(y = fit_top;
        y <= (int16)start_y &&
        error_count < EDGE_FIT_WINDOW;
        y++)
    {
        if(line[y] != 0)
        {
            predicted_x =
                slope * (float)y + intercept;

            error =
                (float)line[y] - predicted_x;

            error_sum += error * error;
            error_count++;
        }
    }

    if(error_count == 0 ||
       error_sum >
       EDGE_MAX_FIT_RMS_SQ * (float)error_count)
    {
        return 0;
    }

        /*
     * 更新左右边线的有效性相关变量。
     */
    fill_top =
        fit_top - EDGE_FILL_MAX_ROWS;

    if(fill_top < EDGE_TOP_Y + 1)
    {
        fill_top = EDGE_TOP_Y + 1;
    }

        // 逐行遍历边线斜率与直线拟合。
    for(y = fit_top - 1;
        y >= fill_top;
        y--)
    {
        predicted_x =
            slope * (float)y + intercept;

        if(predicted_x >= 0.0f)
        {
            predicted_pixel =
                (int16)(predicted_x + 0.5f);
        }
        else
        {
            predicted_pixel =
                (int16)(predicted_x - 0.5f);
        }

                // 检查旧版单边缺口补线是否满足继续处理的条件。
        if(predicted_pixel < 1)
        {
            predicted_pixel = 1;
        }

        if(predicted_pixel > MT9V03X_W - 2)
        {
            predicted_pixel = MT9V03X_W - 2;
        }

        line[y] =
            (uint8)predicted_pixel;
    }

        /*
     * 更新旧版单边缺口补线相关变量。
     */
    *top_y =
        (uint8)fill_top;

    return 1;
}

typedef struct
{
    float slope;
    float intercept;

        // 定义pointcount变量，用于当前图像处理过程。
    uint8 point_count;
} CrossFitModel;

/**
 * @brief 十字补线使用的直线拟合，模型为 x = slope * y + intercept。
 */
static uint8 fit_cross_edge(
    uint8 *line,
    uint8 top_y,
    uint8 start_y,
    CrossFitModel *model)
{
    int16 y;

    uint16 count = 0;
    uint16 error_count = 0;

    float sum_y = 0.0f;
    float sum_x = 0.0f;
    float sum_yy = 0.0f;
    float sum_yx = 0.0f;

    float denominator;
    float predicted_x;
    float error;
    float error_sum = 0.0f;

    if(model == 0)
    {
        return 0;
    }
    model->point_count = 0;
        // 逐行遍历十字补线使用的直线拟合，模型为 x = slope * y + intercept。
    for(y = (int16)start_y;
        y >= (int16)top_y &&
        count < CROSS_FIT_WINDOW;
        y--)
    {
        if(line[y] != 0)
        {
            sum_y += (float)y;
            sum_x += (float)line[y];
            sum_yy += (float)y * (float)y;
            sum_yx += (float)y * (float)line[y];

            count++;
        }
    }

        // 检查左右边线的有效性是否满足继续处理的条件。
    if(count < EDGE_MIN_FIT_POINTS)
    {
        return 0;
    }

        // 检查左右边线的有效性是否满足继续处理的条件。
    if((start_y - top_y) < EDGE_MIN_TRACE_SPAN)
    {
        return 0;
    }

    denominator =
        (float)count * sum_yy -
        sum_y * sum_y;

    if(denominator > -0.001f &&
       denominator <  0.001f)
    {
        return 0;
    }

    model->slope =
        ((float)count * sum_yx -
         sum_y * sum_x) / denominator;

    model->intercept =
        (sum_x -
         model->slope * sum_y) /
        (float)count;

        // 逐行遍历十字补线使用的直线拟合，模型为 x = slope * y + intercept。
    for(y = (int16)start_y;
        y >= (int16)top_y &&
        error_count < CROSS_FIT_WINDOW;
        y--)
    {
        if(line[y] != 0)
        {
            predicted_x =
                model->slope * (float)y +
                model->intercept;

            error =
                (float)line[y] - predicted_x;

            error_sum += error * error;
            error_count++;
        }
    }

    if(error_count == 0)
    {
        return 0;
    }

    if(error_sum >
       CROSS_MAX_FIT_RMS_SQ *
       (float)error_count)
    {
        return 0;
    }

    model->point_count = (uint8)count;

    return 1;
}
// 参数 TURN_FALLBACK_MIN_POINTS：急弯拟合所需的最少点数，当前值为 4。
#define TURN_FALLBACK_MIN_POINTS       4
// 参数 TURN_FALLBACK_MIN_SPAN：急弯拟合所需的最小纵向跨度，当前值为 4。
#define TURN_FALLBACK_MIN_SPAN         4
// 参数 TURN_FALLBACK_MAX_SLOPE：急弯补线允许的最大斜率，当前值为 5.0f。
#define TURN_FALLBACK_MAX_SLOPE        5.0f
// 参数 TURN_FALLBACK_MAX_RMS_SQ：急弯直线拟合允许的最大均方误差，当前值为 25.0f。
#define TURN_FALLBACK_MAX_RMS_SQ       25.0f
// 参数 TURN_FALLBACK_MAX_EXTENSION：急弯补线允许的最大延伸行数，当前值为 48。
#define TURN_FALLBACK_MAX_EXTENSION    48
// 参数 TURN_FALLBACK_MIN_WIDTH：急弯补线允许的最小赛道宽度，当前值为 20。
#define TURN_FALLBACK_MIN_WIDTH        20
// 参数 TURN_FALLBACK_MAX_WIDTH：急弯补线允许的最大赛道宽度。
#define TURN_FALLBACK_MAX_WIDTH        (MT9V03X_W - 8)
// 参数 TURN_FALLBACK_MIN_COMMON_ROWS：急弯补线要求的最少共同有效行，当前值为 6。
#define TURN_FALLBACK_MIN_COMMON_ROWS  6

static int16 turn_abs_i16(int16 value)
{
    return (value < 0) ? -value : value;
}

static int16 turn_limit_pixel(int16 pixel)
{
    if(pixel < 1)
    {
        return 1;
    }

    if(pixel > MT9V03X_W - 2)
    {
        return MT9V03X_W - 2;
    }

    return pixel;
}

/**
 * @brief 对短急弯边线做直线拟合，检查点数、跨度、斜率和误差。
 */
static uint8 fit_short_turn_edge(
    uint8 *line,
    uint8 top_limit,
    uint8 bottom_limit,
    CrossFitModel *model,
    uint8 *valid_top,
    uint8 *valid_bottom)
{
    int16 y;
    int16 first;
    int16 last;
    uint16 count;

    float sum_y;
    float sum_x;
    float sum_yy;
    float sum_yx;
    float denominator;
    float error_sum;
    float predicted;
    float error;

    if(line == 0 ||
       model == 0 ||
       valid_top == 0 ||
       valid_bottom == 0 ||
       top_limit >= bottom_limit)
    {
        return 0;
    }

    first = -1;
    last = -1;
    count = 0;

    sum_y = 0.0f;
    sum_x = 0.0f;
    sum_yy = 0.0f;
    sum_yx = 0.0f;

    for(y = (int16)top_limit;
        y <= (int16)bottom_limit;
        y++)
    {
        if(line[y] != 0)
        {
            if(first < 0)
            {
                first = y;
            }

            last = y;

            sum_y += (float)y;
            sum_x += (float)line[y];
            sum_yy += (float)y * (float)y;
            sum_yx += (float)y * (float)line[y];

            count++;
        }
    }

    if(count < TURN_FALLBACK_MIN_POINTS ||
       first < 0 ||
       last < 0 ||
       (last - first) < TURN_FALLBACK_MIN_SPAN)
    {
        return 0;
    }

    denominator =
        (float)count * sum_yy -
        sum_y * sum_y;

    if(denominator > -0.001f &&
       denominator < 0.001f)
    {
        return 0;
    }

    model->slope =
        ((float)count * sum_yx -
         sum_y * sum_x) / denominator;

    model->intercept =
        (sum_x -
         model->slope * sum_y) /
        (float)count;

    if(model->slope > TURN_FALLBACK_MAX_SLOPE ||
       model->slope < -TURN_FALLBACK_MAX_SLOPE)
    {
        return 0;
    }

    error_sum = 0.0f;

    for(y = first; y <= last; y++)
    {
        if(line[y] != 0)
        {
            predicted =
                model->slope * (float)y +
                model->intercept;

            error =
                (float)line[y] - predicted;

            error_sum += error * error;
        }
    }

    if(error_sum / (float)count > TURN_FALLBACK_MAX_RMS_SQ)
    {
        return 0;
    }

    model->point_count = (uint8)count;
    *valid_top = (uint8)first;
    *valid_bottom = (uint8)last;

    return 1;
}

static int16 turn_predict_from_anchor(
    const CrossFitModel *model,
    uint8 anchor_y,
    uint8 anchor_x,
    int16 y)
{
    float predicted;
    int16 pixel;

    if(model == 0)
    {
        return (int16)anchor_x;
    }

        /*
     * 更新边线斜率与直线拟合相关变量。
     */
    predicted =
        (float)anchor_x +
        model->slope * (float)(y - (int16)anchor_y);

    if(predicted >= 0.0f)
    {
        pixel = (int16)(predicted + 0.5f);
    }
    else
    {
        pixel = (int16)(predicted - 0.5f);
    }

    return turn_limit_pixel(pixel);
}

/**
 * @brief 急弯补线。左右边线拟合后最多向外延伸 48 行，并检查三个位置的赛道宽度。
 */
static uint8 extend_short_turn_edges(
    uint8 *left_line,
    uint8 *right_line,
    uint8 target_top,
    uint8 target_bottom)
{
    CrossFitModel left_model;
    CrossFitModel right_model;

    uint8 left_top;
    uint8 left_bottom;
    uint8 right_top;
    uint8 right_bottom;

    int16 fill_top;
    int16 y;
    int16 sample_y[3];
    int16 sample;
    int16 left_predict;
    int16 right_predict;
    int16 left_actual;
    int16 right_actual;
    int16 width;

    if(left_line == 0 ||
       right_line == 0 ||
       target_top >= target_bottom)
    {
        return 0;
    }

    if(!fit_short_turn_edge(
            left_line,
            target_top,
            target_bottom,
            &left_model,
            &left_top,
            &left_bottom))
    {
        return 0;
    }

    if(!fit_short_turn_edge(
            right_line,
            target_top,
            target_bottom,
            &right_model,
            &right_top,
            &right_bottom))
    {
        return 0;
    }

        /*
     * 更新急弯补线相关变量。
     */
    fill_top =
        (int16)target_bottom -
        TURN_FALLBACK_MAX_EXTENSION;

    if(fill_top < (int16)target_top)
    {
        fill_top = target_top;
    }

    sample_y[0] = fill_top;
    sample_y[1] = (fill_top + (int16)target_bottom) / 2;
    sample_y[2] = target_bottom;

    for(sample = 0; sample < 3; sample++)
    {
        left_predict =
            turn_predict_from_anchor(
                &left_model,
                left_bottom,
                left_line[left_bottom],
                sample_y[sample]);

        right_predict =
            turn_predict_from_anchor(
                &right_model,
                right_bottom,
                right_line[right_bottom],
                sample_y[sample]);

        width = right_predict - left_predict;

        if(left_predict >= right_predict ||
           width < TURN_FALLBACK_MIN_WIDTH ||
           width > TURN_FALLBACK_MAX_WIDTH)
        {
            return 0;
        }
    }

        // 逐行遍历急弯补线。
    for(y = fill_top;
        y <= (int16)target_bottom;
        y++)
    {
        left_predict =
            turn_predict_from_anchor(
                &left_model,
                left_bottom,
                left_line[left_bottom],
                y);

        right_predict =
            turn_predict_from_anchor(
                &right_model,
                right_bottom,
                right_line[right_bottom],
                y);

        left_actual =
            (left_line[y] != 0)
            ? left_line[y]
            : left_predict;

        right_actual =
            (right_line[y] != 0)
            ? right_line[y]
            : right_predict;

        width = right_actual - left_actual;

        if(left_actual >= right_actual ||
           width < TURN_FALLBACK_MIN_WIDTH ||
           width > TURN_FALLBACK_MAX_WIDTH)
        {
            continue;
        }

        if(left_line[y] == 0)
        {
            left_line[y] = (uint8)left_predict;
        }

        if(right_line[y] == 0)
        {
            right_line[y] = (uint8)right_predict;
        }
    }

    return 1;
}

/**
 * @brief 通过已爬到的左右线寻找同时断线的连续行。
 */
static uint8 find_cross_gap(
    uint8 *left_line,
    uint8 *right_line,
    uint8 top_y,
    uint8 bottom_y,
    uint8 left_trace_top,
    uint8 right_trace_top,
    uint8 *gap_top_y)
{
    int16 y;
    int16 top_diff;

    uint8 both_valid;
    uint8 both_missing;
    uint8 common_streak;
    uint8 simultaneous_missing;

    if(left_line == 0 ||
       right_line == 0 ||
       gap_top_y == 0 ||
       top_y >= bottom_y)
    {
        return 0;
    }

        /*
     * 更新八邻域爬线过程相关变量。
     */
    top_diff =
        (int16)left_trace_top -
        (int16)right_trace_top;

    if(top_diff < 0)
    {
        top_diff = -top_diff;
    }

    if(top_diff > CROSS_MAX_TOP_DIFF)
    {
        return 0;
    }

    common_streak = 0;
    simultaneous_missing = 0;

        // 逐行遍历通过已爬到的左右线寻找同时断线的连续行。
    for(y = (int16)bottom_y;
        y >= (int16)top_y;
        y--)
    {
        both_valid =
            (left_line[y] != 0 &&
             right_line[y] != 0 &&
             left_line[y] < right_line[y]);

        both_missing =
            (left_line[y] == 0 &&
             right_line[y] == 0);

                // 检查通过已爬到的左右线寻找同时断线的连续行是否满足继续处理的条件。
        if(both_valid)
        {
            if(common_streak < 255)
            {
                common_streak++;
            }

            simultaneous_missing = 0;
            continue;
        }

                // 检查通过已爬到的左右线寻找同时断线的连续行是否满足继续处理的条件。
        if(both_missing)
        {
                        // 检查十字锁定与十字白区是否满足继续处理的条件。
            if(common_streak <
               CROSS_MIN_COMMON_ROWS_AROUND_GAP)
            {
                simultaneous_missing = 0;
                continue;
            }

            if(simultaneous_missing < 255)
            {
                simultaneous_missing++;
            }

            if(simultaneous_missing >=
               CROSS_MIN_MISSING_ROWS)
            {
                *gap_top_y = (uint8)y;
                return 1;
            }

            continue;
        }

                /*
         * 更新通过已爬到的左右线寻找同时断线的连续行相关变量。
         */
        common_streak = 0;
        simultaneous_missing = 0;
    }

    return 0;
}
/**
 * @brief 统计一条线的首个有效行、最后有效行和有效点数。
 */
static uint8 get_line_span(
    uint8 *line,
    uint8 min_y,
    uint8 max_y,
    uint8 *first_y,
    uint8 *last_y,
    uint8 *valid_count)
{
    int16 y;
    uint8 first = 0;
    uint8 last = 0;
    uint8 count = 0;

    if(line == 0 || first_y == 0 || last_y == 0 ||
       valid_count == 0 || min_y > max_y)
    {
        return 0;
    }

    for(y = (int16)min_y; y <= (int16)max_y; y++)
    {
        if(line[y] != 0)
        {
            if(count == 0)
            {
                first = (uint8)y;
            }

            last = (uint8)y;
            if(count < 255)
            {
                count++;
            }
        }
    }

    if(count < EDGE_MIN_FIT_POINTS)
    {
        return 0;
    }

    if(((uint16)last - (uint16)first) <
       EDGE_MIN_TRACE_SPAN)
    {
        return 0;
    }
    *first_y = first;
    *last_y = last;
    *valid_count = count;
    return 1;
}
// 参数 EDGE_SPIKE_MIN_DELTA：判定边线单点毛刺所需的最小跳变量，当前值为 5。
#define EDGE_SPIKE_MIN_DELTA       5
// 参数 EDGE_SPIKE_NEIGHBOR_TOL：边线毛刺判断使用的邻行容差，当前值为 3。
#define EDGE_SPIKE_NEIGHBOR_TOL    3

/**
 * @brief 使用三数中值过滤边线单点毛刺。
 */
static void filter_edge_single_row_spikes(
    uint8 *line,
    uint8 top_y,
    uint8 bottom_y)
{
    int16 y;

    uint8 value_a;
    uint8 value_b;
    uint8 value_c;
    uint8 median_value;

    int16 delta_a;
    int16 delta_c;
    int16 neighbor_delta;

    if(line == 0 ||
       top_y + 1 >= bottom_y)
    {
        return;
    }

    for(y = (int16)top_y + 1;
        y < (int16)bottom_y;
        y++)
    {
        value_a = line[y - 1];
        value_b = line[y];
        value_c = line[y + 1];

                // 检查使用三数中值过滤边线单点毛刺是否满足继续处理的条件。
        if(value_a == 0 ||
           value_b == 0 ||
           value_c == 0)
        {
            continue;
        }

        delta_a =
            (int16)value_b -
            (int16)value_a;

        if(delta_a < 0)
        {
            delta_a = -delta_a;
        }

        delta_c =
            (int16)value_b -
            (int16)value_c;

        if(delta_c < 0)
        {
            delta_c = -delta_c;
        }

        neighbor_delta =
            (int16)value_a -
            (int16)value_c;

        if(neighbor_delta < 0)
        {
            neighbor_delta = -neighbor_delta;
        }

                // 检查左右边线的有效性是否满足继续处理的条件。
        if(delta_a >= EDGE_SPIKE_MIN_DELTA &&
           delta_c >= EDGE_SPIKE_MIN_DELTA &&
           neighbor_delta <= EDGE_SPIKE_NEIGHBOR_TOL)
        {
                        // 检查使用三数中值过滤边线单点毛刺是否满足继续处理的条件。
            if(value_a > value_b)
            {
                if(value_b > value_c)
                {
                    median_value = value_b;
                }
                else if(value_a > value_c)
                {
                    median_value = value_c;
                }
                else
                {
                    median_value = value_a;
                }
            }
            else
            {
                if(value_a > value_c)
                {
                    median_value = value_a;
                }
                else if(value_b > value_c)
                {
                    median_value = value_c;
                }
                else
                {
                    median_value = value_b;
                }
            }

            line[y] = median_value;
        }
    }
}
// 参数 MID_CONT_JUMP_MIN：中线连续性修复所需的最小异常跳变量，当前值为 7。
#define MID_CONT_JUMP_MIN       7
// 参数 MID_CONT_SLOPE_TOL：中线连续性判断允许的斜率误差，当前值为 3。
#define MID_CONT_SLOPE_TOL      3
// 参数 MID_CONT_RETURN_TOL：中线跳走后返回的第一个容差，当前值为 5。
#define MID_CONT_RETURN_TOL     5
// 参数 MID_CONT_RETURN2_TOL：中线跳走后返回的第二个容差，当前值为 7。
#define MID_CONT_RETURN2_TOL    7
// 参数 MID_CONT_MAX_RUN：中线连续异常段允许修复的最大行数，当前值为 12。
#define MID_CONT_MAX_RUN        12

/**
 * @brief 判断某行左右边线是否同时有效且 left < right。
 */
static uint8 mid_line_row_valid(int16 row)
{
    if(row < 0 ||
       row >= MT9V03X_H)
    {
        return 0;
    }

    if(left_line_list[row] == 0 ||
       right_line_list[row] == 0 ||
       left_line_list[row] >= right_line_list[row])
    {
        return 0;
    }

    return 1;
}
// 参数 SINGLE_SIDE_SEARCH_ROWS：单边补线估计赛道宽度时的搜索行数，当前值为 24。
#define SINGLE_SIDE_SEARCH_ROWS   24
// 参数 SINGLE_SIDE_MIN_WIDTH：单边补线允许的最小赛道宽度，当前值为 30。
#define SINGLE_SIDE_MIN_WIDTH     30
// 参数 SINGLE_SIDE_MAX_WIDTH：单边补线允许的最大赛道宽度。
#define SINGLE_SIDE_MAX_WIDTH     (MT9V03X_W - 8)

/**
 * @brief 在目标行上下 24 行寻找最近的有效赛道宽度。
 */
static uint8 estimate_lane_width_near(
    uint8 *left_line,
    uint8 *right_line,
    uint8 y,
    uint16 width_hint,
    uint16 *width)
{
    int16 offset;
    int16 candidate_y;
    uint16 current_width;

    if(left_line == 0 ||
       right_line == 0 ||
       width == 0)
    {
        return 0;
    }

    for(offset = 0;
        offset <= SINGLE_SIDE_SEARCH_ROWS;
        offset++)
    {
                /*
         * 更新在目标行上下 24 行寻找最近的有效赛道宽度相关变量。
         */
        candidate_y = (int16)y - offset;

        if(candidate_y >= EDGE_TOP_Y + 1 &&
           candidate_y < MT9V03X_H &&
           left_line[candidate_y] != 0 &&
           right_line[candidate_y] != 0 &&
           left_line[candidate_y] < right_line[candidate_y])
        {
            current_width =
                (uint16)right_line[candidate_y] -
                (uint16)left_line[candidate_y];

            if(current_width >= SINGLE_SIDE_MIN_WIDTH &&
               current_width <= SINGLE_SIDE_MAX_WIDTH)
            {
                *width = current_width;
                return 1;
            }
        }

        if(offset == 0)
        {
            continue;
        }

        candidate_y = (int16)y + offset;

        if(candidate_y < MT9V03X_H &&
           left_line[candidate_y] != 0 &&
           right_line[candidate_y] != 0 &&
           left_line[candidate_y] < right_line[candidate_y])
        {
            current_width =
                (uint16)right_line[candidate_y] -
                (uint16)left_line[candidate_y];

            if(current_width >= SINGLE_SIDE_MIN_WIDTH &&
               current_width <= SINGLE_SIDE_MAX_WIDTH)
            {
                *width = current_width;
                return 1;
            }
        }
    }

        // 检查赛道宽度估计是否满足继续处理的条件。
    if(width_hint >= SINGLE_SIDE_MIN_WIDTH &&
       width_hint <= SINGLE_SIDE_MAX_WIDTH)
    {
        *width = width_hint;
        return 1;
    }

    return 0;
}


/**
 * @brief 从真实边缘点中找该侧最靠上的 y。
 */
static uint8 side_real_top_row(uint8 side)
{
uint16 i;
uint16 n;
uint8  best = 127;

if(side == EDGE_LEFT)
{
    n = Left_Edge_Count;

    for(i = 0; (i < n) && (i < MAX_EDGE_PTS); i++)
    {
        if(Left_Edge_Pts[i].y > EDGE_TOP_Y &&
           Left_Edge_Pts[i].y < best)
        {
            best = Left_Edge_Pts[i].y;
        }
    }
}
else
{
    n = Right_Edge_Count;

    for(i = 0; (i < n) && (i < MAX_EDGE_PTS); i++)
    {
        if(Right_Edge_Pts[i].y > EDGE_TOP_Y &&
           Right_Edge_Pts[i].y < best)
        {
            best = Right_Edge_Pts[i].y;
        }
    }
}

return best;
}

/**
 * @brief 旧单边补线顶部边界计算，当前属于关闭功能。
 */
static uint8 one_side_fill_limit(uint8 side)
{
uint8  top = side_real_top_row(side);
uint16 limit;

if(top == 127)
{
    return (uint8)(EDGE_TOP_Y + 1);
}

limit = (uint16)top;

if(limit > (uint16)(EDGE_TOP_Y + 1 + ONE_SIDE_FILL_TOP_MARGIN))
{
    limit -= (uint16)ONE_SIDE_FILL_TOP_MARGIN;
}
else
{
    limit = (uint16)(EDGE_TOP_Y + 1);
}

return (uint8)limit;
}
/**
 * @brief 从真实边缘点中找该侧最靠下、最靠近车的 y。
 */
static uint8 side_real_bottom_row(uint8 side)
{
    uint16 i;
    uint16 n;
    uint8  best = 0;

    if(side == EDGE_LEFT)
    {
        n = Left_Edge_Count;

        for(i = 0; (i < n) && (i < MAX_EDGE_PTS); i++)
        {
            if(Left_Edge_Pts[i].y > best)
            {
                best = Left_Edge_Pts[i].y;
            }
        }
    }
    else
    {
        n = Right_Edge_Count;

        for(i = 0; (i < n) && (i < MAX_EDGE_PTS); i++)
        {
            if(Right_Edge_Pts[i].y > best)
            {
                best = Right_Edge_Pts[i].y;
            }
        }
    }

    return best;
}

/**
 * @brief 根据编译开关决定旧单边补线的底部范围。
 */
static uint8 one_side_fill_bottom(uint8 side, uint8 bottom_y)
{
#if !ONE_SIDE_FILL_LIMIT_TO_REAL_SPAN
    (void)side;
    return bottom_y;
#else
    uint8 b = side_real_bottom_row(side);

    if(b == 0)
    {
        return bottom_y;
    }

    return b;
#endif
}
/**
 * @brief 旧版普通单边补线，所在调用分支由 ENABLE_ONE_SIDE_FILL_NORMAL=0 关闭。
 */
static void complete_one_side_edge(
    uint8 *left_line,
    uint8 *right_line,
    uint8 top_y,
    uint8 bottom_y,
    uint16 width_hint,
    uint8 left_fill_top,       /* 定义leftfilltop变量，用于当前图像处理过程。 */
    uint8 right_fill_top,      /* 定义rightfilltop变量，用于当前图像处理过程。 */
    uint8 left_fill_bottom,    /* 定义leftfillbottom变量，用于当前图像处理过程。 */
    uint8 right_fill_bottom)   /* 定义rightfillbottom变量，用于当前图像处理过程。 */
{
    int16 y;
    int16 predicted;
    uint16 width;

    if(left_line == 0 ||
       right_line == 0)
    {
        return;
    }

    for(y = (int16)bottom_y;
        y >= (int16)top_y;
        y--)
    {
                // 检查左右边线的有效性是否满足继续处理的条件。
        if(left_line[y] == 0 &&
           right_line[y] == 0)
        {
            continue;
        }

                // 检查左右边线的有效性是否满足继续处理的条件。
        if(left_line[y] == 0 &&
           right_line[y] != 0 &&
           (uint8)y >= left_fill_top &&
           (uint8)y <= left_fill_bottom)
        {
            if(estimate_lane_width_near(
                    left_line,
                    right_line,
                    (uint8)y,
                    width_hint,
                    &width))
            {
                predicted =
                    (int16)right_line[y] -
                    (int16)width;

                if(predicted >= 1 &&
                   predicted < MT9V03X_W - 1)
                {
                    left_line[y] = (uint8)predicted;
                }
            }
        }

                // 继续判断左右边线的有效性的其他情况。
        else if(right_line[y] == 0 &&
                left_line[y] != 0 &&
                (uint8)y >= right_fill_top &&
                (uint8)y <= right_fill_bottom)
        {
            if(estimate_lane_width_near(
                    left_line,
                    right_line,
                    (uint8)y,
                    width_hint,
                    &width))
            {
                predicted =
                    (int16)left_line[y] +
                    (int16)width;

                if(predicted >= 1 &&
                   predicted < MT9V03X_W - 1)
                {
                    right_line[y] = (uint8)predicted;
                }
            }
        }
    }
}
// 定义midangletab变量，用于当前图像处理过程。
static const uint8 mid_angle_tab[33] =
{
     0,  2,  4,  5,  7,  9, 11, 12, 14, 16, 17,
    19, 21, 22, 24, 25, 27, 28, 29, 31, 32, 33,
    35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45
};

/*
 * 使用查表近似计算两点连线角度。
 */
static int16 mid_angle_deg(int16 dx, int16 dy)
{
    int16 adx;
    int16 idx;
    int16 ang;

    if(dy <= 0)
    {
        dy = 1;
    }

    if(dx == 0)
    {
        return 0;
    }

    adx = (dx < 0) ? -dx : dx;

    if(adx <= dy)
    {
        idx = (int16)((adx * 32) / dy);
        if(idx > 32)
        {
            idx = 32;
        }
        ang = (int16)mid_angle_tab[idx];
    }
    else
    {
        idx = (int16)((dy * 32) / adx);
        if(idx > 32)
        {
            idx = 32;
        }
        ang = 90 - (int16)mid_angle_tab[idx];
    }

    return (dx < 0) ? -ang : ang;
}

/*
 * 统计一组中心线点的平均角度并剔除离群点。当前由 MID_ANG_ENABLE=0 关闭。
 */
static int16 mid_block_angle(const uint8 *rows, uint16 start, uint16 count)
{
    uint16 k;
    int16  sum1 = 0;
    int16  sum2 = 0;
    int16  avg1;
    int16  ang;
    uint16 cnt2 = 0;
    int16  y0;
    int16  x0;
    int16  dx;
    int16  dy;

    if(count < 2)
    {
        return 0;
    }

    y0 = (int16)rows[start];
    x0 = (int16)mid_line_list[y0];

        // 循环遍历complete_one_side_edge 的主要处理，完成当前统计或筛选。
    for(k = 1; k < count; k++)
    {
        dy = y0 - (int16)rows[start + k];
        dx = (int16)mid_line_list[rows[start + k]] - x0;

        sum1 += mid_angle_deg(dx, dy);
    }

    avg1 = sum1 / (int16)(count - 1);

        // 循环遍历complete_one_side_edge 的主要处理，完成当前统计或筛选。
    for(k = 1; k < count; k++)
    {
        dy = y0 - (int16)rows[start + k];
        dx = (int16)mid_line_list[rows[start + k]] - x0;

        ang = mid_angle_deg(dx, dy);

        if((ang - avg1) > MID_ANG_OUTLIER ||
           (avg1 - ang) > MID_ANG_OUTLIER)
        {
            continue;
        }

        sum2 += ang;
        cnt2++;
    }

    if(cnt2 == 0)
    {
        return avg1;          /* 返回complete_one_side_edge 的主要处理的判断或处理结果。 */
    }

    return sum2 / (int16)cnt2;
}
/**
 * @brief 中线切段时判断某行是否有效。
 */
static uint8 cut_mid_row_ok(int16 row)
{
    if(row <= (int16)EDGE_TOP_Y ||
       row >  (int16)(jidian_search_line - 1))
    {
        return 0;
    }

    if(road_mode == ROAD_MODE_IPM_FILL)
    {
        return (ipm_mid_line_valid[row] != 0) ? 1 : 0;
    }

    if(left_line_list[row] == 0 ||
       right_line_list[row] == 0)
    {
        return 0;
    }

    if(left_line_list[row] >= right_line_list[row])
    {
        return 0;
    }

    return 1;
}

/* ===== 图像处理参数配置 ===== */
#define MID_CUT_NEAR_MIN_DX    0    /* 参数 MID_CUT_NEAR_MIN_DX：近处中线切段所需的最小横向跳变，当前值为 0。 */
#define MID_CUT_FAR_MIN_DX     6    /* 参数 MID_CUT_FAR_MIN_DX：远处中线切段所需的最小横向跳变，当前值为 6。 */
#define MID_CUT_ROW_DEV        6   /* 参数 MID_CUT_ROW_DEV：中线切段时允许的行偏移，当前值为 6。 */
// 定义midcutrows变量，用于当前图像处理过程。
static uint8 mid_cut_rows[MT9V03X_H];
/*
 * 对一组中心线点求平均 x 和鲁棒上下偏移。
 */
static int16 mid_cut_block_offset(uint16 start, uint16 cnt, uint8 *ok)
{
    int16  sum = 0;
    int16  avg;
    int16  x;
    int16  kept_cnt = 0;
    int16  first_x = 0;
    int16  last_x  = 0;
    uint16 j;

    *ok = 0;

    for(j = 0; j < cnt; j++)
    {
        sum = (int16)(sum + (int16)mid_line_list[mid_cut_rows[start + j]]);
    }

    avg = (int16)(sum / (int16)cnt);

    for(j = 0; j < cnt; j++)
    {
        x = (int16)mid_line_list[mid_cut_rows[start + j]];

        if(((x - avg) > (int16)MID_CUT_ROW_DEV) ||
           ((avg - x) > (int16)MID_CUT_ROW_DEV))
        {
            continue;                       /* 跳过本次循环，继续处理下一行。 */
        }

        if(kept_cnt == 0)
        {
            first_x = x;
        }

        last_x = x;
        kept_cnt++;
    }

    if(kept_cnt < 3)
    {
        return 0;
    }

    *ok = 1;

    return (int16)(last_x - first_x);
}
/**
 * @brief 比较两个 8 行块的中线偏移，检测突变并截掉更远部分。
 */
static uint8 cut_mid_line_by_slope_jump(void)
{
    uint16 n = 0;
    uint16 i;
    int16  y;
    int16  near_off;
    int16  far_off;
    int16  near_abs;
    int16  far_abs;
    int16  diff;
    int16  off_diff_limit;
    uint8  near_ok;
    uint8  far_ok;
    uint8  cut_y;

#if !MID_OFF_ENABLE
    return 0;
#endif

    if(cross_lock_active)
    {
        return 0;
    }

    if(ring_state == 2 || ring_state == 3)
    {
        return 0;
    }

        // 环岛状态机的当前状态。
    off_diff_limit = (ring_state != 0)
                   ? (int16)MID_OFF_DIFF_RING
                   : (int16)MID_OFF_DIFF;

        // 逐行遍历左右边线的有效性。
    for(y = (int16)(jidian_search_line - 1); y > (int16)EDGE_TOP_Y; y--)
    {
        if(cut_mid_row_ok(y))
        {
            mid_cut_rows[n] = (uint8)y;
            n++;

            if(n >= MT9V03X_H)
            {
                break;
            }
        }
    }

        // 检查中线的有效性与连续性是否满足继续处理的条件。
    if(n < (uint16)(MID_OFF_BLOCK * 2))
    {
        return 0;
    }

        // 循环遍历中线的有效性与连续性，完成当前统计或筛选。
    for(i = MID_OFF_BLOCK; i + MID_OFF_BLOCK <= n; i += MID_OFF_STEP)
    {
                /*
         * 更新比较两个 8 行块的中线偏移，检测突变并截掉更远部分相关变量。
         */
        near_ok = 0;
        far_ok  = 0;

        near_off = mid_cut_block_offset((uint16)(i - MID_OFF_BLOCK),
                                        (uint16)MID_OFF_BLOCK,
                                        &near_ok);

        far_off  = mid_cut_block_offset((uint16)i,
                                        (uint16)MID_OFF_BLOCK,
                                        &far_ok);

        near_abs = (near_off < 0) ? (int16)(-near_off) : near_off;
        far_abs  = (far_off  < 0) ? (int16)(-far_off)  : far_off;

                // 检查中线的有效性与连续性是否满足继续处理的条件。
        if((near_ok != 0) && (far_ok != 0) &&
           (near_abs > (int16)MID_CUT_NEAR_MIN_DX) &&
           (far_abs  > (int16)MID_CUT_FAR_MIN_DX) &&
           (((near_off > 0) && (far_off < 0)) ||
            ((near_off < 0) && (far_off > 0))))
        {
            diff = (int16)(near_off - far_off);

            if(diff < 0)
            {
                diff = (int16)(-diff);
            }

            if(diff > off_diff_limit)
            {
                cut_y = mid_cut_rows[i];

                                // 逐行遍历左右边线的有效性。
                for(y = (int16)cut_y; y > (int16)EDGE_TOP_Y; y--)
                {
                    left_line_list[y]     = 0;
                    right_line_list[y]    = 0;
                    mid_line_list[y]      = MID_W;
                    ipm_mid_line_valid[y] = 0;
                }

                search_end_line = cut_y;

                return 1;
            }
        }
    }

    return 0;                  /* 返回比较两个 8 行块的中线偏移，检测突变并截掉更远部分的判断或处理结果。 */
}
/**
 * @brief 使用 (left + right) / 2 生成逐行中心线，并执行突变截断。
 */
static uint16 build_mid_line_from_edges(
    uint8 *left_line,
    uint8 *right_line,
    uint8 *mid_line,
    uint8 top_y,
    uint8 bottom_y)
{
    int16 y;
    uint16 valid_count = 0;

    if(left_line == 0 ||
       right_line == 0 ||
       mid_line == 0)
    {
        return 0;
    }

    for(y = (int16)bottom_y;
        y >= (int16)top_y;
        y--)
    {
        if(left_line[y] == 0 ||
           right_line[y] == 0 ||
           left_line[y] >= right_line[y])
        {
            continue;
        }

        mid_line[y] =
            (uint8)(((uint16)left_line[y] +
                     (uint16)right_line[y]) / 2U);

        valid_count++;
    }


    cut_mid_line_by_slope_jump();

    return valid_count;
}

// 参数 MID_SPIKE_MIN_DELTA：中线尖峰判定所需的最小跳变量，当前值为 8。
#define MID_SPIKE_MIN_DELTA       8
// 参数 MID_SPIKE_NEIGHBOR_TOL：中线尖峰判断使用的邻行容差，当前值为 4。
#define MID_SPIKE_NEIGHBOR_TOL    4
#define MID_OUT_GAP_FILL    100    /* 参数 MID_OUT_GAP_FILL：输出中线允许直接补齐的最大间隙，当前值为 100。 */
static int16 abs_i16_value(int16 value)
{
    return (value < 0) ? -value : value;
}
/**
 * @brief 检测并删除边线持续向外跳的异常段。
 */
static void filter_single_edge_outward_jump(
    uint8 *line,
    uint8 top_y,
    uint8 bottom_y,
    uint8 side)
{
    int16 y;
    int16 cursor;
    int16 run;
    int16 step;
    int16 previous_step;
    int16 reference_step;
    int16 outward_total;
    uint8 trend_samples;
    uint8 trend_matches;

    if(line == 0 || top_y >= bottom_y)
    {
        return;
    }

    for(y = (int16)bottom_y - 2;
        y >= (int16)top_y + 2;
        y--)
    {
        if(line[y] == 0 ||
           line[y + 1] == 0 ||
           line[y + 2] == 0)
        {
            continue;
        }

        step =
            (int16)line[y] -
            (int16)line[y + 1];

        previous_step =
            (int16)line[y + 1] -
            (int16)line[y + 2];

        if(abs_i16_value(step - previous_step) <
           SINGLE_EDGE_JUMP_DX)
        {
            continue;
        }

        if(side == EDGE_LEFT)
        {
            if(step >= 0)
            {
                continue;
            }
        }
        else
        {
            if(step <= 0)
            {
                continue;
            }
        }

        run = 1;
        cursor = y - 1;
        reference_step = 0;
        outward_total = abs_i16_value(step);
        trend_samples = 0;
        trend_matches = 0;

        while(cursor >= (int16)top_y &&
              line[cursor] != 0 &&
              line[cursor + 1] != 0 &&
              run < SINGLE_EDGE_MAX_VERIFY_ROWS)
        {
            step =
                (int16)line[cursor] -
                (int16)line[cursor + 1];

            if(side == EDGE_LEFT)
            {
                if(step >= 0)
                {
                    break;
                }
            }
            else
            {
                if(step <= 0)
                {
                    break;
                }
            }

            if(trend_samples == 0)
            {
                reference_step = step;
            }

            trend_samples++;

            if(abs_i16_value(step - reference_step) <=
               SINGLE_EDGE_TREND_TOL)
            {
                trend_matches++;
            }

            outward_total += abs_i16_value(step);
            run++;
            cursor--;
        }

        if(run < SINGLE_EDGE_OUTWARD_RUN ||
           trend_samples < SINGLE_EDGE_OUTWARD_RUN - 1 ||
           trend_matches * 2 < trend_samples + 1 ||
           outward_total < SINGLE_EDGE_MIN_OUTWARD_DX)
        {
            continue;
        }

        for(cursor = 0;
            cursor < run;
            cursor++)
        {
            line[y - cursor] = 0;
        }

        y -= run - 1;
    }
}
/**
 * @brief 统计左右线同时有效且顺序正确的行数。
 */
static uint16 count_common_edge_rows(
    uint8 *left_line,
    uint8 *right_line,
    uint8 top_y,
    uint8 bottom_y)
{
    int16 y;
    uint16 count = 0;

    if(left_line == 0 ||
       right_line == 0 ||
       top_y > bottom_y)
    {
        return 0;
    }

    for(y = (int16)top_y;
        y <= (int16)bottom_y;
        y++)
    {
                // 检查左右边线的有效性是否满足继续处理的条件。
        if(left_line[y] != 0 &&
           right_line[y] != 0 &&
           left_line[y] < right_line[y])
        {
            if(count < 65535)
            {
                count++;
            }
        }
    }

    return count;
}

/**
 * @brief 使用前后行平均替代中线单点尖峰。
 */
static void filter_mid_line_spikes(
    uint8 *mid_line,
    uint8 top_y,
    uint8 bottom_y)
{
    int16 y;
    int16 previous_x;
    int16 current_x;
    int16 next_x;
    int16 neighbor_delta;

    if(mid_line == 0 ||
       top_y + 1 >= bottom_y)
    {
        return;
    }

    for(y = (int16)top_y + 1;
        y < (int16)bottom_y;
        y++)
    {
        if(!mid_line_row_valid(y - 1) ||
           !mid_line_row_valid(y) ||
           !mid_line_row_valid(y + 1))
        {
            continue;
        }

        previous_x = (int16)mid_line[y - 1];
        current_x  = (int16)mid_line[y];
        next_x     = (int16)mid_line[y + 1];

        neighbor_delta =
            abs_i16_value(previous_x - next_x);

                // 检查中线的有效性与连续性是否满足继续处理的条件。
        if(abs_i16_value(current_x - previous_x)
                >= MID_SPIKE_MIN_DELTA &&
           abs_i16_value(current_x - next_x)
                >= MID_SPIKE_MIN_DELTA &&
           neighbor_delta <= MID_SPIKE_NEIGHBOR_TOL)
        {
            mid_line[y] =
                (uint8)((previous_x + next_x) / 2);
        }
    }
}

/* ===== 图像处理参数配置 ===== */
#define MID_OUT_SPIKE_PX    3    /* 参数 MID_OUT_SPIKE_PX：输出中线判定单点尖峰的最小像素差，当前值为 3。 */
#define MID_OUT_PASSES      3    /* 参数 MID_OUT_PASSES：输出中线平滑执行的次数，当前值为 3。 */
#define MID_OUT_MAX_STEP    5    /* 参数 MID_OUT_MAX_STEP：输出中线相邻行允许的最大横向变化，当前值为 5。 */

/**
 * @brief 判断中心线某行能否参与输出平滑。此函数存在重点疑点，见第 15 节。
 */
static uint8 mid_out_row_ok(int16 y)
{
    if(y <= (int16)EDGE_TOP_Y || y > (int16)(jidian_search_line - 1))
    {
        return 0;
    }

    {
        return (ipm_mid_line_valid[y] != 0) ? 1 : 0;
    }

    if(left_line_list[y] == 0 ||
       right_line_list[y] == 0 ||
       left_line_list[y] >= right_line_list[y])
    {
        return 0;
    }

    return 1;
}

/**
 * @brief 中心线输出平滑：小间隙插值、中值滤波、加权平均和最大步长限制。
 */
static void mid_output_smooth(void)
{
    static uint8 src[MT9V03X_H];
    int16 y;
    int16 yy;
    int16 k;
    int16 j;
    int16 t;
    int16 cnt;
    int16 pass;
    int16 v[9];
    int16 med;
    int16 d;
    int16 rep;
    int16 prev_y;
    int16 prev_x;
    int16 gap;
    int32 sum;
        // 检查逆透视补线与 IPM 结果是否满足继续处理的条件。
    if(road_mode == ROAD_MODE_IPM_FILL)
    {
        prev_y = -1;
        prev_x = 0;

        for(y = (int16)(EDGE_TOP_Y + 1); y <= (int16)(jidian_search_line - 1); y++)
        {
            if(ipm_mid_line_valid[y] == 0)
            {
                continue;
            }

            if(prev_y >= 0)
            {
                gap = (int16)(y - prev_y);

                if((gap > 1) && (gap <= (int16)MID_OUT_GAP_FILL))
                {
                    for(k = 1; k < gap; k++)
                    {
                        mid_line_list[prev_y + k] =
                            (uint8)((int16)prev_x +
                                    (int16)(((int32)mid_line_list[y] - (int32)prev_x) * (int32)k / (int32)gap));

                        ipm_mid_line_valid[prev_y + k] = 1;
                    }
                }
            }

            prev_y = y;
            prev_x = (int16)mid_line_list[y];
        }
    }
    for(pass = 0; pass < MID_OUT_PASSES; pass++)
    {
        for(y = 0; y < MT9V03X_H; y++)
        {
            src[y] = mid_line_list[y];
        }

        for(y = (int16)(EDGE_TOP_Y + 1); y <= (int16)(jidian_search_line - 1); y++)
        {
            if(!mid_out_row_ok(y))
            {
                continue;
            }

            cnt = 0;

            for(k = -4; k <= 4; k++)
            {
                yy = (int16)(y + k);

                if(!mid_out_row_ok(yy))
                {
                    continue;
                }

                v[cnt] = (int16)src[yy];
                cnt++;
            }

            if(cnt < 4)
            {
                continue;
            }

            for(j = 1; j < cnt; j++)
            {
                t = v[j];

                for(k = (int16)(j - 1); (k >= 0) && (v[k] > t); k--)
                {
                    v[k + 1] = v[k];
                }

                v[k + 1] = t;
            }

            med = v[cnt >> 1];

            if((mid_line_list[y] > med && (int16)(mid_line_list[y] - med) > (int16)MID_OUT_SPIKE_PX) ||
               (med > mid_line_list[y] && (int16)(med - mid_line_list[y]) > (int16)MID_OUT_SPIKE_PX))
            {
                mid_line_list[y] = (uint8)med;
            }
        }
    }

        // 循环遍历中心线输出平滑：小间隙插值、中值滤波、加权平均和最大步长限制，完成当前统计或筛选。
    for(rep = 0; rep < 2; rep++)
    {
        for(y = 0; y < MT9V03X_H; y++)
        {
            src[y] = mid_line_list[y];
        }

        for(y = (int16)(EDGE_TOP_Y + 3); y <= (int16)(jidian_search_line - 3); y++)
        {
            if(!mid_out_row_ok(y - 3) || !mid_out_row_ok(y - 2) || !mid_out_row_ok(y - 1) ||
               !mid_out_row_ok(y)     || !mid_out_row_ok(y + 1) || !mid_out_row_ok(y + 2) ||
               !mid_out_row_ok(y + 3))
            {
                continue;
            }

            sum =   (int32)4 * (int32)src[y]
                  + (int32)3 * ((int32)src[y - 1] + (int32)src[y + 1])
                  + (int32)2 * ((int32)src[y - 2] + (int32)src[y + 2])
                  +            ((int32)src[y - 3] + (int32)src[y + 3]);

            mid_line_list[y] = (uint8)((sum + 8) / 16);
        }

        for(y = (int16)(EDGE_TOP_Y + 2); y <= (int16)(jidian_search_line - 1); y++)
        {
            if(!mid_out_row_ok(y) || !mid_out_row_ok(y - 1))
            {
                continue;
            }

            d = (int16)mid_line_list[y] - (int16)mid_line_list[y - 1];

            if(d > (int16)MID_OUT_MAX_STEP)
            {
                mid_line_list[y] = (uint8)((int16)mid_line_list[y - 1] + (int16)MID_OUT_MAX_STEP);
            }

            if(d < -(int16)MID_OUT_MAX_STEP)
            {
                mid_line_list[y] = (uint8)((int16)mid_line_list[y - 1] - (int16)MID_OUT_MAX_STEP);
            }
        }
    }
}
/* ===== 函数 filter_ipm_mid_line_spikes ===== */

static void filter_ipm_mid_line_spikes(
    uint8 *mid_line,
    uint8 top_y,
    uint8 bottom_y)
{
    int16 y;
    int16 previous_x;
    int16 current_x;
    int16 next_x;

    if(mid_line == 0 ||
       top_y + 1 >= bottom_y)
    {
        return;
    }

    for(y = (int16)top_y + 1;
        y < (int16)bottom_y;
        y++)
    {
        if(!ipm_mid_line_valid[y - 1] ||
           !ipm_mid_line_valid[y] ||
           !ipm_mid_line_valid[y + 1])
        {
            continue;
        }

        previous_x = mid_line[y - 1];
        current_x  = mid_line[y];
        next_x     = mid_line[y + 1];

        if(abs_i16_value(current_x - previous_x) >=
               MID_SPIKE_MIN_DELTA &&
           abs_i16_value(current_x - next_x) >=
               MID_SPIKE_MIN_DELTA &&
           abs_i16_value(previous_x - next_x) <=
               MID_SPIKE_NEIGHBOR_TOL)
        {
            mid_line[y] =
                (uint8)((previous_x + next_x) / 2);
        }
    }
}


/**
 * @brief 检测中线跳走后又返回的情况，并使用斜率估计填回异常段。
 */
static void repair_mid_line_continuation(
    uint8 *line,
    uint8 first_y,
    uint8 last_y)
{
    uint8 raw_line[MT9V03X_H];
    uint8 work_line[MT9V03X_H];

    uint16 i;

    int16 y;
    int16 j;
    int16 run;

    int16 slope1;
    int16 slope2;
    int16 slope3;
    int16 slope;

    int16 slope_delta12;
    int16 slope_delta23;

    int16 predicted;
    int16 predicted_at_return;
    int16 predicted_after_return;

    int16 current_delta;
    int16 return_delta;
    int16 after_return_delta;

    int16 current_value;
    int16 return_value;
    int16 after_return_value;

    int16 return_y;
    int16 after_return_y;

    int16 replacement;
    uint8 repaired;

    if(line == 0)
    {
        return;
    }

    if(first_y >= MT9V03X_H)
    {
        return;
    }

    if(last_y >= MT9V03X_H)
    {
        last_y = MT9V03X_H - 1;
    }

    if((int16)last_y <= (int16)first_y + 8)
    {
        return;
    }

        // 循环遍历检测中线跳走后又返回的情况，并使用斜率估计填回异常段，完成当前统计或筛选。
    for(i = 0; i < MT9V03X_H; i++)
    {
        raw_line[i] = line[i];
        work_line[i] = line[i];
    }

        // 逐行遍历检测中线跳走后又返回的情况，并使用斜率估计填回异常段。
    for(y = (int16)last_y - 4;
        y >= (int16)first_y + 2;
        y--)
    {
        repaired = 0;

                // 检查中线的有效性与连续性是否满足继续处理的条件。
        if(mid_line_row_valid(y) == 0 ||
           mid_line_row_valid(y + 1) == 0 ||
           mid_line_row_valid(y + 2) == 0 ||
           mid_line_row_valid(y + 3) == 0 ||
           mid_line_row_valid(y + 4) == 0)
        {
            continue;
        }

                /*
         * 更新边线斜率与直线拟合相关变量。
         */
        slope1 =
            (int16)work_line[y + 1] -
            (int16)work_line[y + 2];

        slope2 =
            (int16)work_line[y + 2] -
            (int16)work_line[y + 3];

        slope3 =
            (int16)work_line[y + 3] -
            (int16)work_line[y + 4];

        slope_delta12 = slope1 - slope2;

        if(slope_delta12 < 0)
        {
            slope_delta12 = -slope_delta12;
        }

        slope_delta23 = slope2 - slope3;

        if(slope_delta23 < 0)
        {
            slope_delta23 = -slope_delta23;
        }

                // 检查中线的有效性与连续性是否满足继续处理的条件。
        if(slope_delta12 > MID_CONT_SLOPE_TOL ||
           slope_delta23 > MID_CONT_SLOPE_TOL)
        {
            continue;
        }

                /*
         * 更新边线斜率与直线拟合相关变量。
         */
        slope =
            (slope1 + slope2 + slope3) / 3;

        predicted =
            (int16)work_line[y + 1] +
            slope;

        if(predicted < 1)
        {
            predicted = 1;
        }

        if(predicted > MT9V03X_W - 2)
        {
            predicted = MT9V03X_W - 2;
        }

        current_value =
            (int16)raw_line[y];

        current_delta =
            current_value -
            predicted;

        if(current_delta < 0)
        {
            current_delta = -current_delta;
        }

                // 检查中线的有效性与连续性是否满足继续处理的条件。
        if(current_delta < MID_CONT_JUMP_MIN)
        {
            continue;
        }

                // 循环遍历检测中线跳走后又返回的情况，并使用斜率估计填回异常段，完成当前统计或筛选。
        for(run = 1;
            run <= MID_CONT_MAX_RUN;
            run++)
        {
            return_y =
                y - run;

            after_return_y =
                return_y - 1;

            if(return_y < (int16)first_y ||
               after_return_y < (int16)first_y)
            {
                break;
            }

            if(mid_line_row_valid(return_y) == 0 ||
               mid_line_row_valid(after_return_y) == 0)
            {
                continue;
            }

                        /*
             * 更新边线斜率与直线拟合相关变量。
             */
            predicted_at_return =
                predicted +
                slope * run;

            predicted_after_return =
                predicted_at_return +
                slope;

            if(predicted_at_return < 1)
            {
                predicted_at_return = 1;
            }

            if(predicted_at_return > MT9V03X_W - 2)
            {
                predicted_at_return = MT9V03X_W - 2;
            }

            if(predicted_after_return < 1)
            {
                predicted_after_return = 1;
            }

            if(predicted_after_return > MT9V03X_W - 2)
            {
                predicted_after_return = MT9V03X_W - 2;
            }

            return_value =
                (int16)raw_line[return_y];

            after_return_value =
                (int16)raw_line[after_return_y];

            return_delta =
                return_value -
                predicted_at_return;

            if(return_delta < 0)
            {
                return_delta = -return_delta;
            }

            after_return_delta =
                after_return_value -
                predicted_after_return;

            if(after_return_delta < 0)
            {
                after_return_delta = -after_return_delta;
            }

                        // 检查中线的有效性与连续性是否满足继续处理的条件。
            if(return_delta <= MID_CONT_RETURN_TOL &&
               after_return_delta <= MID_CONT_RETURN2_TOL)
            {
                                // 循环遍历检测中线跳走后又返回的情况，并使用斜率估计填回异常段，完成当前统计或筛选。
                for(j = 0; j < run; j++)
                {
                    replacement =
                        predicted +
                        slope * j;

                    if(replacement < 1)
                    {
                        replacement = 1;
                    }

                    if(replacement > MT9V03X_W - 2)
                    {
                        replacement = MT9V03X_W - 2;
                    }

                    work_line[y - j] =
                        (uint8)replacement;

                    line[y - j] =
                        (uint8)replacement;
                }

                repaired = 1;

                                /*
                 * 执行检测中线跳走后又返回的情况，并使用斜率估计填回异常段相关操作。
                 */
                y -= run - 1;

                break;
            }
        }

        if(repaired)
        {
            continue;
        }
    }
}
static int16 cross_abs_i16_value(int16 value)
{
    return (value < 0) ? -value : value;
}

/**
 * @brief 检测中线突变时清空对应的左右边线。
 */
static void invalidate_cross_mid_jump_segment(
    uint8 *left_line,
    uint8 *right_line,
    uint8 top_y,
    uint8 bottom_y)
{
    uint8 raw_mid[MT9V03X_H];

    int16 y;
    int16 run;
    int16 j;

    int16 slope1;
    int16 slope2;
    int16 slope3;
    int16 slope;

    int16 predicted;
    int16 predicted_return;
    int16 predicted_after;

    int16 return_y;
    int16 after_return_y;

    int16 current_delta;
    int16 return_delta;
    int16 after_delta;

    if(left_line == 0 ||
       right_line == 0 ||
       top_y >= bottom_y)
    {
        return;
    }

    memset(raw_mid, 0, sizeof(raw_mid));

        // 逐行遍历检测中线突变时清空对应的左右边线。
    for(y = (int16)top_y;
        y <= (int16)bottom_y;
        y++)
    {
        if(left_line[y] != 0 &&
           right_line[y] != 0 &&
           left_line[y] < right_line[y])
        {
            raw_mid[y] =
                (uint8)(((uint16)left_line[y] +
                         (uint16)right_line[y]) / 2U);
        }
    }

        // 逐行遍历检测中线突变时清空对应的左右边线。
    for(y = (int16)bottom_y - 4;
        y >= (int16)top_y + 2;
        y--)
    {
        if(raw_mid[y] == 0 ||
           raw_mid[y + 1] == 0 ||
           raw_mid[y + 2] == 0 ||
           raw_mid[y + 3] == 0 ||
           raw_mid[y + 4] == 0)
        {
            continue;
        }

        slope1 =
            (int16)raw_mid[y + 1] -
            (int16)raw_mid[y + 2];

        slope2 =
            (int16)raw_mid[y + 2] -
            (int16)raw_mid[y + 3];

        slope3 =
            (int16)raw_mid[y + 3] -
            (int16)raw_mid[y + 4];

        if(cross_abs_i16_value(slope1 - slope2) >
               MID_CONT_SLOPE_TOL ||
           cross_abs_i16_value(slope2 - slope3) >
               MID_CONT_SLOPE_TOL)
        {
            continue;
        }

        slope =
            (slope1 + slope2 + slope3) / 3;

        predicted =
            (int16)raw_mid[y + 1] +
            slope;

        current_delta =
            cross_abs_i16_value(
                (int16)raw_mid[y] -
                predicted);

                // 检查中线的有效性与连续性是否满足继续处理的条件。
        if(current_delta < MID_CONT_JUMP_MIN)
        {
            continue;
        }

        for(run = 1;
            run <= MID_CONT_MAX_RUN;
            run++)
        {
            return_y =
                y - run;

            after_return_y =
                return_y - 1;

            if(return_y < (int16)top_y ||
               after_return_y < (int16)top_y)
            {
                break;
            }

            if(raw_mid[return_y] == 0 ||
               raw_mid[after_return_y] == 0)
            {
                continue;
            }

            predicted_return =
                predicted +
                slope * run;

            predicted_after =
                predicted_return +
                slope;

            return_delta =
                cross_abs_i16_value(
                    (int16)raw_mid[return_y] -
                    predicted_return);

            after_delta =
                cross_abs_i16_value(
                    (int16)raw_mid[after_return_y] -
                    predicted_after);

                        // 检查中线的有效性与连续性是否满足继续处理的条件。
            if(return_delta <= MID_CONT_RETURN_TOL &&
               after_delta <= MID_CONT_RETURN2_TOL)
            {
                for(j = 0; j < run; j++)
                {
                                        /*
                     * 更新左右边线的有效性对应的数组状态。
                     */
                    left_line[y - j] = 0;
                    right_line[y - j] = 0;
                    raw_mid[y - j] = 0;
                }

                y -= run - 1;
                break;
            }
        }
    }
}
static int16 model_to_pixel(
    const CrossFitModel *model,
    uint8 y)
{
    float x;
    int16 pixel;

    x = model->slope * (float)y + model->intercept;

    if(x >= 0.0f)
    {
        pixel = (int16)(x + 0.5f);
    }
    else
    {
        pixel = (int16)(x - 0.5f);
    }

    if(pixel < 1)
    {
        pixel = 1;
    }

    if(pixel > MT9V03X_W - 2)
    {
        pixel = MT9V03X_W - 2;
    }

    return pixel;
}

/**
 * @brief 使用上下两个模型填充边线，当前没有调用。
 */
static void fill_edge_bidirectional(
    uint8 *line,
    uint8 target_top,
    uint8 target_bottom,
    uint8 valid_top,
    uint8 valid_bottom,
    const CrossFitModel *top_model,
    const CrossFitModel *bottom_model)
{
    int16 y;
    int16 predicted_pixel;
    const CrossFitModel *model;

    if(line == 0 || top_model == 0 || bottom_model == 0 ||
       target_top > target_bottom ||
       valid_top > valid_bottom)
    {
        return;
    }

    for(y = (int16)target_top; y <= (int16)target_bottom; y++)
    {
        if(y < (int16)valid_top)
        {
            model = top_model;
        }
        else if(y > (int16)valid_bottom)
        {
            model = bottom_model;
        }
        else if(line[y] != 0)
        {
            continue;
        }
        else if(y - (int16)valid_top <=
                (int16)valid_bottom - y)
        {
            model = top_model;
        }
        else
        {
            model = bottom_model;
        }

        predicted_pixel =
            model_to_pixel(model, (uint8)y);

        line[y] = (uint8)predicted_pixel;
    }
}
/**
 * @brief 根据上下模型的有效性和点数选择模型进行预测。
 */
static uint8 predict_edge_from_models(
    uint8 y,
    uint8 valid_top,
    uint8 valid_bottom,
    uint8 top_ok,
    uint8 bottom_ok,
    const CrossFitModel *top_model,
    const CrossFitModel *bottom_model,
    int16 *pixel)
{
    const CrossFitModel *model = 0;
    const CrossFitModel *preferred_model = 0;

    if(pixel == 0)
    {
        return 0;
    }

        // 检查根据上下模型的有效性和点数选择模型进行预测是否可用；条件不满足时进入回退分支。
    if(top_ok &&
       (!bottom_ok ||
        top_model->point_count >= bottom_model->point_count))
    {
        preferred_model = top_model;
    }
    else if(bottom_ok)
    {
        preferred_model = bottom_model;
    }

    if(preferred_model == 0)
    {
        return 0;
    }

        // 检查根据上下模型的有效性和点数选择模型进行预测是否满足继续处理的条件。
    if(y < valid_top ||
       y > valid_bottom)
    {
        model = preferred_model;
    }
    else if(top_ok && bottom_ok)
    {
                // 检查根据上下模型的有效性和点数选择模型进行预测是否满足继续处理的条件。
        if((uint16)y - (uint16)valid_top <=
           (uint16)valid_bottom - (uint16)y)
        {
            model = top_model;
        }
        else
        {
            model = bottom_model;
        }
    }
    else if(top_ok)
    {
        model = top_model;
    }
    else
    {
        model = bottom_model;
    }

    if(model == 0)
    {
        return 0;
    }

    *pixel = model_to_pixel(model, y);

    return 1;
}

/**
 * @brief 十字补线前采样顶部、中部、底部，检查宽度和斜率是否合理。
 */
static uint8 mixed_models_are_plausible(
    uint8 target_top,
    uint8 target_bottom,
    uint8 left_valid_top,
    uint8 left_valid_bottom,
    uint8 left_top_ok,
    uint8 left_bottom_ok,
    const CrossFitModel *left_top_model,
    const CrossFitModel *left_bottom_model,
    uint8 right_valid_top,
    uint8 right_valid_bottom,
    uint8 right_top_ok,
    uint8 right_bottom_ok,
    const CrossFitModel *right_top_model,
    const CrossFitModel *right_bottom_model)
{
    uint8 sample;
    uint8 sample_y[3];
    int16 left_pixel;
    int16 right_pixel;
    int16 width;

    if(target_top > target_bottom ||
       (!left_top_ok && !left_bottom_ok) ||
       (!right_top_ok && !right_bottom_ok))
    {
        return 0;
    }

    if((left_top_ok &&
        (left_top_model->slope > CROSS_MAX_SLOPE ||
         left_top_model->slope < -CROSS_MAX_SLOPE)) ||
       (left_bottom_ok &&
        (left_bottom_model->slope > CROSS_MAX_SLOPE ||
         left_bottom_model->slope < -CROSS_MAX_SLOPE)) ||
       (right_top_ok &&
        (right_top_model->slope > CROSS_MAX_SLOPE ||
         right_top_model->slope < -CROSS_MAX_SLOPE)) ||
       (right_bottom_ok &&
        (right_bottom_model->slope > CROSS_MAX_SLOPE ||
         right_bottom_model->slope < -CROSS_MAX_SLOPE)))
    {
        return 0;
    }

    sample_y[0] = target_top;
    sample_y[1] = (uint8)(((uint16)target_top +
                            (uint16)target_bottom) / 2U);
    sample_y[2] = target_bottom;

    for(sample = 0; sample < 3; sample++)
    {
        if(!predict_edge_from_models(
               sample_y[sample],
               left_valid_top,
               left_valid_bottom,
               left_top_ok,
               left_bottom_ok,
               left_top_model,
               left_bottom_model,
               &left_pixel) ||
           !predict_edge_from_models(
               sample_y[sample],
               right_valid_top,
               right_valid_bottom,
               right_top_ok,
               right_bottom_ok,
               right_top_model,
               right_bottom_model,
               &right_pixel))
        {
            return 0;
        }

        width = right_pixel - left_pixel;
        if(left_pixel >= right_pixel ||
           width < CROSS_MIN_WIDTH ||
           width > MT9V03X_W - 4)
        {
            return 0;
        }
    }

    return 1;
}

static int16 clamp_edge_pixel(int16 pixel)
{
    if(pixel < 1)
    {
        return 1;
    }

    if(pixel > MT9V03X_W - 2)
    {
        return MT9V03X_W - 2;
    }

    return pixel;
}

static int16 model_pixel_anchored(
    const CrossFitModel *model,
    int16 anchor_y,
    int16 anchor_x,
    int16 y)
{
    float predicted;
    int16 result;

    if(model == 0)
    {
        return anchor_x;
    }

        /*
     * 更新边线斜率与直线拟合相关变量。
     */
    predicted =
        (float)anchor_x +
        model->slope * (float)(y - anchor_y);

    if(predicted >= 0.0f)
    {
        result = (int16)(predicted + 0.5f);
    }
    else
    {
        result = (int16)(predicted - 0.5f);
    }

    return clamp_edge_pixel(result);
}

/**
 * @brief 补边线核心：已有值不动，中间缺行线性插值，外部缺行按模型外推。
 */
static void fill_edge_with_available_models(
    uint8 *line,
    uint8 target_top,
    uint8 target_bottom,
    uint8 valid_top,
    uint8 valid_bottom,
    uint8 top_ok,
    uint8 bottom_ok,
    const CrossFitModel *top_model,
    const CrossFitModel *bottom_model)
{
    int16 y;
    int16 upper_y;
    int16 lower_y;
    int16 predicted;
    int16 upper_x;
    int16 lower_x;
    const CrossFitModel *model;

    if(line == 0 ||
       target_top > target_bottom ||
       valid_top > valid_bottom)
    {
        return;
    }

    for(y = (int16)target_top;
        y <= (int16)target_bottom;
        y++)
    {
                // 检查补边线核心：已有值不动，中间缺行线性插值，外部缺行按模型外推是否满足继续处理的条件。
        if(line[y] != 0)
        {
            continue;
        }

                // 检查补边线核心：已有值不动，中间缺行线性插值，外部缺行按模型外推是否满足继续处理的条件。
        if(y < (int16)valid_top)
        {
            if(!top_ok || top_model == 0)
            {
                continue;
            }

            predicted =
                model_pixel_anchored(
                    top_model,
                    valid_top,
                    line[valid_top],
                    y);

            line[y] = (uint8)predicted;
            continue;
        }

                // 检查补边线核心：已有值不动，中间缺行线性插值，外部缺行按模型外推是否满足继续处理的条件。
        if(y > (int16)valid_bottom)
        {
            if(!bottom_ok || bottom_model == 0)
            {
                continue;
            }

            predicted =
                model_pixel_anchored(
                    bottom_model,
                    valid_bottom,
                    line[valid_bottom],
                    y);

            line[y] = (uint8)predicted;
            continue;
        }

                /*
         * 更新补边线核心：已有值不动，中间缺行线性插值，外部缺行按模型外推相关变量。
         */
        upper_y = y - 1;
        while(upper_y >= (int16)valid_top &&
              line[upper_y] == 0)
        {
            upper_y--;
        }

        lower_y = y + 1;
        while(lower_y <= (int16)valid_bottom &&
              line[lower_y] == 0)
        {
            lower_y++;
        }

                // 检查补边线核心：已有值不动，中间缺行线性插值，外部缺行按模型外推是否满足继续处理的条件。
        if(upper_y >= (int16)valid_top &&
           lower_y <= (int16)valid_bottom)
        {
            upper_x = line[upper_y];
            lower_x = line[lower_y];

            predicted =
                upper_x +
                (lower_x - upper_x) *
                (y - upper_y) /
                (lower_y - upper_y);

            line[y] =
                (uint8)clamp_edge_pixel(predicted);

            continue;
        }

                // 检查补边线核心：已有值不动，中间缺行线性插值，外部缺行按模型外推是否满足继续处理的条件。
        if(upper_y >= (int16)valid_top)
        {
            upper_x = line[upper_y];

            if(top_ok && top_model != 0)
            {
                model = top_model;
            }
            else if(bottom_ok && bottom_model != 0)
            {
                model = bottom_model;
            }
            else
            {
                continue;
            }

            predicted =
                model_pixel_anchored(
                    model,
                    upper_y,
                    upper_x,
                    y);

            line[y] = (uint8)predicted;
        }
        else if(lower_y <= (int16)valid_bottom)
        {
            lower_x = line[lower_y];

            if(bottom_ok && bottom_model != 0)
            {
                model = bottom_model;
            }
            else if(top_ok && top_model != 0)
            {
                model = top_model;
            }
            else
            {
                continue;
            }

            predicted =
                model_pixel_anchored(
                    model,
                    lower_y,
                    lower_x,
                    y);

            line[y] = (uint8)predicted;
        }
    }
}
/**
 * @brief 检查左右完整模型是否保持合理宽度和斜率，当前没有直接调用。
 */
static uint8 cross_models_are_plausible(
    const CrossFitModel *left_model,
    const CrossFitModel *right_model,
    uint8 from_y,
    uint8 to_y)
{
    float left_from;
    float right_from;
    float left_to;
    float right_to;
    float width_from;
    float width_to;

    if(left_model == 0 || right_model == 0 || to_y <= from_y)
    {
        return 0;
    }

    if(left_model->slope > CROSS_MAX_SLOPE ||
       left_model->slope < -CROSS_MAX_SLOPE ||
       right_model->slope > CROSS_MAX_SLOPE ||
       right_model->slope < -CROSS_MAX_SLOPE)
    {
        return 0;
    }

    left_from = left_model->slope * (float)from_y + left_model->intercept;
    right_from = right_model->slope * (float)from_y + right_model->intercept;
    left_to = left_model->slope * (float)to_y + left_model->intercept;
    right_to = right_model->slope * (float)to_y + right_model->intercept;

    width_from = right_from - left_from;
    width_to = right_to - left_to;

    if(left_from < 1.0f || left_from > (float)(MT9V03X_W - 2) ||
       right_from < 1.0f || right_from > (float)(MT9V03X_W - 2) ||
       left_to < 1.0f || left_to > (float)(MT9V03X_W - 2) ||
       right_to < 1.0f || right_to > (float)(MT9V03X_W - 2))
    {
        return 0;
    }

    if(width_from < (float)CROSS_MIN_WIDTH ||
       width_to < (float)CROSS_MIN_WIDTH ||
       width_from > (float)(MT9V03X_W - 4) ||
       width_to > (float)(MT9V03X_W - 4))
    {
        return 0;
    }

    return 1;
}
/**
 * @brief 从起止行向下填充模型预测值，当前没有直接调用。
 */
static void cross_fill_down(
    uint8 *line,
    uint8 from_y,
    uint8 to_y,
    const CrossFitModel *model)
{
    int16 y;

    float predicted_x;
    int16 predicted_pixel;

    if(line == 0 || model == 0)
    {
        return;
    }

        // 检查从起止行向下填充模型预测值，当前没有直接调用是否满足继续处理的条件。
    if(to_y <= from_y)
    {
        return;
    }

        // 逐行遍历从起止行向下填充模型预测值，当前没有直接调用。
    for(y = (int16)from_y + 1;
        y <= (int16)to_y;
        y++)
    {
                /*
         * 更新边线斜率与直线拟合相关变量。
         */
        predicted_x =
            model->slope * (float)y +
            model->intercept;

                // 检查从起止行向下填充模型预测值，当前没有直接调用是否满足继续处理的条件。
        if(predicted_x >= 0.0f)
        {
            predicted_pixel =
                (int16)(predicted_x + 0.5f);
        }
        else
        {
            predicted_pixel =
                (int16)(predicted_x - 0.5f);
        }

                // 检查从起止行向下填充模型预测值，当前没有直接调用是否满足继续处理的条件。
        if(predicted_pixel < 1)
        {
            predicted_pixel = 1;
        }

        if(predicted_pixel > MT9V03X_W - 2)
        {
            predicted_pixel = MT9V03X_W - 2;
        }

                /*
         * 更新从起止行向下填充模型预测值，当前没有直接调用对应的数组状态。
         */
        line[y] = (uint8)predicted_pixel;
    }
}
/**
 * @brief 左右种子距离过近或顺序反转时，丢弃离中心更远的一侧。
 */
static void validate_single_seed_pair(
    uint8 *left_ok,
    uint8 *right_ok,
    uint8 left_x,
    uint8 right_x)
{
    uint16 left_distance;
    uint16 right_distance;

    if(left_ok == 0 ||
       right_ok == 0)
    {
        return;
    }

    if(!(*left_ok) ||
       !(*right_ok))
    {
        return;
    }

        // 检查巡线种子点是否满足继续处理的条件。
    if(right_x > left_x &&
       ((uint16)right_x - (uint16)left_x)
       >= SINGLE_SEED_MIN_X_GAP)
    {
        return;
    }

        // 检查中线的有效性与连续性是否满足继续处理的条件。
    if(left_x < MID_W)
    {
        left_distance =
            (uint16)MID_W - (uint16)left_x;
    }
    else
    {
        left_distance = 0;
    }

    if(right_x > MID_W)
    {
        right_distance =
            (uint16)right_x - (uint16)MID_W;
    }
    else
    {
        right_distance = 0;
    }

    if(left_distance >= right_distance)
    {
        *right_ok = 0;
    }
    else
    {
        *left_ok = 0;
    }
}

/**
 * @brief 清空结果后重新选择种子并左右独立重爬。
 */
static uint8 retry_single_side_trace(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 min_y,
    uint8 max_y,
    uint8 *left_ok_out,
    uint8 *right_ok_out,
    uint8 *left_start_y_out,
    uint8 *right_start_y_out,
    uint8 *left_start_x_out,
    uint8 *right_start_x_out,
    uint8 *left_top_out,
    uint8 *right_top_out)
{
    uint8 left_x = 0;
    uint8 right_x = 0;
    uint8 left_y = 0;
    uint8 right_y = 0;

    uint8 left_ok;
    uint8 right_ok;

    uint8 left_trace_top;
    uint8 right_trace_top;

    if(img == 0 ||
       left_ok_out == 0 ||
       right_ok_out == 0 ||
       left_start_y_out == 0 ||
       right_start_y_out == 0 ||
       left_start_x_out == 0 ||
       right_start_x_out == 0 ||
       left_top_out == 0 ||
       right_top_out == 0 ||
       min_y > max_y)
    {
        return 0;
    }

    left_ok =
        choose_single_side_seed(
            img,
            min_y,
            max_y,
            EDGE_LEFT,
            &left_y,
            &left_x);

    right_ok =
        choose_single_side_seed(
            img,
            min_y,
            max_y,
            EDGE_RIGHT,
            &right_y,
            &right_x);
        /*
     * 左右种子距离过近或顺序反转时，丢弃离中心更远的一侧。
     */
    validate_single_seed_pair(
        &left_ok,
        &right_ok,
        left_x,
        right_x);
        /*
     * 左右种子距离过近或顺序反转时，丢弃离中心更远的一侧。
     */
    validate_single_seed_pair(
        &left_ok,
        &right_ok,
        left_x,
        right_x);

        // 检查清空结果后重新选择种子并左右独立重爬是否可用；条件不满足时进入回退分支。
    if(!left_ok && !right_ok)
    {
        return 0;
    }

        // 左侧边线逐行缓存；每一行保存一个横坐标。
    memset(left_line_list, 0, sizeof(left_line_list));
    memset(right_line_list, 0, sizeof(right_line_list));

    memset(Left_Edge_Pts, 0, sizeof(Left_Edge_Pts));
    memset(Right_Edge_Pts, 0, sizeof(Right_Edge_Pts));

    Left_Edge_Count = 0;
    Right_Edge_Count = 0;

    left_trace_top = EDGE_TOP_Y;
    right_trace_top = EDGE_TOP_Y;

    if(left_ok)
    {
        if(RING_LONG_TIGHT(EDGE_LEFT))          /* 检查环岛状态与环岛边线是否满足继续处理的条件。 */
        {
            Left_Edge_Count =
                trace_edge_cross(
                    img, left_x, left_y, EDGE_LEFT,
                    Left_Edge_Pts, left_line_list, &left_trace_top);
        }
        else
        {
            Left_Edge_Count =
                trace_edge(
                    img, left_x, left_y, EDGE_LEFT,
                    Left_Edge_Pts, left_line_list, &left_trace_top);
        }
    }

    if(right_ok)
    {
        if(RING_LONG_TIGHT(EDGE_RIGHT))
        {
            Right_Edge_Count =
                trace_edge_cross(
                    img, right_x, right_y, EDGE_RIGHT,
                    Right_Edge_Pts, right_line_list, &right_trace_top);
        }
        else
        {
            Right_Edge_Count =
                trace_edge(
                    img, right_x, right_y, EDGE_RIGHT,
                    Right_Edge_Pts, right_line_list, &right_trace_top);
        }
    }

    *left_ok_out = left_ok;
    *right_ok_out = right_ok;

    *left_start_y_out = left_y;
    *right_start_y_out = right_y;

    *left_start_x_out = left_x;
    *right_start_x_out = right_x;

    *left_top_out = left_trace_top;
    *right_top_out = right_trace_top;

    return 1;
}
/**
 * @brief 从较可靠的一侧补出另一侧和中心候选。
 */
static uint8 fill_missing_side_by_ipm(
    uint8 *left_line,
    uint8 *right_line,
    uint8 top_y,
    uint8 bottom_y,
    uint16 left_count,
    uint16 right_count);
/**
 * @brief 将 IPM 中心候选写回中心线数组。
 */
static uint16 build_mid_line_from_ipm_candidates(
    uint8 *mid_line,
    uint8 top_y,
    uint8 bottom_y);
/**
 * @brief 更严格的十字白区二次检测。
 */
static uint8 detect_cross_white_gap_double(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 top_y,
    uint8 bottom_y,
    uint8 *gap_top_y);
/**
 * @brief 统计某条线在指定 y 范围内的非零行数。
 */
static uint16 count_valid_edge_rows(
    uint8 *line,
    uint8 top_y,
    uint8 bottom_y)
{
    int16 y;
    uint16 count = 0;

    if(line == 0 ||
       top_y > bottom_y)
    {
        return 0;
    }

    for(y = (int16)top_y;
        y <= (int16)bottom_y;
        y++)
    {
        if(line[y] != 0)
        {
            count++;
        }
    }

    return count;
}
/* ===== 图像处理参数配置 ===== */
#define EDGE_REFILL_GAP_ROWS    3

static uint8 edge_tmp_x[MT9V03X_H];      /* 定义edgetmpx变量，用于当前图像处理过程。 */

/* ===== 图像处理参数配置 ===== */
#define EDGE_CUT_STEP_ROWS      3    /* 参数 EDGE_CUT_STEP_ROWS：边线尾部向外漂移检查的分组行数，当前值为 3。 */
#define EDGE_CUT_OUTWARD_SUM    7    /* 参数 EDGE_CUT_OUTWARD_SUM：边线尾部连续向外漂移的累计判定阈值，当前值为 7。 */

static uint16 edge_cut_outward_tail(uint8 *line, uint8 side, uint8 top_y, uint8 bot_y)
{
    uint16 rows[MT9V03X_H];
    uint16 cnt = 0;
    uint16 i;
    uint16 k;
    uint16 n = 0;
    int16  y;
    int16  step;
    int16  total;
    uint8  all_out;

    if(line == 0)
    {
        return 0;
    }

        // 逐行遍历每 3 个有效行一组检查，持续向外漂移时删除尾部。
    for(y = (int16)bot_y; y >= (int16)top_y; y--)
    {
        if(line[y] != 0)
        {
            rows[cnt] = (uint16)y;
            cnt++;
        }
    }

    if(cnt < (uint16)EDGE_CUT_STEP_ROWS)
    {
        return 0;
    }

        // 逐行遍历左右边线的有效性。
    for(i = 0; (i + (uint16)EDGE_CUT_STEP_ROWS) <= cnt; i += (uint16)EDGE_CUT_STEP_ROWS)
    {
        all_out = 1;

        for(k = 1; k < (uint16)EDGE_CUT_STEP_ROWS; k++)
        {
            step = (int16)line[rows[i + k]] - (int16)line[rows[i]];

            if(side == EDGE_LEFT)
            {
                if(step >= 0)               /* 检查每 3 个有效行一组检查，持续向外漂移时删除尾部是否满足继续处理的条件。 */
                {
                    all_out = 0;
                    break;
                }
            }
            else
            {
                if(step <= 0)               /* 检查每 3 个有效行一组检查，持续向外漂移时删除尾部是否满足继续处理的条件。 */
                {
                    all_out = 0;
                    break;
                }
            }
        }

        if(all_out == 0)
        {
            continue;
        }

        total = (int16)line[rows[i + (uint16)EDGE_CUT_STEP_ROWS - 1]] -
                (int16)line[rows[i]];

        if(total < 0)
        {
            total = (int16)(-total);
        }

        if(total > (int16)EDGE_CUT_OUTWARD_SUM)
        {
                        // 逐行遍历左右边线的有效性。
            for(k = (uint16)(i + (uint16)EDGE_CUT_STEP_ROWS); k < cnt; k++)
            {
                if(line[rows[k]] != 0)
                {
                    line[rows[k]] = 0;
                    n++;
                }
            }

            return n;
        }
    }

    return 0;
}
/* ===== 函数 edge_line_smooth_refill ===== */
static uint16 edge_line_smooth_refill(uint8 *line, uint8 top_y, uint8 bot_y)
{
    int16  y;
    int16  prev_y;
    int16  prev_x;
    int16  gap;
    int16  k;
    int16  yy;
    int16  xi;
    int16  w;
    int32  sum;
    int32  wsum;
    uint8  ok;
    uint16 n = 0;

    if(line == 0 || top_y > bot_y)
    {
        return 0;
    }

        // 逐行遍历边线 7 行加权平滑并线性填补最多 3 行小间隙。
    for(y = (int16)top_y; y <= (int16)bot_y; y++)
    {
        edge_tmp_x[y] = line[y];
    }

        // 参数 EDGE_BLUR_HALF：边线平滑窗口的半宽，当前值为 3。
    #define EDGE_BLUR_HALF   3      /* 参数 EDGE_BLUR_HALF：边线平滑窗口的半宽，当前值为 3。 */

    for(y = (int16)(top_y + EDGE_BLUR_HALF); y <= (int16)(bot_y - EDGE_BLUR_HALF); y++)
    {
        if(line[y] == 0)
        {
            continue;
        }

        ok = 1;

        for(k = -(int16)EDGE_BLUR_HALF; k <= (int16)EDGE_BLUR_HALF; k++)
        {
            if(edge_tmp_x[y + k] == 0)
            {
                ok = 0;
                break;
            }
        }

        if(!ok)
        {
            continue;                       /* 跳过本次循环，继续处理下一行。 */
        }

        sum  = 0;
        wsum = 0;

        for(k = -(int16)EDGE_BLUR_HALF; k <= (int16)EDGE_BLUR_HALF; k++)
        {
            w = (int16)((int16)EDGE_BLUR_HALF + 1 - ((k < 0) ? -k : k));

            sum  += (int32)w * (int32)edge_tmp_x[y + k];
            wsum += (int32)w;
        }

        xi = (int16)((sum + (wsum / 2)) / wsum);

        if(xi >= 1 && xi <= MT9V03X_W - 2)
        {
            line[y] = (uint8)xi;
        }
    }

        /*
     * 更新边线 7 行加权平滑并线性填补最多 3 行小间隙相关变量。
     */
    prev_y = -1;
    prev_x = 0;

    for(y = (int16)top_y; y <= (int16)bot_y; y++)
    {
        if(line[y] == 0)
        {
            continue;
        }

        if(prev_y >= 0)
        {
            gap = (int16)(y - prev_y);

            if(gap > 1 && gap <= (int16)EDGE_REFILL_GAP_ROWS)
            {
                for(k = 1; k < gap; k++)
                {
                    yy = (int16)(prev_y + k);

                    if(line[yy] != 0)
                    {
                        continue;
                    }

                    xi = (int16)((int32)prev_x +
                                 ((int32)line[y] - (int32)prev_x) * (int32)k / (int32)gap);

                    if(xi < 1 || xi > MT9V03X_W - 2)
                    {
                        continue;
                    }

                    line[yy] = (uint8)xi;
                    n++;
                }
            }
        }

        prev_y = y;
        prev_x = line[y];
    }

    return n;
}
/**
 * @brief 巡线主流程：选择边线种子、执行八邻域爬线、处理十字/急弯/IPM 补线，生成并平滑中线，同时更新当前道路模式。
 */
void image_deal(uint8 index[MT9V03X_H][MT9V03X_W])
{uint16 left_valid_row_count;
uint16 right_valid_row_count;
    uint8 left_start_y;
    uint8 right_start_y;
    uint8 independent_seed_mode;
    uint8 left_seed_ok;
    uint8 right_seed_ok;
    uint8 independent_retry_done;

    uint8 retry_left_start_y;
    uint8 retry_right_start_y;
    uint8 retry_left_start;
    uint8 retry_right_start;
    uint8 retry_left_ok;
    uint8 retry_right_ok;

    uint8 cross_scan_bottom;


    uint8 left_span_ok;
    uint8 right_span_ok;
    uint16 lane_width_hint;

    uint8 cross_gap_ok;
    uint8 cross_gap_top;

        // 定义crossdetectnow变量，用于当前图像处理过程。
    uint8 cross_detect_now;

    uint8 left_start = 0;
    uint8 right_start = 0;

    uint8 actual_start_y;
    uint8 bottom_target_y;

    uint8 left_top;
    uint8 right_top;
    uint8 common_top;

    uint8 left_valid_top;
    uint8 left_valid_bottom;
    uint8 right_valid_top;
    uint8 right_valid_bottom;
    uint8 left_valid_count;
    uint8 right_valid_count;

    uint8 left_top_fit_ok;
    uint8 left_bottom_fit_ok;
    uint8 right_top_fit_ok;
    uint8 right_bottom_fit_ok;
    uint8 completion_ok;
    uint8 turn_fallback_ok;

    uint8 left_top_end;
    uint8 right_top_end;
    uint8 left_bottom_begin;
    uint8 right_bottom_begin;

    CrossFitModel left_top_model;
    CrossFitModel left_bottom_model;
    CrossFitModel right_top_model;
    CrossFitModel right_bottom_model;

    uint8 start_band_top;

    uint8 best_start_y;
    uint8 best_left_start;
    uint8 best_right_start;

    uint16 mid_valid_count;
    uint16 common_mid_count;
    uint16 edge_count_diff;
    uint8 ipm_known_side;
    int16 y;
        // 检查十字锁定与十字白区是否可用；条件不满足时进入回退分支。
    if(cross_lock_active && !RING_SUPPRESS_CROSS)
    {
        road_mode = ROAD_MODE_CROSS;
    }
    else
    {
        road_mode = ROAD_MODE_NORMAL;
    }

    left_start_y = 0;
    right_start_y = 0;
    independent_seed_mode = 0;
    left_seed_ok = 0;
    right_seed_ok = 0;
    independent_retry_done = 0;

    left_start_y = 0;
    right_start_y = 0;

    memset(mid_exact_ok, 0, sizeof(mid_exact_ok));   /* 标记对应行的精确中线是否有效。 */
        // 是否暂时丢弃 IPM 补出的某一条边线。
    ipm_discard_active = 0;

    if(ipm_discard_hold > 0)
    {
        ipm_discard_hold--;
    }
    retry_left_start = 0;
    retry_right_start = 0;
    retry_left_ok = 0;
    retry_right_ok = 0;


    Left_Edge_Count = 0;
    Right_Edge_Count = 0;
    left_jidian = 0;
    right_jidian = 0;

    for(y = 0; y < MT9V03X_H; y++)
    {

        left_line_list[y] = 0;
        right_line_list[y] = 0;
        mid_line_list[y] = MID_W;
    }

    bottom_target_y = jidian_search_line - 1;

    cross_gap_top = 0;
    cross_detect_now = 0;
    ipm_known_side = IPM_KNOWN_NONE;

        // 以下代码继续处理image_deal 的主要处理。
    {
        uint8 cross_dbg_gap_top = 0;
        uint8 miss_th_save      = cross_missing_rows_th;    /* 定义missthsave变量，用于当前图像处理过程。 */
        uint8 pct_th_save       = cross_white_pct_th;

                // 检查环岛状态与环岛边线是否满足继续处理的条件。
        if(ring_state != 0)
        {
            cross_missing_rows_th = CROSS_MIN_MISSING_ROWS_RING;
            cross_white_pct_th    = CROSS_WHITE_MIN_PERCENT_RING;
        }

        cross_dbg_hit = detect_cross_white_gap(index,
                                               EDGE_TOP_Y + 1,
                                               bottom_target_y,
                                               &cross_dbg_gap_top);

        cross_missing_rows_th = miss_th_save;               /* 更新image_deal 的主要处理相关变量。 */
        cross_white_pct_th    = pct_th_save;
    }

        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if(RING_SUPPRESS_CROSS)
    {
        cross_lock_active = 0;
        cross_hold_frames = 0;
    }

        // 检查十字锁定与十字白区是否满足继续处理的条件。
    if(cross_lock_active)
    {
        if(cross_hold_frames > 0)
        {
            cross_hold_frames--;
        }

        if(cross_hold_frames == 0)
        {
            cross_lock_active = 0;
        }
    }

        // 检查十字锁定与十字白区是否可用；条件不满足时进入回退分支。
    if(!RING_SUPPRESS_CROSS &&
       !cross_lock_active)
    {
        uint16 rows_l;
        uint16 rows_r;

        cross_detect_now =
            detect_cross_white_gap(
                index,
                EDGE_TOP_Y + 1,
                bottom_target_y,
                &cross_gap_top);

                // 检查image_deal 的主要处理是否满足继续处理的条件。
        if(cross_detect_now)
        {
            rows_l = count_image_side_rows(index, EDGE_LEFT,  EDGE_TOP_Y + 1, bottom_target_y);
            rows_r = count_image_side_rows(index, EDGE_RIGHT, EDGE_TOP_Y + 1, bottom_target_y);

            if(((rows_l + (uint16)CROSS_MAX_SIDE_ROW_DIFF) <= rows_r) ||
               ((rows_r + (uint16)CROSS_MAX_SIDE_ROW_DIFF) <= rows_l))
            {
                cross_detect_now = 0;
                   cross_dbg_hit = 0;
            }
        }

        if(cross_detect_now)
        {
                        // 是否处于十字锁定状态。
            cross_lock_active = 1;
            cross_hold_frames = CROSS_HOLD_FRAMES;
        }
    }

    cross_gap_ok =
        cross_lock_active
        ? 1
        : 0;

    best_start_y = 0;
    best_left_start = 0;
    best_right_start = 0;

        // 检查十字锁定与十字白区是否满足继续处理的条件。
    if(cross_lock_active)
    {
        best_start_y = 0;
        best_left_start = 0;
        best_right_start = 0;

        if(choose_start_row(
                index,
                EDGE_TOP_Y + 2,
                bottom_target_y,
                &best_start_y,
                &best_left_start,
                &best_right_start))
        {
            left_seed_ok = 1;
            right_seed_ok = 1;

            left_start_y = best_start_y;
            right_start_y = best_start_y;

            left_start = best_left_start;
            right_start = best_right_start;
        }
        else
        {
            left_seed_ok = 0;
            right_seed_ok = 0;

            left_start_y = 0;
            right_start_y = 0;
            left_start = 0;
            right_start = 0;
        }
    }
    else
    {
                /*
         * 从底部向上选择单边种子；环岛时使用更宽松条件。
         */
        left_seed_ok =
            choose_single_side_seed(
                index,
                (zebra_flag != 0U)
                    ? (uint8)(EDGE_TOP_Y + 2U)
                    : (uint8)SINGLE_SEED_MIN_Y,
                (zebra_flag != 0U)
                    ? (uint8)ZEBRA_SEED_BASE_Y
                    : bottom_target_y,
                EDGE_LEFT,
                &left_start_y,
                &left_start);

        right_seed_ok =
            choose_single_side_seed(
                index,
                (zebra_flag != 0U)
                    ? (uint8)(EDGE_TOP_Y + 2U)
                    : (uint8)SINGLE_SEED_MIN_Y,
                (zebra_flag != 0U)
                    ? (uint8)ZEBRA_SEED_BASE_Y
                    : bottom_target_y,
                EDGE_RIGHT,
                &right_start_y,
                &right_start);
    }

        // 检查巡线种子点是否满足继续处理的条件。
    if(left_seed_ok && right_seed_ok &&
       ((uint16)left_start >= (uint16)right_start))
    {
        if(ring_state != 0U)
        {
                        // 检查中线的有效性与连续性是否满足继续处理的条件。
            if((uint16)left_start < (uint16)MID_W)
            {
                memset(right_line_list, 0, sizeof(right_line_list));
                memset(Right_Edge_Pts, 0, sizeof(Right_Edge_Pts));

                Right_Edge_Count = 0;
                right_seed_ok    = 0;
                right_jidian     = 0;
                right_start      = 0;
                right_start_y    = 0;
                right_top        = EDGE_TOP_Y;
            }
            else if((uint16)right_start > (uint16)MID_W)
            {
                memset(left_line_list, 0, sizeof(left_line_list));
                memset(Left_Edge_Pts, 0, sizeof(Left_Edge_Pts));

                Left_Edge_Count = 0;
                left_seed_ok    = 0;
                left_jidian     = 0;
                left_start      = 0;
                left_start_y    = 0;
                left_top        = EDGE_TOP_Y;
            }

                        // 是否暂时丢弃 IPM 补出的某一条边线。
            ipm_discard_active = 0;
        }
        else
        {
                        // 检查中线的有效性与连续性是否满足继续处理的条件。
            if((uint16)left_start < (uint16)MID_W)
            {
                memset(right_line_list, 0, sizeof(right_line_list));

                ipm_discard_side   = EDGE_RIGHT;
                ipm_discard_active = 1;
            }
            else if((uint16)right_start > (uint16)MID_W)
            {
                memset(left_line_list, 0, sizeof(left_line_list));

                ipm_discard_side   = EDGE_LEFT;
                ipm_discard_active = 1;
            }

            if(ipm_discard_active != 0U)
            {
                ipm_discard_hold = IPM_DISCARD_CROSS_HOLD_ARM;

                cross_lock_active = 0;
                cross_hold_frames = 0;
                cross_gap_ok      = 0;
                cross_gap_top     = 0;
                cross_detect_now  = 0;
                cross_dbg_hit     = 0;
            }
        }
    }


        // 检查巡线种子点是否可用；条件不满足时进入回退分支。
    if(!left_seed_ok && !right_seed_ok)
    {
        if(cross_lock_active)
        {
                        // 检查左右边线的有效性是否满足继续处理的条件。
            if(detect_cross_white_gap_double(
                    index,
                    EDGE_TOP_Y + 1,
                    bottom_target_y,
                    &cross_gap_top))
            {
                cross_hold_frames = CROSS_HOLD_FRAMES;
                cross_lock_active = 1;
                cross_gap_ok = 1;
            }

            road_mode = ROAD_MODE_CROSS;
        }
        else
        {
            road_mode = ROAD_MODE_LOST;
        }

        search_end_line = bottom_target_y;
        mid_output_smooth();        /* 中心线输出平滑：小间隙插值、中值滤波、加权平均和最大步长限制。 */
        return;
    }

    independent_seed_mode = 1;

        // 检查十字锁定与十字白区是否满足继续处理的条件。
    if(cross_lock_active)
    {
        road_mode = ROAD_MODE_CROSS;
    }
    else
    {
        road_mode = ROAD_MODE_NORMAL;
    }

        // 检查巡线种子点是否满足继续处理的条件。
    if(left_seed_ok && right_seed_ok)
    {
        edge_start_y =
            (left_start_y > right_start_y)
            ? left_start_y
            : right_start_y;
    }
    else if(left_seed_ok)
    {
        edge_start_y = left_start_y;
    }
    else
    {
        edge_start_y = right_start_y;
    }

    actual_start_y = edge_start_y;

        // 左侧巡线起始点的横坐标。
    left_jidian =
        left_seed_ok ? left_start : 0;

    right_jidian =
        right_seed_ok ? right_start : 0;

        /*
     * 更新左右边线的有效性相关变量。
     */
    left_top =
        left_seed_ok ? left_start_y : EDGE_TOP_Y;

    right_top =
        right_seed_ok ? right_start_y : EDGE_TOP_Y;

        // 检查巡线种子点是否满足继续处理的条件。
    if(left_seed_ok)
    {
        if(cross_lock_active || RING_LONG_TIGHT(EDGE_LEFT))
        {
            Left_Edge_Count =
                trace_edge_cross(
                    index,
                    left_start,
                    left_start_y,
                    EDGE_LEFT,
                    Left_Edge_Pts,
                    left_line_list,
                    &left_top);
        }
        else
        {
            Left_Edge_Count =
                trace_edge(
                    index,
                    left_start,
                    left_start_y,
                    EDGE_LEFT,
                    Left_Edge_Pts,
                    left_line_list,
                    &left_top);
        }
    }

    if(right_seed_ok)
    {
        if(cross_lock_active || RING_LONG_TIGHT(EDGE_RIGHT))
        {
            Right_Edge_Count =
                trace_edge_cross(
                    index,
                    right_start,
                    right_start_y,
                    EDGE_RIGHT,
                    Right_Edge_Pts,
                    right_line_list,
                    &right_top);
        }
        else
        {
            Right_Edge_Count =
                trace_edge(
                    index,
                    right_start,
                    right_start_y,
                    EDGE_RIGHT,
                    Right_Edge_Pts,
                    right_line_list,
                    &right_top);
        }
    }

        // 检查逆透视补线与 IPM 结果是否满足继续处理的条件。
    if(ipm_discard_active != 0)
    {
        if(ipm_discard_side == EDGE_LEFT)
        {
            memset(left_line_list, 0, sizeof(left_line_list));
        }
        else
        {
            memset(right_line_list, 0, sizeof(right_line_list));
        }
    }

        // 检查十字锁定与十字白区是否可用；条件不满足时进入回退分支。
    if(!cross_lock_active)
    {
        filter_single_edge_outward_jump(
            left_line_list,
            EDGE_TOP_Y + 1,
            bottom_target_y,
            EDGE_LEFT);

        filter_single_edge_outward_jump(
            right_line_list,
            EDGE_TOP_Y + 1,
            bottom_target_y,
            EDGE_RIGHT);

                // 检查环岛状态与环岛边线是否满足继续处理的条件。
        if(ring_state == 0)
        {
            (void)edge_cut_outward_tail(
                      left_line_list,
                      EDGE_LEFT,
                      EDGE_TOP_Y + 1,
                      bottom_target_y);

            (void)edge_cut_outward_tail(
                      right_line_list,
                      EDGE_RIGHT,
                      EDGE_TOP_Y + 1,
                      bottom_target_y);
        }
    }
        // 左侧边线逐行缓存；每一行保存一个横坐标。
    (void)edge_line_smooth_refill(left_line_list,  EDGE_TOP_Y + 1, bottom_target_y);
    (void)edge_line_smooth_refill(right_line_list, EDGE_TOP_Y + 1, bottom_target_y);
        // 检查十字锁定与十字白区是否可用；条件不满足时进入回退分支。
    if(!RING_SUPPRESS_CROSS &&
       !cross_lock_active)
    {
        uint16 rows_l_retry;
        uint16 rows_r_retry;

        if(detect_cross_white_gap(
               index,
               EDGE_TOP_Y + 1,
               bottom_target_y,
               &cross_gap_top))
        {
            rows_l_retry =
                count_image_side_rows(
                    index,
                    EDGE_LEFT,
                    EDGE_TOP_Y + 1,
                    bottom_target_y);

            rows_r_retry =
                count_image_side_rows(
                    index,
                    EDGE_RIGHT,
                    EDGE_TOP_Y + 1,
                    bottom_target_y);

            if(!(((rows_l_retry + (uint16)CROSS_MAX_SIDE_ROW_DIFF) <= rows_r_retry) ||
                 ((rows_r_retry + (uint16)CROSS_MAX_SIDE_ROW_DIFF) <= rows_l_retry)))
            {
                cross_lock_active = 1;
                cross_hold_frames = CROSS_HOLD_FRAMES;
                cross_gap_ok = 1;
            }
            else
            {
                                /*
                 * 更新image_deal 的主要处理相关变量。
                 */
                cross_dbg_hit = 0;
            }
        }
    }

        // 检查十字锁定与十字白区是否可用；条件不满足时进入回退分支。
    if(!RING_SUPPRESS_CROSS && cross_lock_active)
    {
        if(detect_cross_white_gap_double(
                index,
                EDGE_TOP_Y + 1,
                bottom_target_y,
                &cross_gap_top))
        {
            cross_hold_frames = CROSS_HOLD_FRAMES;
            cross_lock_active = 1;
            cross_gap_ok = 1;
        }
    }
        /*
     * 更新左右边线的有效性相关变量。
     */


    cross_scan_bottom = EDGE_TOP_Y;

        // 检查十字锁定与十字白区是否可用；条件不满足时进入回退分支。
    if(!RING_SUPPRESS_CROSS &&
       cross_lock_active &&
       left_seed_ok &&
       right_seed_ok &&
       Left_Edge_Count >= CROSS_MIN_TRACE_POINTS &&
       Right_Edge_Count >= CROSS_MIN_TRACE_POINTS)
    {
        cross_scan_bottom =
            (left_start_y < right_start_y)
            ? left_start_y
            : right_start_y;

        if(cross_scan_bottom > EDGE_TOP_Y + 1)
        {
            if(find_cross_gap(
                    left_line_list,
                    right_line_list,
                    EDGE_TOP_Y + 1,
                    cross_scan_bottom,
                    left_top,
                    right_top,
                    &cross_gap_top))
            {
                                /*
                 * 更新image_deal 的主要处理相关变量。
                 */
                cross_gap_ok = 1;
                cross_hold_frames = CROSS_HOLD_FRAMES;
                cross_lock_active = 1;
            }
        }
    }
        // 检查十字锁定与十字白区是否满足继续处理的条件。
    if(cross_lock_active &&
       left_seed_ok &&
       right_seed_ok)
    {
        invalidate_cross_mid_jump_segment(
            left_line_list,
            right_line_list,
            EDGE_TOP_Y + 1,
            bottom_target_y);
    }
        // 检查巡线种子点是否满足继续处理的条件。
    if(left_seed_ok &&
       right_seed_ok)
    {
        uint8  few_side;
        uint8  few_x;
        int16  many_x;
        int16  merge_dx;
        uint8  base_y;
        uint8  k;
        uint8  yy;
        uint8 *other_line;

        left_valid_row_count =
            count_valid_edge_rows(
                left_line_list,
                EDGE_TOP_Y + 1,
                bottom_target_y);

        right_valid_row_count =
            count_valid_edge_rows(
                right_line_list,
                EDGE_TOP_Y + 1,
                bottom_target_y);

        if(left_valid_row_count <= right_valid_row_count)
        {
                        /*
             * 更新左右边线的有效性相关变量。
             */
            few_side = EDGE_LEFT;
            base_y   = (uint8)left_start_y;
        }
        else
        {
                        /*
             * 更新左右边线的有效性相关变量。
             */
            few_side = EDGE_RIGHT;
            base_y   = (uint8)right_start_y;
        }

        few_x = (few_side == EDGE_LEFT) ? left_line_list[base_y]
                                        : right_line_list[base_y];

        other_line = (few_side == EDGE_LEFT) ? right_line_list : left_line_list;

                /*
         * 更新image_deal 的主要处理相关变量。
         */
        many_x = (int16)other_line[base_y];

        if(many_x == 0)
        {
            for(k = 1; (k <= (uint8)IPM_MERGE_NEAREST_MAX_ROWS) && (many_x == 0); k++)
            {
                if(((int16)base_y + (int16)k) <= (int16)bottom_target_y)
                {
                    yy = (uint8)((int16)base_y + (int16)k);

                    if(other_line[yy] != 0)
                    {
                        many_x = (int16)other_line[yy];
                        break;
                    }
                }

                if(((int16)base_y - (int16)k) >= (int16)(EDGE_TOP_Y + 1))
                {
                    yy = (uint8)((int16)base_y - (int16)k);

                    if(other_line[yy] != 0)
                    {
                        many_x = (int16)other_line[yy];
                        break;
                    }
                }
            }
        }

        if(few_x != 0 && many_x != 0)
        {
            merge_dx = many_x - (int16)few_x;

            if(merge_dx < 0)
            {
                merge_dx = -merge_dx;
            }

            if(merge_dx <= (int16)IPM_MERGE_MAX_GAP_X)
            {
                ipm_known_side = (few_side == EDGE_LEFT)
                               ? IPM_KNOWN_RIGHT
                               : IPM_KNOWN_LEFT;
            }
        }
                // 检查环岛状态与环岛边线是否可用；条件不满足时进入回退分支。
        if(!RING_SUPPRESS_CROSS)
        {
            if((uint16)left_valid_row_count >= (uint16)right_valid_row_count)
            {
                if(((uint16)(left_valid_row_count - right_valid_row_count) >=
                    IPM_MIN_EDGE_COUNT_DIFF) &&
                   (right_valid_row_count < IPM_SINGLE_SIDE_FEW_ROWS_MAX))
                {
                    ipm_known_side = IPM_KNOWN_LEFT;
                }
            }
            else
            {
                if(((uint16)(right_valid_row_count - left_valid_row_count) >=
                    IPM_MIN_EDGE_COUNT_DIFF) &&
                   (left_valid_row_count < IPM_SINGLE_SIDE_FEW_ROWS_MAX))
                {
                    ipm_known_side = IPM_KNOWN_RIGHT;
                }
            }
        }
        if(ipm_known_side != IPM_KNOWN_NONE)
        {
            independent_retry_done = 1;

            if(ipm_known_side == IPM_KNOWN_LEFT)
            {
                memset(
                    right_line_list,
                    0,
                    sizeof(right_line_list));
            }
            else
            {
                memset(
                    left_line_list,
                    0,
                    sizeof(left_line_list));
            }

                        // 检查环岛状态与环岛边线是否满足继续处理的条件。
            if(ring_state == 0U)
            {
                if(ipm_known_side == IPM_KNOWN_LEFT)
                {
                    ipm_discard_side = EDGE_RIGHT;
                }
                else
                {
                    ipm_discard_side = EDGE_LEFT;
                }

                ipm_discard_active = 1;
                ipm_discard_hold   = IPM_DISCARD_CROSS_HOLD_ARM;

                cross_lock_active = 0;
                cross_hold_frames = 0;
                cross_gap_ok      = 0;
                cross_gap_top     = 0;
                cross_detect_now  = 0;
                cross_dbg_hit     = 0;
            }
        }
    }


#if IPM_CHECK_TWO_LINES_MERGED
        // 检查十字锁定与十字白区是否可用；条件不满足时进入回退分支。
    if(!cross_lock_active &&
       ipm_known_side == IPM_KNOWN_NONE)
    {
        uint8 pair_rows_ok = 0;

                // 逐行遍历左右边线的有效性。
        for(y = EDGE_TOP_Y + 1; y <= bottom_target_y; y++)
        {
            if(left_line_list[y] != 0 &&
               right_line_list[y] != 0 &&
               ((int16)right_line_list[y] -
                (int16)left_line_list[y]) >= IPM_PAIR_MIN_GAP_X)
            {
                pair_rows_ok = 1;
                break;
            }
        }

        if(!pair_rows_ok)
        {
                        /*
             * 左侧边线逐行缓存；每一行保存一个横坐标。
             */
            left_valid_row_count =
                count_valid_edge_rows(
                    left_line_list,
                    EDGE_TOP_Y + 1,
                    bottom_target_y);

            right_valid_row_count =
                count_valid_edge_rows(
                    right_line_list,
                    EDGE_TOP_Y + 1,
                    bottom_target_y);

                        // 检查image_deal 的主要处理是否满足继续处理的条件。
            if(left_valid_row_count >= right_valid_row_count)
            {
                ipm_known_side = IPM_KNOWN_LEFT;
            }
            else
            {
                ipm_known_side = IPM_KNOWN_RIGHT;
            }

                        // 检查逆透视补线与 IPM 结果是否满足继续处理的条件。
            if(((ipm_known_side == IPM_KNOWN_LEFT) &&
                (left_valid_row_count < IPM_MIN_KNOWN_POINTS)) ||
               ((ipm_known_side == IPM_KNOWN_RIGHT) &&
                (right_valid_row_count < IPM_MIN_KNOWN_POINTS)))
            {
                ipm_known_side = IPM_KNOWN_NONE;
            }
        }
    }
#endif

    turn_fallback_ok = 0;      /* 更新image_deal 的主要处理相关变量。 */


        // 检查逆透视补线与 IPM 结果是否满足继续处理的条件。
    if(ipm_known_side != IPM_KNOWN_NONE)
    {
        left_valid_row_count =
            count_valid_edge_rows(
                left_line_list,
                EDGE_TOP_Y + 1,
                bottom_target_y);

        right_valid_row_count =
            count_valid_edge_rows(
                right_line_list,
                EDGE_TOP_Y + 1,
                bottom_target_y);

        if((ipm_known_side == IPM_KNOWN_LEFT &&
            left_valid_row_count >= IPM_MIN_KNOWN_POINTS) ||
           (ipm_known_side == IPM_KNOWN_RIGHT &&
            right_valid_row_count >= IPM_MIN_KNOWN_POINTS))
        {
            road_mode = ROAD_MODE_IPM_TRY;

            if(fill_missing_side_by_ipm(
                    left_line_list,
                    right_line_list,
                    EDGE_TOP_Y + 1,
                    bottom_target_y,
                    Left_Edge_Count,
                    Right_Edge_Count))
            {
                road_mode = ROAD_MODE_IPM_FILL;

                mid_valid_count =
                    build_mid_line_from_ipm_candidates(
                        mid_line_list,
                        EDGE_TOP_Y + 1,
                        bottom_target_y);

                                // 检查逆透视补线与 IPM 结果是否可用；条件不满足时进入回退分支。
                if(mid_valid_count >= IPM_MIN_ACCEPT_ROWS &&
                   !cross_gap_ok)
                {
                    search_end_line = EDGE_TOP_Y;

                    filter_ipm_mid_line_spikes(
                        mid_line_list,
                        EDGE_TOP_Y + 1,
                        bottom_target_y);

                    mid_output_smooth();        /* 中心线输出平滑：小间隙插值、中值滤波、加权平均和最大步长限制。 */

                    return;
                }
            }

            road_mode = ROAD_MODE_IPM_FAIL;
        }
    }



        // 检查image_deal 的主要处理是否满足继续处理的条件。
    if(turn_fallback_ok)
    {road_mode = ROAD_MODE_TURN_FILL;
        mid_valid_count =
            build_mid_line_from_edges(
                left_line_list,
                right_line_list,
                mid_line_list,
                EDGE_TOP_Y + 1,
                bottom_target_y);

        if(mid_valid_count >= MID_MIN_VALID_ROWS)
        {
            search_end_line = EDGE_TOP_Y;
            mid_output_smooth();        /* 中心线输出平滑：小间隙插值、中值滤波、加权平均和最大步长限制。 */
            return;
        }
        else
        {
            search_end_line = actual_start_y;

                        /*
             * 执行image_deal 的主要处理相关操作。
             */
            goto image_deal_build_mid;
        }
    }

        // 检查左右边线的有效性是否满足继续处理的条件。
    if(Left_Edge_Count < EDGE_MIN_FIT_POINTS &&
       Right_Edge_Count < EDGE_MIN_FIT_POINTS)
    {
        road_mode = ROAD_MODE_LOST;
        search_end_line = actual_start_y;

    }

        /*
     * 更新image_deal 的主要处理相关变量。
     */
    left_valid_count = 0;
    right_valid_count = 0;

    lane_width_hint =
        (right_start > left_start)
        ? (uint16)(right_start - left_start)
        : 0;

    left_span_ok =
        get_line_span(
            left_line_list,
            EDGE_TOP_Y + 1,
            bottom_target_y,
            &left_valid_top,
            &left_valid_bottom,
            &left_valid_count);

    right_span_ok =
        get_line_span(
            right_line_list,
            EDGE_TOP_Y + 1,
            bottom_target_y,
            &right_valid_top,
            &right_valid_bottom,
            &right_valid_count);

    if(!left_span_ok || !right_span_ok)
    {
                // 检查逆透视补线与 IPM 结果是否可用；条件不满足时进入回退分支。
        if(!cross_gap_ok &&
           ipm_known_side == IPM_KNOWN_NONE)
        {
            turn_fallback_ok =
                extend_short_turn_edges(
                    left_line_list,
                    right_line_list,
                    EDGE_TOP_Y + 1,
                    bottom_target_y);

            if(turn_fallback_ok)
            {
                road_mode = ROAD_MODE_TURN_FILL;

                mid_valid_count =
                    build_mid_line_from_edges(
                        left_line_list,
                        right_line_list,
                        mid_line_list,
                        EDGE_TOP_Y + 1,
                        bottom_target_y);

                                // 检查中线的有效性与连续性是否满足继续处理的条件。
                if(mid_valid_count >= MID_MIN_VALID_ROWS)
                {
                    search_end_line = EDGE_TOP_Y;

                    filter_mid_line_spikes(
                        mid_line_list,
                        EDGE_TOP_Y + 1,
                        bottom_target_y);

                    mid_output_smooth();        /* 中心线输出平滑：小间隙插值、中值滤波、加权平均和最大步长限制。 */

                    return;
                }
            }

                        /*
             * 左侧边线逐行缓存；每一行保存一个横坐标。
             */

            mid_valid_count =
                build_mid_line_from_edges(
                    left_line_list,
                    right_line_list,
                    mid_line_list,
                    EDGE_TOP_Y + 1,
                    bottom_target_y);

            if(mid_valid_count >= MID_MIN_VALID_ROWS)
            {
                search_end_line = EDGE_TOP_Y;

                filter_mid_line_spikes(
                    mid_line_list,
                    EDGE_TOP_Y + 1,
                    bottom_target_y);

                mid_output_smooth();        /* 中心线输出平滑：小间隙插值、中值滤波、加权平均和最大步长限制。 */

                return;
            }
        }

                /*
         * 更新image_deal 的主要处理相关变量。
         */
        mid_valid_count = 0;

        goto image_deal_build_mid;
    }
#if ENABLE_ONE_SIDE_FILL_NORMAL
    else
    {
                /*
         * 左侧边线逐行缓存；每一行保存一个横坐标。
         */
        common_mid_count =
            count_common_edge_rows(
                left_line_list,
                right_line_list,
                EDGE_TOP_Y + 1,
                bottom_target_y);

        if(left_valid_count >= right_valid_count)
        {
            edge_count_diff = (uint16)(left_valid_count - right_valid_count);
        }
        else
        {
            edge_count_diff = (uint16)(right_valid_count - left_valid_count);
        }

        if((common_mid_count < (uint16)IPM_TRIGGER_MIN_MID_ROWS) ||
           (edge_count_diff >= IPM_MIN_EDGE_COUNT_DIFF))
        {
            complete_one_side_edge(
                left_line_list,
                right_line_list,
                EDGE_TOP_Y + 1,
                bottom_target_y,
                lane_width_hint,
                EDGE_TOP_Y + 1,          /* 执行左右边线的有效性相关操作。 */
                EDGE_TOP_Y + 1,          /* 执行左右边线的有效性相关操作。 */
                bottom_target_y,
                bottom_target_y);
        }
    }
#endif
    left_top_end =
        (uint8)(left_valid_top + CROSS_FIT_WINDOW - 1);
    if(left_top_end > left_valid_bottom)
    {
        left_top_end = left_valid_bottom;
    }

    right_top_end =
        (uint8)(right_valid_top + CROSS_FIT_WINDOW - 1);
    if(right_top_end > right_valid_bottom)
    {
        right_top_end = right_valid_bottom;
    }

    left_bottom_begin =
        (left_valid_bottom > CROSS_FIT_WINDOW - 1)
        ? (uint8)(left_valid_bottom - CROSS_FIT_WINDOW + 1)
        : left_valid_top;
    if(left_bottom_begin < left_valid_top)
    {
        left_bottom_begin = left_valid_top;
    }

    right_bottom_begin =
        (right_valid_bottom > CROSS_FIT_WINDOW - 1)
        ? (uint8)(right_valid_bottom - CROSS_FIT_WINDOW + 1)
        : right_valid_top;
    if(right_bottom_begin < right_valid_top)
    {
        right_bottom_begin = right_valid_top;
    }

    left_top_fit_ok =
        fit_cross_edge(
            left_line_list,
            left_valid_top,
            left_top_end,
            &left_top_model);

    left_bottom_fit_ok =
        fit_cross_edge(
            left_line_list,
            left_bottom_begin,
            left_valid_bottom,
            &left_bottom_model);

    right_top_fit_ok =
        fit_cross_edge(
            right_line_list,
            right_valid_top,
            right_top_end,
            &right_top_model);

    right_bottom_fit_ok =
        fit_cross_edge(
            right_line_list,
            right_bottom_begin,
            right_valid_bottom,
            &right_bottom_model);
    completion_ok = 0;

        // 检查左右边线的有效性是否满足继续处理的条件。
    if(cross_gap_ok &&
       (left_top_fit_ok || left_bottom_fit_ok) &&
       (right_top_fit_ok || right_bottom_fit_ok) &&
       mixed_models_are_plausible(
           EDGE_TOP_Y + 1,
           bottom_target_y,
           left_valid_top,
           left_valid_bottom,
           left_top_fit_ok,
           left_bottom_fit_ok,
           &left_top_model,
           &left_bottom_model,
           right_valid_top,
           right_valid_bottom,
           right_top_fit_ok,
           right_bottom_fit_ok,
           &right_top_model,
           &right_bottom_model))
    {
        fill_edge_with_available_models(
            left_line_list,
            EDGE_TOP_Y + 1,
            bottom_target_y,
            left_valid_top,
            left_valid_bottom,
            left_top_fit_ok,
            left_bottom_fit_ok,
            &left_top_model,
            &left_bottom_model);

        fill_edge_with_available_models(
            right_line_list,
            EDGE_TOP_Y + 1,
            bottom_target_y,
            right_valid_top,
            right_valid_bottom,
            right_top_fit_ok,
            right_bottom_fit_ok,
            &right_top_model,
            &right_bottom_model);

        completion_ok = 1;
        road_mode = ROAD_MODE_CROSS;
    }
        // 检查十字锁定与十字白区是否可用；条件不满足时进入回退分支。
    if(!completion_ok &&
       !cross_lock_active)
    {
        common_mid_count =
            count_common_edge_rows(
                left_line_list,
                right_line_list,
                EDGE_TOP_Y + 1,
                bottom_target_y);

        if(common_mid_count < TURN_FALLBACK_MIN_COMMON_ROWS)
        {
            turn_fallback_ok =
                extend_short_turn_edges(
                    left_line_list,
                    right_line_list,
                    EDGE_TOP_Y + 1,
                    bottom_target_y);

            if(turn_fallback_ok)
            {
                road_mode = ROAD_MODE_TURN_FILL;

                mid_valid_count =
                    build_mid_line_from_edges(
                        left_line_list,
                        right_line_list,
                        mid_line_list,
                        EDGE_TOP_Y + 1,
                        bottom_target_y);

                if(mid_valid_count >= MID_MIN_VALID_ROWS)
                {
                    search_end_line = EDGE_TOP_Y;

                    filter_mid_line_spikes(
                        mid_line_list,
                        EDGE_TOP_Y + 1,
                        bottom_target_y);

                    mid_output_smooth();        /* 中心线输出平滑：小间隙插值、中值滤波、加权平均和最大步长限制。 */

                    return;
                }
            }
        }
    }
    common_top =
        (left_top > right_top)
        ? left_top
        : right_top;

    if(completion_ok)
    {
                /*
         * 更新左右边线的有效性相关变量。
         */
        search_end_line = EDGE_TOP_Y;
    }
    else if(common_top > EDGE_TOP_Y)
    {
        search_end_line = common_top - 1;
    }
    else
    {
        search_end_line = EDGE_TOP_Y;
    }

        /*
     * 更新image_deal 的主要处理相关变量。
     */
    image_deal_build_mid:

        mid_valid_count = 0;

        for(y = bottom_target_y;
            y > EDGE_TOP_Y;
            y--)
    {
        if(left_line_list[y] == 0 ||
           right_line_list[y] == 0 ||
           left_line_list[y] >= right_line_list[y])
        {
            continue;
        }

        mid_line_list[y] =
            (uint8)(((uint16)left_line_list[y] +
                     (uint16)right_line_list[y]) / 2U);

        mid_valid_count++;
    }

        // 检查中线的有效性与连续性是否满足继续处理的条件。
    if(cut_mid_line_by_slope_jump())
    {
                /*
         * 左侧边线逐行缓存；每一行保存一个横坐标。
         */
        mid_valid_count =
            count_common_edge_rows(
                left_line_list,
                right_line_list,
                EDGE_TOP_Y + 1,
                bottom_target_y);
    }

        // 检查十字锁定与十字白区是否满足继续处理的条件。
        if(cross_lock_active &&
           mid_valid_count >= 6)
        {
            repair_mid_line_continuation(
                mid_line_list,
                EDGE_TOP_Y + 1,
                bottom_target_y);
        }

    if(completion_ok)
    {
        if(mid_valid_count >= 6)
        {
            road_mode = ROAD_MODE_CROSS;
            search_end_line = EDGE_TOP_Y;

            filter_mid_line_spikes(
                mid_line_list,
                EDGE_TOP_Y + 1,
                bottom_target_y);

            mid_output_smooth();        /* 中心线输出平滑：小间隙插值、中值滤波、加权平均和最大步长限制。 */

            return;
        }

                        /*
             * 更新image_deal 的主要处理相关变量。
             */
            completion_ok = 0;

            if(cross_lock_active)
            {
                road_mode = ROAD_MODE_CROSS;
            }
            else
            { road_mode = ROAD_MODE_NORMAL;}
        }
        /*
     * 左侧边线逐行缓存；每一行保存一个横坐标。
     */
    left_valid_row_count =
        count_valid_edge_rows(
            left_line_list,
            EDGE_TOP_Y + 1,
            bottom_target_y);

    right_valid_row_count =
        count_valid_edge_rows(
            right_line_list,
            EDGE_TOP_Y + 1,
            bottom_target_y);

    if(left_valid_row_count >= right_valid_row_count)
    {
        edge_count_diff =
            left_valid_row_count -
            right_valid_row_count;
    }
    else
    {
        edge_count_diff =
            right_valid_row_count -
            left_valid_row_count;
    }
        if(!completion_ok &&
           !independent_retry_done &&
           !cross_lock_active &&
           ((edge_count_diff >= IPM_MIN_EDGE_COUNT_DIFF) ||
            (mid_valid_count < IPM_TRIGGER_MIN_MID_ROWS)))
        {
            independent_retry_done = 1;

            retry_left_ok = 0;
            retry_right_ok = 0;

                        // 检查斑马线特征是否满足继续处理的条件。
            if(retry_single_side_trace(
                    index,
                    (zebra_flag != 0U)
                        ? (uint8)(EDGE_TOP_Y + 2U)
                        : (uint8)SINGLE_SEED_MIN_Y,
                    (zebra_flag != 0U)
                        ? (uint8)ZEBRA_SEED_BASE_Y
                        : bottom_target_y,
                    &retry_left_ok,
                    &retry_right_ok,
                    &retry_left_start_y,
                    &retry_right_start_y,
                    &retry_left_start,
                    &retry_right_start,
                    &left_top,
                    &right_top))
            {
                independent_seed_mode = 1;
                road_mode = ROAD_MODE_SINGLE_SEED;

                left_seed_ok = retry_left_ok;
                right_seed_ok = retry_right_ok;

                left_start_y = retry_left_start_y;
                right_start_y = retry_right_start_y;

                left_start = retry_left_start;
                right_start = retry_right_start;

                left_jidian =
                    left_seed_ok ? left_start : 0;

                right_jidian =
                    right_seed_ok ? right_start : 0;

                filter_edge_single_row_spikes(
                    left_line_list,
                    EDGE_TOP_Y + 1,
                    bottom_target_y);

                filter_edge_single_row_spikes(
                    right_line_list,
                    EDGE_TOP_Y + 1,
                    bottom_target_y);

                if(!cross_lock_active)
                {
                    filter_single_edge_outward_jump(
                        left_line_list,
                        EDGE_TOP_Y + 1,
                        bottom_target_y,
                        EDGE_LEFT);

                    filter_single_edge_outward_jump(
                        right_line_list,
                        EDGE_TOP_Y + 1,
                        bottom_target_y,
                        EDGE_RIGHT);
                }

                left_valid_row_count =
                    count_valid_edge_rows(
                        left_line_list,
                        EDGE_TOP_Y + 1,
                        bottom_target_y);

                right_valid_row_count =
                    count_valid_edge_rows(
                        right_line_list,
                        EDGE_TOP_Y + 1,
                        bottom_target_y);
                                // 逐行遍历image_deal 的主要处理。
                for(y = 0; y < MT9V03X_H; y++)
                {
                    mid_line_list[y] = MID_W;
                }
                                /*
                 * 更新image_deal 的主要处理相关变量。
                 */

                mid_valid_count = 0;

                                // 检查逆透视补线与 IPM 结果是否满足继续处理的条件。
                if(left_valid_row_count >= IPM_MIN_KNOWN_POINTS ||
                   right_valid_row_count >= IPM_MIN_KNOWN_POINTS)
                {
                    road_mode = ROAD_MODE_IPM_TRY;

                    if(fill_missing_side_by_ipm(
                            left_line_list,
                            right_line_list,
                            EDGE_TOP_Y + 1,
                            bottom_target_y,
                            Left_Edge_Count,
                            Right_Edge_Count))
                    {
                        road_mode = ROAD_MODE_IPM_FILL;

                        mid_valid_count =
                            build_mid_line_from_ipm_candidates(
                                mid_line_list,
                                EDGE_TOP_Y + 1,
                                bottom_target_y);

                        if(mid_valid_count >= IPM_MIN_ACCEPT_ROWS &&
                           !cross_gap_ok)
                        {
                            search_end_line = EDGE_TOP_Y;

                            filter_ipm_mid_line_spikes(
                                mid_line_list,
                                EDGE_TOP_Y + 1,
                                bottom_target_y);

                            mid_output_smooth();        /* 中心线输出平滑：小间隙插值、中值滤波、加权平均和最大步长限制。 */

                            return;
                        }

                                                /*
                         * 执行image_deal 的主要处理相关操作。
                         */
                        goto image_deal_build_mid;
                    }

                    road_mode = ROAD_MODE_IPM_FAIL;
                }
            }
        }

                // 检查中线的有效性与连续性是否满足继续处理的条件。
        if(mid_valid_count >= MID_MIN_VALID_ROWS)
        {
            search_end_line = EDGE_TOP_Y;
        }
        else if(cross_lock_active)
        {
                        // 当前道路模式，用于区分普通、单边、IPM、十字和急弯等状态。
            road_mode = ROAD_MODE_CROSS;
            search_end_line = actual_start_y;
        }
        else
        {
            road_mode = ROAD_MODE_LOST;
            search_end_line = actual_start_y;
        }


        /*
     * 由左右边线计算得到的逐行中心线。
     */
        filter_mid_line_spikes(
            mid_line_list,
            EDGE_TOP_Y + 1,
            bottom_target_y);

        mid_output_smooth();        /* 中心线输出平滑：小间隙插值、中值滤波、加权平均和最大步长限制。 */
    }

uint8 mid_weight_list[120]=
{
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    6,6,6,6,6,6,6,6,6,6,
    7,8,9,10,11,12,13,14,15,16,
    17,18,19,20,20,20,20,19,18,17,
    16,15,14,13,12,11,10,9,8,7,
    6,6,6,6,6,6,6,6,6,6,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
};

uint8 final_mid_line = MID_W;     /* 定义finalmidline变量，用于当前图像处理过程。 */
uint8 last_mid_line = MID_W;      /* 定义lastmidline变量，用于当前图像处理过程。 */

/**
 * @brief 从下往上遍历有效中线，使用 mid_weight_list[] 加权求和，再与上一帧混合。
 */
uint8 find_mid_line_weight(void)
{
    uint8 mid_line_value = MID_W;
    uint8 mid_line = MID_W;

    uint32 weight_midline_sum = 0;
    uint32 weight_sum = 0;

    uint8 i;

        // 循环遍历巡线种子点，完成当前统计或筛选。
    for(i = jidian_search_line - 1;
        i > search_end_line;
        i--)
    {
                // 检查逆透视补线与 IPM 结果是否满足继续处理的条件。
        if(road_mode == ROAD_MODE_IPM_FILL)
        {
            if(!ipm_mid_line_valid[i])
            {
                continue;
            }
        }
        else
        {
            if(left_line_list[i] == 0 ||
               right_line_list[i] == 0 ||
               left_line_list[i] >= right_line_list[i])
            {
                continue;
            }
        }

        weight_midline_sum +=
            (uint32)mid_line_list[i] *
            (uint32)mid_weight_list[i];

        weight_sum +=
            (uint32)mid_weight_list[i];
    }

    if(weight_sum != 0)
    {
        mid_line =
            (uint8)(weight_midline_sum / weight_sum);
    }

        /*
     * 更新中线的有效性与连续性相关变量。
     */
    mid_line_value =
        (uint8)(last_mid_line * 0.2f +
                mid_line * 0.8f);

    last_mid_line = mid_line_value;

    return mid_line_value;
}
/*
 * 更新从下往上遍历有效中线，使用 mid_weight_list[] 加权求和，再与上一帧混合相关变量。
 */
static int16 last_lookahead_error = 0;


// 定义lookaheadvalidcount变量，用于当前图像处理过程。
uint16 lookahead_valid_count = 0;

// 定义linefoundflag变量，用于当前图像处理过程。
uint8 line_found_flag = 0;

// 定义zebraflag变量，用于当前图像处理过程。
uint8 zebra_flag = 0;

// 参数 ZEBRA_RING_HOLD_FRAMES：斑马线出现后抑制环岛判断的帧数，当前值为 6U。
#define ZEBRA_RING_HOLD_FRAMES          6U

// 定义zebraringholdframes变量，用于当前图像处理过程。
static uint8 zebra_ring_hold_frames = 0U;

// 定义zebraflaglast变量，用于当前图像处理过程。
static uint8 zebra_flag_last = 0U;


/**
 * @brief 判断前瞻某行是否有效，并遵守 search_end_line。
 */
static uint8 mid_row_valid_at(int16 y)
{
    if(y <= (int16)EDGE_TOP_Y ||
       y >  (int16)(jidian_search_line - 1))
    {
        return 0;
    }

    if(y <= (int16)search_end_line)
    {
        return 0;
    }

    if(road_mode == ROAD_MODE_IPM_FILL)
    {
        return (ipm_mid_line_valid[y] != 0) ? 1 : 0;
    }

    if(left_line_list[y] == 0 ||
       right_line_list[y] == 0 ||
       left_line_list[y] >= right_line_list[y])
    {
        return 0;
    }

    return 1;
}

int16 find_mid_line_lookahead_error(void)
{
    int16  prospect_y;
    int16  y;
    int16  k;

    int32  middle_sum = 0;      /* 更新判断前瞻某行是否有效，并遵守 search_end_line相关变量。 */
    uint16 valid_count = 0;
    int16  middle_true;         /* 执行判断前瞻某行是否有效，并遵守 search_end_line相关操作。 */

    lookahead_valid_count = 0;

        /* ===== mid_row_valid_at 处理步骤 ===== */
    static int16 prospect_y_used = PROSPECT_LINE_NORMAL;
    int16 prospect_y_target;

        // 根据环岛状态与环岛边线的不同状态分支处理。
    switch(ring_state)
    {
        case 1:
                        /*
             * 更新环岛状态与环岛边线相关变量。
             */
            prospect_y_target = PROSPECT_LINE_RING_S1;
            break;

        case 2:
                        /*
             * 更新环岛状态与环岛边线相关变量。
             */
            prospect_y_target = PROSPECT_LINE_RING_S2;
            break;

        case 3:
                        /*
             * 更新环岛状态与环岛边线相关变量。
             */
            prospect_y_target = PROSPECT_LINE_RING_S3;
            break;

        default:
                        /*
             * 更新判断前瞻某行是否有效，并遵守 search_end_line相关变量。
             */
            prospect_y_target = PROSPECT_LINE_NORMAL;
            break;
    }

        // 检查判断前瞻某行是否有效，并遵守 search_end_line是否满足继续处理的条件。
    if(prospect_y_used < prospect_y_target)
    {
        prospect_y_used += PROSPECT_MOVE_STEP;

        if(prospect_y_used > prospect_y_target)
        {
            prospect_y_used = prospect_y_target;
        }
    }
    else if(prospect_y_used > prospect_y_target)
    {
        prospect_y_used -= PROSPECT_MOVE_STEP;

        if(prospect_y_used < prospect_y_target)
        {
            prospect_y_used = prospect_y_target;
        }
    }

    prospect_y = prospect_y_used;

    if(prospect_y <= (int16)EDGE_TOP_Y)
    {
        prospect_y = (int16)(EDGE_TOP_Y + 1);
    }

    if(prospect_y > (int16)(jidian_search_line - 1 - PROSPECT_ROW_STEP * (PROSPECT_ROW_COUNT - 1)))
    {
        prospect_y = (int16)(jidian_search_line - 1 - PROSPECT_ROW_STEP * (PROSPECT_ROW_COUNT - 1));
    }

        // 检查判断前瞻某行是否有效，并遵守 search_end_line是否满足继续处理的条件。
    if(prospect_y <= (int16)search_end_line)
    {
        prospect_y = (int16)(search_end_line + 1);

        if(prospect_y > (int16)(jidian_search_line - 1 - PROSPECT_ROW_STEP * (PROSPECT_ROW_COUNT - 1)))
        {
            prospect_y = (int16)(jidian_search_line - 1 - PROSPECT_ROW_STEP * (PROSPECT_ROW_COUNT - 1));
        }
    }

        /* ===== mid_row_valid_at 处理步骤 ===== */
    for(k = 0; k < PROSPECT_ROW_COUNT; k++)
    {
        y = (int16)(prospect_y + k * (int16)PROSPECT_ROW_STEP);

                // 检查中线的有效性与连续性是否满足继续处理的条件。
        if(mid_exact_ok[y] != 0)
        {
            middle_true = (int16)mid_exact_x[y];
        }
        else
        {
            if(!mid_row_valid_at(y))
            {
                continue;
            }

            middle_true = (int16)mid_line_list[y];

            if((middle_true < 1) || (middle_true > (int16)(MT9V03X_W - 1)))
            {
                continue;
            }
        }

        middle_sum += middle_true;
        valid_count++;
    }

        /* ===== mid_row_valid_at 处理步骤 ===== */
    if(valid_count == 0)
    {
                // 定义fallbackcount变量，用于当前图像处理过程。
        uint16 fallback_count = 0;
        int32  fallback_sum   = 0;

        for(y = (int16)(EDGE_TOP_Y + 1);
            y <= (int16)(jidian_search_line - 1);
            y++)
        {
            if(mid_exact_ok[y] != 0)
            {
                middle_true = (int16)mid_exact_x[y];
            }
            else
            {
                if(!mid_row_valid_at(y))
                {
                    continue;
                }

                middle_true = (int16)mid_line_list[y];

                if((middle_true < 1) ||
                   (middle_true > (int16)(MT9V03X_W - 1)))
                {
                    continue;
                }
            }

            fallback_sum += middle_true;
            fallback_count++;

            if(fallback_count >= 3U)
            {
                lookahead_valid_count = fallback_count;

                middle_true =
                    (int16)(fallback_sum / (int32)fallback_count);

                last_lookahead_error =
                    (int16)(middle_true - (int16)MID_W);

                return last_lookahead_error;
            }
        }

                /*
         * 更新判断前瞻某行是否有效，并遵守 search_end_line相关变量。
         */
        lookahead_valid_count = 0;
        return last_lookahead_error;
    }

    lookahead_valid_count = valid_count;

    middle_true = (int16)(middle_sum / (int32)valid_count);

    last_lookahead_error = (int16)(middle_true - (int16)MID_W);
    return last_lookahead_error;
}
// 参数 ERROR_FILT_MAX_STEP：用于配置误差FILT最大步长，当前值为 10。
#define ERROR_FILT_MAX_STEP   10

int16 error_output_filter(int16 raw)
{
    static int16 hist[3] = {0, 0, 0};
    static uint8 idx = 0;
    static uint8 filled = 0;
    static int16 out = 0;
    int16 a, b, c;
    int16 med;

    hist[idx] = raw;
    idx = (uint8)((idx + 1U) % 3U);

    if(filled < 3U)
    {
        filled++;
    }

    if(filled < 3U)
    {
        med = raw;
    }
    else
    {
        a = hist[0];
        b = hist[1];
        c = hist[2];

        if(((a <= b) && (b <= c)) || ((c <= b) && (b <= a)))
        {
            med = b;
        }
        else if(((b <= a) && (a <= c)) || ((c <= a) && (a <= b)))
        {
            med = a;
        }
        else
        {
            med = c;
        }
    }

    if(med > (int16)(out + (int16)ERROR_FILT_MAX_STEP))
    {
        med = (int16)(out + (int16)ERROR_FILT_MAX_STEP);
    }

    if(med < (int16)(out - (int16)ERROR_FILT_MAX_STEP))
    {
        med = (int16)(out - (int16)ERROR_FILT_MAX_STEP);
    }

    out = med;

    return out;
}


/* ===== 全局图像数据 ===== */

static const float IPM_MAT1[3][3] =       /* 定义逆透视MAT1变量，用于当前图像处理过程。 */
{
    { 0.9580907495f, -0.7001432400f,  1.8056325663f },
    {-0.0147398577f,  0.1916181499f, -0.2653174383f },
    { 0.0000000000f, -0.0073699288f,  1.0096802514f }
};

static const float IPM_MAT2[3][3] =       /* 定义逆透视MAT2变量，用于当前图像处理过程。 */
{
    { 1.1053291536f, 4.0031347962f, -0.9247648903f },
    { 0.0858934169f, 5.5830721003f,  1.3134796238f },
    { 0.0006269592f, 0.0407523511f,  1.0000000000f }
};
// 参数 IPM_DEFAULT_LANE_WIDTH：IPM 未获得可靠宽度时使用的默认赛道宽度，当前值为 30.0f。
#define IPM_DEFAULT_LANE_WIDTH      30.0f
// 参数 IPM_MIN_LANE_WIDTH：IPM 可接受的最小赛道宽度，当前值为 10.0f。
#define IPM_MIN_LANE_WIDTH          10.0f
// 参数 IPM_MAX_LANE_WIDTH：IPM 可接受的最大赛道宽度，当前值为 100.0f。
#define IPM_MAX_LANE_WIDTH          100.0f
typedef struct
{
    int16 x;
    int16 y;
} IpmGeneratedPoint;

// 参数 IPM_GENERATED_POINT_MAX：IPM 补线最多允许生成的点数。
#define IPM_GENERATED_POINT_MAX (MT9V03X_H * 2)

static IpmGeneratedPoint ipm_generated_edge[
    IPM_GENERATED_POINT_MAX
];

static uint16 ipm_generated_edge_count = 0;

static int16 ipm_mid_candidate_x[MT9V03X_H];
static uint8 ipm_mid_candidate_valid[MT9V03X_H];

static uint8 ipm_candidate_valid[MT9V03X_H];
static uint8 ipm_candidate_x[MT9V03X_H];

/**
 * @brief 使用 IPM_MAT2 将原图坐标转换到鸟瞰图坐标。
 */
static uint8 ipm_original_to_bev(
    float src_x,
    float src_y,
    float *bev_x,
    float *bev_y)
{
    float tx;
    float ty;
    float w;

    if(bev_x == 0 || bev_y == 0)
    {
        return 0;
    }

    tx =
        IPM_MAT2[0][0] * src_x +
        IPM_MAT2[0][1] * src_y +
        IPM_MAT2[0][2];

    ty =
        IPM_MAT2[1][0] * src_x +
        IPM_MAT2[1][1] * src_y +
        IPM_MAT2[1][2];

    w =
        IPM_MAT2[2][0] * src_x +
        IPM_MAT2[2][1] * src_y +
        IPM_MAT2[2][2];

    if(w > -0.0001f && w < 0.0001f)
    {
        return 0;
    }

    *bev_x = tx / w;
    *bev_y = ty / w;

    return 1;
}

/**
 * @brief 使用 IPM_MAT1 将鸟瞰图坐标转回原图。
 */
static uint8 ipm_bev_to_original(
    float bev_x,
    float bev_y,
    float *src_x,
    float *src_y)
{
    float tx;
    float ty;
    float w;

    if(src_x == 0 || src_y == 0)
    {
        return 0;
    }

    tx =
        IPM_MAT1[0][0] * bev_x +
        IPM_MAT1[0][1] * bev_y +
        IPM_MAT1[0][2];

    ty =
        IPM_MAT1[1][0] * bev_x +
        IPM_MAT1[1][1] * bev_y +
        IPM_MAT1[1][2];

    w =
        IPM_MAT1[2][0] * bev_x +
        IPM_MAT1[2][1] * bev_y +
        IPM_MAT1[2][2];

    if(w > -0.0001f && w < 0.0001f)
    {
        return 0;
    }

    *src_x = tx / w;
    *src_y = ty / w;

    return 1;
}

/**
 * @brief 估计鸟瞰赛道宽度，当前没有调用。
 */
static float estimate_ipm_lane_width(
    uint8 *left_line,
    uint8 *right_line,
    uint8 top_y,
    uint8 bottom_y)
{uint16 left_valid_rows;
uint16 right_valid_rows;
    int16 y;
    uint16 count = 0;
    float width_sum = 0.0f;
    float left_u;
    float left_v;
    float right_u;
    float right_v;
    float width;

    for(y = (int16)top_y;
        y <= (int16)bottom_y;
        y++)
    {
        if(left_line[y] == 0 ||
           right_line[y] == 0 ||
           left_line[y] >= right_line[y])
        {
            continue;
        }

        if(!ipm_original_to_bev(
                (float)left_line[y],
                (float)y,
                &left_u,
                &left_v))
        {
            continue;
        }

        if(!ipm_original_to_bev(
                (float)right_line[y],
                (float)y,
                &right_u,
                &right_v))
        {
            continue;
        }

        width = right_u - left_u;

        if(width < 0.0f)
        {
            width = -width;
        }

        if(width >= IPM_MIN_LANE_WIDTH &&
           width <= IPM_MAX_LANE_WIDTH)
        {
            width_sum += width;

            if(count < 65535)
            {
                count++;
            }
        }
    }

    if(count >= 3)
    {
        return width_sum / (float)count;
    }

    return IPM_DEFAULT_LANE_WIDTH;
}
/* ===== 图像处理参数配置 ===== */
#define RING_IPM_TANGENT_STEP   2       /* 参数 RING_IPM_TANGENT_STEP：计算环岛 IPM 边线切线时使用的行步长，当前值为 2。 */

static uint8 ipm_row_ok[MT9V03X_H];      /* 定义ipmrowok变量，用于当前图像处理过程。 */
static float ipm_row_u [MT9V03X_H];
static float ipm_row_v [MT9V03X_H];
static float ipm_row_nu[MT9V03X_H];      /* 定义ipmrownu变量，用于当前图像处理过程。 */
static float ipm_row_nv[MT9V03X_H];

// 参数 RING_IPM_GAP_FILL_ROWS：环岛 IPM 中线允许补齐的最大间隙，当前值为 6。
#define RING_IPM_GAP_FILL_ROWS  6
// 参数 RING_MID_SMALL_GAP：环岛中线可视为小间隙的最大行数，当前值为 6。
#define RING_MID_SMALL_GAP   6    /* 参数 RING_MID_SMALL_GAP：环岛中线可视为小间隙的最大行数，当前值为 6。 */
#define RING_MID_SPIKE_PX    3    /* 参数 RING_MID_SPIKE_PX：环岛中线判定尖峰的最小像素差，当前值为 3。 */

static float ipm_smu[MT9V03X_H];         /* 定义ipmsmu变量，用于当前图像处理过程。 */
static float ipm_smv[MT9V03X_H];

// 定义ringlongshist变量，用于当前图像处理过程。
static uint8 ring_long_s_hist[8]     = {0, 0, 0, 0, 0, 0, 0, 0};
static uint8 ring_freeze_mid[MT9V03X_H];
static uint8 ring_freeze_valid[MT9V03X_H];
static uint8 ring_freeze_ok          = 0;

static uint8 ring_short_mid[MT9V03X_H];
static uint8 ring_short_valid[MT9V03X_H];
static uint8 ring_short_ok           = 0;
/**
 * @brief 将已知边线映射到鸟瞰图，计算每行指向道路内部的法向量。
 */
static uint16 ipm_build_row_normals(const uint8 *line, uint8 side, uint8 top_y, uint8 bot_y)
{
    int16  y;
    uint8  i;
    uint8  i0;
    uint8  i1;
    uint8  cnt = 0;
    uint8  idx[MT9V03X_H];
    float  du;
    float  dv;
    float  dn;
    uint16 ok_rows = 0;

    memset(ipm_row_ok, 0, sizeof(ipm_row_ok));

        // 逐行遍历将已知边线映射到鸟瞰图，计算每行指向道路内部的法向量。
    for(y = (int16)bot_y; y >= (int16)top_y; y--)
    {
        if(line[y] == 0)
        {
            continue;
        }

        if(!ipm_original_to_bev((float)line[y], (float)y,
                                &ipm_row_u[y], &ipm_row_v[y]))
        {
            continue;
        }

        idx[cnt] = (uint8)y;
        cnt++;

        if(cnt >= (uint8)MT9V03X_H)
        {
            break;
        }
    }

    if(cnt == 0)
    {
        return 0;
    }

        // 检查将已知边线映射到鸟瞰图，计算每行指向道路内部的法向量是否满足继续处理的条件。
    if(cnt >= 3)
    {
        for(i = 0; i < cnt; i++)
        {
            uint8 ya = idx[i];
            uint8 y0 = (i > 0)          ? idx[i - 1] : ya;
            uint8 y2 = ((i + 1) < cnt)  ? idx[i + 1] : ya;

            ipm_smu[ya] = (ipm_row_u[y0] + 2.0f * ipm_row_u[ya] + ipm_row_u[y2]) * 0.25f;
            ipm_smv[ya] = (ipm_row_v[y0] + 2.0f * ipm_row_v[ya] + ipm_row_v[y2]) * 0.25f;
        }

        for(i = 0; i < cnt; i++)
        {
            uint8 ya = idx[i];

            ipm_row_u[ya] = ipm_smu[ya];
            ipm_row_v[ya] = ipm_smv[ya];
        }
    }

        // 循环遍历将已知边线映射到鸟瞰图，计算每行指向道路内部的法向量，完成当前统计或筛选。
    for(i = 0; i < cnt; i++)
    {
        uint8 ya = idx[i];

        i0 = (i >= (uint8)RING_IPM_TANGENT_STEP)
           ? (uint8)(i - (uint8)RING_IPM_TANGENT_STEP)
           : 0;
        i1 = (uint8)(i + (uint8)RING_IPM_TANGENT_STEP);

        if(i1 >= cnt)
        {
            i1 = (uint8)(cnt - 1);
        }

        if(i1 == i0)
        {
                        /*
             * 更新逆透视补线与 IPM 结果对应的数组状态。
             */
            ipm_row_nu[ya] = (side == EDGE_LEFT) ?  1.0f : -1.0f;
            ipm_row_nv[ya] =  0.0f;
        }
        else
        {
            du = ipm_row_u[idx[i1]] - ipm_row_u[idx[i0]];
            dv = ipm_row_v[idx[i1]] - ipm_row_v[idx[i0]];

            dn = sqrtf(du * du + dv * dv);

            if(dn < 0.0001f)
            {
                ipm_row_nu[ya] = (side == EDGE_LEFT) ?  1.0f : -1.0f;
                ipm_row_nv[ya] =  0.0f;
            }
            else
            {
                du /= dn;
                dv /= dn;

                if(side == EDGE_LEFT)
                {
                    ipm_row_nu[ya] = -dv;      /* 更新逆透视补线与 IPM 结果对应的数组状态。 */
                    ipm_row_nv[ya] =  du;
                }
                else
                {
                    ipm_row_nu[ya] =  dv;      /* 更新逆透视补线与 IPM 结果对应的数组状态。 */
                    ipm_row_nv[ya] = -du;
                }
            }
        }

        ipm_row_ok[ya] = 1;
        ok_rows++;
    }

    return ok_rows;
}
/**
 * @brief 从较可靠的一侧补出另一侧和中心候选。
 */
static uint8 fill_missing_side_by_ipm(
    uint8 *left_line,
    uint8 *right_line,
    uint8 top_y,
    uint8 bottom_y,
    uint16 left_count,
    uint16 right_count)
{
    uint8 fill_left;
    uint8 *known_line;
    uint8 *missing_line;

    int16 y;
    int16 mapped_y;
    int16 generated_x;
    int16 generated_y;
    int16 mid_x;
    int16 mid_y;

    uint16 left_valid_rows;
    uint16 right_valid_rows;
    uint16 mid_valid_count;

    float road_width;
    float source_u;
    float source_v;
    float target_u;
    float target_v;      /* 定义targetv变量，用于当前图像处理过程。 */
    float target_x;
    float target_y;
    float mid_u;
    float mid_v;         /* 定义midv变量，用于当前图像处理过程。 */
    float mid_x_float;
    float mid_y_float;
    float nu;            /* 定义nu变量，用于当前图像处理过程。 */
    float nv;            /* 定义nv变量，用于当前图像处理过程。 */

    (void)left_count;
    (void)right_count;

    if(left_line == 0 ||
       right_line == 0 ||
       top_y > bottom_y)
    {
        return 0;
    }

    left_valid_rows =
        count_valid_edge_rows(
            left_line,
            top_y,
            bottom_y);

    right_valid_rows =
        count_valid_edge_rows(
            right_line,
            top_y,
            bottom_y);

    if(left_valid_rows < IPM_MIN_KNOWN_POINTS &&
       right_valid_rows < IPM_MIN_KNOWN_POINTS)
    {
        return 0;
    }

        // 检查从较可靠的一侧补出另一侧和中心候选是否满足继续处理的条件。
    if(left_valid_rows <= right_valid_rows)
    {
        fill_left = 1;
        known_line = right_line;
        missing_line = left_line;
    }
    else
    {
        fill_left = 0;
        known_line = left_line;
        missing_line = right_line;
    }

    road_width = IPM_CALIBRATED_LANE_WIDTH;

        // 逐行遍历从较可靠的一侧补出另一侧和中心候选。
    for(y = (int16)top_y;
        y <= (int16)bottom_y;
        y++)
    {
        missing_line[y] = 0;
    }
        /*
     * 将已知边线映射到鸟瞰图，计算每行指向道路内部的法向量。
     */
    (void)ipm_build_row_normals(known_line,
                                (fill_left != 0) ? EDGE_RIGHT : EDGE_LEFT,
                                top_y,
                                bottom_y);
    ipm_generated_edge_count = 0;

    memset(
        ipm_mid_candidate_valid,
        0,
        sizeof(ipm_mid_candidate_valid));

    memset(
        ipm_mid_candidate_x,
        0,
        sizeof(ipm_mid_candidate_x));

        // 逐行遍历从较可靠的一侧补出另一侧和中心候选。
    for(y = (int16)bottom_y;
        y >= (int16)top_y;
        y--)
    {
        if(known_line[y] == 0)
        {
            continue;
        }

        if(!ipm_original_to_bev(
                (float)known_line[y],
                (float)y,
                &source_u,
                &source_v))
        {
            continue;
        }

                // 检查逆透视补线与 IPM 结果是否满足继续处理的条件。
        if(ipm_row_ok[y])
        {
            nu = ipm_row_nu[y];
            nv = ipm_row_nv[y];
        }
        else
        {
                        /*
             * 更新从较可靠的一侧补出另一侧和中心候选相关变量。
             */
            nu = (fill_left != 0) ? -1.0f : 1.0f;
            nv =  0.0f;
        }

        target_u = source_u + nu * road_width;
        target_v = source_v + nv * road_width;

        mid_u = source_u + nu * (road_width * 0.5f);
        mid_v = source_v + nv * (road_width * 0.5f);

                // 检查逆透视补线与 IPM 结果是否满足继续处理的条件。
        if(ipm_bev_to_original(
                target_u,
                target_v,
                &target_x,
                &target_y))        {
            generated_x =
                (int16)(target_x >= 0.0f
                        ? target_x + 0.5f
                        : target_x - 0.5f);

            generated_y =
                (int16)(target_y >= 0.0f
                        ? target_y + 0.5f
                        : target_y - 0.5f);

            if(ipm_generated_edge_count <
               IPM_GENERATED_POINT_MAX)
            {
                ipm_generated_edge[
                    ipm_generated_edge_count].x =
                    generated_x;

                ipm_generated_edge[
                    ipm_generated_edge_count].y =
                    generated_y;

                ipm_generated_edge_count++;
            }
        }

                // 检查逆透视补线与 IPM 结果是否满足继续处理的条件。
        if(ipm_bev_to_original(
                mid_u,
                mid_v,
                &mid_x_float,
                &mid_y_float))
        {
            mid_x =
                (int16)(mid_x_float >= 0.0f
                        ? mid_x_float + 0.5f
                        : mid_x_float - 0.5f);

            mid_y =
                (int16)(mid_y_float >= 0.0f
                        ? mid_y_float + 0.5f
                        : mid_y_float - 0.5f);

            if(mid_y >= (int16)top_y &&
               mid_y <= (int16)bottom_y)
            {
                if(ipm_mid_candidate_valid[mid_y])
                {
                    ipm_mid_candidate_x[mid_y] =
                        (int16)(
                            (ipm_mid_candidate_x[mid_y] +
                             mid_x) / 2);
                }
                else
                {
                    ipm_mid_candidate_x[mid_y] = mid_x;
                    ipm_mid_candidate_valid[mid_y] = 1;
                }
            }
        }
    }

    mid_valid_count = 0;

    for(y = (int16)top_y;
        y <= (int16)bottom_y;
        y++)
    {
        if(ipm_mid_candidate_valid[y])
        {
            mid_valid_count++;
        }
    }

        /*
     * 返回从较可靠的一侧补出另一侧和中心候选的判断或处理结果。
     */
    return (mid_valid_count > 0) ? 1 : 0;
}
/**
 * @brief 将 IPM 中心候选写回中心线数组。
 */
static uint16 build_mid_line_from_ipm_candidates(
    uint8 *mid_line,
    uint8 top_y,
    uint8 bottom_y)
{
    int16 y;
    int16 x;
    uint16 valid_count = 0;

    if(mid_line == 0 ||
       top_y > bottom_y)
    {
        return 0;
    }

        /*
     * 标记某行中线是否由 IPM 补线生成。
     */
    memset(
        ipm_mid_line_valid,
        0,
        sizeof(ipm_mid_line_valid));

        // 逐行遍历将 IPM 中心候选写回中心线数组。
    for(y = (int16)top_y;
        y <= (int16)bottom_y;
        y++)
    {
        mid_line[y] = MID_W;
    }

        // 逐行遍历将 IPM 中心候选写回中心线数组。
    for(y = (int16)top_y;
        y <= (int16)bottom_y;
        y++)
    {
        if(!ipm_mid_candidate_valid[y])
        {
            continue;
        }

        x = ipm_mid_candidate_x[y];

                // 保存经过精确判断后确认的中线横坐标。
        mid_exact_x[y]  = (int16)x;
        mid_exact_ok[y] = 1;

                // 检查将 IPM 中心候选写回中心线数组是否满足继续处理的条件。
        if(x < 1 ||
           x >= MT9V03X_W - 1)
        {
            continue;
        }

        mid_line[y] = (uint8)x;
        ipm_mid_line_valid[y] = 1;
        valid_count++;
    }

        // 检查中线的有效性与连续性是否满足继续处理的条件。
    if(cut_mid_line_by_slope_jump())
    {
        valid_count = 0;

        for(y = (int16)top_y; y <= (int16)bottom_y; y++)
        {
            if(ipm_mid_line_valid[y])
            {
                valid_count++;
            }
        }
    }

    return valid_count;
}
/**
 * @brief 生成整幅逆透视灰度预览图，当前关闭。
 */
void ipm_warp_gray(
    uint8 src[MT9V03X_H][MT9V03X_W],
    uint8 dst[MT9V03X_H][MT9V03X_W])
{
    uint16 dst_x;
    uint16 dst_y;

    int16 src_x;
    int16 src_y;

    float transformed_x;
    float transformed_y;
    float transformed_w;

    for(dst_y = 0;
        dst_y < MT9V03X_H;
        dst_y++)
    {
        for(dst_x = 0;
            dst_x < MT9V03X_W;
            dst_x++)
        {
                        /*
             * 更新逆透视补线与 IPM 结果相关变量。
             */
            transformed_x =
                IPM_MAT1[0][0] * (float)dst_x +
                IPM_MAT1[0][1] * (float)dst_y +
                IPM_MAT1[0][2];

            transformed_y =
                IPM_MAT1[1][0] * (float)dst_x +
                IPM_MAT1[1][1] * (float)dst_y +
                IPM_MAT1[1][2];

            transformed_w =
                IPM_MAT1[2][0] * (float)dst_x +
                IPM_MAT1[2][1] * (float)dst_y +
                IPM_MAT1[2][2];

                        // 检查生成整幅逆透视灰度预览图，当前关闭是否满足继续处理的条件。
            if(transformed_w > -0.0001f &&
               transformed_w <  0.0001f)
            {
                dst[dst_y][dst_x] = 0;
                continue;
            }

                        /*
             * 执行生成整幅逆透视灰度预览图，当前关闭相关操作。
             */
            transformed_x /= transformed_w;
            transformed_y /= transformed_w;

                        /*
             * 更新生成整幅逆透视灰度预览图，当前关闭相关变量。
             */
            src_x = (int16)(transformed_x + 0.5f);
            src_y = (int16)(transformed_y + 0.5f);

                        // 检查生成整幅逆透视灰度预览图，当前关闭是否满足继续处理的条件。
            if(src_x < 0 ||
               src_x >= MT9V03X_W ||
               src_y < 0 ||
               src_y >= MT9V03X_H)
            {
                dst[dst_y][dst_x] = 0;
                continue;
            }

                        /*
             * 执行生成整幅逆透视灰度预览图，当前关闭相关操作。
             */
            dst[dst_y][dst_x] =
                src[src_y][src_x];
        }
    }
}
// 参数 BORDER_ARTIFACT_ENABLE：去图像边框黑色伪影关闭。
#define BORDER_ARTIFACT_ENABLE       0
// 参数 BORDER_ARTIFACT_SIDE：用于配置图像边框伪影边线，当前值为 1。
#define BORDER_ARTIFACT_SIDE         1

// 参数 BORDER_ARTIFACT_Y_MIN：用于配置图像边框伪影Y最小，当前值为 35。
#define BORDER_ARTIFACT_Y_MIN        35
// 参数 BORDER_ARTIFACT_Y_MAX：用于配置图像边框伪影Y最大，当前值为 100。
#define BORDER_ARTIFACT_Y_MAX        100
// 参数 BORDER_ARTIFACT_MAX_X：用于配置图像边框伪影最大X，当前值为 26。
#define BORDER_ARTIFACT_MAX_X        26
// 参数 BORDER_ARTIFACT_MAX_RUN：用于配置图像边框伪影最大连续段，当前值为 18。
#define BORDER_ARTIFACT_MAX_RUN      18

/**
 * @brief 去除图像边框附近的黑色伪影，当前关闭。
 */
static void remove_border_black_artifact(
    uint8 bin[MT9V03X_H][MT9V03X_W])
{
    uint16 y;
    uint16 k;
    uint16 run_length;

#if BORDER_ARTIFACT_ENABLE

    for(y = BORDER_ARTIFACT_Y_MIN;
        y <= BORDER_ARTIFACT_Y_MAX &&
        y < MT9V03X_H;
        y++)
    {
        run_length = 0;

#if BORDER_ARTIFACT_SIDE == 0

                // 在左右边线的有效性条件持续成立期间重复处理。
        while(run_length < MT9V03X_W &&
              run_length <= BORDER_ARTIFACT_MAX_X &&
              bin[y][run_length] == EDGE_BLACK)
        {
            run_length++;
        }

                // 检查左右边线的有效性是否满足继续处理的条件。
        if(run_length > 0 &&
           run_length <= BORDER_ARTIFACT_MAX_RUN &&
           run_length < MT9V03X_W &&
           bin[y][run_length] == EDGE_WHITE)
        {
            for(k = 0; k < run_length; k++)
            {
                bin[y][k] = EDGE_WHITE;
            }
        }

#else

                // 在左右边线的有效性条件持续成立期间重复处理。
        while(run_length < MT9V03X_W &&
              run_length <= BORDER_ARTIFACT_MAX_X &&
              bin[y][MT9V03X_W - 1 - run_length] == EDGE_BLACK)
        {
            run_length++;
        }

                // 检查左右边线的有效性是否满足继续处理的条件。
        if(run_length > 0 &&
           run_length <= BORDER_ARTIFACT_MAX_RUN &&
           run_length < MT9V03X_W &&
           bin[y][MT9V03X_W - 1 - run_length] == EDGE_WHITE)
        {
            for(k = 0; k < run_length; k++)
            {
                bin[y][MT9V03X_W - 1 - k] = EDGE_WHITE;
            }
        }

#endif
    }

#endif
}
/**
 * @brief 孤立黑点周围至少 7 个白点时，将黑点改为白色。
 */
void fill_small_black_holes(
    uint8 bin[MT9V03X_H][MT9V03X_W])
{
    uint16 x;
    uint16 y;
    uint8 white_count;

    for(y = 1;
        y < MT9V03X_H - 1;
        y++)
    {
        for(x = 1;
            x < MT9V03X_W - 1;
            x++)
        {
            if(bin[y][x] != 0)
            {
                continue;
            }

            white_count = 0;

            if(bin[y - 1][x - 1] == 255)
            {
                white_count++;
            }

            if(bin[y - 1][x] == 255)
            {
                white_count++;
            }

            if(bin[y - 1][x + 1] == 255)
            {
                white_count++;
            }

            if(bin[y][x - 1] == 255)
            {
                white_count++;
            }

            if(bin[y][x + 1] == 255)
            {
                white_count++;
            }

            if(bin[y + 1][x - 1] == 255)
            {
                white_count++;
            }

            if(bin[y + 1][x] == 255)
            {
                white_count++;
            }

            if(bin[y + 1][x + 1] == 255)
            {
                white_count++;
            }

                        // 检查孤立黑点周围至少 7 个白点时，将黑点改为白色是否满足继续处理的条件。
            if(white_count >= BLACK_DOT_MIN_WHITE_NEIGHBORS)
            {
                bin[y][x] = 255;
            }
        }
    }
}

/**
 * @brief 逐行统计黑色短条带，满足多黑带和多行条件时返回 1。
 */
uint8 detect_zebra_crossing(
    uint8 bin[MT9V03X_H][MT9V03X_W])
{
    int16 y;
    int16 x;

    uint16 x_start;
    uint16 x_end;

    uint16 row_band_count;
    uint16 black_run;
    uint16 white_run;
    uint16 gap_before_black;

    uint16 good_rows = 0;

    x_start = ZEBRA_SCAN_X_START;
    x_end   = ZEBRA_SCAN_X_END;

        // 检查逐行统计黑色短条带，满足多黑带和多行条件时返回 1是否满足继续处理的条件。
    if(x_start >= MT9V03X_W)
    {
        x_start = 0;
    }

    if(x_end >= MT9V03X_W)
    {
        x_end = MT9V03X_W - 1;
    }

    if(x_start >= x_end)
    {
        return 0;
    }

    for(y = ZEBRA_SCAN_Y_START;
        y <= ZEBRA_SCAN_Y_END &&
        y < MT9V03X_H;
        y++)
    {
        row_band_count = 0;
        black_run = 0;

                /*
         * 更新斑马线特征相关变量。
         */
        white_run = ZEBRA_WHITE_GAP_MIN;

                /*
         * 更新斑马线特征相关变量。
         */
        gap_before_black = ZEBRA_WHITE_GAP_MIN;

        for(x = x_start; x <= x_end; x++)
        {
            if(bin[y][x] == 0)
            {
                                // 检查逐行统计黑色短条带，满足多黑带和多行条件时返回 1是否满足继续处理的条件。
                if(black_run == 0)
                {
                    gap_before_black = white_run;
                }

                if(black_run < 65535)
                {
                    black_run++;
                }

                                /*
                 * 更新逐行统计黑色短条带，满足多黑带和多行条件时返回 1相关变量。
                 */
                white_run = 0;
            }
            else
            {
                                // 检查斑马线特征是否满足继续处理的条件。
                if(black_run >= ZEBRA_BLACK_RUN_MIN)
                {
                                        // 检查斑马线特征是否满足继续处理的条件。
                    if(row_band_count == 0 ||
                       gap_before_black >= ZEBRA_WHITE_GAP_MIN)
                    {
                        if(row_band_count < 255)
                        {
                            row_band_count++;
                        }
                    }
                }

                black_run = 0;

                if(white_run < 65535)
                {
                    white_run++;
                }
            }
        }

                // 检查斑马线特征是否满足继续处理的条件。
        if(black_run >= ZEBRA_BLACK_RUN_MIN)
        {
            if(row_band_count == 0 ||
               gap_before_black >= ZEBRA_WHITE_GAP_MIN)
            {
                if(row_band_count < 255)
                {
                    row_band_count++;
                }
            }
        }

                // 检查斑马线特征是否满足继续处理的条件。
        if(row_band_count >= ZEBRA_MIN_BANDS_PER_ROW)
        {
            good_rows++;
        }
    }

        // 检查斑马线特征是否满足继续处理的条件。
    if(good_rows >= ZEBRA_MIN_GOOD_ROWS)
    {
        return 1;
    }

    return 0;
}
/**
 * @brief 更严格的十字白区二次检测。
 */
static uint8 detect_cross_white_gap_double(
    uint8 img[MT9V03X_H][MT9V03X_W],
    uint8 top_y,
    uint8 bottom_y,
    uint8 *gap_top_y)
{
    int16 y;

    uint8 left_x;
    uint8 right_x;
    uint8 left_found;
    uint8 right_found;

    uint8 missing_streak;
    uint8 support_rows;

    if(img == 0 ||
       gap_top_y == 0 ||
       top_y >= bottom_y)
    {
        return 0;
    }

    missing_streak = 0;

        // 逐行遍历更严格的十字白区二次检测。
    for(y = (int16)bottom_y;
        y >= (int16)top_y;
        y--)
    {
        left_found =
            find_side_edge_in_row(
                img,
                (uint8)y,
                EDGE_LEFT,
                &left_x);

        right_found =
            find_side_edge_in_row(
                img,
                (uint8)y,
                EDGE_RIGHT,
                &right_x);

        if(!left_found &&
           !right_found &&
           row_is_nearly_all_white(
               img,
               (uint8)y))
        {
            if(missing_streak < 255)
            {
                missing_streak++;
            }

            if(missing_streak >=
               CROSS_MIN_MISSING_ROWS)
            {
                *gap_top_y = (uint8)y;
                break;
            }
        }
        else
        {
            missing_streak = 0;
        }
    }

    if(missing_streak <
       CROSS_MIN_MISSING_ROWS)
    {
        return 0;
    }

        /*
     * 更新更严格的十字白区二次检测相关变量。
     */
    support_rows = 0;

    for(y = (int16)bottom_y;
        y >= (int16)top_y;
        y--)
    {
        left_found =
            find_side_edge_in_row(
                img,
                (uint8)y,
                EDGE_LEFT,
                &left_x);

        right_found =
            find_side_edge_in_row(
                img,
                (uint8)y,
                EDGE_RIGHT,
                &right_x);

        if(left_found &&
           right_found &&
           left_x < right_x)
        {
            if(support_rows < 255)
            {
                support_rows++;
            }
        }
    }

    if(support_rows < CROSS_MIN_SUPPORT_ROWS)
    {
        return 0;
    }

    return 1;
}

/* ===== 图像处理参数配置 ===== */
#define RING_ENABLE            1        /* 参数 RING_ENABLE：环岛处理开启。 */
#define RING_DEBUG_ONLY        0        /* 参数 RING_DEBUG_ONLY：不是只检测模式。 */

// 参数 RING_TOP_Y：环岛入口检测的顶部行，当前值为 15。
#define RING_TOP_Y             15

// 参数 RING_BREAK_TOP_Y：环岛入口断线检测的顶部行，当前值为 50。
#define RING_BREAK_TOP_Y       50

// 参数 RING_BREAK_MAX_ROWS：环岛入口断线允许连续存在的最大行数，当前值为 6。
#define RING_BREAK_MAX_ROWS    6

/*
 * 执行环岛状态与环岛边线相关操作。
 */
#if (RING_TOP_Y >= RING_BREAK_TOP_Y)
#error "RING_TOP_Y must be smaller than RING_BREAK_TOP_Y"
#endif

// 参数 RING_STRAIGHT_TOL：环岛长边直线判定容差，当前值为 25。
#define RING_STRAIGHT_TOL      25      /* 参数 RING_STRAIGHT_TOL：环岛长边直线判定容差，当前值为 25。 */

// 参数 RING_SLOPE_SEED_ROWS：环岛斜率判断使用的种子行数，当前值为 6。
#define RING_SLOPE_SEED_ROWS   6       /* 参数 RING_SLOPE_SEED_ROWS：环岛斜率判断使用的种子行数，当前值为 6。 */
#define RING_SLOPE_DIFF_TOL    1      /* 参数 RING_SLOPE_DIFF_TOL：环岛边线斜率差判断容差，当前值为 1。 */
#define RING_SLOPE_PASS_PCT    95     /* 参数 RING_SLOPE_PASS_PCT：环岛边线满足斜率条件所需的百分比，当前值为 95。 */

// 参数 RING_CONFIRM_FRAMES：环岛入口检测所需的连续确认帧数，当前值为 3。
#define RING_CONFIRM_FRAMES    3

// 参数 RING_S1_TOP_A：用于配置环岛S1顶部A，当前值为 20。
#define RING_S1_TOP_A          20       /* 参数 RING_S1_TOP_A：用于配置环岛S1顶部A，当前值为 20。 */
#define RING_S1_TOP_B          28       /* 参数 RING_S1_TOP_B：用于配置环岛S1顶部B，当前值为 28。 */

// 参数 RING_FRAME_MS：环岛状态机估算使用的单帧时间，当前值为 20。
#define RING_FRAME_MS          20       /* 参数 RING_FRAME_MS：环岛状态机估算使用的单帧时间，当前值为 20。 */
// 参数 RING_S1_TIMEOUT_MS：用于配置环岛S1TIMEOUT毫秒，当前值为 5000。
#define RING_S1_TIMEOUT_MS      5000
#define RING_S1_TIMEOUT_FRAMES  (RING_S1_TIMEOUT_MS / RING_FRAME_MS)   /* 宏函数 RING_S1_TIMEOUT_FRAMES(RING_S1_TIMEOUT_MS / RING_FRAME_MS)：根据输入参数计算环 岛S1TIMEOUT帧数。 */
// 参数 RING_INSIDE_MS：进入环岛内部状态后的最小持续时间，当前值为 200。
#define RING_INSIDE_MS         200   /* 参数 RING_INSIDE_MS：进入环岛内部状态后的最小持续时间，当前值为 200。 */
#define RING_COOLDOWN_MS       5000     /* 参数 RING_COOLDOWN_MS：环岛处理结束后的冷却时间，当前值为 5000。 */
// 参数 RING_EXIT_LONG_TOP_Y：环岛出口时长边顶部判定行，当前值为 15。
#define RING_EXIT_LONG_TOP_Y   15       /* 参数 RING_EXIT_LONG_TOP_Y：环岛出口时长边顶部判定行，当前值为 15。 */
#define RING_EXIT_SHORT_TOP_Y  30       /* 参数 RING_EXIT_SHORT_TOP_Y：环岛出口时短边顶部判定行，当前值为 30。 */
// 参数 RING_MID_FALLBACK_ROWS：环岛单边补线失败时触发回退的最少有效行，当前值为 5。
#define RING_MID_FALLBACK_ROWS 5        /* 参数 RING_MID_FALLBACK_ROWS：环岛单边补线失败时触发回退的最少有效行，当前值为 5。 */

// 参数 RING_T_LONG_FB_SHORT_PTS：环岛 T 字场景长短边点数差阈值，当前值为 5。
#define RING_T_LONG_FB_SHORT_PTS   5
#define RING_EXIT_STRAIGHT_PCT 90       /* 参数 RING_EXIT_STRAIGHT_PCT：环岛出口长边直线判定比例，当前值为 90。 */

#define RING_INSIDE_FRAMES     (RING_INSIDE_MS / RING_FRAME_MS)

// 参数 RING_COOLDOWN_FRAMES：环岛处理结束后的冷却帧数。
#define RING_COOLDOWN_FRAMES   (RING_COOLDOWN_MS / RING_FRAME_MS)



// 参数 RING_ENTER_FRAMES：环岛入口状态要求的基础连续帧数，当前值为 12。
#define RING_ENTER_FRAMES      12
// 参数 RING_S1_STAGE2_CONFIRM_FRAMES：用于配置环岛S1STAGE2确认帧数，当前值为 2。
#define RING_S1_STAGE2_CONFIRM_FRAMES 2

// 参数 RING_BIG_EXIT_GYRO_ANGLE_DEG：大环退出所需的累计陀螺仪转角，当前值为 265.0f。
#define RING_BIG_EXIT_GYRO_ANGLE_DEG        265.0f
// 参数 RING_SMALL_EXIT_GYRO_ANGLE_DEG：小环退出所需的累计陀螺仪转角，当前值为 250.0f。
#define RING_SMALL_EXIT_GYRO_ANGLE_DEG      250.0f

// 参数 RING_BIG_AFTER_EXIT_FORCE_ERROR_MIN：大环出环后强制误差的最小值，当前值为 32。
#define RING_BIG_AFTER_EXIT_FORCE_ERROR_MIN      32
// 参数 RING_SMALL_AFTER_EXIT_FORCE_ERROR_MIN：小环出环后强制误差的最小值，当前值为 36。
#define RING_SMALL_AFTER_EXIT_FORCE_ERROR_MIN    36
// 参数 RING_AFTER_EXIT_FORCE_ERROR_MS：出环后强制转向误差保持的毫秒数，当前值为 280。
#define RING_AFTER_EXIT_FORCE_ERROR_MS       280
// 参数 RING_AFTER_EXIT_FORCE_ERROR_FRAMES：出环后强制转向误差保持的帧数。
#define RING_AFTER_EXIT_FORCE_ERROR_FRAMES   (RING_AFTER_EXIT_FORCE_ERROR_MS / RING_FRAME_MS)

// 环岛陀螺仪类型：不使用 IMU。
#define RING_IMU_TYPE_NONE                  0U
// 环岛陀螺仪类型：使用 IMU660RB。
#define RING_IMU_TYPE_IMU660RB              2U

// 参数 RING_INSIDE_MIN_FRAMES：环岛内部状态允许退出的最小帧数，当前值为 20。

#define RING_INSIDE_MIN_FRAMES 20
// 参数 RING_EXIT_FRAMES：环岛出口状态保持的帧数，当前值为 12。
#define RING_EXIT_FRAMES       12
// 参数 RING_MAX_FRAMES：环岛状态机允许持续的最大帧数，当前值为 400。
#define RING_MAX_FRAMES        400

// 参数 RING_ENTER_WIDTH：环岛入口允许的最小赛道宽度。
#define RING_ENTER_WIDTH       (IPM_CALIBRATED_LANE_WIDTH * 0.50f)   /* 参数 RING_ENTER_WIDTH：环岛入口允许的最小赛道宽度。 */
#define RING_BIAS_MIN_ROWS     3      /* 参数 RING_BIAS_MIN_ROWS：环岛中线加入内侧偏置所需的最少有效行，当前值为 3。 */
#define RING_INSIDE_WIDTH      (IPM_CALIBRATED_LANE_WIDTH * 0.50f)   /* 参数 RING_INSIDE_WIDTH：环岛内侧补线使用的宽度。 */
// 参数 RING_FALLBACK_LONG_WIDTH：环岛长边回退补线使用的宽度。
#define RING_FALLBACK_LONG_WIDTH   (IPM_CALIBRATED_LANE_WIDTH * 0.50f)
// 参数 RING_MID_EXTRAP_ROWS：环岛中线允许外推的最大行数，当前值为 25。
#define RING_MID_EXTRAP_ROWS   25      /* 参数 RING_MID_EXTRAP_ROWS：环岛中线允许外推的最大行数，当前值为 25。 */
#define RING_MID_GAP_FILL_ROWS 60      /* 参数 RING_MID_GAP_FILL_ROWS：环岛中线允许线性补齐的最大间隙，当前值为 60。 */
#define RING_SHORT_MIN_ROWS    6     /* 参数 RING_SHORT_MIN_ROWS：环岛短边判定所需的最少有效行，当前值为 6。 */
// 参数 RING_SHORT_TOP_EXTRAP_ENABLE：短边顶部外推关闭。
#define RING_SHORT_TOP_EXTRAP_ENABLE 0
// 参数 RING_T_LONG_S_PASS：环岛 T 字长边通过判定阈值，当前值为 85。
#define RING_T_LONG_S_PASS      85
// 参数 RING_T_LONG_S_FRAMES：环岛 T 字长边连续确认帧数，当前值为 5。
#define RING_T_LONG_S_FRAMES    5
// 参数 RING_LONG_MIN_MID_ROWS：环岛长边生成中线所需的最少有效行，当前值为 12。
#define RING_LONG_MIN_MID_ROWS   12
// 参数 RING_LONG_MIN_POINTS：判定环岛长边所需的最少爬线点，当前值为 10。
#define RING_LONG_MIN_POINTS     10
// 参数 RING_LONG_FIT_PTS：环岛长边拟合使用的最少点数，当前值为 10。
#define RING_LONG_FIT_PTS        10
// 参数 RING_LONG_FIT_BIAS：环岛长边直线拟合的偏置，当前值为 1。
#define RING_LONG_FIT_BIAS       1
// 参数 RING_LONG_FIT_MAX_DX：环岛长边拟合允许的最大横向残差，当前值为 3。
#define RING_LONG_FIT_MAX_DX     3
// 参数 RING_LONG_FIT_MAX_ROWS：环岛长边拟合允许使用的最大行数，当前值为 95。
#define RING_LONG_FIT_MAX_ROWS   95
// 参数 RING_LONG_FIT_ENV：环岛长边拟合允许的包络误差，当前值为 3。
#define RING_LONG_FIT_ENV        3
#define RING_LONG_FIT_SLOPE_PX   2     /* 参数 RING_LONG_FIT_SLOPE_PX：环岛长边拟合使用的斜率像素阈值，当前值为 2。 */
// 参数 RING_EDGE_EXT_REF_ROWS：环岛边线外推使用的参考行数，当前值为 12。
#define RING_EDGE_EXT_REF_ROWS   12    /* 参数 RING_EDGE_EXT_REF_ROWS：环岛边线外推使用的参考行数，当前值为 12。 */
#define RING_EDGE_EXT_MAX_SLOPE  3     /* 参数 RING_EDGE_EXT_MAX_SLOPE：环岛边线外推允许的最大斜率，当前值为 3。 */
#define RING_EDGE_EXT_MAX_CURV   3     /* 参数 RING_EDGE_EXT_MAX_CURV：环岛边线外推允许的最大曲率，当前值为 3。 */
#define RING_EDGE_EXT_MAX_DX     60    /* 参数 RING_EDGE_EXT_MAX_DX：环岛边线外推允许的最大横向距离，当前值为 60。 */
#define RING_EXIT_CONFIRM_FRAMES  5     /* 参数 RING_EXIT_CONFIRM_FRAMES：确认离开环岛所需的连续帧数，当前值为 5。 */
// 参数 RING_MID_EXTRAP_REF_ROWS：环岛中线外推使用的参考行数，当前值为 20。
#define RING_MID_EXTRAP_REF_ROWS   20  /* 参数 RING_MID_EXTRAP_REF_ROWS：环岛中线外推使用的参考行数，当前值为 20。 */
#define RING_MID_CURVE_MAX_DX      45  /* 参数 RING_MID_CURVE_MAX_DX：环岛二次曲线中线外推允许的最大横向距离，当前值为 45。 */
#define RING_MID_EXTRAP_MAX_SLOPE  3  /* 参数 RING_MID_EXTRAP_MAX_SLOPE：环岛中线外推允许的最大斜率，当前值为 3。 */

// 参数 RING_MID_CURVE_SCALE：环岛二次曲线中线外推的比例系数，当前值为 60。
#define RING_MID_CURVE_SCALE       60
// 参数 RING_FILL_TOP_Y：环岛中线填充区域的顶部行，当前值为 20。
#define RING_FILL_TOP_Y        20
// 参数 RING_FILL_BOT_Y：环岛中线填充区域的底部行，当前值为 117。
#define RING_FILL_BOT_Y        117

// 参数 RING_SCAN_Y_TOP：环岛边线扫描的顶部行，当前值为 20。

#define RING_SCAN_Y_TOP        20       /* 参数 RING_SCAN_Y_TOP：环岛边线扫描的顶部行，当前值为 20。 */
#define RING_SCAN_Y_BOT        70       /* 参数 RING_SCAN_Y_BOT：环岛边线扫描的底部行，当前值为 70。 */
#define RING_NEAR_Y            110      /* 参数 RING_NEAR_Y：环岛近景判断使用的参考行，当前值为 110。 */

// 参数 RING_NEAR_SCAN_TOP：环岛近景扫描的顶部行，当前值为 80。
#define RING_NEAR_SCAN_TOP     80    /* 参数 RING_NEAR_SCAN_TOP：环岛近景扫描的顶部行，当前值为 80。 */
#define RING_NEAR_SCAN_BOT     117                 /* 参数 RING_NEAR_SCAN_BOT：环岛近景扫描的底部行，当前值为 117。 */
#define RING_NEAR_MIN_ROWS     2                /* 参数 RING_NEAR_MIN_ROWS：近处环岛边线判定所需的最少行数，当前值为 2。 */

// 参数 RING_SLOPE_SCAN_TOP：环岛斜率扫描的顶部行。
#define RING_SLOPE_SCAN_TOP    (EDGE_TOP_Y + 1)            /* 参数 RING_SLOPE_SCAN_TOP：环岛斜率扫描的顶部行。 */
#define RING_SLOPE_SCAN_BOT    (jidian_search_line - 1)    /* 参数 RING_SLOPE_SCAN_BOT：环岛斜率扫描的底部行。 */

volatile uint8  ring_state    = 0;      /* 环岛状态机的当前状态。 */
volatile uint8  ring_dir      = 0;      /* 更新更严格的十字白区二次检测相关变量。 */
uint16 ring_frames   = 0;      /* 定义ringframes变量，用于当前图像处理过程。 */
uint8  ring_confirm  = 0;      /* 定义ringconfirm变量，用于当前图像处理过程。 */


uint16 ring_cooldown = 0;

// 定义ringafterexitforceerrorframes变量，用于当前图像处理过程。
uint16 ring_after_exit_force_error_frames = 0;

uint8  ring_started  = 0;

/*
 * 更新更严格的十字白区二次检测相关变量。
 */
volatile float ring_gyro_angle_deg = 0.0f;
static float ring_gyro_last_yaw_deg = 0.0f;
static uint8 ring_gyro_integrating = 0;
static uint8 ring_gyro_ready = 0;
volatile uint8 ring_imu_type = RING_IMU_TYPE_NONE;

// 环岛入口短边所在的边线方向。
uint8  ring_short_side = 0;
uint8  ring_cross_latch = 0;   /* 定义ringcrosslatch变量，用于当前图像处理过程。 */
uint8  ring_exit_cnt    = 0;   /* 定义ringexitcnt变量，用于当前图像处理过程。 */
uint8  ring_cross_cnt   = 0;   /* 定义ringcrosscnt变量，用于当前图像处理过程。 */
uint8  ring_dbg_top_l  = 127;  /* 定义ringdbgtopl变量，用于当前图像处理过程。 */
uint8  gocross          = 0;   /* 定义gocross变量，用于当前图像处理过程。 */
uint8  ring_dbg_top_r  = 127;  /* 定义ringdbgtopr变量，用于当前图像处理过程。 */
uint8  ring_dbg_lok    = 0;    /* 定义ringdbglok变量，用于当前图像处理过程。 */
uint8  ring_dbg_rok    = 0;    /* 定义ringdbgrok变量，用于当前图像处理过程。 */
uint8  ring_done_cnt   = 0;    /* 定义ringdonecnt变量，用于当前图像处理过程。 */

uint16 ring_dbg_cnt_l  = 0;    /* 定义ringdbgcntl变量，用于当前图像处理过程。 */
uint16 ring_dbg_cnt_r  = 0;    /* 定义ringdbgcntr变量，用于当前图像处理过程。 */
uint8  ring_dbg_rows_l = 0;    /* 定义ringdbgrowsl变量，用于当前图像处理过程。 */
uint8  ring_dbg_rows_r = 0;    /* 定义ringdbgrowsr变量，用于当前图像处理过程。 */

uint8  ring_dbg_straight   = 0;   /* 定义ringdbgstraight变量，用于当前图像处理过程。 */
uint8  ring_dbg_straight_r = 0;   /* 定义ringdbgstraightr变量，用于当前图像处理过程。 */
uint8  ring_dbg_short_s   = 0;   /* 定义ringdbgshorts变量，用于当前图像处理过程。 */
// 定义ringfitx变量，用于当前图像处理过程。
uint8  ring_fit_x[RING_FIT_PTS_MAX];
uint8  ring_fit_y[RING_FIT_PTS_MAX];
uint16 ring_fit_count = 0;
// 定义ringissmall变量，用于当前图像处理过程。
uint8  ring_is_small = 0;
uint8  ring_dbg_t2     = 0;    /* 定义ringdbgt2变量，用于当前图像处理过程。 */
uint8  ring_dbg_cd     = 0;    /* 定义ringdbgcd变量，用于当前图像处理过程。 */
uint8  ring_dbg_fb     = 0;    /* 定义ringdbgfb变量，用于当前图像处理过程。 */
 uint8  ring_dbg_exit;       /* 定义ringdbgexit变量，用于当前图像处理过程。 */
/**
 * @brief 统计某条行线数组在指定 y 范围内的非零行数。
 */
static uint16 ring_side_valid_rows(uint8 *line, uint8 y1, uint8 y2)
{
    uint16 y;
    uint16 n = 0;

    for(y = y1; y <= y2; y++)
    {
        if(line[y] != 0)
        {
            n++;
        }
    }

    return n;
}

/**
 * @brief 找某条线最靠上的有效 y，找不到返回 127。
 */
static uint8 ring_line_top(uint8 *line)
{
    int16 y;

    for(y = EDGE_TOP_Y + 1; y <= (int16)(jidian_search_line - 1); y++)
    {
        if(line[y] != 0)
        {
            return (uint8)y;
        }
    }

    return 127;
}
/**
 * @brief 返回某侧真实爬线点数，并处理 IPM 丢弃边。
 */
static uint16 ring_side_pts(uint8 side)
{
    if((ring_state == 0U) &&
       (ipm_discard_active != 0U) &&
       (side == ipm_discard_side))
    {
        return 0;
    }

    return (side == EDGE_LEFT)
         ? Left_Edge_Count
         : Right_Edge_Count;
}
/**
 * @brief 遍历原始爬线点，找该侧最靠上的真实点。
 */
static uint8 ring_real_top(uint8 side)
{
    uint16 i;
    uint16 n;
    uint8  best = 127;

        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if((ring_state == 0U) &&
       (ipm_discard_active != 0U) &&
       (side == ipm_discard_side))
    {
        return 127;
    }

    if(side == EDGE_LEFT)
    {
        n = Left_Edge_Count;

        for(i = 0; (i < n) && (i < MAX_EDGE_PTS); i++)
        {
            if(Left_Edge_Pts[i].y < best)
            {
                best = Left_Edge_Pts[i].y;
            }
        }
    }
    else
    {
        n = Right_Edge_Count;

        for(i = 0; (i < n) && (i < MAX_EDGE_PTS); i++)
        {
            if(Right_Edge_Pts[i].y < best)
            {
                best = Right_Edge_Pts[i].y;
            }
        }
    }

    return best;
}

/**
 * @brief 统计原始爬线点在指定 y 范围覆盖了多少不同行。
 */
static uint16 ring_real_rows(uint8 side, uint8 y1, uint8 y2)
{
    uint8  hit[MT9V03X_H];
    uint16 i;
    uint16 n;
    uint16 cnt = 0;
    uint16 y;

        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if((ring_state == 0U) &&
       (ipm_discard_active != 0U) &&
       (side == ipm_discard_side))
    {
        return 0;
    }

    memset(hit, 0, sizeof(hit));

    if(side == EDGE_LEFT)
    {
        n = Left_Edge_Count;

        for(i = 0; (i < n) && (i < MAX_EDGE_PTS); i++)
        {
            if(Left_Edge_Pts[i].y >= y1 && Left_Edge_Pts[i].y <= y2)
            {
                hit[Left_Edge_Pts[i].y] = 1;
            }
        }
    }
    else
    {
        n = Right_Edge_Count;

        for(i = 0; (i < n) && (i < MAX_EDGE_PTS); i++)
        {
            if(Right_Edge_Pts[i].y >= y1 && Right_Edge_Pts[i].y <= y2)
            {
                hit[Right_Edge_Pts[i].y] = 1;
            }
        }
    }

    for(y = y1; y <= y2; y++)
    {
        if(hit[y])
        {
            cnt++;
        }
    }

    return cnt;
}
/*
 * 执行统计原始爬线点在指定 y 范围覆盖了多少不同行相关操作。
 */
static int16 ring_row_x[MT9V03X_H];
static uint8 ring_row_ok[MT9V03X_H];

/**
 * @brief 检查一条边线是否近似为直线。
 */
static uint8 ring_side_line_ok(uint8 side, uint8 y1, uint8 y2, uint8 *dbg_out)
{
    uint16 i;
    uint16 n;
    int16  y;

        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if((ring_state == 0U) &&
       (ipm_discard_active != 0U) &&
       (side == ipm_discard_side))
    {
        if(dbg_out != 0)
        {
            *dbg_out = 0;
        }

        return 0;
    }
    int16  prev_y = -1;
    int16  prev_x = 0;

    int32  seed_dx = 0;             /* 更新巡线种子点相关变量。 */
    int32  seed_dy = 0;             /* 更新巡线种子点相关变量。 */
    uint8  seed_cnt = 0;

    uint8  avg_ready = 0;
    int32  avg_dx = 0;
    int32  avg_dy = 0;

    int32  dx;
    int32  dy;
    int32  abs_dx;
    int32  abs_avg_dx;
    int32  lhs;
    int32  rhs;

    uint16 total_rows = 0;          /* 定义totalrows变量，用于当前图像处理过程。 */
    uint16 good_rows  = 0;          /* 定义goodrows变量，用于当前图像处理过程。 */
    uint8  score = 0;

    memset(ring_row_ok, 0, sizeof(ring_row_ok));
    memset(ring_row_x, 0, sizeof(ring_row_x));

        // 检查左右边线的有效性是否满足继续处理的条件。
    if(side == EDGE_LEFT)
    {
        n = Left_Edge_Count;

        for(i = 0; (i < n) && (i < MAX_EDGE_PTS); i++)
        {
            y = (int16)Left_Edge_Pts[i].y;

            if(y >= (int16)y1 && y <= (int16)y2)
            {
                if(ring_row_ok[y] == 0 ||
                   (int16)Left_Edge_Pts[i].x > ring_row_x[y])
                {
                    ring_row_x[y] = (int16)Left_Edge_Pts[i].x;
                    ring_row_ok[y] = 1;
                }
            }
        }
    }
    else
    {
        n = Right_Edge_Count;

        for(i = 0; (i < n) && (i < MAX_EDGE_PTS); i++)
        {
            y = (int16)Right_Edge_Pts[i].y;

            if(y >= (int16)y1 && y <= (int16)y2)
            {
                if(ring_row_ok[y] == 0 ||
                   (int16)Right_Edge_Pts[i].x < ring_row_x[y])
                {
                    ring_row_x[y] = (int16)Right_Edge_Pts[i].x;
                    ring_row_ok[y] = 1;
                }
            }
        }
    }

        // 逐行遍历检查一条边线是否近似为直线。
    for(y = (int16)y2; y >= (int16)y1; y--)
    {
        if(ring_row_ok[y] == 0)
        {
            continue;
        }

        total_rows++;                       /* 执行检查一条边线是否近似为直线相关操作。 */

                // 检查环岛状态与环岛边线是否满足继续处理的条件。
        if(seed_cnt < RING_SLOPE_SEED_ROWS)
        {
            if(seed_cnt > 0)
            {
                seed_dx += (int32)ring_row_x[y] - (int32)prev_x;
                seed_dy += (int32)prev_y - (int32)y;
            }

            seed_cnt++;
            good_rows++;                    /* 执行检查一条边线是否近似为直线相关操作。 */

            prev_y = y;
            prev_x = ring_row_x[y];
            continue;
        }

        if(avg_ready == 0)
        {
            avg_dx = seed_dx;
            avg_dy = seed_dy;
            avg_ready = 1;

            if(avg_dy <= 0)
            {
                if(dbg_out != 0)
                {
                    *dbg_out = 0;
                }

                return 0;
            }
        }

        dx = (int32)ring_row_x[y] - (int32)prev_x;
        dy = (int32)prev_y - (int32)y;              /* 更新检查一条边线是否近似为直线相关变量。 */

        abs_dx = (dx < 0) ? -dx : dx;
        abs_avg_dx = (avg_dx < 0) ? -avg_dx : avg_dx;

                // 检查检查一条边线是否近似为直线是否满足继续处理的条件。
        if((avg_dx > 0 && dx < 0) ||
           (avg_dx < 0 && dx > 0))
        {
                        // 以下代码继续处理检查一条边线是否近似为直线。
        }
        else
        {
                        /*
             * 更新检查一条边线是否近似为直线相关变量。
             */
            lhs = abs_dx * avg_dy;
            rhs = abs_avg_dx * dy +
                  (int32)RING_SLOPE_DIFF_TOL * dy * avg_dy;

            if(lhs <= rhs)
            {
                good_rows++;
            }
        }

        prev_y = y;
        prev_x = ring_row_x[y];
    }

    if(seed_cnt < RING_SLOPE_SEED_ROWS ||
       total_rows == 0)
    {
        if(dbg_out != 0)
        {
            *dbg_out = 0;
        }

        return 0;
    }

    score = (uint8)((good_rows * 100U) / total_rows);

    if(dbg_out != 0)
    {
        *dbg_out = score;
    }

    return (score >= (uint8)RING_SLOPE_PASS_PCT) ? 1 : 0;
}

/**
 * @brief 将原始真实爬线点补回行边线数组。
 */
static void ring_restore_real_edge(uint8 side)
{
    uint16 i;
    uint16 n;
    int16  y;

        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if((ring_state == 0U) &&
       (ipm_discard_active != 0U) &&
       (side == ipm_discard_side))
    {
        return;
    }

    if(side == EDGE_LEFT)
    {
        n = Left_Edge_Count;

        for(i = 0; (i < n) && (i < MAX_EDGE_PTS); i++)
        {
            y = (int16)Left_Edge_Pts[i].y;

            if(y <= (int16)EDGE_TOP_Y || y > (int16)(jidian_search_line - 1))
            {
                continue;
            }

            if(left_line_list[y] == 0)
            {
                left_line_list[y] = (uint8)Left_Edge_Pts[i].x;
            }
        }
    }
    else
    {
        n = Right_Edge_Count;

        for(i = 0; (i < n) && (i < MAX_EDGE_PTS); i++)
        {
            y = (int16)Right_Edge_Pts[i].y;

            if(y <= (int16)EDGE_TOP_Y || y > (int16)(jidian_search_line - 1))
            {
                continue;
            }

            if(right_line_list[y] == 0)
            {
                right_line_list[y] = (uint8)Right_Edge_Pts[i].x;
            }
        }
    }
}


        /**
         * @brief 从已知边线沿鸟瞰法向量移动半个车道宽度，生成中心候选。
         */
        static uint16 ring_mid_from_side(uint8 side, uint8 top_y, uint8 bot_y, float width)
        {
            int16  y;
            int16  mx;
            int16  my;
            float  u2;
            float  v2;
            float  tx;
            float  ty;

                        /*
             * 执行从已知边线沿鸟瞰法向量移动半个车道宽度，生成中心候选相关操作。
             */
            int16  cand_x[MT9V03X_H];
            uint8  cand_ok[MT9V03X_H];
            uint8  wrote[MT9V03X_H];        /* 定义wrote变量，用于当前图像处理过程。 */
            uint16 n = 0;

            memset(cand_ok, 0, sizeof(cand_ok));
            memset(wrote,   0, sizeof(wrote));

                        // 左侧边线逐行缓存；每一行保存一个横坐标。
            (void)ipm_build_row_normals((side == EDGE_LEFT) ? left_line_list : right_line_list,
                                        side,
                                        top_y,
                                        bot_y);

                        // 逐行遍历从已知边线沿鸟瞰法向量移动半个车道宽度，生成中心候选。
            for(y = (int16)bot_y; y >= (int16)top_y; y--)
            {
                                // 检查逆透视补线与 IPM 结果是否可用；条件不满足时进入回退分支。
                if(!ipm_row_ok[y])
                {
                    continue;
                }

                                /*
                 * 更新逆透视补线与 IPM 结果相关变量。
                 */
                u2 = ipm_row_u[y] + ipm_row_nu[y] * width;
                v2 = ipm_row_v[y] + ipm_row_nv[y] * width;

                if(!ipm_bev_to_original(u2, v2, &tx, &ty))
                {
                    continue;
                }

                mx = (int16)(tx >= 0.0f ? tx + 0.5f : tx - 0.5f);
                my = (int16)(ty >= 0.0f ? ty + 0.5f : ty - 0.5f);      /* 更新从已知边线沿鸟瞰法向量移动半个车道宽度，生成中心候选相关变量。 */

                if(my < (int16)top_y || my > (int16)bot_y)
                {
                    continue;
                }

                if(cand_ok[my] != 0)
                {
                    cand_x[my] = (int16)((cand_x[my] + mx) / 2);        /* 更新从已知边线沿鸟瞰法向量移动半个车道宽度，生成中心候选对应的数组状态。 */
                }
                else
                {
                    cand_x[my] = mx;
                    cand_ok[my] = 1;
                }
            }

                        // 以下代码继续处理从已知边线沿鸟瞰法向量移动半个车道宽度，生成中心候选。
            {
                int16 y_prev = -1;
                int16 x_prev = 0;
                int16 gap, k, xi;

                for(y = (int16)top_y; y <= (int16)bot_y; y++)
                {
                    if(cand_ok[y] == 0)
                    {
                        continue;
                    }

                    if(y_prev >= 0)
                    {
                        gap = (int16)(y - y_prev);

                        if(gap > 1 && gap <= (int16)RING_IPM_GAP_FILL_ROWS)
                        {
                            for(k = 1; k < gap; k++)
                            {
                                xi = (int16)((int32)x_prev +
                                             ((int32)cand_x[y] - (int32)x_prev) * (int32)k / (int32)gap);

                                cand_x[y_prev + k]  = xi;
                                cand_ok[y_prev + k] = 1;
                            }
                        }
                    }

                    y_prev = y;
                    x_prev = cand_x[y];
                }
            }

                        // 逐行遍历从已知边线沿鸟瞰法向量移动半个车道宽度，生成中心候选。
            for(y = (int16)top_y; y <= (int16)bot_y; y++)
            {
                int16 v;

                if(cand_ok[y] == 0)
                {
                    continue;
                }

                if(ipm_mid_line_valid[y] != 0)
                {
                    continue;                       /* 跳过本次循环，继续处理下一行。 */
                }

                v = cand_x[y];

                                // 保存经过精确判断后确认的中线横坐标。
                mid_exact_x[y]  = (int16)v;
                mid_exact_ok[y] = 1;

                if(v < 0)              { v = 0; }
                if(v > MT9V03X_W - 1)  { v = MT9V03X_W - 1; }

                mid_line_list[y]      = (uint8)v;
                ipm_mid_line_valid[y] = 1;
                wrote[y]              = 1;
                n++;
            }

                        // 以下代码继续处理从已知边线沿鸟瞰法向量移动半个车道宽度，生成中心候选。
            {
                int16 yy;
                static uint8 tmp_mid[MT9V03X_H];        /* 定义tmpmid变量，用于当前图像处理过程。 */

                for(yy = 0; yy < MT9V03X_H; yy++)
                {
                    tmp_mid[yy] = mid_line_list[yy];
                }

                for(yy = (int16)top_y + 2; yy <= (int16)bot_y - 2; yy++)
                {
                    if(wrote[yy] == 0)
                    {
                        continue;
                    }

                    if((ipm_mid_line_valid[yy - 2] == 0) || (ipm_mid_line_valid[yy - 1] == 0) ||
                       (ipm_mid_line_valid[yy]     == 0) || (ipm_mid_line_valid[yy + 1] == 0) ||
                       (ipm_mid_line_valid[yy + 2] == 0))
                    {
                        continue;
                    }

                    mid_line_list[yy] = (uint8)((      (int16)tmp_mid[yy - 2]
                                                + 2 * (int16)tmp_mid[yy - 1]
                                                + 3 * (int16)tmp_mid[yy]
                                                + 2 * (int16)tmp_mid[yy + 1]
                                                +     (int16)tmp_mid[yy + 2]) / 9);
                }
            }

            return n;
        }
                /**
         * @brief 环岛中线清洗：小间隙填补、中值滤波、5 行加权平滑。
         */
        static void ring_mid_cleanup(void)
        {
            static uint8 src[MT9V03X_H];
            int16 y;
            int16 yy;
            int16 k;
            int16 j;
            int16 t;
            int16 cnt;
            int16 pass;
            int16 v[9];
            int16 med;
            int16 y_prev;
            int16 x_prev;
            int16 gap;
            int16 a, b, c, d, e;
            int32 sum;

                        /*
             * 更新环岛中线清洗：小间隙填补、中值滤波、5 行加权平滑相关变量。
             */
            y_prev = -1;
            x_prev = 0;

            for(y = (int16)(EDGE_TOP_Y + 1); y <= (int16)(jidian_search_line - 1); y++)
            {
                if(ipm_mid_line_valid[y] == 0)
                {
                    continue;
                }

                if(y_prev >= 0)
                {
                    gap = (int16)(y - y_prev);

                    if(gap > 1 && gap <= (int16)RING_MID_SMALL_GAP)
                    {
                        for(k = 1; k < gap; k++)
                        {
                            mid_line_list[y_prev + k] =
                                (uint8)((int16)x_prev +
                                        (int16)(((int32)mid_line_list[y] - (int32)x_prev) * (int32)k / (int32)gap));

                            ipm_mid_line_valid[y_prev + k] = 1;
                        }
                    }
                }

                y_prev = y;
                x_prev = (int16)mid_line_list[y];
            }

                        // 循环遍历环岛中线清洗：小间隙填补、中值滤波、5 行加权平滑，完成当前统计或筛选。
            for(pass = 0; pass < 2; pass++)
            {
                for(y = 0; y < MT9V03X_H; y++)
                {
                    src[y] = mid_line_list[y];
                }

                for(y = (int16)(EDGE_TOP_Y + 1); y <= (int16)(jidian_search_line - 1); y++)
                {
                    if(ipm_mid_line_valid[y] == 0)
                    {
                        continue;
                    }

                    cnt = 0;

                    for(k = -4; k <= 4; k++)
                    {
                        yy = (int16)(y + k);

                        if(yy < (int16)(EDGE_TOP_Y + 1) || yy > (int16)(jidian_search_line - 1))
                        {
                            continue;
                        }

                        if(ipm_mid_line_valid[yy] == 0)
                        {
                            continue;
                        }

                        v[cnt] = (int16)src[yy];
                        cnt++;
                    }

                    if(cnt < 4)
                    {
                        continue;                       /* 跳过本次循环，继续处理下一行。 */
                    }

                                        // 循环遍历环岛中线清洗：小间隙填补、中值滤波、5 行加权平滑，完成当前统计或筛选。
                    for(j = 1; j < cnt; j++)
                    {
                        t = v[j];

                        for(k = (int16)(j - 1); (k >= 0) && (v[k] > t); k--)
                        {
                            v[k + 1] = v[k];
                        }

                        v[k + 1] = t;
                    }

                    med = v[cnt >> 1];

                    if((mid_line_list[y] > med && (int16)(mid_line_list[y] - med) > (int16)RING_MID_SPIKE_PX) ||
                       (med > mid_line_list[y] && (int16)(med - mid_line_list[y]) > (int16)RING_MID_SPIKE_PX))
                    {
                        mid_line_list[y] = (uint8)med;
                    }
                }
            }

                        // 逐行遍历环岛中线清洗：小间隙填补、中值滤波、5 行加权平滑。
            for(y = 0; y < MT9V03X_H; y++)
            {
                src[y] = mid_line_list[y];
            }

            for(y = (int16)(EDGE_TOP_Y + 3); y <= (int16)(jidian_search_line - 3); y++)
            {
                if((ipm_mid_line_valid[y - 2] == 0) || (ipm_mid_line_valid[y - 1] == 0) ||
                   (ipm_mid_line_valid[y]     == 0) || (ipm_mid_line_valid[y + 1] == 0) ||
                   (ipm_mid_line_valid[y + 2] == 0))
                {
                    continue;
                }

                a = (int16)src[y - 2];
                b = (int16)src[y - 1];
                c = (int16)src[y];
                d = (int16)src[y + 1];
                e = (int16)src[y + 2];

                sum = (int32)a + (int32)(2 * b) + (int32)(3 * c) + (int32)(2 * d) + (int32)e;

                mid_line_list[y] = (uint8)((sum + 4) / 9);
            }
        }


/**
 * @brief 在两个端点之间填充直线中心线，当前没有实际调用。
 */
static void ring_pull_line(uint8 y1, int16 x1, uint8 y2, int16 x2)
{
    int16 y;
    int16 t;
    int16 x;
    int16 dy;
    int32 dx;

    if(y1 > y2)
    {
        t = y1;  y1 = y2;  y2 = t;
        t = x1;  x1 = x2;  x2 = t;
    }

    dy = (int16)y2 - (int16)y1;

    if(dy <= 0)
    {
        return;
    }

    dx = (int32)x2 - (int32)x1;

    for(y = y1; y <= y2; y++)
    {
        x = (int16)((int32)x1 + dx * ((int32)y - (int32)y1) / dy);

        if(x < 1)
        {
            x = 1;
        }

        if(x > MT9V03X_W - 2)
        {
            x = MT9V03X_W - 2;
        }

        mid_line_list[y] = (uint8)x;
        ipm_mid_line_valid[y] = 1;
    }
}

/**
 * @brief 左右真实边线都有效时，将对应中心线标记为可用。
 */
static void ring_mark_normal_mid_valid(void)
{
    int16 y;

    for(y = EDGE_TOP_Y + 1; y <= (int16)(jidian_search_line - 1); y++)
    {
        if(left_line_list[y] != 0 &&
           right_line_list[y] != 0 &&
           left_line_list[y] < right_line_list[y])
        {
                        // 标记某行中线是否由 IPM 补线生成。
            ipm_mid_line_valid[y] = 1;
        }
    }
}
/**
 * @brief 使用三个参考点做二次曲线外推，当前被宏关闭。
 */
static uint16 ring_extend_mid_curve(void)
{
    int32  trend;                       /* 执行使用三个参考点做二次曲线外推，当前被宏关闭相关操作。 */
    int16  x_lin;
    int16 ys[RING_MID_EXTRAP_REF_ROWS];
    int16 xs[RING_MID_EXTRAP_REF_ROWS];

    uint8  cnt = 0;
    uint8  k;
    int16  y;
    int16  y0;
    int16  y1;
    int16  y2;
    int16  x0;
    int16  x1;
    int16  x2;
    int16  x;
    int16  x_last;
    int32  term0;
    int32  term1;
    int32  term2;
    uint16 n = 0;

        // 逐行遍历环岛状态与环岛边线。
    for(y = (int16)RING_FILL_TOP_Y; y <= (int16)RING_FILL_BOT_Y; y++)
    {
        if(ipm_mid_line_valid[y] != 0)
        {
            ys[cnt] = y;
            xs[cnt] = (int16)mid_line_list[y];
            cnt++;

            if(cnt >= (uint8)RING_MID_EXTRAP_REF_ROWS)
            {
                break;
            }
        }
    }

    if(cnt < 3)
    {
        return 0;                       /* 返回使用三个参考点做二次曲线外推，当前被宏关闭的判断或处理结果。 */
    }

    y0 = ys[0];              x0 = xs[0];            /* 更新使用三个参考点做二次曲线外推，当前被宏关闭相关变量。 */
    y1 = ys[cnt >> 1];       x1 = xs[cnt >> 1];      /* 更新使用三个参考点做二次曲线外推，当前被宏关闭相关变量。 */
    y2 = ys[cnt - 1];        x2 = xs[cnt - 1];       /* 更新使用三个参考点做二次曲线外推，当前被宏关闭相关变量。 */

    if(y1 <= y0 || y2 <= y1)
    {
        return 0;
    }

        /*
     * 更新使用三个参考点做二次曲线外推，当前被宏关闭相关变量。
     */
    trend = (int32)x2 - (int32)x0;

    x_last = x0;

        // 循环遍历环岛状态与环岛边线，完成当前统计或筛选。
    for(k = 1; (int16)(y0 - (int16)k) >= (int16)RING_FILL_TOP_Y; k++)
    {
        y = (int16)(y0 - (int16)k);

                /*
         * 更新使用三个参考点做二次曲线外推，当前被宏关闭相关变量。
         */
        term0 = (int32)x0 * (int32)(y - y1) * (int32)(y - y2) /
                ((int32)(y0 - y1) * (int32)(y0 - y2));

        term1 = (int32)x1 * (int32)(y - y0) * (int32)(y - y2) /
                ((int32)(y1 - y0) * (int32)(y1 - y2));

        term2 = (int32)x2 * (int32)(y - y0) * (int32)(y - y1) /
                ((int32)(y2 - y0) * (int32)(y2 - y1));

        x = (int16)(term0 + term1 + term2);

                /*
         * 更新环岛状态与环岛边线相关变量。
         */
        x = (int16)((int32)x0 + ((int32)(x - x0) * (int32)RING_MID_CURVE_SCALE) / 100);

                // 检查使用三个参考点做二次曲线外推，当前被宏关闭是否满足继续处理的条件。
        if(trend >= 0)
        {
            if(x > x0)
            {
                break;
            }
        }
        else
        {
            if(x < x0)
            {
                break;
            }
        }
                // 检查环岛状态与环岛边线是否满足继续处理的条件。
        if((x - x0) > (int16)RING_MID_CURVE_MAX_DX ||
           (x0 - x) > (int16)RING_MID_CURVE_MAX_DX)
        {
            break;
        }

                // 检查环岛状态与环岛边线是否满足继续处理的条件。
        if((x - x_last) > (int16)RING_MID_EXTRAP_MAX_SLOPE)
        {
            x = (int16)(x_last + (int16)RING_MID_EXTRAP_MAX_SLOPE);
        }

        if((x_last - x) > (int16)RING_MID_EXTRAP_MAX_SLOPE)
        {
            x = (int16)(x_last - (int16)RING_MID_EXTRAP_MAX_SLOPE);
        }

                // 检查使用三个参考点做二次曲线外推，当前被宏关闭是否满足继续处理的条件。
        if(x < 1 || x > MT9V03X_W - 2)
        {
            break;
        }

        mid_line_list[y]      = (uint8)x;
        ipm_mid_line_valid[y] = 1;
        x_last                = x;
        n++;
    }

    return n;
}
/**
 * @brief 向远端和近端线性外推中心线，当前被宏关闭。
 */
static void ring_extend_mid(void)
{
    int16 y;
    int16 y_a;
    int16 y_b;
    int16 x_a = 0;
    int16 x_b = 0;
    int16 dy;
    int32 dx;
    int16 x;
    uint8 rows;
    uint8 ref_cnt;

        // 以下代码继续处理向远端和近端线性外推中心线，当前被宏关闭。
    {
        int16 y_prev = -1;
        int16 x_prev = 0;
        int16 gap;
        int16 k;
        int16 xi;

        for(y = (int16)RING_FILL_TOP_Y; y <= (int16)RING_FILL_BOT_Y; y++)
        {
            if(ipm_mid_line_valid[y] == 0)
            {
                continue;
            }

            if(y_prev >= 0)
            {
                gap = y - y_prev;

                if(gap > 1 && gap <= (int16)RING_MID_GAP_FILL_ROWS)
                {
                    for(k = 1; k < gap; k++)
                    {
                        xi = (int16)((int32)x_prev +
                                     ((int32)mid_line_list[y] - (int32)x_prev) *
                                     (int32)k / (int32)gap);

                                                // 检查向远端和近端线性外推中心线，当前被宏关闭是否满足继续处理的条件。
                        if(xi < 1 || xi > MT9V03X_W - 2)
                        {
                            continue;
                        }

                        mid_line_list[y_prev + k]      = (uint8)xi;
                        ipm_mid_line_valid[y_prev + k] = 1;
                    }
                }
            }

            y_prev = y;
            x_prev = (int16)mid_line_list[y];
        }
    }

        // 检查向远端和近端线性外推中心线，当前被宏关闭是否满足继续处理的条件。
    if(ring_extend_mid_curve() == 0)
    {
        y_a = -1;
        y_b = -1;
        ref_cnt = 0;

        for(y = (int16)RING_FILL_TOP_Y; y <= (int16)RING_FILL_BOT_Y; y++)
        {
            if(ipm_mid_line_valid[y] != 0)
            {
                if(y_a < 0)
                {
                    y_a = y;
                    x_a = (int16)mid_line_list[y];
                }

                y_b = y;
                x_b = (int16)mid_line_list[y];
                ref_cnt++;

                if(ref_cnt >= (uint8)RING_MID_EXTRAP_REF_ROWS)
                {
                    break;
                }
            }
        }

        if(y_a >= 0 && y_b > y_a && ref_cnt >= 2)
        {
            dy = y_b - y_a;
            dx = (int32)x_b - (int32)x_a;

            if(dx > (int32)RING_MID_EXTRAP_MAX_SLOPE * (int32)dy)
            {
                dx = (int32)RING_MID_EXTRAP_MAX_SLOPE * (int32)dy;
            }

            if(dx < -(int32)RING_MID_EXTRAP_MAX_SLOPE * (int32)dy)
            {
                dx = -(int32)RING_MID_EXTRAP_MAX_SLOPE * (int32)dy;
            }

            for(rows = 0; rows < RING_MID_EXTRAP_ROWS; rows++)
            {
                y = y_a - (int16)(rows + 1);

                if(y < (int16)RING_FILL_TOP_Y)
                {
                    break;
                }

                x = (int16)((int32)x_a - (dx * (int32)(rows + 1)) / (int32)dy);

                                /*
                 * 更新环岛状态与环岛边线相关变量。
                 */
                x = (int16)((int32)x_a + ((int32)(x - x_a) * (int32)RING_MID_CURVE_SCALE) / 100);

                                // 检查向远端和近端线性外推中心线，当前被宏关闭是否满足继续处理的条件。
                if(x < 1 || x > MT9V03X_W - 2)
                {
                    break;
                }

                mid_line_list[y]      = (uint8)x;
                ipm_mid_line_valid[y] = 1;
            }
        }
    }

        /*
     * 更新向远端和近端线性外推中心线，当前被宏关闭相关变量。
     */
    y_a = -1;
    y_b = -1;
    ref_cnt = 0;

    for(y = (int16)RING_FILL_BOT_Y; y >= (int16)RING_FILL_TOP_Y; y--)
    {
        if(ipm_mid_line_valid[y] != 0)
        {
            if(y_a < 0)
            {
                y_a = y;
                x_a = (int16)mid_line_list[y];
            }

            y_b = y;
            x_b = (int16)mid_line_list[y];
            ref_cnt++;

            if(ref_cnt >= (uint8)RING_MID_EXTRAP_REF_ROWS)
            {
                break;
            }
        }
    }

    if(y_a >= 0 && y_b < y_a && ref_cnt >= 2)
    {
        dy = y_a - y_b;
        dx = (int32)x_a - (int32)x_b;

        if(dx > (int32)RING_MID_EXTRAP_MAX_SLOPE * (int32)dy)
        {
            dx = (int32)RING_MID_EXTRAP_MAX_SLOPE * (int32)dy;
        }

        if(dx < -(int32)RING_MID_EXTRAP_MAX_SLOPE * (int32)dy)
        {
            dx = -(int32)RING_MID_EXTRAP_MAX_SLOPE * (int32)dy;
        }

        for(rows = 0; rows < RING_MID_EXTRAP_ROWS; rows++)
        {
            y = y_a + (int16)(rows + 1);

            if(y > (int16)RING_FILL_BOT_Y)
            {
                break;
            }

            x = (int16)((int32)x_a + (dx * (int32)(rows + 1)) / (int32)dy);

                        // 检查向远端和近端线性外推中心线，当前被宏关闭是否满足继续处理的条件。
            if(x < 1 || x > MT9V03X_W - 2)
            {
                break;
            }

            mid_line_list[y]      = (uint8)x;
            ipm_mid_line_valid[y] = 1;
        }
    }
}

/**
 * @brief 统计环岛补线中心线的有效行数。
 */
static uint16 ring_ipm_valid_rows(void);
/**
 * @brief 环岛单边补线主工具。
 */
static uint8 ring_try_one_side_mid(void)
{
    uint8  many_side;
    uint8  few_side;
    uint8  y;
    uint8  few_base_y = 0;
    int16  few_x;
    int16  many_x;
    int16  dx;
    uint16 rows_left;
    uint16 rows_right;
    uint16 rows_many;
    uint16 n;
    float  width;
    uint8  mid_backup[MT9V03X_H];
    uint8  valid_backup[MT9V03X_H];

        // 左侧边线逐行缓存；每一行保存一个横坐标。
    rows_left  = count_valid_edge_rows(left_line_list,  EDGE_TOP_Y + 1, jidian_search_line - 1);
    rows_right = count_valid_edge_rows(right_line_list, EDGE_TOP_Y + 1, jidian_search_line - 1);

        // 检查环岛单边补线主工具是否满足继续处理的条件。
    if(rows_left == rows_right)
    {
        many_side = ring_short_side;
    }
    else if(rows_left > rows_right)
    {
        many_side = EDGE_LEFT;
    }
    else
    {
        many_side = EDGE_RIGHT;
    }

    few_side  = (many_side == EDGE_LEFT) ? EDGE_RIGHT : EDGE_LEFT;
    rows_many = (many_side == EDGE_LEFT) ? rows_left : rows_right;

        // 逐行遍历左右边线的有效性。
    for(y = EDGE_TOP_Y + 1; y <= jidian_search_line - 1; y++)
    {
        if((few_side == EDGE_LEFT) ? (left_line_list[y] != 0)
                                   : (right_line_list[y] != 0))
        {
            few_base_y = y;
        }
    }

        /*
     * 更新环岛单边补线主工具相关变量。
     */
    dx = 32767;

    if(few_base_y != 0)
    {
        few_x  = (few_side  == EDGE_LEFT) ? (int16)left_line_list[few_base_y]
                                          : (int16)right_line_list[few_base_y];
        many_x = (many_side == EDGE_LEFT) ? (int16)left_line_list[few_base_y]
                                          : (int16)right_line_list[few_base_y];

        if(many_x != 0 && few_x != 0)
        {
            dx = many_x - few_x;

            if(dx < 0)
            {
                dx = -dx;
            }
        }
    }

        // 检查逆透视补线与 IPM 结果是否满足继续处理的条件。
    if(dx <= (int16)IPM_MERGE_MAX_GAP_X)
    {
        return 0;
    }

        // 检查逆透视补线与 IPM 结果是否满足继续处理的条件。
    if(rows_many < (uint16)IPM_MIN_KNOWN_POINTS)
    {
        return 0;
    }

        // 逐行遍历环岛单边补线主工具。
    for(y = 0; y < MT9V03X_H; y++)
    {
        mid_backup[y]   = mid_line_list[y];
        valid_backup[y] = ipm_mid_line_valid[y];
    }

    memset(ipm_mid_line_valid, 0, sizeof(ipm_mid_line_valid));

        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if((ring_state == 1) && (many_side == RING_LONG_SIDE()))
    {
        width = RING_ENTER_WIDTH;                       /* 更新环岛状态与环岛边线相关变量。 */
    }
    else
    {
        width = (many_side == RING_LONG_SIDE()) ? RING_FALLBACK_LONG_WIDTH
                                                : RING_INSIDE_WIDTH;
    }

    n = ring_mid_from_side(many_side,
                           RING_FILL_TOP_Y,
                           RING_FILL_BOT_Y,
                           width);
        /*
     * 执行环岛状态与环岛边线相关操作。
     */

#if (RING_SHORT_TOP_EXTRAP_ENABLE)
    if(many_side == ring_short_side)
    {
        (void)ring_extend_mid_curve();
        n = ring_ipm_valid_rows();
    }
#endif

    if(n < (uint16)RING_MID_FALLBACK_ROWS)
    {
                // 逐行遍历环岛单边补线主工具。
        for(y = 0; y < MT9V03X_H; y++)
        {
            mid_line_list[y]      = mid_backup[y];
            ipm_mid_line_valid[y] = valid_backup[y];
        }

        return 0;
    }

    road_mode       = ROAD_MODE_IPM_FILL;
    search_end_line = RING_FILL_TOP_Y - 1;

    return 1;
}
/**
 * @brief 环岛状态 1 的单边补线包装函数。
 */
static void ring_state1_fill_guard(void)
{
    (void)ring_try_one_side_mid();
}
/**
 * @brief 统计环岛补线中心线的有效行数。
 */
static uint16 ring_ipm_valid_rows(void)
{
    int16 y;
    uint16 n = 0;

    for(y = EDGE_TOP_Y + 1;
        y <= (int16)(jidian_search_line - 1);
        y++)
    {
        if(ipm_mid_line_valid[y] != 0)
        {
            n++;
        }
    }

    return n;
}
/**
 * @brief 允许中心线中间断最多 60 行，并使用首尾线性插值。
 */
static void ring_fill_mid_gap(void)
{
    int16 y;
    int16 y_prev = -1;
    int16 x_prev = 0;
    int16 gap;
    int16 k;
    int16 xi;

    for(y = (int16)RING_FILL_TOP_Y; y <= (int16)RING_FILL_BOT_Y; y++)
    {
        if(ipm_mid_line_valid[y] == 0)
        {
            continue;
        }

        if(y_prev >= 0)
        {
            gap = y - y_prev;

            if(gap > 1 && gap <= (int16)RING_MID_GAP_FILL_ROWS)
            {
                for(k = 1; k < gap; k++)
                {
                    xi = (int16)((int32)x_prev +
                                 ((int32)mid_line_list[y] - (int32)x_prev) *
                                 (int32)k / (int32)gap);

                    if(xi < 1)             { xi = 1; }
                    if(xi > MT9V03X_W - 2) { xi = MT9V03X_W - 2; }

                    mid_line_list[y_prev + k]      = (uint8)xi;
                    ipm_mid_line_valid[y_prev + k] = 1;
                }
            }
        }

        y_prev = y;
        x_prev = (int16)mid_line_list[y];
    }
}

/**
 * @brief 构造带曲率外推的边线，当前没有调用。
 */
static uint8 ring_build_ext_line(uint8 side, uint8 *out_line)
{
    int16 ys[RING_EDGE_EXT_REF_ROWS];
    int16 xs[RING_EDGE_EXT_REF_ROWS];

    uint16 i;
    uint16 n;
    int16  y;
    int16  cnt = 0;
    int16  y_top;
    int16  y_mid;
    int16  y_bot;
    int16  x_top;
    int16  x_mid;
    int16  x_bot;
    int16  x;
    int16  trend;
    int32  slope;              /* 执行边线斜率与直线拟合相关操作。 */
    int32  slope2;
    int32  curv;               /* 执行构造带曲率外推的边线，当前没有调用相关操作。 */
    int32  cur;
    int32  x_acc;              /* 执行构造带曲率外推的边线，当前没有调用相关操作。 */
    uint8  ext = 0;

        // 逐行遍历构造带曲率外推的边线，当前没有调用。
    for(y = 0; y < MT9V03X_H; y++)
    {
        out_line[y] = 0;
    }

    n = ring_side_pts(side);        /* 返回某侧真实爬线点数，并处理 IPM 丢弃边。 */
    for(i = 0; (i < n) && (i < MAX_EDGE_PTS); i++)
    {
        y = (int16)((side == EDGE_LEFT) ? Left_Edge_Pts[i].y : Right_Edge_Pts[i].y);

        if(y <= (int16)EDGE_TOP_Y || y > (int16)(jidian_search_line - 1))
        {
            continue;
        }

        x = (int16)((side == EDGE_LEFT) ? Left_Edge_Pts[i].x : Right_Edge_Pts[i].x);

        if(side == EDGE_LEFT)
        {
            if(out_line[y] == 0 || x > (int16)out_line[y])
            {
                out_line[y] = (uint8)x;
            }
        }
        else
        {
            if(out_line[y] == 0 || x < (int16)out_line[y])
            {
                out_line[y] = (uint8)x;
            }
        }
    }

        // 逐行遍历左右边线的有效性。
    for(y = (int16)EDGE_TOP_Y + 1;
        (y <= (int16)RING_FILL_BOT_Y) && (cnt < (int16)RING_EDGE_EXT_REF_ROWS);
        y++)
    {
        if(out_line[y] != 0)
        {
            ys[cnt] = y;
            xs[cnt] = (int16)out_line[y];
            cnt++;
        }
    }

    if(cnt < 3)
    {
        return 0;                   /* 返回构造带曲率外推的边线，当前没有调用的判断或处理结果。 */
    }

    y_top = ys[0];
    x_top = xs[0];
    y_mid = ys[cnt / 2];
    x_mid = xs[cnt / 2];
    y_bot = ys[cnt - 1];
    x_bot = xs[cnt - 1];

    if(y_top <= (int16)RING_FILL_TOP_Y || y_mid <= y_top || y_bot <= y_mid)
    {
        return 0;
    }

        /*
     * 更新边线斜率与直线拟合相关变量。
     */
    slope  = (((int32)x_mid - (int32)x_top) << 4) / (int32)(y_mid - y_top);
    slope2 = (((int32)x_bot - (int32)x_top) << 4) / (int32)(y_bot - y_top);

    curv = (slope2 - slope) / (int32)(y_bot - y_mid);

    if(curv >  (int32)RING_EDGE_EXT_MAX_CURV)
    {
        curv =  (int32)RING_EDGE_EXT_MAX_CURV;
    }

    if(curv < -(int32)RING_EDGE_EXT_MAX_CURV)
    {
        curv = -(int32)RING_EDGE_EXT_MAX_CURV;
    }

        /*
     * 更新边线斜率与直线拟合相关变量。
     */
    trend = (slope > 0) ? 1 : ((slope < 0) ? -1 : 0);

        /*
     * 更新边线斜率与直线拟合相关变量。
     */
    cur   = slope;
    x_acc = ((int32)x_top) << 4;

    for(y = y_top - 1; y >= (int16)RING_FILL_TOP_Y; y--)
    {
        x_acc -= cur;                          /* 执行构造带曲率外推的边线，当前没有调用相关操作。 */

        if(cur >  ((int32)RING_EDGE_EXT_MAX_SLOPE << 4))
        {
            cur =  ((int32)RING_EDGE_EXT_MAX_SLOPE << 4);
        }

        if(cur < -((int32)RING_EDGE_EXT_MAX_SLOPE << 4))
        {
            cur = -((int32)RING_EDGE_EXT_MAX_SLOPE << 4);
        }

        x = (int16)(x_acc >> 4);

        if(x < 1)             { x = 1; }
        if(x > MT9V03X_W - 2) { x = MT9V03X_W - 2; }

                // 检查构造带曲率外推的边线，当前没有调用是否满足继续处理的条件。
        if((trend > 0) && ((int32)x > (int32)x_top))
        {
            break;
        }

        if((trend < 0) && ((int32)x < (int32)x_top))
        {
            break;
        }

                // 检查环岛状态与环岛边线是否满足继续处理的条件。
        if(((int32)x - (int32)x_top) >  (int32)RING_EDGE_EXT_MAX_DX ||
           ((int32)x_top - (int32)x) >  (int32)RING_EDGE_EXT_MAX_DX)
        {
            break;
        }

        out_line[y] = (uint8)x;
        ext++;

        cur += curv;                           /* 执行构造带曲率外推的边线，当前没有调用相关操作。 */
    }

    return ext;
}

/**
 * @brief 从任意边线生成中心线，当前没有调用。
 */
static uint16 ring_mid_from_line(const uint8 *line, uint8 side, uint8 top_y, uint8 bot_y, float width)
{
    int16 y;
    int16 x;
    int16 mx;

    float u;
    float v;
    float u2;
    float tx;
    float ty;

    uint16 n = 0;

    for(y = (int16)bot_y; y >= (int16)top_y; y--)
    {
        if(ipm_mid_line_valid[y] != 0)
        {
            continue;                       /* 跳过本次循环，继续处理下一行。 */
        }

        x = (int16)line[y];

        if(x == 0)
        {
            continue;
        }

        if(!ipm_original_to_bev((float)x, (float)y, &u, &v))
        {
            continue;
        }

        u2 = (side == EDGE_LEFT) ? (u + width) : (u - width);

        if(!ipm_bev_to_original(u2, v, &tx, &ty))
        {
            continue;
        }

        mx = (int16)(tx >= 0.0f ? tx + 0.5f : tx - 0.5f);

        if(mx < 1 || mx > MT9V03X_W - 2)
        {
            continue;
        }

        mid_line_list[y] = (uint8)mx;
        ipm_mid_line_valid[y] = 1;
        n++;
    }

    return n;
}

/**
 * @brief 按固定小斜率延长环岛长边，直到周围不再有白色道路。
 */
static uint16 ring_long_extend_to_edge(uint8 side)
{
    uint8 *line;
    int16  ys[RING_LONG_FIT_PTS];
    int16  xs[RING_LONG_FIT_PTS];
    uint8  cnt = 0;

    int16  y;
    int16  k;
    int16  kk;
    int16  y0;
    int16  x0;
    int16  x;
    int16  x_last;
    int16  slope;
    int16  xx;

    uint8  white_env;
    uint16 n = 0;

    line = (side == EDGE_LEFT) ? left_line_list : right_line_list;

        // 逐行遍历环岛状态与环岛边线。
    for(y = (int16)RING_FILL_TOP_Y; y <= (int16)RING_FILL_BOT_Y; y++)
    {
        if(line[y] != 0)
        {
            ys[cnt] = y;
            xs[cnt] = (int16)line[y];
            cnt++;

            if(cnt >= (uint8)RING_LONG_FIT_PTS)
            {
                break;
            }
        }
    }

    if(cnt < 3)
    {
        return 0;                       /* 返回按固定小斜率延长环岛长边，直到周围不再有白色道路的判断或处理结果。 */
    }

    y0 = ys[0];              x0 = xs[0];             /* 更新按固定小斜率延长环岛长边，直到周围不再有白色道路相关变量。 */

        // 环岛入口短边所在的边线方向。
    slope = (ring_short_side == EDGE_LEFT) ? -(int16)RING_LONG_FIT_SLOPE_PX
                                           :  (int16)RING_LONG_FIT_SLOPE_PX;

    x_last = x0;

        // 循环遍历环岛状态与环岛边线，完成当前统计或筛选。
    for(k = 1; (int16)(y0 - k) >= (int16)RING_FILL_TOP_Y; k++)
    {
        if(k > (int16)RING_LONG_FIT_MAX_ROWS)
        {
            break;
        }

        y = (int16)(y0 - k);

                /*
         * 更新边线斜率与直线拟合相关变量。
         */
        x = (int16)(x0 + slope * (int16)k);

        if(x < 1 || x > (int16)(MT9V03X_W - 2))
        {
            break;                      /* 满足条件后退出当前循环。 */
        }

                /*
         * 更新按固定小斜率延长环岛长边，直到周围不再有白色道路相关变量。
         */
        white_env = 0;

        for(kk = 0; kk < (int16)RING_LONG_FIT_ENV; kk++)
        {
            xx = (int16)(x - (int16)(RING_LONG_FIT_ENV / 2) + kk);

            if(xx < 0 || xx > (int16)(MT9V03X_W - 1))
            {
                continue;
            }

            if(image[y][(uint8)xx] != EDGE_BLACK)
            {
                white_env++;
            }
        }

        if(white_env == 0)
        {
            break;
        }

                // 检查按固定小斜率延长环岛长边，直到周围不再有白色道路是否满足继续处理的条件。
        if(line[y] == 0)
        {
            line[y] = (uint8)x;
            n++;

            if(ring_fit_count < (uint16)RING_FIT_PTS_MAX)
            {
                ring_fit_x[ring_fit_count] = (uint8)x;
                ring_fit_y[ring_fit_count] = (uint8)y;
                ring_fit_count++;
            }
        }

        x_last = x;
    }

    return n;
}
/**
 * @brief 环岛中线覆盖核心。
 */
static void ring_apply_mid_inside(void)
{
    uint8  short_side;
    uint8  long_side;
    uint16 n_ipm;
    uint16 n_mid;
    uint8  long_fb = 0;                /* 定义longfb变量，用于当前图像处理过程。 */
    uint8  t_long_ok = 0  ;            /* 定义tlongok变量，用于当前图像处理过程。 */
    int16  y;
    uint8  mid_backup[MT9V03X_H];
    uint8  valid_backup[MT9V03X_H];

    ring_dbg_fb = 0;                   /* 更新环岛中线覆盖核心相关变量。 */
    ring_fit_count = 0;                /* 更新边线斜率与直线拟合相关变量。 */
    
    short_side = ring_short_side;      /* 环岛入口短边所在的边线方向。 */
    long_side  = (ring_short_side == EDGE_RIGHT) ? EDGE_LEFT : EDGE_RIGHT;

    {
        uint8 long_pct = 0;

        (void)ring_side_line_ok(long_side,
                                RING_SLOPE_SCAN_TOP, RING_SLOPE_SCAN_BOT,
                                &long_pct);

        {
            uint8 i;

            for(i = 7; i > 0; i--)               /* 循环遍历环岛中线覆盖核心，完成当前统计或筛选。 */
            {
                ring_long_s_hist[i] = ring_long_s_hist[i - 1];
            }
        }

        ring_long_s_hist[0] = long_pct;          /* 更新环岛状态与环岛边线对应的数组状态。 */
    }

        /*
     * 更新环岛中线覆盖核心相关变量。
     */
    t_long_ok = 1;

    if(ring_state == 2)
    {
        uint8 i;

        for(i = 0; i < (uint8)RING_T_LONG_S_FRAMES; i++)
        {
            if(ring_long_s_hist[i] >= (uint8)RING_T_LONG_S_PASS)
            {
                t_long_ok = 0;
                break;
            }
        }
    }

        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if((ring_state != 2) || (t_long_ok != 0))
    {
        if(ring_try_one_side_mid())
        {
            return;
        }
    }
        // 逐行遍历环岛中线覆盖核心。
    for(y = 0; y < MT9V03X_H; y++)
    {
        mid_backup[y]   = mid_line_list[y];
        valid_backup[y] = ipm_mid_line_valid[y];
    }

    memset(ipm_mid_line_valid, 0, sizeof(ipm_mid_line_valid));

        /*
     * 将原始真实爬线点补回行边线数组。
     */
    ring_restore_real_edge(short_side);

        /*
     * 从已知边线沿鸟瞰法向量移动半个车道宽度，生成中心候选。
     */
    n_ipm = ring_mid_from_side(short_side,
                               RING_FILL_TOP_Y,
                               RING_FILL_BOT_Y,
                               RING_INSIDE_WIDTH);

        // 参数 RING_MID_EXTRAP_ENABLE：环岛整段中线外推关闭。
    #define RING_MID_EXTRAP_ENABLE   0

        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if(n_ipm >= (uint16)RING_SHORT_MIN_ROWS)
    {
#if (RING_MID_EXTRAP_ENABLE)
    ring_extend_mid();
#else
    ring_fill_mid_gap();

#if (RING_SHORT_TOP_EXTRAP_ENABLE)
    (void)ring_extend_mid_curve();
#endif
#endif

n_ipm = ring_ipm_valid_rows();
    } /* 继续处理环岛中线覆盖核心。 */

        // 参数 RING_SHORT_SAVE_MIN_ROWS：环岛短边可保留的最少有效行，当前值为 5。
    #define RING_SHORT_SAVE_MIN_ROWS   5

    if(ring_ipm_valid_rows() >= (uint16)RING_SHORT_SAVE_MIN_ROWS)
    {
        for(y = 0; y < MT9V03X_H; y++)
        {
            ring_short_mid[y]   = mid_line_list[y];
            ring_short_valid[y] = ipm_mid_line_valid[y];
        }

        ring_short_ok = 1;
    }

    n_mid = ring_ipm_valid_rows();

    if(((ring_state == 2) &&
        (ring_side_pts(short_side) < (uint16)RING_T_LONG_FB_SHORT_PTS)) ||
       ((ring_state != 2) &&
        (n_mid < (uint16)RING_MID_FALLBACK_ROWS)))       /* 执行环岛状态与环岛边线相关操作。 */
    {
                // 检查环岛状态与环岛边线是否满足继续处理的条件。
        if((ring_state == 2) && (t_long_ok == 0))
        {
            if(ring_short_ok)
            {
                for(y = 0; y < MT9V03X_H; y++)
                {
                    mid_line_list[y]      = ring_short_mid[y];
                    ipm_mid_line_valid[y] = ring_short_valid[y];
                }
            }
                        /*
             * 更新环岛中线覆盖核心相关变量。
             */

            ring_dbg_fb = 2;             /* 更新环岛中线覆盖核心相关变量。 */
            long_fb     = 0;
        }
        else
        {
            ring_dbg_fb = 1;                     /* 更新环岛中线覆盖核心相关变量。 */
            long_fb     = 1;                     /* 更新环岛中线覆盖核心相关变量。 */

            ring_restore_real_edge(long_side);

            ring_mid_from_side(long_side,
                               RING_FILL_TOP_Y,
                               RING_FILL_BOT_Y,
                               RING_FALLBACK_LONG_WIDTH);     /* 执行环岛状态与环岛边线相关操作。 */

            n_mid = ring_ipm_valid_rows();
        }
    }
        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if((ring_state != 2) &&
       long_fb && (n_mid < (uint16)RING_LONG_MIN_MID_ROWS))
    {
        uint16 long_pts = ring_side_pts(long_side);

        if(cross_dbg_hit &&
           (long_pts > (uint16)RING_LONG_MIN_POINTS))
        {
            if(ring_long_extend_to_edge(long_side) > 0)
            {
                ring_mid_from_side(long_side,
                                   RING_FILL_TOP_Y,
                                   RING_FILL_BOT_Y,
                                   RING_FALLBACK_LONG_WIDTH);

                n_mid = ring_ipm_valid_rows();
            }
        }
    }
        /*
     * 统计环岛补线中心线的有效行数。
     */
    n_mid = ring_ipm_valid_rows();

    if(n_mid < (uint16)RING_MID_FALLBACK_ROWS)
    {
        ring_dbg_fb = 1;                   /* 更新环岛中线覆盖核心相关变量。 */

        ring_mark_normal_mid_valid();
        n_mid = ring_ipm_valid_rows();

        if(n_mid < (uint16)RING_MID_FALLBACK_ROWS)
        {
                        // 逐行遍历环岛中线覆盖核心。
            for(y = 0; y < MT9V03X_H; y++)
            {
                mid_line_list[y]      = mid_backup[y];
                ipm_mid_line_valid[y] = valid_backup[y];
            }

            return;
        }
    }

        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if(n_mid >= (uint16)RING_MID_FALLBACK_ROWS)
    {
        road_mode       = ROAD_MODE_IPM_FILL;
        search_end_line = RING_FILL_TOP_Y - 1;
    }

        // 逐行遍历环岛中线覆盖核心。
    for(y = 0; y < MT9V03X_H; y++)
    {
        ring_freeze_mid[y]   = mid_line_list[y];
        ring_freeze_valid[y] = ipm_mid_line_valid[y];
    }
    ring_freeze_ok = 1;
}
// 参数 RING_GYRO_TEMP_DISABLE：不临时禁用陀螺仪。
#define RING_GYRO_TEMP_DISABLE      0

/**
 * @brief 初始化 IMU660RB 偏航角。
 */
uint8 ring_gyro_init(void)
{
    ring_gyro_angle_deg = 0.0f;
    ring_gyro_last_yaw_deg = 0.0f;
    ring_gyro_integrating = 0;
    ring_gyro_ready = 0;
    ring_imu_type = RING_IMU_TYPE_NONE;

#if !RING_GYRO_TEMP_DISABLE
    if(imu660rb_yaw_init() == 0)
    {
        ring_imu_type = RING_IMU_TYPE_IMU660RB;
        ring_gyro_ready = 1;
    }
#endif

    return ring_gyro_ready;
}

/**
 * @brief 读取当前 yaw。
 */
static float ring_imu_yaw_read(void)
{
#if RING_GYRO_TEMP_DISABLE
    return 0.0f;
#else
    return imu660rb_yaw_get();
#endif
}

/**
 * @brief 进入环岛时记录 yaw 积分起点。
 */
static void ring_gyro_start(void)
{
    ring_gyro_angle_deg = 0.0f;

#if RING_GYRO_TEMP_DISABLE
    ring_gyro_last_yaw_deg = 0.0f;
    ring_gyro_integrating = 0;
#else
    ring_gyro_last_yaw_deg = ring_imu_yaw_read();
    ring_gyro_integrating = ring_gyro_ready;
#endif
}

/**
 * @brief 计算 yaw 差值，处理 ±180 度环绕并累计总转角。
 */
static void ring_gyro_update(void)
{
#if RING_GYRO_TEMP_DISABLE
    return;
#else
    float yaw_now;
    float yaw_delta;

    if(ring_gyro_integrating == 0)
    {
        return;
    }

    yaw_now = ring_imu_yaw_read();
    yaw_delta = yaw_now - ring_gyro_last_yaw_deg;

    while(yaw_delta > 180.0f)
    {
        yaw_delta -= 360.0f;
    }

    while(yaw_delta < -180.0f)
    {
        yaw_delta += 360.0f;
    }

    ring_gyro_angle_deg += yaw_delta;
    ring_gyro_last_yaw_deg = yaw_now;
#endif
}

int16 ring_apply_after_exit_error_floor(int16 error)
{
    int16 force_error_min;

    if(ring_after_exit_force_error_frames == 0)
    {
        return error;
    }

    force_error_min = ring_is_small
                    ? (int16)RING_SMALL_AFTER_EXIT_FORCE_ERROR_MIN
                    : (int16)RING_BIG_AFTER_EXIT_FORCE_ERROR_MIN;

    if(error < force_error_min)
    {
        error = force_error_min;
    }

    ring_after_exit_force_error_frames--;

    return error;
}

/**
 * @brief 清空环岛状态，但保留 ring_done_cnt。
 */
static void ring_reset(void)
{    ring_exit_cnt    = 0;      /* 更新清空环岛状态，但保留 ring_done_cnt相关变量。 */
    ring_state      = 0;
    ring_dir        = 0;
    ring_frames     = 0;
    ring_confirm    = 0;
    ring_started    = 0;
    ring_short_side = 0;
    ring_relax_trace = 0;
    ring_cross_latch = 0;
    ring_cross_cnt   = 0;      /* 更新清空环岛状态，但保留 ring_done_cnt相关变量。 */
    gocross          = 0;      /* 更新清空环岛状态，但保留 ring_done_cnt相关变量。 */
    ring_fit_count = 0;                /* 更新边线斜率与直线拟合相关变量。 */
        /*
     * 执行清空环岛状态，但保留 ring_done_cnt相关操作。
     */
    memset(ring_freeze_valid, 0, sizeof(ring_freeze_valid));
    ring_freeze_ok      = 0;

    memset(ring_short_valid, 0, sizeof(ring_short_valid));   /* 执行环岛状态与环岛边线相关操作。 */
    ring_short_ok       = 0;

    memset(ring_long_s_hist, 0, sizeof(ring_long_s_hist));   /* 执行环岛状态与环岛边线相关操作。 */
    ring_gyro_angle_deg = 0.0f;
    ring_gyro_integrating = 0;
}
/**
 * @brief 检测环岛入口并判断短边在左还是右。
 */
static uint8 ring_detect(uint8 *dir)
{
    uint8  top_l;
    uint8  top_r;
    uint16 rows_l;
    uint16 rows_r;
    uint8  good_left_ok;
    uint8  good_right_ok;

    if(zebra_flag ||
       zebra_ring_hold_frames ||
       cross_lock_active)
    {
        return 0;
    }

        /*
     * 遍历原始爬线点，找该侧最靠上的真实点。
     */
    top_l = ring_real_top(EDGE_LEFT);
    top_r = ring_real_top(EDGE_RIGHT);

    rows_l = ring_real_rows(EDGE_LEFT,  RING_SCAN_Y_TOP, RING_SCAN_Y_BOT);
    rows_r = ring_real_rows(EDGE_RIGHT, RING_SCAN_Y_TOP, RING_SCAN_Y_BOT);

        /*
     * 更新检测环岛入口并判断短边在左还是右相关变量。
     */
    ring_dbg_top_l  = top_l;
    ring_dbg_top_r  = top_r;
    ring_dbg_rows_l = (uint8)rows_l;
    ring_dbg_rows_r = (uint8)rows_r;
    ring_dbg_cnt_l  = Left_Edge_Count;
    ring_dbg_cnt_r  = Right_Edge_Count;

        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if(ring_real_rows(EDGE_LEFT,  RING_NEAR_SCAN_TOP, RING_NEAR_SCAN_BOT) < RING_NEAR_MIN_ROWS)
    {
        return 0;
    }

    if(ring_real_rows(EDGE_RIGHT, RING_NEAR_SCAN_TOP, RING_NEAR_SCAN_BOT) < RING_NEAR_MIN_ROWS)
    {
        return 0;
    }

        /*
     * 检查一条边线是否近似为直线。
     */
    good_left_ok  = ring_side_line_ok(EDGE_LEFT,  RING_SLOPE_SCAN_TOP, RING_SLOPE_SCAN_BOT,
                                      &ring_dbg_straight);
    good_right_ok = ring_side_line_ok(EDGE_RIGHT, RING_SLOPE_SCAN_TOP, RING_SLOPE_SCAN_BOT,
                                      &ring_dbg_straight_r);

        // 环岛入口短边所在的边线方向。
    ring_dbg_short_s = (ring_short_side == EDGE_LEFT) ? ring_dbg_straight
                                                      : ring_dbg_straight_r;

        // 检查检测环岛入口并判断短边在左还是右是否满足继续处理的条件。
    if(top_l > top_r)
    {
        ring_dbg_straight = ring_dbg_straight_r;    /* 更新检测环岛入口并判断短边在左还是右相关变量。 */
    }

        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if(top_l < RING_TOP_Y && good_left_ok &&
       top_r > RING_BREAK_TOP_Y)
    {
        *dir = 1;
        return 1;
    }

        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if(top_r < RING_TOP_Y && good_right_ok &&
       top_l > RING_BREAK_TOP_Y)
    {
        *dir = 2;
        return 1;
    }

    return 0;
}

/**
 * @brief 环岛 0/1/2/3 状态机总控。
 */
void ring_process(void)
{
    uint8 short_side;                   /* 定义shortside变量，用于当前图像处理过程。 */
    uint8 dir = 0;                      /* 定义dir变量，用于当前图像处理过程。 */
    uint8 detected;                     /* 定义detected变量，用于当前图像处理过程。 */
    uint16 n_mid;                       /* 定义nmid变量，用于当前图像处理过程。 */
    uint16 n_ipm;                       /* 定义nipm变量，用于当前图像处理过程。 */

#if !RING_ENABLE
    return;
#endif

    ring_gyro_update();

        // 检查斑马线特征是否满足继续处理的条件。
    if(zebra_flag != zebra_flag_last)
    {
        zebra_flag_last = zebra_flag;

        if(zebra_flag == 0U)
        {
            zebra_ring_hold_frames = ZEBRA_RING_HOLD_FRAMES;
        }
        else
        {
            zebra_ring_hold_frames = 0U;
        }
    }
    else if((zebra_flag == 0U) &&
            (zebra_ring_hold_frames > 0U))
    {
        zebra_ring_hold_frames--;
    }

        // 检查环岛 0/1/2/3 状态机总控是否满足继续处理的条件。
    if(ring_after_exit_force_error_frames > 0)
    {
        if(ring_cooldown > 0)
        {
            ring_cooldown--;
        }

        ring_dbg_cd = (uint8)(((uint32)ring_cooldown * RING_FRAME_MS) / 100U);
        ring_dbg_t2 = 0;

        return;
    }

        /*
     * 检测环岛入口并判断短边在左还是右。
     */
    detected = ring_detect(&dir);

        // 检查环岛状态与环岛边线是否满足继续处理的条件。
    if(ring_state == 3)
    {
                // 以下代码继续处理环岛状态与环岛边线。
    }
        /* ===== 环岛识别与补线 ===== */
    if(ring_state == 0)
    {
                // 检查环岛 0/1/2/3 状态机总控是否满足继续处理的条件。
        if(ring_cooldown > 0)
        {
            ring_cooldown--;
            ring_confirm = 0;

            ring_dbg_cd = (uint8)(((uint32)ring_cooldown * RING_FRAME_MS) / 100U);
            ring_dbg_t2 = 0;

            return;
        }

        ring_dbg_cd = 0;
        ring_dbg_t2 = 0;

        if(detected)
        {
            if(ring_confirm < 255)
            {
                ring_confirm++;
            }
        }
        else
        {
            ring_confirm = 0;
        }

        if(ring_confirm >= RING_CONFIRM_FRAMES)
        {
                        /*
             * 更新环岛 0/1/2/3 状态机总控相关变量。
             */
            ring_is_small = (uint8)((ring_done_cnt & 1U) != 0U);
            ring_gyro_start();
            ring_dir        = dir;
            ring_short_side = (dir == 1) ? EDGE_RIGHT : EDGE_LEFT;   /* 环岛入口短边所在的边线方向。 */
            ring_state      = 1;
            ring_frames     = 0;
            ring_confirm    = 0;
            ring_started    = 0;                   /* 更新环岛 0/1/2/3 状态机总控相关变量。 */                  /* 更新环岛 0/1/2/3 状态机总控相关变量。 */
        }
        else
        {
            return;                     /* 返回环岛 0/1/2/3 状态机总控的判断或处理结果。 */
        }
    }

        /* ===== 环岛识别与补线 ===== */
#if RING_DEBUG_ONLY
    ring_state   = 0;
    ring_frames  = 0;
    ring_confirm = 0;
    return;
#endif
        /* ===== 环岛识别与补线 ===== */
    if(ring_state == 1)
    {
        
        short_side = ring_short_side;           /* 环岛入口短边所在的边线方向。 */

        ring_frames++;

                // 检查环岛状态与环岛边线是否满足继续处理的条件。
        if(ring_frames >= (uint16)RING_S1_TIMEOUT_FRAMES)
        {
            ring_reset();                                   /* 清空环岛状态，但保留 ring_done_cnt。 */
            ring_cooldown = (uint16)RING_COOLDOWN_FRAMES;   /* 更新环岛状态与环岛边线相关变量。 */
            ring_dbg_t2   = 0;
            ring_dbg_cd   = (uint8)(((uint32)ring_cooldown * RING_FRAME_MS) / 100U);

            return;
        }

                // 检查环岛 0/1/2/3 状态机总控是否满足继续处理的条件。
        if(ring_started == 0)
        {
            if(ring_real_top(short_side) < (uint8)RING_S1_TOP_A)
            {
                                /*
                 * 更新环岛状态与环岛边线相关变量。
                 */
                ring_confirm = (uint8)RING_CONFIRM_FRAMES;
            }
            else
            {
                ring_confirm = 0;
            }

            if(ring_confirm >= RING_CONFIRM_FRAMES)
            {
                ring_started = 1;
                ring_confirm = 0;
            }

            ring_dbg_t2 = 0;
            ring_state1_fill_guard();
            return;
        }

                // 检查环岛状态与环岛边线是否满足继续处理的条件。
        if(ring_real_top(short_side) > (uint8)RING_S1_TOP_B)
        {
            if(ring_confirm < 255)
            {
                ring_confirm++;
            }
        }
        else
        {
            ring_confirm = 0;
        }

        if(ring_confirm >= RING_S1_STAGE2_CONFIRM_FRAMES)
        {
            ring_state       = 2;               /* 环岛状态机的当前状态。 */
            ring_frames      = 0;
            ring_confirm     = 0;
            ring_relax_trace = 1;               /* 环岛场景下是否允许放宽边线爬取限制。 */
            gocross          = 1;               /* 更新环岛 0/1/2/3 状态机总控相关变量。 */
        }

        ring_dbg_t2 = 0;
        ring_state1_fill_guard();               /* 环岛状态 1 的单边补线包装函数。 */
        return;                                 /* 返回环岛 0/1/2/3 状态机总控的判断或处理结果。 */
                                                                        // 以下代码继续处理环岛状态与环岛边线。
    }

        /* ===== 环岛识别与补线 ===== */
    if(ring_state == 2)
    {
        ring_relax_trace = 1;                   /* 环岛场景下是否允许放宽边线爬取限制。 */

        ring_frames++;

                // 检查环岛状态与环岛边线是否满足继续处理的条件。
        if(ring_frames < (uint16)RING_INSIDE_FRAMES)
        {
            ring_dbg_t2 = (uint8)(((uint32)(RING_INSIDE_FRAMES - ring_frames) *
                                   RING_FRAME_MS) / 100U);
        }
        else
        {
            ring_dbg_t2 = 0;
        }

                /*
         * 环岛中线覆盖核心。
         */
        ring_apply_mid_inside();
        ring_mid_cleanup();                 /* 环岛中线清洗：小间隙填补、中值滤波、5 行加权平滑。 */

                // 检查环岛状态与环岛边线是否满足继续处理的条件。
        if(ring_frames >= (uint16)RING_INSIDE_FRAMES)
        {
            ring_dbg_t2      = 0;
            ring_state       = 3;
            ring_frames      = 0;
            ring_relax_trace = 1;               /* 环岛场景下是否允许放宽边线爬取限制。 */
        }

        return;
    }

        /* ===== 环岛识别与补线 ===== */
    if(ring_state == 3)
    {
        ring_relax_trace = 1;           /* 环岛场景下是否允许放宽边线爬取限制。 */

        if((ring_gyro_angle_deg >
            (ring_is_small ? RING_SMALL_EXIT_GYRO_ANGLE_DEG
                           : RING_BIG_EXIT_GYRO_ANGLE_DEG)) ||
           (ring_gyro_angle_deg <
            -(ring_is_small ? RING_SMALL_EXIT_GYRO_ANGLE_DEG
                            : RING_BIG_EXIT_GYRO_ANGLE_DEG)))
        {
            if(ring_done_cnt < 255)
            {
                ring_done_cnt++;
            }

            ring_after_exit_force_error_frames =
                (uint16)RING_AFTER_EXIT_FORCE_ERROR_FRAMES;

            ring_reset();                                   /* 清空环岛状态，但保留 ring_done_cnt。 */
            ring_cooldown = (uint16)RING_COOLDOWN_FRAMES;   /* 更新环岛状态与环岛边线相关变量。 */
            ring_dbg_t2   = 0;
            ring_dbg_cd   = (uint8)(((uint32)ring_cooldown * RING_FRAME_MS) / 100U);

            return;
        }

                /*
         * 环岛中线覆盖核心。
         */
        ring_apply_mid_inside();
        ring_mid_cleanup();                 /* 环岛中线清洗：小间隙填补、中值滤波、5 行加权平滑。 */

        ring_dbg_t2 = 0;

        return;
    }
}
