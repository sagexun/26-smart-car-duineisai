#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"

// ==================== 搜线参数宏定义 ====================
#define jidian_search_line 118          // 基点搜索行 (避开最底部119行边缘)
#define search_start_line  118          // 搜线起始行
#define MID_W              93           // 赛道中值
/*
 * 前瞻窗口参数。
 *
 * 图像坐标中 y 越大越靠近车辆，
 * y 越小越远离车辆。
 *
 * 当前窗口为 y=75 到 y=95，
 * 共使用 21 行。
 */
#define LOOKAHEAD_NEAR_Y   95
#define LOOKAHEAD_FAR_Y    75
/* 【新增·基础前瞻】照霹雳火2025：固定前瞻行 + 相邻 2 行共 3 行求平均
 *   PROSPECT_LINE     : 前瞻行（图像行号，越小越远、越大越靠近车）
 *   PROSPECT_ROW_STEP : 相邻两行的间隔（2025 用的是 26/28/30 → 间隔 2）
 *   元素（环岛/十字/坡道）里的动态前瞻先不做，留给队友在这个基础上加。 */
/* 普通道路：50、52、54 */
#define PROSPECT_LINE_NORMAL       40                    //原始44

/* 环岛状态1，准备入环：56、58、60，让小车晚一点转 */
#define PROSPECT_LINE_RING_S1      40                   //保持不动
//看得近一点

/* 环岛状态2，已经入环：50、52、54 */
#define PROSPECT_LINE_RING_S2      28                   //保持不动
//恢复普通路

/* 环岛状态3，准备出环：54、56、58，减小出环抖动 */
#define PROSPECT_LINE_RING_S3      40                   //往近的调 原始70
//出环岛有个渐变的效果，防止抖动

/* 每行间隔 */
#define PROSPECT_ROW_STEP           2

/* 总共取3行 */
#define PROSPECT_ROW_COUNT          3

/* 前瞻切换时，每帧最多移动2行 */
#define PROSPECT_MOVE_STEP          2
// ==================== 逆透视与相量法专属结构体 ====================
typedef struct {
    uint8 x;
    uint8 y;
} Point2D_u8;

#define MAX_EDGE_PTS 180               // 单边最大爬边点数
extern uint8 edge_start_y;
// ==================== 全局变量声明 (extern) ====================
extern uint8 base_image[MT9V03X_H][MT9V03X_W];
extern uint8 image[MT9V03X_H][MT9V03X_W];
extern uint8 img_threshold;
extern uint8 left_jidian;
extern uint8 right_jidian;

// 原有的行边界数组（完全兼容你的display.c和find_mid_line_weight）
extern uint8 left_line_list[MT9V03X_H];
extern uint8 right_line_list[MT9V03X_H];
extern uint8 mid_line_list[MT9V03X_H];
extern uint8 final_mid_line;
extern uint8 last_mid_line;
extern uint8 ipm_mid_line_valid[MT9V03X_H];
// 动态截止行变量（八邻域根据前方赛道自动给出）
extern uint8 search_end_line;

// 专门为后面逆透视与相量法准备的有序边缘点集
extern Point2D_u8 Left_Edge_Pts[MAX_EDGE_PTS];
extern uint16    Left_Edge_Count;
extern Point2D_u8 Right_Edge_Pts[MAX_EDGE_PTS];
extern uint16    Right_Edge_Count;

// ==================== 图像处理函数声明 ====================
uint8 Ostu(uint8 index[MT9V03X_H][MT9V03X_W]);
void set_image_twovalues(uint8 value);
void find_jidian(uint8 index[MT9V03X_H][MT9V03X_W]);
void image_deal(uint8 index[MT9V03X_H][MT9V03X_W]);
uint8 Limit_uint8(uint8 a, uint8 b, uint8 c);
uint8 find_mid_line_weight(void);
/*
 * 获取前瞻区域相对于图像中心的有符号偏差。
 */
int16 find_mid_line_lookahead_error(void);
int16 find_mid_line_lookahead_error(void);
int16 error_output_filter(int16 raw);      /* 【本次需求】error 输出滤波 */
/*
 * 当前前瞻区域内有效的行数
 */
extern uint16 lookahead_valid_count;

/*
 * 线路状态：
 * 1：找到线路
 * 0：连续丢线
 */
extern uint8 line_found_flag;

/*
 * 斑马线状态：
 * 1：检测到斑马线
 * 0：没有检测到斑马线
 */
extern uint8 zebra_flag;
#define ROAD_MODE_NORMAL       0
#define ROAD_MODE_SINGLE_SEED  1
#define ROAD_MODE_IPM_TRY      2
#define ROAD_MODE_IPM_FILL     3
#define ROAD_MODE_CROSS        4
#define ROAD_MODE_TURN_FILL    5
#define ROAD_MODE_IPM_FAIL     6
#define ROAD_MODE_LOST         7
extern float ipm_segment_width[4];
extern uint16 ipm_segment_valid_rows[4];

void ipm_measure_lane_width_four_segments(
    uint8 *left_line,
    uint8 *right_line,
    uint8 top_y,
    uint8 bottom_y);
extern uint8 road_mode;
extern uint8 cross_lock_active;
extern uint8 cross_hold_frames;
/*
 * 斑马线检测函数
 */
uint8 detect_zebra_crossing(
    uint8 bin[MT9V03X_H][MT9V03X_W]);
/*
 * 斑马线检测区域
 *
 * 斑马线与赛道方向平行，
 * 因此应该在每一行沿X方向寻找多个黑色条带。
 */
#define ZEBRA_SCAN_Y_START          45
#define ZEBRA_SCAN_Y_END            105

#define ZEBRA_SCAN_X_START          18
#define ZEBRA_SCAN_X_END            170

/*
 * 单个黑带最小横向宽度
 */
#define ZEBRA_BLACK_RUN_MIN         3

/*
 * 两个黑带之间至少需要的白色间隔
 */
#define ZEBRA_WHITE_GAP_MIN         2

/*
 * 一行中至少要有这么多条黑带
 */
#define ZEBRA_MIN_BANDS_PER_ROW     4

/*
 * 至少有这么多行满足条件
 */
#define ZEBRA_MIN_GOOD_ROWS         8
extern uint8  ring_cross_latch;
extern uint8  ring_cross_cnt;
extern uint8  ring_dbg_fb;

/*
 * 逆透视预览图。
 *
 * 该数组只保存逆透视后的灰度图，
 * 不参与原来的二值化和寻线逻辑。
 */
extern uint8  ring_dbg_exit;       /* 出环条件位掩码（屏幕 E） */
extern uint8 ipm_preview[MT9V03X_H][MT9V03X_W];
extern uint8  ring_short_side;   /* 【新增】断边(短边)：0=EDGE_LEFT 1=EDGE_RIGHT */
/*
 * 使用 Mat1 生成逆透视灰度图。
 * src：原始灰度图，坐标形式为 src[y][x]
 * dst：逆透视灰度图，坐标形式为 dst[y][x]
 */
void ipm_warp_gray(
    uint8 src[MT9V03X_H][MT9V03X_W],
    uint8 dst[MT9V03X_H][MT9V03X_W]);
void fill_small_black_holes(
    uint8 bin[MT9V03X_H][MT9V03X_W]);
/* ==================== 环岛模块 ==================== */
extern uint8 ring_dbg_straight;
extern uint8 ring_dbg_straight_r;   /* 右侧直线分数(%) */
extern uint8 ring_dbg_short_s;      /* 【显示用】短边一侧的直线分数(%) → 屏幕 SS */
/* 【显示用】长边延长段的点：只在屏幕上画青色线，不参与任何判断 */
#define RING_FIT_PTS_MAX 120
extern uint8  ring_fit_x[RING_FIT_PTS_MAX];
extern uint8  ring_fit_y[RING_FIT_PTS_MAX];
extern uint16 ring_fit_count;
extern volatile uint8 ring_state;
extern volatile uint8 ring_dir;
extern uint8  ring_dbg_island;
extern uint8  ring_dbg_lok;
extern uint8  ring_dbg_rok;
extern uint8  ring_done_cnt;

/* 【新增】真实爬线点集的统计（补线不会影响这些值） */
extern uint16 ring_dbg_cnt_l  ;    /* 左边真实点数（=Left_Edge_Count） */
extern uint16 ring_dbg_cnt_r  ;    /* 右边真实点数（=Right_Edge_Count） */
extern uint8  ring_dbg_rows_l ;    /* 左边真实点落在 y=20~70 的行数 */
extern uint8  ring_dbg_rows_r;    /* 右边真实点落在 y=20~70 的行数 */
/* 好的一侧的 x 总变化量（用来标 RING_STRAIGHT_TOL） */
extern uint8  ring_dbg_straight ;
void ring_process(void);
extern uint8  ring_dbg_top_l;
extern uint8  ring_dbg_top_r;
extern uint8  ring_confirm;

/* 【新增】环岛状态机用 */
extern uint16 ring_cooldown;      /* 环岛结束后禁止检测的剩余帧数 */
extern uint8  ring_dbg_t2;        /* 状态2 剩余时间(0.1 秒，0~30) */
extern uint8  ring_dbg_cd;        /* 冷却剩余时间(0.1 秒，0~50) */
extern uint8 cross_dbg_hit;
extern uint8 gocross;             /* 【新增】1 = 正在"短边补线拐进环岛"（状态2 起）→ 出环清 0 */
#endif /* CODE_IMAGE_H_ */
