/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu1_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "image.h"    // 引入你的图像处理算法头文件
#include "display.h"
#include "Motor.h"  // 引入你的屏幕绘图头文件

uint8 ring_gyro_init(void);
int16 ring_apply_after_exit_error_floor(int16 error);

/*
 * 连续多少帧没有足够有效前瞻行后，判定为丢线
 */
#define LINE_LOST_CONFIRM_FRAMES  5

/*
 * 至少需要3行有效数据，才认为前瞻有效
 */
#define LOOKAHEAD_MIN_VALID_ROWS  1

/*
 * 斑马线需要连续检测2帧才置1
 */
#define ZEBRA_ON_CONFIRM_FRAMES   2

/*
 * 连续4帧没检测到才清零，防止标志抖动
 */
#define ZEBRA_OFF_CONFIRM_FRAMES  4
#define PC_IMAGE_ENABLE     0
#define PC_IMAGE_INTERVAL   5
#define IPM_PREVIEW_ENABLE  0
#define ENABLE_BLACK_HOLE_FILL  1
static uint8 pc_image_count = 0;

#define RING_PID_ERROR_MIN    27
/* Entry force error is independently tunable for big and small rings. */
#define RING_BIG_ENTRY_FORCE_ERROR    31
#define RING_SMALL_ENTRY_FORCE_ERROR  34

extern uint8 ring_is_small;

/* 【开关】输出方式二选一：
 *   1 = 只显示 TFT 屏幕（灰度图 + 三条线 + 环岛调试数字），不跑 WiFi 图传
 *   0 = 只跑 WiFi 图传（逐飞助手里看图和线），屏幕不刷图也不显示调试数字
 */

#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// **************************** 代码区域 ****************************

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等
    //gpio_init(P15_4, GPO, GPIO_HIGH, GPO_PUSH_PULL);
        //gpio_set_level(P15_4, GPIO_HIGH);  // 或者直接写 1
//        wireless_uart_init();
        imu_ready = ring_gyro_init();
//        tft180_set_dir(TFT180_PORTAIT_180);
//        tft180_init();

        mt9v03x_init();
        cpu_wait_event_ready();         // 等待所有核心初始化完毕
        while (TRUE)
           {



            if (mt9v03x_finish_flag == 1)
                       {
                           mt9v03x_finish_flag =0;


                           memcpy(base_image[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
               #if IPM_PREVIEW_ENABLE

                   ipm_warp_gray(base_image, ipm_preview);

               #endif
               #if PC_IMAGE_ENABLE

                   pc_image_count++;

                   if (pc_image_count >= PC_IMAGE_INTERVAL)
                   {
                       pc_image_count = 0;


                       sendimg_zoom(base_image[0],
                                    MT9V03X_W,
                                    MT9V03X_H,
                                    MT9V03X_W ,
                                    MT9V03X_H );
                   }

               #endif

                           img_threshold = Ostu(base_image);      //        ֵ
                           set_image_twovalues(img_threshold);    //
               #if ENABLE_BLACK_HOLE_FILL

                   fill_small_black_holes(image);
               #endif
                   /*
                    * 当前帧斑马线候选结果
                    */
                   zebra_now =
                       detect_zebra_crossing(image);
                   if(zebra_now)
                   {
                       zebra_off_frames = 0;

                       if(zebra_on_frames < 255)
                       {
                           zebra_on_frames++;
                       }

                       if((zebra_on_frames >= ZEBRA_ON_CONFIRM_FRAMES) &&
                          (zebra_flag == 0U))
                       {
                           if(zebra_count < 255U)
                           {
                               zebra_count++;
                           }
                           zebra_flag = 1;
                       }
                   }
                   else
                   {
                       zebra_on_frames = 0;

                       if(zebra_off_frames < 255)
                       {
                           zebra_off_frames++;
                       }

                       if(zebra_off_frames >= ZEBRA_OFF_CONFIRM_FRAMES)
                       {
                           zebra_flag = 0;
                       }
                   }
                   image_deal(image);

                   ring_process();                     /* 【新增】环岛识别与处理 */

                   final_mid_line = find_mid_line_weight();

                           pid_error = find_mid_line_lookahead_error();
                           pid_error = ring_apply_after_exit_error_floor(pid_error);

                           if(ring_state == 2U)
                           {
                               if(pid_error <= RING_PID_ERROR_MIN)
                               {
                                    pid_error = ring_is_small
                                              ? RING_SMALL_ENTRY_FORCE_ERROR
                                              : RING_BIG_ENTRY_FORCE_ERROR;
                               }
                           }
                           /*
                            * 丢线判断
                            */
                           if(lookahead_valid_count >= LOOKAHEAD_MIN_VALID_ROWS)
                           {
                               line_lost_frames = 0;
                               line_found_flag = 1;
                           }
                           else
                           {
                               if(line_lost_frames < 255)
                               {
                                   line_lost_frames++;
                               }

                               if(line_lost_frames >= LINE_LOST_CONFIRM_FRAMES)
                               {
                                   line_found_flag = 0;
                               }
                           }

//                           tft180_show_gray_image(0,0,image[0],MT9V03X_W, MT9V03X_H,128, 81,0);
//                           draw_line();
//                           tft180_show_string(5, 100, "M:");
//                           tft180_show_int(
//                               28,
//                               100,
//                               (int32)road_mode,
//                               1);
//
//                           tft180_show_string(55, 100, "L:");
//                           tft180_show_int(
//                               72,
//                               100,
//                               (int32)Left_Edge_Count,
//                               3);
//
//                           tft180_show_string(5, 120, "R:");
//                           tft180_show_int(
//                               20,
//                               120,
//                               (int32)Right_Edge_Count,
//                               3);
//                           tft180_show_string(5, 140, "C:");
//                           tft180_show_int(
//                               28,
//                               140,
//                               (int32)cross_lock_active,
//                               1);
//
//                           tft180_show_string(55, 140, "T:");
//                           tft180_show_int(
//                               72,
//                               140,
//                               (int32)cross_hold_frames,
//                               2);
//                           /* 环岛调试：LT/RT = 左右边线最上面有效行；LK/RK = 20~70 段有效行数 */
//                           /*
//                            * 环岛调试：
//                            *   LT/RT = 左右边线"最上面有效行"（越小=爬得越高；127=这一侧完全没有线）
//                            *   LK/RK = 左右边线在 y=20~70 里的有效行数（越大=越完整）
//                            *   RS    = 环岛状态（第一步应为 0）
//                            */
//                           /*
//                            * 环岛调试（TFT180 默认 8x16 字体：每字符 8 像素宽、16 像素高）
//                            *   LT/RT = 左右边线最上面有效行（越小=爬得越高；127=这一侧完全没线）
//                            *   LK/RK = 左右边线在 y=20~70 的有效行数
//                            *   RS    = 环岛状态（第一步应为 0）
//                            *   D     = 环岛方向（1=右环 2=左环）
//                            *   S     = 好的一侧在 20~70 的 x 总变化量（用来定 RING_STRAIGHT_TOL）
//                            */
//                           /*
//                            * 环岛调试（TFT180 默认 8x16 字体：每字符 8 像素宽）
//                            *   L/R   = 左右【真实爬线】爬到的最远行（越小=爬得越远；127=这一侧没爬到）
//                            *   CL/CR = 左右【真实爬线】的点数（就是做中线的有效点数）
//                            *   LK/RK = 左右真实点落在 y=20~70 的行数（断线那一侧会很小）
//                            *   M/D   = 环岛状态 / 方向
//                            */
//                           tft180_show_string(2, 100, "L:");
//                           tft180_show_int(20, 100, (int32)ring_dbg_top_l, 3);
//                           tft180_show_string(56, 100, "R:");
//                           tft180_show_int(72, 100, (int32)ring_dbg_top_r, 3);
//                           /* 【新增】X = 这一帧图像里是否检测到十字(1=检测到) */
//                           tft180_show_string(104, 100, "X:");
//                           tft180_show_int(112, 100, (int32)cross_dbg_hit, 1);
//                           /* 【改动】CD = 环岛结束后"禁止检测"倒计时(0.1 秒，0~50)
//                            *   ST = 当前状态号（0=正常巡线 1=已锁定在等短边 2=拎边线） */
//                           /* 【新增】C = 十字锁定(1=正在按十字处理)；环岛流程里必须一直是 0 */
//                           /* 【改动】F = 这一帧短边补线有没有走保底：1=用了正常逻辑的中线 */
//                           tft180_show_string(104, 100, "RM");
//                           tft180_show_int(112, 100, (int32)road_mode, 1);
//                           tft180_show_string(2, 120, "CD:");
//                           tft180_show_int(26, 120, (int32)ring_dbg_cd, 3);
//                           tft180_show_string(62, 120, "ST:");
//                           tft180_show_int(86, 120, (int32)ring_state, 1);
//                           tft180_show_string(2, 140, "LK:");
//                           tft180_show_int(28, 140, (int32)ring_dbg_rows_l, 2);
//                           tft180_show_string(48, 140, "RK:");
//                           tft180_show_int(72, 140, (int32)ring_dbg_rows_r, 2);
//                           tft180_show_string(96, 140, "M:");
//                           tft180_show_int(112, 140, (int32)ring_dir, 1);
//                           /* 【改动】SL / SR = 左边 / 右边在 20~70 段的“直线分数”(%)；
//                            *   分母就是屏幕上那一侧的 LK / RK，可以直接对照 */
//                           /* 【改动】S = 爬得高那一侧的直线分数(%)；T = 状态2 剩余时间(0.1 秒，0~30)
//                            *   坐标说明：show_int 占 8*(位数+1) 像素，最后一个字符起始 x 不能超过 120 */
//                           tft180_show_string(2, 84, "S:");
//                           tft180_show_int(20, 84, (int32)ring_dbg_straight, 3);
//                           /* 【改动】T = 状态2 剩余时间(0.1 秒，100~0)，所以要 3 位 */
//                           tft180_show_string(56, 84, "T:");
//                           tft180_show_int(72, 84, (int32)ring_dbg_t2, 3);
//                           /* 【新增】E = 出环条件位掩码：1=长边<15  2=短边>30  4=没十字；E=7 就出环 */
//                           tft180_show_string(102, 120, "E:");
//                           tft180_show_int(110, 120, (int32)ring_dbg_exit, 1);
//                           /*
//                            * 第一行：线路和斑马线标志
//                            */
//                           tft180_show_string(5, 100, "LF:");
//                           tft180_show_int(
//                               28,
//                               100,
//                               (int32)line_found_flag,
//                               1);
//
//                           tft180_show_string(55, 100, "Z:");
//                           tft180_show_int(
//                               72,
//                               100,
//                               (int32)zebra_flag,
//                               1);
//
//                           /*
//                            * 第二行：有效前瞻行数和PID误差
//                            */
//                           tft180_show_string(5, 120, "V:");

//                           tft180_show_int(
//                               20,
//                               120,
//                               (int32)lookahead_valid_count,
//                               2);
//
//                           tft180_show_string(72, 120, "E:");
//                           tft180_show_int(
//                               72,
//                               100,
//                               (int32)pid_error,
//                               4);

                       }
           }
}
#pragma section all restore
// **************************** 代码区域 ****************************
