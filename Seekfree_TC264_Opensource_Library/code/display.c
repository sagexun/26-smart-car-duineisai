/*
 * display.c
 *
 *  Created on: 2026年9月6日
 *      Author: gry
 */

// ==========================================================
// 函数功能：限制大小范围，防止数组越界导致单片机死机
// 输入参数：a:下限, b:需要判断的值, c:上限
// 返回值  ：安全合法的数值
// ==========================================================

#include "zf_common_headfile.h"
#include "image.h"

static void draw_real_edge_point(
    uint16 x,
    uint16 y,
    uint16 color)
{
    tft180_draw_point(x, y, color);

    if(x > 0)
    {
        tft180_draw_point(x - 1, y, color);
    }

    if(x < 127)
    {
        tft180_draw_point(x + 1, y, color);
    }

    if(y > 0)
    {
        tft180_draw_point(x, y - 1, color);
    }

    if(y < 80)
    {
        tft180_draw_point(x, y + 1, color);
    }
}
// ==========================================================
// 函数功能：边线和中线显示
// 备    注：调用此函数后在屏幕上显示左右边线以及中线
// ==========================================================
void draw_line(void)
{
    uint16 i;
    int16 y;

    /*
     * 绘制左边界点。
     */
    for(i = 0; i < Left_Edge_Count; i++)
    {
        uint16 x;
        uint16 py;

        x =
            (uint32)Left_Edge_Pts[i].x * 127U /
            (MT9V03X_W - 1U);

        py =
            (uint32)Left_Edge_Pts[i].y * 80U /
            (MT9V03X_H - 1U);

        draw_real_edge_point(
            x,
            py,
            RGB565_BLUE);
    }

    /*
     * 绘制右边界点。
     */
    for(i = 0; i < Right_Edge_Count; i++)
    {
        uint16 x;
        uint16 py;

        x =
            (uint32)Right_Edge_Pts[i].x * 127U /
            (MT9V03X_W - 1U);

        py =
            (uint32)Right_Edge_Pts[i].y * 80U /
            (MT9V03X_H - 1U);

        draw_real_edge_point(
            x,
            py,
            RGB565_GREEN);
    }

    /*
     * 绘制中线。
     */
    {
        uint8 previous_valid = 0;
        uint16 previous_x = 0;
        uint16 previous_y = 0;
        int16 previous_source_y = -1;

        for(y = MT9V03X_H - 1;
            y >= 0;
            y--)
        {
            uint16 middle_x;
            uint16 display_x;
            uint16 display_y;

            /*
             * search_end_line 以上的行不属于本次有效巡线区域。
             *
             * 如果继续绘制，这些行的 mid_line_list
             * 可能仍是初始化值 MID_W，
             * 屏幕上就会出现固定的竖直线。
             */


            /*
             * 左右边界必须同时有效。
             */
            if(road_mode == ROAD_MODE_IPM_FILL)
            {
                /*
                 * IPM 模式下，中线可能只有一侧真实边界。
                 * 不能再要求 left_line_list 和 right_line_list 同时有效。
                 */
                if(!ipm_mid_line_valid[y])
                {
                    previous_valid = 0;
                    previous_source_y = -1;
                    continue;
                }
            }
            else
            {
                /*
                 * 普通巡线仍然要求左右真实边界同时有效。
                 */
                if(left_line_list[y] == 0 ||
                   right_line_list[y] == 0)
                {
                    previous_valid = 0;
                    previous_source_y = -1;
                    continue;
                }

                if(left_line_list[y] >= right_line_list[y])
                {
                    previous_valid = 0;
                    previous_source_y = -1;
                    continue;
                }
            }

            /*
             * 使用 image_deal() 最后生成的中线。
             * 这里包含拟合补线后的结果。
             */
            middle_x = mid_line_list[y];

            if(middle_x < 1 ||
               middle_x >= MT9V03X_W - 1)
            {
                previous_valid = 0;
                previous_source_y = -1;
                continue;
            }

            display_x =
                (uint32)middle_x * 127U /
                (MT9V03X_W - 1U);

            display_y =
                (uint32)y * 80U /
                (MT9V03X_H - 1U);

            /*
             * 多个摄像头行可能映射到同一个 TFT 屏幕行。
             *
             * 例如：
             * 原图第 116 行 -> 屏幕第 77 行
             * 原图第 115 行 -> 屏幕第 77 行
             *
             * 如果两个点都画出来，就会出现水平短线。
             * 同一屏幕行只保留后处理到的点。
             */
            if(previous_valid &&
               previous_y == display_y)
            {
                previous_x = display_x;
                previous_y = display_y;
                previous_source_y = y;
                continue;
            }

            /*
             * 画当前中线点。
             */
            tft180_draw_point(
                display_x,
                display_y,
                RGB565_RED);

            /*
             * 只有原图中相邻、屏幕坐标也不同，
             * 才连接前后两个中线点。
             */
            if(previous_valid &&
               previous_source_y == y + 1 &&
               previous_y != display_y)
            {
                tft180_draw_line(
                    previous_x,
                    previous_y,
                    display_x,
                    display_y,
                    RGB565_RED);
            }

            previous_x = display_x;
            previous_y = display_y;
            previous_source_y = y;
            previous_valid = 1;
        }
    }
}
