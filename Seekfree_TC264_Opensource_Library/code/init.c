/*
 * init.c
 *
 *  Created on: 2026年9月16日
 *      Author: 薰
 */

#include "init.h"

volatile int16 pid_error = 0;
uint8 line_lost_frames = 0;
uint8 zebra_on_frames = 0;
uint8 zebra_off_frames = 0;
uint8 zebra_now = 0;
float set_val = 0.0f;

/* 初始化成功为 1，失败为 0 */
volatile uint8 imu_ready = 0;

/* 三轴加速度，单位：g */
volatile float angle_z = 0.0f;

/* 在调试器里观察：255=尚未返回，0=库报告成功，1=失败 */
volatile uint8 wireless_init_result = 255;

//显示错误标志位
uint8 debug_test = 0;



void init(void){

    //外设初始化
    led_Init();
    //无线串口调试初始化
    wireless_uart_init();
//    wireless_init_result = wireless_uart_init();
//
//    if (wireless_init_result != 0)
//    {
//        /*
//         * 当前库在自动波特率失败时，可能没有恢复 RTS 为输入。
//         * 这里只恢复引脚方向，不代表初始化已经成功。
//         */
//        gpio_init(WIRELESS_UART_RTS_PIN,
//                  GPI,
//                  GPIO_HIGH,
//                  GPI_PULL_UP);
//    }
//    imu_ready = (imu660rb_yaw_init() == 0);
//    if (imu_ready)
//    {
//        uint8_t buf1[] = "IMU660RB init OK\r\n";
//        wireless_uart_send_buffer(buf1,sizeof(buf1));
//    }
//    else
//    {
//        imu_ready = 0;
//
//        uint8_t buf2[] = "IMU660RB init FAILED\r\n";
//        wireless_uart_send_buffer(buf2,sizeof(buf2));
//    }


    //电机初始化
    Motor_Init();
    //编码器初始化
    Encoder_Init();
    //按键初始化
    key_init(10);
    //定时中断
    pit_ms_init(CCU61_CH0,5);
}
