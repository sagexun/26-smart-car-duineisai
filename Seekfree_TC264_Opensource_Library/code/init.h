/*
 * init.h
 *
 *  Created on: 2026年9月16日
 *      Author: 薰
 */

#ifndef CODE_INIT_H_
#define CODE_INIT_H_

#include "zf_common_headfile.h"

extern volatile int16 pid_error;
extern uint8 line_lost_frames;
extern uint8 zebra_on_frames;
extern uint8 zebra_off_frames;
extern uint8 zebra_now;
extern float Kp_p;
extern float set_val;
extern uint8_t debug_test;

/* 初始化成功为 1，失败为 0 */
extern volatile uint8 imu_ready;

/* 三轴加速度，单位：g */
extern volatile float angle_z;

extern volatile uint8 wireless_init_result;

void init(void);

#endif /* CODE_INIT_H_ */
