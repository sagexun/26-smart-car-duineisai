/*
 * Motor.h
 *
 *  Created on: 2026锟斤拷9锟斤拷7锟斤拷
 *      Author: 薰
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_
#include "zf_common_headfile.h"

#define MotorL_pwm      ATOM0_CH2_P21_4
#define MotorL_turn     P21_5

#define MotorR_pwm      ATOM0_CH0_P21_2
#define MotorR_turn     P21_3

#define pid_limit      8000

/* Positive PWM must produce positive encoder_speed. Keep calibrated signs. */
#define ENCODER_L_SIGN  (-1)
#define ENCODER_R_SIGN  (1)

extern volatile uint8 start;
/* 已确认的独立斑马线次数，第二次确认后锁停 */
extern volatile uint8 zebra_count;
extern float get_turn_pid_output;
extern float get_turn_cmd;         /* 实际转向命令，范围 [-1, 1] */
extern struct PID ring_pid;        /* 圆环专用转向 PID，和 image_pid 分开调 */
typedef struct motor1
{
    volatile float target_speed;      // 目锟斤拷锟劫讹拷
    volatile int duty;              // PWM占锟秸憋拷锟斤拷锟�
    volatile float encoder_speed;     // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟绞碉拷锟斤拷俣锟�
    int16 encoder_raw;       // 锟斤拷锟轿诧拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    int32 total_encoder;   // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟桔计ｏ拷锟斤拷蹋锟�:锟叫断筹拷锟斤拷锟斤拷锟剿讹拷远
}motor1;

extern struct motor1 motor_L;
extern struct motor1 motor_R;
extern float base_speed;

extern volatile uint8 count;

void Motor_Init(void);
void car_start(void);
void MotorL_SetSpeed(int pwm);
void MotorR_SetSpeed(int pwm);
void Encoder_Init(void);
void Encoder_Data_get(void);
void Motor_Set_Target(motor1 *motor,float target_motor_speed);
void pid_sync_to_motor(void);
void Motor_PID_Control(void);
void Turn_PID_Control(
    float base_speed,
    float outer_add_ratio,
    float inner_reduce_ratio,
    float pid_output_limit);

#endif /* CODE_MOTOR_H_ */
