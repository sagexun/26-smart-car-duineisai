/*
 * pid.h
 *
 *  Created on: 2026年9月14日
 *      Author: 薰
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_typedef.h"

#define pid_out_I_Limit  1000

extern struct PID motor_pid_L;
extern struct PID motor_pid_R;
extern struct PID image_pid;

typedef struct PID
{
    float Kp;
    float Ki;
    float Kd;
    float LowPass;      //低通滤波系数（一般用来给微分项降噪）

    float Out_P;
    float Out_I;
    float Out_D;

    float PrevError;    //上上次误差
    float LastError;    //上次误差
    float Error;        //当前误差
    float LastData;     //上次得到的电机速度，陀螺仪的角度
}PID;

#define PID_CREATE(_kp,_ki,_kd,_low_pass)\
{\
    .Kp=_kp,\
    .Ki=_ki,\
    .Kd=_kd,\
    .LowPass=_low_pass,\
    .Out_P=0,\
    .Out_I=0,\
    .Out_D=0,\
}

void PID_Reset(PID *pid);
float PID_Increase(PID *PID,float NowData,float Target);
float PID_Position(PID *PID,float error);

#endif /* CODE_PID_H_ */
