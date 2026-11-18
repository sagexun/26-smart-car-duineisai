/*
 * pid.c
 *
 *  Created on: 2026年9月14日
 *      Author: 薰
 */

#include "pid.h"

static float pid_out_limit = 20;//根据实际情况
#define TURN_D_ERROR_LIMIT 20.0f
/*
 * 电机闭环增量式pid结构体以及初始化
 */
PID motor_pid_L = PID_CREATE(27,1.994,0,1);    //LowPass设置为1即用不到低通滤波
PID motor_pid_R = PID_CREATE(27,2.596,0,1);
PID image_pid = PID_CREATE(10,0,0,1); //转向环


/* Clear dynamic state without changing tuning parameters. */
void PID_Reset(PID *pid)
{
    pid->Out_P = 0.0f;
    pid->Out_I = 0.0f;
    pid->Out_D = 0.0f;
    pid->PrevError = 0.0f;
    pid->LastError = 0.0f;
    pid->Error = 0.0f;
    pid->LastData = 0.0f;
}

//PD位置式PID:主要功能计算速度差值
float PID_Position(PID *PID,float error){
    PID->Error = error;
    PID->Out_P = error;
    PID->Out_I += PID->Error;
    //积分限幅
    if(PID->Out_I > pid_out_limit){
        PID->Out_I = pid_out_limit;
    }
    if(PID->Out_I < -pid_out_limit){
        PID->Out_I = -pid_out_limit;
    }
    //微分项
    PID->Out_D = PID->Error - PID->LastError;

//    /* 只限制D项看到的单帧误差跳变 */
    if(PID->Out_D > TURN_D_ERROR_LIMIT)
    {
        PID->Out_D = TURN_D_ERROR_LIMIT;
    }
    else if(PID->Out_D < -TURN_D_ERROR_LIMIT)
    {
        PID->Out_D = -TURN_D_ERROR_LIMIT;
    }

    //更新误差
    PID->LastError = 0.9*PID->Error + 0.1 *PID->LastError;
    return(PID->Kp*PID->Out_P + PID->Ki*PID->Out_I + PID->Kd*PID->Out_D);
}

//增量式PID：用于控制电机
float PID_Increase(PID *PID,float NowData,float Target){
    PID->Error = Target - NowData;
    PID->Out_P = (PID->Error - PID->LastError);
    PID->Out_I = PID->Error;
//    PID->Out_I = Limit_float(PID->Error);
    PID->Out_D = (PID->Error - 2 * PID->LastError + PID->PrevError);

    PID->PrevError = PID->LastError;
    PID->LastError = PID->Error;
//    PID->PrevError = 0.9*PID->LastError + 0.1 *PID->PrevError;
//    PID->LastError = 0.9*PID->Error + 0.1 *PID->LastError;
    PID->LastData = NowData;

    return (PID->Kp * PID->Out_P + PID->Ki * PID->Out_I + PID->Kd * PID->Out_D);
}
//还有一个新思路，就是把这个Kp搞成Kp*x的平方，这样的话，只要一遇到很大的误差就会放大使得系统响应更快
