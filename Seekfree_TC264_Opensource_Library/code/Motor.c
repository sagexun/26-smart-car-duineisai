/*
 * Motor.c
 *
 *  Created on: 2026锟斤拷9锟斤拷7锟斤拷
 *      Author: 薰
 */

#include "Motor.h"
#include "uart.h"
#include "init.h"
#include "image.h"     // 新增

motor1 motor_L;
motor1 motor_R;

/*
 * 普通路继续使用 image_pid。
 * ring_state != 0 时改走 ring_pid，初始值和当前普通路的源码默认值一致，
 * 后面圆环参数可以直接在这里单独改，不会影响普通路。
 */
PID ring_pid = PID_CREATE(1.3f, 0.0f, 11.8f, 1);

volatile uint8 start = 0;
volatile uint8 zebra_count = 0U;

/* Only the 2 ms loop changes this gate; the 10 ms loop reads it. */
static volatile uint8 motor_control_enabled = 0;
static volatile uint8 motor_target_ready = 0;
static uint8 turn_pid_ring_mode = 0;

#define MOTOR_SPEED_DT_S          0.005f
#define ENCODER_LINES             1024.0f
#define MOTOR_GEAR_RATIO          0.4412f
#define WHEEL_CIRCUMFERENCE_CM    26.80f

static float enc_l_y1 = 0.0f;
//static float enc_l_y2 = 0.0f;
static float enc_r_y1 = 0.0f;
//static float enc_r_y2 = 0.0f;

#define MOTOR_DEADZONE_PWM          900
#define MOTOR_START_SPEED_CM_S      5.0f

#define RING_ENTRY_ERROR_MIN       10.0f

#define RING_ENTRY_TURN_MIN         0.60f
#define RING_ENTRY_TURN_MAX         0.70f

/* 转向外环20ms一次，4拍就是80ms */
#define RING_ENTRY_HOLD_TICKS       6u


static float ring_entry_sign       = 0.0f;

static uint8 ring_entry_hold_ticks = 0;
static uint8 ring_entry_used       = 0;

/* state1切到state2以后，继续允许触发5拍，也就是100ms */
#define RING_ENTRY_S2_TRIGGER_TICKS  5u

static uint8 ring_entry_s2_trigger_ticks = 0u;
volatile uint8 count = 0;

float base_speed = 240.0f;

float get_turn_pid_output = 0.0f;
float get_turn_cmd = 0.0f;

static float Ring_Entry_Assist(float normal_turn, float error)
{
    float expected_sign;
    float magnitude;
    uint8 state;
    uint8 dir;
    uint8 can_trigger = 0u;

    state = ring_state;
    dir   = ring_dir;

    /* 回到普通道路，重新允许下一个环岛触发 */
    if(state == 0u)
    {
        ring_entry_hold_ticks       = 0u;
        ring_entry_used             = 0u;
        ring_entry_sign             = 0.0f;
        ring_entry_s2_trigger_ticks = 0u;

        return normal_turn;
    }

    /* 环岛方向无效或者已经进入出环状态 */
    if(((dir != 1u) && (dir != 2u)) || (state == 3u))
    {
        ring_entry_hold_ticks       = 0u;
        ring_entry_sign             = 0.0f;
        ring_entry_s2_trigger_ticks = 0u;

        return normal_turn;
    }

    /* dir=1右环，turn为正；dir=2左环，turn为负 */
    expected_sign = (dir == 1u) ? 1.0f : -1.0f;

    /* 保持期间环岛方向发生变化，才取消保持 */
    if((ring_entry_hold_ticks > 0u) &&
       (ring_entry_sign * expected_sign < 0.0f))
    {
        ring_entry_hold_ticks = 0u;
        ring_entry_sign       = 0.0f;

        return normal_turn;
    }

    /*
     * state1期间一直允许触发，同时不断刷新state2触发窗口。
     * 这样1→2切换帧产生的error跳变不会被漏掉。
     */
    if(state == 1u)
    {
        can_trigger = 1u;
        ring_entry_s2_trigger_ticks =
            RING_ENTRY_S2_TRIGGER_TICKS;
    }
    else if(state == 2u)
    {
        if(ring_entry_s2_trigger_ticks > 0u)
        {
            can_trigger = 1u;
            ring_entry_s2_trigger_ticks--;
        }
    }
    else
    {
        ring_entry_hold_ticks = 0u;
        ring_entry_sign       = 0.0f;

        return normal_turn;
    }

    /*
     * 不再强制要求单拍jump>=6。
     * state1或刚进入state2时，方向正确且error达到10就触发。
     */
    if(can_trigger &&
       (ring_entry_used == 0u) &&
       (expected_sign * error >= RING_ENTRY_ERROR_MIN))
    {
        ring_entry_used       = 1u;
        ring_entry_sign       = expected_sign;
        ring_entry_hold_ticks = RING_ENTRY_HOLD_TICKS;
    }

    if(ring_entry_hold_ticks == 0u)
    {
        return normal_turn;
    }

    /*
     * 保持期间不再因为pid_error瞬间反向而退出。
     * 80ms内完全相信已经确认过的ring_dir。
     */
    magnitude = normal_turn >= 0.0f
              ? normal_turn
              : -normal_turn;

    magnitude = Limit_float(
        RING_ENTRY_TURN_MIN,
        magnitude,
        RING_ENTRY_TURN_MAX
    );

    ring_entry_hold_ticks--;

    return ring_entry_sign * magnitude;
}
//static float Encoder_Filter(float input, float *last, float *last2)
//{
//    float output = 0.2f * input
//                 + 0.4f * (*last)
//                 + 0.4f * (*last2);
//
//    *last2 = *last;
//    *last = output;
//
//    return output;
//}

static float Encoder_Filter(float input, float *last)
{
    float output = 0.7f * input + 0.3f * (*last);

    *last = output;

    return output;
}

static void Motor_Stop_And_Reset(void)
{
    motor_target_ready = 0;

    ring_entry_sign             = 0.0f;
    ring_entry_hold_ticks       = 0u;
    ring_entry_used             = 0u;
    ring_entry_s2_trigger_ticks = 0u;

    get_turn_pid_output = 0.0f;
    get_turn_cmd = 0.0f;

    Motor_Set_Target(&motor_L, 0.0f);
    Motor_Set_Target(&motor_R, 0.0f);
    PID_Reset(&motor_pid_L);
    PID_Reset(&motor_pid_R);
    PID_Reset(&image_pid);
    PID_Reset(&ring_pid);
    MotorL_SetSpeed(0);
    MotorR_SetSpeed(0);
}


void Motor_Init(void){
    gpio_init(MotorL_turn, GPO, 0, GPO_PUSH_PULL);
    pwm_init(MotorL_pwm,17000,0);
    gpio_init(MotorR_turn, GPO, 1, GPO_PUSH_PULL);
    pwm_init(MotorR_pwm,17000,0);
    motor_control_enabled = 0;
    start = 0;
    zebra_count = 0U;
    Motor_Stop_And_Reset();
}

//锟斤拷锟斤拷锟斤拷锟斤拷
//void car_start(void){
//    /*
//     * CPU1 每确认一次独立斑马线，zebra_count 加 1。
//     * 第一次正常通过，第二次锁停并清空目标、PID和两路PWM。
//     */
//    if(key_get_state(KEY_1) == KEY_SHORT_PRESS){
//        key_clear_state(KEY_1);
//        /* Block the 10 ms loop while resetting both wheels. */
//        motor_control_enabled = 0;
//        Motor_Stop_And_Reset();
//        zebra_count = 0U;
//        start++;
//        if(start == 1u)   /* 第一次发车时清空环岛计数，否则重跑会沿用上次的done_cnt */
//        {
//            ring_done_cnt = 0;
//        }
//
//        motor_control_enabled = (uint8)((start & 1u) && car_stop == 0 && line_found_flag);
//        /* Wait for fresh targets from the next 10 ms tick. */
//        return;
//    }
//    if((zebra_count >= 2U) || car_stop || (start & 1u) == 0u){
//        motor_control_enabled = 0;
//        Motor_Stop_And_Reset();
//        return;
//    }else{
//        motor_control_enabled = 1;
//    }
////    if(line_found_flag == 0){
////        motor_control_enabled = 0;
////        Motor_Stop_And_Reset();
////    }
////还需要进行丢线判断
//    if(motor_control_enabled && motor_target_ready && car_stop == 0){
//        Motor_PID_Control();
//    }
//}
void car_start(void)
{
    /* 按键处理：启动、停止，并清空本轮斑马线计数 */
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_1);

        motor_control_enabled = 0;
        Motor_Stop_And_Reset();

        zebra_count = 0U;
        start++;

        if(start == 1u)
        {
            ring_done_cnt = 0;
        }

        motor_control_enabled =
            (uint8)((start & 1u) && car_stop == 0 && line_found_flag);

        /* 等待下一次转向环给出新目标 */
        return;
    }

    /* 正常停车或还没发车：立即停车 */
    if(car_stop || (start & 1u) == 0u)
    {
        motor_control_enabled = 0;
        Motor_Stop_And_Reset();
        return;
    }

    /*
     * 第二次识别到斑马线：
     * 不再直接清 PWM，而是把目标速度设为 0，
     * 继续运行速度 PI，让 PWM 自己逐渐减小。
     */
    if(zebra_count >= 2U)
    {
        motor_control_enabled = 1;
        motor_target_ready = 1;

        Motor_Set_Target(&motor_L, 0.0f);
        Motor_Set_Target(&motor_R, 0.0f);

        Motor_PID_Control();
        return;
    }

    /* 普通行驶 */
    motor_control_enabled = 1;

    if(motor_control_enabled &&
       motor_target_ready &&
       car_stop == 0)
    {
        Motor_PID_Control();
    }
}

void MotorL_SetSpeed(int pwm){
    pwm = Limit_int(-pid_limit, pwm, pid_limit);
    motor_L.duty = pwm;
    if(pwm >= 0)
    {
        gpio_set_level(MotorL_turn,1);
        pwm_set_duty(MotorL_pwm,pwm);           //正转
    }else
    {
        gpio_set_level(MotorL_turn,0);
        pwm_set_duty(MotorL_pwm,-pwm);          //反转
    }
}

void MotorR_SetSpeed(int pwm){
    pwm = Limit_int(-pid_limit, pwm, pid_limit);
    motor_R.duty = pwm;
    if(pwm >= 0)
    {
        gpio_set_level(MotorR_turn,1);
        pwm_set_duty(MotorR_pwm,pwm);           //锟斤拷锟斤拷锟斤拷转
    }else
    {
        gpio_set_level(MotorR_turn,0);
        pwm_set_duty(MotorR_pwm,-pwm);          //锟斤拷锟街凤拷转
    }
}

//锟斤拷锟斤拷锟斤拷锟斤拷始锟斤拷
void Encoder_Init(void){
    //锟斤拷锟斤拷锟斤拷锟�
    encoder_dir_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);//20.3为A通锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷20.0为B通锟斤拷锟斤拷锟斤拷锟斤拷
    //锟揭憋拷锟斤拷锟斤拷
    encoder_dir_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);//33.7为A通锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷33.6为B通锟斤拷锟斤拷锟斤拷锟斤拷
    //锟斤拷锟揭憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
}

//精度不行
//void Encoder_Data_get(void){
//    //M法，低速时精度很低，适用于高速
//    motor_L.encoder_raw = ENCODER_L_SIGN * SecondOrderLagFilter_L(encoder_get_count(TIM6_ENCODER));//一阶滞后滤波
//    motor_L.encoder_speed = (((float)motor_L.encoder_raw/1024)*0.4412*26.80)/0.01;//cm/s
//    motor_L.total_encoder += motor_L.encoder_raw;
//    encoder_clear_count(TIM6_ENCODER);
//
//    motor_R.encoder_raw = ENCODER_R_SIGN * SecondOrderLagFilter_R(encoder_get_count(TIM2_ENCODER));//一阶滞后滤波
//    motor_R.encoder_speed = (((float)motor_R.encoder_raw/1024)*0.4412*26.80)/0.01;//cm/s
//    motor_R.total_encoder += motor_R.encoder_raw;
//    encoder_clear_count(TIM2_ENCODER);
//    //T法：适用于低速，但是不知道咋搞，如果自己不能独立写先让ai来写，或者直接把速度搞成1编码器encoder
//}

void Encoder_Data_get(void)
{
    int16 raw_l;
    int16 raw_r;
    float filtered_l;
    float filtered_r;
    const float count_to_speed =
        MOTOR_GEAR_RATIO *
        WHEEL_CIRCUMFERENCE_CM /
        ENCODER_LINES /
        MOTOR_SPEED_DT_S;

    raw_l = ENCODER_L_SIGN * encoder_get_count(TIM6_ENCODER);
    raw_r = ENCODER_R_SIGN * encoder_get_count(TIM2_ENCODER);

    encoder_clear_count(TIM6_ENCODER);
    encoder_clear_count(TIM2_ENCODER);

    filtered_l = Encoder_Filter((float)raw_l, &enc_l_y1);
    filtered_r = Encoder_Filter((float)raw_r, &enc_r_y1);

    motor_L.encoder_raw = raw_l;
    motor_R.encoder_raw = raw_r;

    motor_L.encoder_speed = filtered_l * count_to_speed;
    motor_R.encoder_speed = filtered_r * count_to_speed;

    // 里程必须累加未滤波的原始脉冲
    motor_L.total_encoder += raw_l;
    motor_R.total_encoder += raw_r;
}

//设置左右电机速度，用在10ms的定时中断转向环
void Motor_Set_Target(motor1 *motor,float target_motor_speed){
    motor->target_speed = target_motor_speed;
}

void Turn_PID_Control(
    float base_speed,
    float outer_add_ratio,
    float inner_reduce_ratio,
    float pid_output_limit)
{
    float error;
    float pid_output;
    float turn;
    float magnitude;
    float target_l;
    float target_r;
    PID *turn_pid;
    uint8 use_ring_pid;
    if(!motor_control_enabled || pid_output_limit <= 0.0f)
    {
        return;
    }
    /* Turn_PID_Control() 开头，防止转向环重新覆盖目标速度 */
    if((zebra_count >= 2U) ||
       !motor_control_enabled ||
       pid_output_limit <= 0.0f)
    {
        return;
    }
    // 防止 2 ms 内环恰好读到一新一旧两个目标
    motor_target_ready = 0;

    error = (float)Limit_int(-45,pid_error,45);

    /* 普通路/圆环只切换 PID 对象，后面的差速分配完全共用。 */
    use_ring_pid = (ring_state != 0U) ? 1U : 0U;
    if(use_ring_pid != turn_pid_ring_mode)
    {
        /* 切换路线时清掉上一段路留下的积分和旧误差。 */
        if(use_ring_pid)
        {
            PID_Reset(&ring_pid);
        }
        else
        {
            PID_Reset(&image_pid);
        }
        turn_pid_ring_mode = use_ring_pid;
    }

    if(use_ring_pid)
    {
        /* 圆环 P/D 由串口 P2/D2 写入 NUM1.anglev_p / NUM1.anglev_d。 */
        ring_pid.Kp = 2.2 + 0.0032 * pid_error * pid_error;
        ring_pid.Kd = 25.0;
        turn_pid = &ring_pid;
    }
    else
    {
        /* 普通路保持原来的参数来源和非线性 P。 */
        image_pid.Kp = 1 + 0.00345 * pid_error * pid_error;
        image_pid.Kd = 25.0;
        turn_pid = &image_pid;
    }

    /* 增益和 PID 输出使用同一个误差。 */
    pid_output = PID_Position(turn_pid, error);
    get_turn_pid_output = pid_output;
    turn = Limit_float(
    -1.0f,
    pid_output / pid_output_limit,
    1.0f);
    get_turn_cmd = turn;
        // 将外环输出归一化到 [-1, 1]
    /* 新增：state1出现error跳变后，保持受控差速80ms */
//    turn = Ring_Entry_Assist(turn, error);
    /* 环内（已进入环内 state2/3）按已完成环数压 |turn| 上限，
     * 避免入环时左右轮速度差过大导致顿挫。
     * 第1个环 done_cnt=0 → 上限0.6；第2个环 done_cnt=1 → 上限0.7 */
//    if((ring_state == 2u) || (ring_state == 3u))
//    {
//        float turn_cap;
//        if(ring_done_cnt == 0u)
//        {
//            turn_cap = 0.60f;
//        }
//        else if(ring_done_cnt == 1u)
//        {
//            turn_cap = 0.65f;
//        }
////        else
////        {
////            turn_cap = 0.70f;   /* 第3个及以后，默认0.7，按需要调 */
////        }
////        float turn_cap;
////        if(ring_done_cnt == 0u)
////        {
////            turn_cap = 0.66f;
////        }
//        turn = Limit_float(-turn_cap, turn, turn_cap);
//    }
    magnitude = turn >= 0.0f ? turn : -turn;


    if(turn > 0.0f)
    {
        // 左侧为外轮，右侧内
        target_r = base_speed *(1.0f - inner_reduce_ratio * magnitude);

        target_l = base_speed *(1.0f + outer_add_ratio * magnitude);
    }
    else
    {
        // 右侧为外轮，左侧内
        target_l = base_speed *(1.0f - inner_reduce_ratio * magnitude);

        target_r = base_speed *(1.0f + outer_add_ratio * magnitude);
    }

    Motor_Set_Target(&motor_L, limit_more_than0(target_l));
    Motor_Set_Target(&motor_R, limit_more_than0(target_r));

    motor_target_ready = 1;
}

//差速转向环,放在10ms的定时器中断
/*
 * 时间：10ms我个人认为最好还是只是搞个标志位就好了，中断最好不要有太多的计算
 * 参数:k1,加速的程度，k2，减速的程度
 * 需要时刻的查找当前的error，然后根据error来调这个小车的目标速度
 */
//void Turn_PID_Control(float Basic_speed,float k1,float k2,float limit){
//    float error;
//    float pid_val;
//
//    if(!motor_control_enabled){
//        return;
//    }
//    error = (float)pid_error;
//
//    pid_val = Limit_float(-limit, PID_Position(&image_pid, error), limit);
////    pid_val = set_val;
////    Motor_Set_Target(&motor_L, Basic_speed + pid_val);
////    Motor_Set_Target(&motor_R, Basic_speed + pid_val);
//    if(pid_error >= 0){
//        Motor_Set_Target(&motor_L, Basic_speed - k1 * pid_val);
//        Motor_Set_Target(&motor_R, limit_more_than0(Basic_speed - k2 * pid_val));
//    }else{
//        Motor_Set_Target(&motor_L, limit_more_than0(Basic_speed + k2 * pid_val));
//        Motor_Set_Target(&motor_R, Basic_speed + k1 * pid_val);
//    }
//    /* Publish readiness only after both targets have been assigned. */
//    motor_target_ready = 1;
//}

void pid_sync_to_motor(void)
{
    motor_pid_L.Kp = NUM1.speed_p;
    motor_pid_L.Ki = NUM1.speed_i;
    motor_pid_L.Kd = NUM1.speed_d;//左电机PI

    motor_pid_R.Kp = NUM2.speed_p;
    motor_pid_R.Ki = NUM2.speed_i;
    motor_pid_R.Kd = NUM2.speed_d;//右电机PI

    base_speed     = NUM1.angle_i;
//    image_pid.Kp   = NUM1.angle_p;
//    image_pid.Kd   = NUM1.angle_d;
}

static int Motor_Speed_PI_Step(motor1 *motor, PID *pid)
{
    float delta_pwm;
    int next_pwm;

    delta_pwm = PID_Increase(
        pid,
        motor->encoder_speed,
        motor->target_speed
    );

    next_pwm = (int)Limit_float(
        -(float)pid_limit,
        (float)motor->duty + delta_pwm,
        (float)pid_limit
    );

    // 启动死区补偿
    if(motor->target_speed > 0.0f &&
       motor->encoder_speed < motor->target_speed &&
       motor->encoder_speed > -MOTOR_START_SPEED_CM_S &&
       motor->encoder_speed <  MOTOR_START_SPEED_CM_S &&
       next_pwm > 0 &&
       next_pwm < MOTOR_DEADZONE_PWM)
    {
        next_pwm = MOTOR_DEADZONE_PWM;
    }
    // 目标已经是零，车轮也已经停下时，不继续反向驱动
    if(motor->target_speed <= 0.0f &&
       motor->encoder_speed <= 0.0f &&
       next_pwm < 0)
    {
        next_pwm = 0;
    }

    return next_pwm;
}

void Motor_PID_Control(void)
{
    motor_L.duty = Motor_Speed_PI_Step(&motor_L, &motor_pid_L);
    motor_R.duty = Motor_Speed_PI_Step(&motor_R, &motor_pid_R);

    MotorL_SetSpeed(motor_L.duty);
    MotorR_SetSpeed(motor_R.duty);
}

////增量式pid控制
//void Motor_PID_Control(void){
//    //pid
//    motor_L.duty = Limit_int(-pid_limit,motor_L.duty+PID_Increase(&motor_pid_L,motor_L.encoder_speed,motor_L.target_speed),pid_limit);
//    motor_R.duty = Limit_int(-pid_limit,motor_R.duty+PID_Increase(&motor_pid_R,motor_R.encoder_speed,motor_R.target_speed),pid_limit);
//    //设置占空比
//    MotorL_SetSpeed(motor_L.duty);
//    MotorR_SetSpeed(motor_R.duty);
//}
