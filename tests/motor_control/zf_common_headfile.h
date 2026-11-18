#ifndef TEST_HEADFILE_H
#define TEST_HEADFILE_H
#include <stdio.h>
#include "zf_common_typedef.h"
#include "pid.h"
#include "Function.h"
#include "init.h"
#include "Motor.h"
#include "image.h"
#define ATOM0_CH2_P21_4 0
#define ATOM0_CH0_P21_2 1
#define P21_5 0
#define P21_3 1
#define GPO 0
#define GPO_PUSH_PULL 0
#define TIM6_ENCODER 0
#define TIM2_ENCODER 1
#define TIM6_ENCODER_CH1_P20_3 0
#define TIM6_ENCODER_CH2_P20_0 0
#define TIM2_ENCODER_CH1_P33_7 0
#define TIM2_ENCODER_CH2_P33_6 0
#define KEY_1 0
#define KEY_SHORT_PRESS 1
#define UART_3 0
#define UART3_TX_P15_6 0
#define UART3_RX_P15_7 0
uint32 wireless_uart_read_buffer(uint8 *buffer, uint32 length);
uint32 wireless_uart_send_string(const char *string);
void uart_init(int uart, uint32 baud, int tx, int rx);
void uart_write_string(int uart, const char *string);
int key_get_state(int key);
void key_clear_state(int key);
void gpio_init(int pin, int mode, int level, int type);
void gpio_set_level(int pin, int level);
void pwm_init(int channel, int frequency, int duty);
void pwm_set_duty(int channel, int duty);
void encoder_dir_init(int channel, int a, int b);
int16 encoder_get_count(int channel);
void encoder_clear_count(int channel);
extern int16 imu660rb_gyro_z;
void imu660rb_yaw_update(float dt_s);
float imu660rb_yaw_get(void);
float imu660rb_gyro_transition(int16 gyro_value);
#endif
