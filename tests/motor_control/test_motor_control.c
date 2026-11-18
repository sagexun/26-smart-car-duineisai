#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "Motor.h"
#include "uart.h"

uint8 line_found_flag = 1;
volatile uint8 ring_state = 0;
volatile uint8 ring_dir = 0;
uint8 ring_done_cnt = 0;
uint8 debug_test = 0;
volatile int16 pid_error;
volatile float ring_gyro_angle_deg = 0.0f;
volatile uint8 imu_ready = 1;
volatile uint8 ring_imu_type = 2;
int16 imu660rb_gyro_z = 0;
static int pressed, pwm[2], pwm_writes, interrupt_during_stop;
static char last_uart_frame[240];
int key_get_state(int key) { (void)key; return pressed; }
void key_clear_state(int key) { (void)key; pressed = 0; }
void gpio_init(int p, int m, int l, int t) { (void)p; (void)m; (void)l; (void)t; }
void gpio_set_level(int pin, int level) {
    (void)pin; (void)level;
    if (interrupt_during_stop) Turn_PID_Control(80, .9f, 1.1f, 20);
}
void pwm_set_duty(int channel, int duty) { pwm[channel] = duty; pwm_writes++; }
void pwm_init(int channel, int frequency, int duty) {
    (void)frequency; pwm_set_duty(channel, duty);
}
void encoder_dir_init(int c, int a, int b) { (void)c; (void)a; (void)b; }
int16 encoder_get_count(int channel) { (void)channel; return 0; }
void encoder_clear_count(int channel) { (void)channel; }
void imu660rb_yaw_update(float dt_s) { (void)dt_s; }
float imu660rb_yaw_get(void) { return 0.0f; }
float imu660rb_gyro_transition(int16 gyro_value) { return (float)gyro_value; }
void uart_init(int uart, uint32 baud, int tx, int rx) {
    (void)uart; (void)baud; (void)tx; (void)rx;
}
void uart_write_string(int uart, const char *string) {
    (void)uart; (void)string;
}
static uint8 rx_buffer[64];
static uint32 rx_length;
uint32 wireless_uart_read_buffer(uint8 *buffer, uint32 length) {
    uint32 count = rx_length < length ? rx_length : length;
    for (uint32 i = 0; i < count; ++i) buffer[i] = rx_buffer[i];
    for (uint32 i = count; i < rx_length; ++i) rx_buffer[i - count] = rx_buffer[i];
    rx_length -= count;
    return count;
}
uint32 wireless_uart_send_string(const char *string) {
    snprintf(last_uart_frame, sizeof(last_uart_frame), "%s", string);
    return 0;
}
static void receive_frame(const char *frame) {
    while (*frame) rx_buffer[rx_length++] = (uint8)*frame++;
    pid_tuning_handler();
}
static void press(void) { pressed = KEY_SHORT_PRESS; car_start(); }
static void expect_stopped(void) {
    assert(pwm[0] == 0 && pwm[1] == 0);
    assert(motor_L.duty == 0 && motor_R.duty == 0);
    assert(motor_L.target_speed == 0 && motor_R.target_speed == 0);
    assert(motor_pid_L.LastError == 0 && motor_pid_R.LastError == 0);
    assert(motor_pid_L.PrevError == 0 && motor_pid_R.PrevError == 0);
    assert(image_pid.Out_I == 0 && image_pid.LastError == 0);
}
int main(void) {
    PID sample = PID_CREATE(2, 3, 4, 1);
    int writes;
    float ring_turn;

    NUM1.angle_p = 0.0f;
    NUM1.angle_i = 0.0f;
    NUM1.angle_d = 0.0f;
    NUM1.speed_p = 8.0f;
    NUM1.speed_i = 1.0f;
    NUM1.speed_d = 0.0f;
    NUM1.anglev_p = 1.3f;
    NUM1.anglev_d = 11.8f;
    NUM2.angle_p = 0.0025f;
    NUM2.anglev_p = 0.001f;
    NUM2.speed_p = 10.9f;
    car_stop = 0;

    ring_gyro_angle_deg = 123.45f;
    pid_tuning_send_current();
    assert(strstr(last_uart_frame, ",123.45,1,2,0.00,0.00\n") != NULL);
    ring_gyro_angle_deg = 0.0f;

    PID_Position(&sample, 2);
    assert(sample.Error == 2 && sample.Out_I == 2 && sample.Out_D == 2);
    PID_Reset(&sample);
    assert(sample.Kp == 2 && sample.Ki == 3 && sample.Kd == 4);
    assert(sample.Error == 0 && sample.Out_I == 0 && sample.LastError == 0);

    receive_frame("P2=1.25!");
    assert(ring_state == 0 && NUM1.anglev_p == 1.25f);
    receive_frame("P2=4!");
    assert(ring_state == 0 && NUM1.anglev_p == 4);
    receive_frame("D2=0.5!");
    assert(ring_state == 0 && NUM1.anglev_d == 0.5f);
    receive_frame("T3=2!");
    assert(ring_state == 2);
    receive_frame("T3=4!");
    assert(ring_state == 2);
    receive_frame("T3=0!");
    assert(ring_state == 0);

    Motor_Init();
    pid_sync_to_motor();
    image_pid.Kp = 5;
    expect_stopped();
    Turn_PID_Control(80, .9f, 1.1f, 20);
    expect_stopped();
    press();
    assert(start == 1);
    car_start(); /* No target published yet: do not run speed PID. */
    expect_stopped();
    writes = pwm_writes;
    Turn_PID_Control(80, .9f, 1.1f, 20);
    assert(pwm_writes == writes); /* 10 ms loop only updates targets. */
    assert(motor_L.target_speed == 80 && motor_R.target_speed == 80);
    car_start();
    assert(motor_L.duty > 0 && motor_R.duty > 0);
    assert(start == 1); /* Consuming the key event prevents duplicate toggles. */
    pid_error = 10;
    Turn_PID_Control(80, .9f, 1.1f, 20);
    assert(motor_L.target_speed == 89 && motor_R.target_speed == 69);
    pid_error = -10;
    Turn_PID_Control(80, .9f, 1.1f, 20);
    assert(motor_L.target_speed == 69 && motor_R.target_speed == 89);

    /* ring_state != 0 must load P2/D2 values into ring_pid. */
    NUM1.anglev_p = 1.0f;
    NUM1.anglev_d = 0.0f;
    ring_state = 2;
    pid_error = 10;
    Turn_PID_Control(80, .9f, 1.1f, 20);
    assert(ring_pid.Kp == 1.1f && ring_pid.Kd == 0.0f);
    ring_turn = ring_pid.Kp * 10.0f / 20.0f;
    assert(fabsf(motor_L.target_speed - 80.0f * (1.0f + 0.9f * ring_turn)) < 0.01f);
    assert(fabsf(motor_R.target_speed - 80.0f * (1.0f - 1.1f * ring_turn)) < 0.01f);
    pid_error = -10;
    Turn_PID_Control(80, .9f, 1.1f, 20);
    assert(fabsf(motor_L.target_speed - 80.0f * (1.0f - 1.1f * ring_turn)) < 0.01f);
    assert(fabsf(motor_R.target_speed - 80.0f * (1.0f + 0.9f * ring_turn)) < 0.01f);
    ring_state = 0;
    pid_error = 0;
    Turn_PID_Control(80, .9f, 1.1f, 20);

    /* Reproduce stale saturated output and large old errors, then stop. */
    MotorL_SetSpeed(3000);
    MotorR_SetSpeed(-3000);
    motor_pid_L.LastError = 704.36f;
    motor_pid_R.LastError = -704.36f;
    motor_pid_L.PrevError = 700;
    motor_pid_R.PrevError = -700;
    interrupt_during_stop = 1;
    press(); /* Inject a 10 ms tick partway through stopping. */
    interrupt_during_stop = 0;
    assert(start == 2);
    expect_stopped();
    Turn_PID_Control(80, .9f, 1.1f, 20);
    car_start();
    expect_stopped();

    /* Stopping must survive repeated speed and steering ticks. */
    for (int tick = 0; tick < 100; ++tick) {
        car_start();
        Turn_PID_Control(80, .9f, 1.1f, 20);
        car_start();
        expect_stopped();
    }
    car_stop = 1;
    press(); /* Odd start cannot override the host stop. */
    assert(start == 3);
    for (int tick = 0; tick < 100; ++tick) {
        Turn_PID_Control(80, .9f, 1.1f, 20);
        car_start();
        expect_stopped();
    }
    car_stop = 0;
    car_start();
    Turn_PID_Control(80, .9f, 1.1f, 20);
    car_start();
    assert(motor_L.duty > 0 && motor_R.duty > 0);
    car_stop = 1;
    car_start();
    expect_stopped();
    press(); /* Even start remains stopped after releasing host stop. */
    car_stop = 0;
    car_start();
    Turn_PID_Control(80, .9f, 1.1f, 20);
    car_start();
    expect_stopped();

    for (int i = 0; i < 130; ++i) { /* Includes uint8 start wraparound. */
        press();
        assert(start & 1u);
        car_start();
        expect_stopped();
        pid_error = 0;
        Turn_PID_Control(80, .9f, 1.1f, 20);
        car_start();
        assert(motor_L.duty > 0 && motor_R.duty > 0);
        assert(motor_pid_L.Kp == 8 && motor_pid_R.Kp == 10.9f);
        assert(image_pid.Kp == 0);
        press();
        expect_stopped();
    }
    puts("PASS: reset, zero-error targets, turn limits, stop preemption, telemetry, 130 restarts and counter wraparound");
    return 0;
}
