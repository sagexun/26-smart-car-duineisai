/*
 * led.c
 *
 *  Created on: 2026Äê9ÔÂ7ÈÕ
 *      Author: Þ¹
 */

#include "led.h"

void led_Init(void){
//    pwm_init(ATOM0_CH7_P20_8, 50, 0);
    gpio_init(P20_8, GPO, 1, GPO_PUSH_PULL);
}

