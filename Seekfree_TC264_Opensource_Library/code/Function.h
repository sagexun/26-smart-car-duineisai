/*
 * Function.h
 *
 *  Created on: 2026Äê9ÔÂ15ÈÕ
 *      Author: Þ¹
 */

#ifndef CODE_FUNCTION_H_
#define CODE_FUNCTION_H_

#include "zf_common_headfile.h"

int Limit_int(int a,int b,int c);
float Limit_float(float a,float b,float c);
int16 SecondOrderLagFilter_L(int16 value);
int16 SecondOrderLagFilter_R(int16 value);
float limit_more_than0(float speed);

#endif /* CODE_FUNCTION_H_ */
