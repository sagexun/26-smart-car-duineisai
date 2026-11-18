/*
 * Function.c
 *
 *  Created on: 2026年9月15日
 *      Author: 薰
 */

#include "Function.h"

/*a----min c----max
 * limit b into from a to c(int)
 */
int Limit_int(int a,int b,int c){
    if((b>=a)&&(b<=c)){
        return b;
    }else if(b < a){
        return a;
    }else if(b > c){
        return c;
    }
    return 0;
}

/*
 * a----min c----max
 * limit b from a to c
 */
float Limit_float(float a,float b,float c){
    if((b>=a)&&(b<=c)){
        return b;
    }else if(b < a){
        return a;
    }else if(b > c){
        return c;
    }
    return 0;
}

/*
 * 滤波函数：二阶滞后滤波算法，左编码器滤波
 *  */
int16 SecondOrderLagFilter_L(int16 value){
//    static int16 last_value,last_2_value;
    static int16 last_value;
//    value=0.7*value+0.3*last_value+0.0*last_2_value;     //二阶滞后滤波
    value=0.7*value+0.3*last_value;
//    last_2_value=last_value;
    last_value=value;

    return value;
}


/*
 * 滤波函数：二阶滞后滤波算法，右编码器滤波
 *  */
int16 SecondOrderLagFilter_R(int16 value){
//    static int16 last_value,last_2_value;
    static int16 last_value;
//    value=0.7*value+0.3*last_value+0.0*last_2_value;     //二阶滞后滤波
    value=0.7*value+0.3*last_value;
//    last_2_value=last_value;
    last_value=value;

    return value;
}

/*
 *限制电机速度不能小于10
 */
float limit_more_than0(float speed){
    if(speed < 0.0f){
        return 0;
    }
    return speed;
}
////求取平均值函数，减少尖峰
//void Average(void){
//
//}
