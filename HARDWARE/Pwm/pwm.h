#ifndef __PWM_H
#define __PWM_H

#include "sys.h"

// TIM4 4 路 PWM (PB6~PB9), 默认 10kHz, CCR 0~100 = 0~100%
void TIM4_PWM_Init_4CH(u16 arr, u16 psc);

// 设置某一路 PWM 占空比, ch=1..4, ccr=0..100
void TIM4_SetPWM(u8 ch, u16 ccr);

#endif
