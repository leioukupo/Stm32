#ifndef __TIMER_H
#define __TIMER_H
#include"sys.h"
void TIM3_Tick_Init(void);   // 1ms 时基: 维护 system_ms (心跳超时/DMP get_ms)
void TIM3_Init(u16 arr, u16 psc);
void TIM3_PWM_Init(u16 arr, u16 psc);
void TIM2_Cap_Init(u16 arr, u16 psc);
#endif
