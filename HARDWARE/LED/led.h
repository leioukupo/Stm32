#ifndef __LED_H
#define __LED_H

#include "sys.h"

void LED_Init(void);   // 初始化 PA0 输出
void LED_Set(u8 on);   // 1=亮 0=灭
void LED_Flip(void);   // 翻转

#endif
