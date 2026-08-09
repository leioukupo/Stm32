#ifndef __TB6612_H
#define __TB6612_H

#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////
// TB6612 双电机驱动 (Phase 3, 计划 §2)
//
// 实物已确认 2 块 TB6612 (4 路 PWM):
//   左板: A=FL, B=RL, STBY=PA1, 方向 PB12/PB13(A) PB14/PB15(B)
//   右板: A=FR, B=RR, STBY=PA3, 方向 PA4/PA5(A) PA6/PA7(B)
//   PWM:  TIM4 CH1=FL(PB6) CH2=RL(PB7) CH3=FR(PB8) CH4=RR(PB9)
//
// TB6612 方向逻辑: AIN1=1,AIN2=0 正转; AIN1=0,AIN2=1 反转; 00 停止.
// (若实测电机转向相反, 对调对应电机的 AIN1/AIN2 即可.)
//////////////////////////////////////////////////////////////////////////////////

#define TB6612_WHEELS      4       // 0=FL 1=FR 2=RL 3=RR (与 mecanum.h 一致)

// 初始化: 方向脚 + STBY 上电拉高 + TIM4 4 路 PWM
void tb6612_init(void);

// 设置单轮: duty -100..100 (符号=方向, 绝对值=占空比)
int tb6612_set_wheel(u8 idx, int duty);

// 一次设置 4 轮
void tb6612_set_wheels(const int duty[4]);

// 全部失电停止 (PWM=0, 方向清零)
void tb6612_stop(void);

#endif
