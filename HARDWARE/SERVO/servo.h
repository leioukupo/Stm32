#ifndef __SERVO_H
#define __SERVO_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////
// 单总线舵机驱动
// 使用USART2半双工单线模式与舵机通信
// PA2 作为单总线数据线
//////////////////////////////////////////////////////////////////////////////////

#define SERVO_REC_LEN 200 // 舵机接收缓冲最大字节数

extern u8  SERVO_RX_BUF[SERVO_REC_LEN]; // 舵机接收缓冲
extern u16 SERVO_RX_STA;                // 舵机接收状态标记

void Servo_Init(u32 bound);             // 初始化舵机串口(USART2半双工)
void Servo_SendByte(u8 data);           // 通过单总线发送一个字节
void Servo_RX_Reset(void);              // 清空接收缓冲与状态(丢弃历史数据/回显)

#endif
