#include "servo.h"
#include "delay.h"

//////////////////////////////////////////////////////////////////////////////////
// 单总线舵机驱动
// 使用USART2半双工单线模式(PA2)与舵机通信
// 半双工模式下,PA2既是TX也是RX,硬件自动切换方向
//////////////////////////////////////////////////////////////////////////////////

u8  SERVO_RX_BUF[SERVO_REC_LEN]; // 舵机接收缓冲
u16 SERVO_RX_STA = 0;            // 接收状态标记
                                  // bit15: 帧接收完成(空闲超时)
                                  // bit14~0: 已接收字节数

// 初始化USART2为半双工单线模式,用于舵机通信
void Servo_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1.使能USART2和GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // 2.初始化PA2为复用推挽输出(单总线数据线)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3.USART2初始化
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &USART_InitStructure);

    // 4.使能半双工模式
    USART_HalfDuplexCmd(USART2, ENABLE);

    // 5.使能接收中断
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);  // 线路空闲=帧完成
    USART_Cmd(USART2, ENABLE);

    // 6.中断优先级设置 (优先级低于USART1,避免抢占半双工线)
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// 通过单总线发送一个字节到舵机
void Servo_SendByte(u8 data)
{
    // 等待上一次发送完成
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    USART_SendData(USART2, data);
}

// 清空接收缓冲与状态
void Servo_RX_Reset(void)
{
    SERVO_RX_STA = 0;
}

// USART2中断服务程序 — 接收舵机返回数据,存入缓冲
void USART2_IRQHandler(void)
{
    u8 res;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        res = USART_ReceiveData(USART2);
        // 存入接收缓冲,等待主循环转发给PC
        if ((SERVO_RX_STA & 0x8000) == 0) // 上一帧未被取走
        {
            u16 idx = SERVO_RX_STA & 0x3FFF;
            if (idx < SERVO_REC_LEN - 1)
            {
                SERVO_RX_BUF[idx] = res;
                SERVO_RX_STA = idx + 1;     // 更新计数
            }
        }
    }
    // 线路空闲: 一帧数据接收完毕, 置位 bit15 帧完成标记
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        USART_ReceiveData(USART2);  // 读 DR 清除 IDLE 标志
        SERVO_RX_STA |= 0x8000;
    }
}
