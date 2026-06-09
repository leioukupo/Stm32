#include "delay.h"
#include "usart.h"
#include "servo.h"

//////////////////////////////////////////////////////////////////////////////////
// STM32F103 串口-舵机 透明转发桥
// PC(USB-TTL) <--USART1--> STM32 <--USART2半双工单线--> 舵机
// 电脑发送的命令原封不动通过单总线发给舵机
// 舵机的回应原封不动通过USB-TTL发给电脑
// 接线:
//   PC端:  PA9(TX), PA10(RX) 接USB-TTL模块
//   舵机端: PA2 接舵机单总线数据线
//////////////////////////////////////////////////////////////////////////////////

// 初始化USART1(PC端),自行实现以避开系统usart.c的帧检测
void PC_Uart_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    // PA9 TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA10 RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // NVIC
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART1
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

// USART1中断 — 接收PC数据,原封不动转发给舵机
void USART1_IRQHandler(void)
{
    u8 res;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        res = USART_ReceiveData(USART1); // 从PC接收一个字节
        Servo_SendByte(res);             // 原封不动转发给舵机
    }
}

int main(void)
{
    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    PC_Uart_Init(115200); // USART1: 与PC通信(PA9/PA10)
    Servo_Init(115200);   // USART2: 与舵机通信(PA2,半双工单线)

    printf("STM32 UART-Servo Bridge Ready\r\n");
    printf("PC <--USART1--> STM32 <--USART2--> Servo\r\n");

    // 空闲超时检测相关变量
    u16 last_servo_cnt = 0;      // 上一次舵机接收计数
    u16 idle_cnt = 0;            // 空闲计数(每100us累加一次)

    while (1)
    {
        u16 cur_cnt = SERVO_RX_STA & 0x3FFF;

        // 检测舵机是否有新数据
        if (cur_cnt > 0)
        {
            if (cur_cnt != last_servo_cnt)
            {
                // 有新数据到达,更新计数和计时
                last_servo_cnt = cur_cnt;
                idle_cnt = 0;
            }
            else
            {
                // 数据计数不变,累计空闲时间
                idle_cnt++;
                // 约3ms无新数据(30 * 100us),认为一帧接收完毕
                if (idle_cnt > 30)
                {
                    // 将舵机返回的数据原封不动转发给PC
                    for (u16 i = 0; i < cur_cnt; i++)
                    {
                        while ((USART1->SR & 0x40) == 0); // 等待USART1发送就绪
                        USART1->DR = SERVO_RX_BUF[i];
                    }
                    // 清空缓冲和状态
                    SERVO_RX_STA = 0;
                    last_servo_cnt = 0;
                    idle_cnt = 0;
                }
            }
        }
        else
        {
            last_servo_cnt = 0;
            idle_cnt = 0;
        }

        delay_us(100); // 提供约100us的时基用于空闲检测
    }
}
