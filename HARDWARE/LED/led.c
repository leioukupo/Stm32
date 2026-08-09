#include "led.h"
#include "stm32f10x.h"

//////////////////////////////////////////////////////////////////////////////////
// 心跳/状态指示灯 (Phase 6)
// 原 PB8 已被 TIM4 CH3 PWM(麦轮)占用, 心跳灯改 PA0 (计划 §2)
//////////////////////////////////////////////////////////////////////////////////

void LED_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);   // 默认灭
}

void LED_Set(u8 on)
{
    if (on)
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
    else
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
}

void LED_Flip(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_0,
                  (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0)));
}
