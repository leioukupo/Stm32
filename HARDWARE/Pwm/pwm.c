#include "pwm.h"

//////////////////////////////////////////////////////////////////////////////////
// TIM4 4 路 PWM -> 2× TB6612 (计划 §2, Phase 3)
//
//   TIM4 CH1 -> PB6 -> 左板 PWMA (FL)
//   TIM4 CH2 -> PB7 -> 左板 PWMB (RL)
//   TIM4 CH3 -> PB8 -> 右板 PWMA (FR)
//   TIM4 CH4 -> PB9 -> 右板 PWMB (RR)
//
// 默认 10kHz: 72MHz / (psc+1) / (arr+1) = 72M / 72 / 100 = 10kHz,
// CCR 0~100 对应 0~100% 占空比 (CCR=100 时 CNT 恒小于 CCR, 输出 100%).
// 注: 已移除原 PB8 LED 翻转逻辑; 心跳灯改 PA0 (Phase 6 实现).
//////////////////////////////////////////////////////////////////////////////////

void TIM4_PWM_Init_4CH(u16 arr, u16 psc)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;
    u8 ch;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // PB6~PB9 全部配置为复用推挽
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    tim.TIM_Period = arr;               // 计数 0~arr (默认 99)
    tim.TIM_Prescaler = psc;            // 默认 71
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &tim);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;                   // 初始 0% 占空比

    TIM_OC1Init(TIM4, &oc);
    TIM_OC2Init(TIM4, &oc);
    TIM_OC3Init(TIM4, &oc);
    TIM_OC4Init(TIM4, &oc);

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // 全部通道初始输出 0
    for (ch = 1; ch <= 4; ch++)
        TIM4_SetPWM(ch, 0);

    TIM_Cmd(TIM4, ENABLE);
}

// 设置某一路 PWM 占空比, ch=1..4, ccr=0..100 (对应 0~100%)
void TIM4_SetPWM(u8 ch, u16 ccr)
{
    switch (ch) {
    case 1: TIM_SetCompare1(TIM4, ccr); break;
    case 2: TIM_SetCompare2(TIM4, ccr); break;
    case 3: TIM_SetCompare3(TIM4, ccr); break;
    case 4: TIM_SetCompare4(TIM4, ccr); break;
    default: break;
    }
}
