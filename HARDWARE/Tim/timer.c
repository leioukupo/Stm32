#include"timer.h"
#include"led.h"
// TODO:占空比
// 高电平占周期的比例

void TIM3_Init(u16 arr, u16 psc){
    LED_Init();
    TIM_TimeBaseInitTypeDef a;
    NVIC_InitTypeDef b;
    a.TIM_Period = arr; //自动重装载值
    a.TIM_Prescaler = psc;  // 预分频系数
    //溢出时间= (ARR+1)*(PSC+1) / TCLK
    //                  ^^^^^^^^^^^^^--->一个周期时间长度 72mhz     72m/(7199+1)=10mhz
    a.TIM_CounterMode = TIM_CounterMode_Up;//计数模式
    a.TIM_ClockDivision = TIM_CKD_DIV1;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//使能时钟
    TIM_TimeBaseInit(TIM3, &a); //定时器参数初始化
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//中断使能
    b.NVIC_IRQChannel = TIM3_IRQn; //TIM_3中断
    b.NVIC_IRQChannelPreemptionPriority = 0;//先占优先级0级
    b.NVIC_IRQChannelSubPriority = 3;//从优先级3级
    b.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
    NVIC_Init(&b);//中断优先级分组设置
    TIM_Cmd(TIM3,ENABLE); //定时器使能
}
// TIM3_IRQHandler 已移至 src/stm32f10x_it.c (1ms 时基 -> system_ms)
void TIM3_PWM_Init(u16 arr, u16 psc){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
    TIM_TimeBaseInitTypeDef a;
    TIM_OCInitTypeDef tim;
    a.TIM_Period = arr;//设置自动重装载的周期数
    a.TIM_Prescaler = psc;//预分频系数
    a.TIM_CounterMode = TIM_CounterMode_Up;
    a.TIM_ClockDivision = TIM_CKD_DIV1;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseInit(TIM3, &a);
    tim.TIM_OCMode=TIM_OCMode_PWM2;
    tim.TIM_OCPolarity=TIM_OCPolarity_High;
    tim.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OC2Init(TIM3, &tim);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_Cmd(TIM3, ENABLE);
}
TIM_ICInitTypeDef TIM2_Cap;
void TIM2_Cap_Init(u16 arr, u16 psc){
    GPIO_InitTypeDef b;
    TIM_TimeBaseInitTypeDef tim_base;
    NVIC_InitTypeDef nvic;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    b.GPIO_Pin = GPIO_Pin_0;
    b.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &b);
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);//PA0下拉

    tim_base.TIM_Period = arr;
    tim_base.TIM_Prescaler = psc;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim_base);
    TIM2_Cap.TIM_Channel = TIM_Channel_1;
    TIM2_Cap.TIM_ICPolarity = TIM_ICPolarity_Rising;//上升沿捕获
    TIM2_Cap.TIM_ICSelection = TIM_ICSelection_DirectTI;//直接映射
    TIM2_Cap.TIM_ICPrescaler = TIM_ICPSC_DIV1;//配置输入分频    不分频
    TIM2_Cap.TIM_ICFilter = 0x00;//配置输入滤波器    不滤波
    TIM_ICInit(TIM2, &TIM2_Cap);
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}
u8 TIM2ch1_cap_sta = 0;//输入捕获状态
u16 TIM2ch1_cap_val; //输入捕获值
void TIM2_IRQHandler(void){
    if ((TIM2ch1_cap_sta&0x80)==0)//还没有成功捕获
    {
        if (TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
        {
            if (TIM2ch1_cap_sta&0x40)//已经捕获到高电平了
            {
                if ((TIM2ch1_cap_sta&0x3F)==0x3F)//高电平太长了
                {
                    TIM2ch1_cap_sta |= 0x80;//强制标记成功捕获了一次
                    TIM2ch1_cap_val++;
                }else
                {
                    TIM2ch1_cap_sta++;//计数器溢出次数加一次
                }
            }
            
        }
        
    }
    if (TIM_GetITStatus(TIM2,TIM_IT_CC1)!=RESET)//捕获1发生捕获时间
    {
        if (TIM2ch1_cap_sta&0x40)//捕获到一个下降沿
        {
            TIM2ch1_cap_sta |= 0x80; //标记成功捕获到一次下降沿
            TIM2ch1_cap_val = TIM_GetCapture1(TIM2);
            TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Rising); //CC1P=0  设置为上升沿捕获
        }else  // 还没有开始  第一次捕获上升沿
        {
            TIM2ch1_cap_sta = 0; //清空
            TIM2ch1_cap_val = 0;
            TIM_SetCounter(TIM2, 0);
            TIM2ch1_cap_sta |= 0x40;//标记捕获到了上升沿
            TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Falling);//cc1p=1   设置为下降沿捕获
        }       
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1 | TIM_IT_Update);//清楚中断标志位
}
