#include "tb6612.h"
#include "pwm.h"

//////////////////////////////////////////////////////////////////////////////////
// TB6612 双电机驱动实现 (Phase 3)
//////////////////////////////////////////////////////////////////////////////////

// 每轮硬件映射 (PWM 通道 + 方向脚 + 所在板 STBY)
typedef struct {
    GPIO_TypeDef *ain1_port;
    u16           ain1_pin;
    GPIO_TypeDef *ain2_port;
    u16           ain2_pin;
    GPIO_TypeDef *stby_port;
    u16           stby_pin;
    u8            pwm_ch;        // TIM4 CH1..4
} tb6612_wheel_t;

static const tb6612_wheel_t s_wheels[TB6612_WHEELS] = {
    // FL: 左板 A
    { GPIOB, GPIO_Pin_12, GPIOB, GPIO_Pin_13, GPIOA, GPIO_Pin_1, 1 },
    // FR: 右板 A
    { GPIOA, GPIO_Pin_4,  GPIOA, GPIO_Pin_5,  GPIOA, GPIO_Pin_3, 3 },
    // RL: 左板 B
    { GPIOB, GPIO_Pin_14, GPIOB, GPIO_Pin_15, GPIOA, GPIO_Pin_1, 2 },
    // RR: 右板 B
    { GPIOA, GPIO_Pin_6,  GPIOA, GPIO_Pin_7,  GPIOA, GPIO_Pin_3, 4 },
};

static void tb6612_pin_init(GPIO_TypeDef *port, u16 pins)
{
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin = pins;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &gpio);
}

void tb6612_init(void)
{
    u8 i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    // 方向脚: 左板 PB12~PB15, 右板 PA4~PA7
    tb6612_pin_init(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    tb6612_pin_init(GPIOA, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);

    // STBY: PA1(左板)/PA3(右板), 上电拉高使能
    tb6612_pin_init(GPIOA, GPIO_Pin_1 | GPIO_Pin_3);
    GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_3);

    // PWM: TIM4 4 通道, 10kHz
    TIM4_PWM_Init_4CH(100 - 1, 72 - 1);

    for (i = 0; i < TB6612_WHEELS; i++)
        tb6612_set_wheel(i, 0);
}

int tb6612_set_wheel(u8 idx, int duty)
{
    const tb6612_wheel_t *w;
    u16 ccr;

    if (idx >= TB6612_WHEELS)
        return -1;
    if (duty > 100)
        duty = 100;
    if (duty < -100)
        duty = -100;

    w = &s_wheels[idx];
    if (duty > 0) {
        GPIO_SetBits(w->ain1_port, w->ain1_pin);
        GPIO_ResetBits(w->ain2_port, w->ain2_pin);
        ccr = (u16)duty;
    } else if (duty < 0) {
        GPIO_ResetBits(w->ain1_port, w->ain1_pin);
        GPIO_SetBits(w->ain2_port, w->ain2_pin);
        ccr = (u16)(-duty);
    } else {
        GPIO_ResetBits(w->ain1_port, w->ain1_pin);
        GPIO_ResetBits(w->ain2_port, w->ain2_pin);
        ccr = 0;
    }
    TIM4_SetPWM(w->pwm_ch, ccr);
    return 0;
}

void tb6612_set_wheels(const int duty[4])
{
    u8 i;

    if (duty == 0)
        return;
    for (i = 0; i < TB6612_WHEELS; i++)
        tb6612_set_wheel(i, duty[i]);
}

void tb6612_stop(void)
{
    u8 i;

    for (i = 0; i < TB6612_WHEELS; i++)
        tb6612_set_wheel(i, 0);
}
