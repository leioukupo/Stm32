#include "delay.h"
#include "stm32f10x.h"
#include "myusart.h"
#include "oled.h"
#include "../HARDWARE/MPU6050/mpu6050.h"
#include "../HARDWARE/Tim/timer2.h"
// Make Pitch, Roll, Yaw global floats so IRQ and main can access them
float Pitch, Roll, Yaw;
void TIM2_IRQHandler(void)
{
    // 检查更新中断
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        // 定时读取DMP数据，保证FIFO不溢出也不读空
        MPU6050_ReadDMP(&Pitch, &Roll, &Yaw);

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
int main(void)
{
    if (SysTick_Config(SystemCoreClock / 1000))
        while (1);
    // use global Pitch, Roll, Yaw
    // 初始化外设
    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    my_uart1_init();
    OLED_Init();
    MPU6050_DMPInit(); // 加载DMP固件，初始化MPU6050
    TIM2_Init(5000 - 1, 720 - 1);      // 开启定时器中断

    while (1) {
        // 刷新显示，数据由中断自动更新
        OLED_ShowString(0, 0, "MPU6050 DMP Test", 16);
        // OLED_SHowString(0, 16, "Pitch:%+06.1f", Pitch);
        // OLED_SHowString(0, 32, "Roll :%+06.1f", Roll);
        // OLED_SHowString(0, 48, "Yaw  :%+06.1f", Yaw);    
        OLED_Refresh();
    }
}
