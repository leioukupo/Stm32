#include "Ds18b20.h"
#include "delay.h"
#include "usart.h"
int main(void)
{
    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    uart_init(115200);
    if (Ds18b20_Init() == 0)
    {
        printf("Init success\n");
    }
    else
    {
        printf("Init failed\n");
    }
    while (1)
    {
        unsigned int temp = ReadTemperature();
        if(temp<0)
			{
				printf("Temperature: - %d.%d\n", temp / 10, temp % 10);					//转为正数
			}
        else printf("Temperature: %d.%d\n", temp / 10, temp % 10);
        delay_ms(100);
    }
}
