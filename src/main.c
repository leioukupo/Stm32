#include "Ds18b20.h"
#include "delay.h"
#include "usart.h"
#include  "adc.h"
#include "key.h"
#include "../HARDWARE/Dac/dac.h"

int main(void)
{
    u16 adcx;
    float temp;
    u8 t       = 0;
    u16 dacval = 0;
    u8 key;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置中断优先级分组2
    delay_init();                                   // 延时函数初始化
    uart_init(115200);                                // 串口初始化为9600
    Adc_Init();                                     // ADC初始化
    Dac1_init();
    while (1)
    {
        t++;
        key = KEY_Scan(0);
        if (key == WKUP_PRES)
        {
            if(dacval<4000)dacval+=200;
			DAC_SetChannel1Data(DAC_Align_12b_R, dacval);//加DAC值  设置DAC通道值
        }
        else if (key==KEY0_PRES)
        {
            if(dacval>200)dacval-=200;
			else dacval=0;
			DAC_SetChannel1Data(DAC_Align_12b_R, dacval);//减DAC值  设置DAC通道值
        }
        if(t==10||key==KEY0_PRES||key==WKUP_PRES) 	//WKUP/KEY1按下了,或者定时时间到了
		{	  
 			adcx=DAC_GetDataOutputValue(DAC_Channel_1);// dac的输出值
            printf("Dac value : %d\n", adcx); //显示DAC寄存器值
			temp=(float)adcx*(3.3/4096);			//得到DAC电压值
			adcx=temp;
            printf("Dac Voltage value integer : %d\n", adcx); 	//显示电压值整数部分
 			temp-=adcx;
			temp*=1000;
            printf("Dac Voltage value decimal: %d\n", temp); 	//显示电压值的小数部分
 			adcx=Get_Adc_Average(ADC_Channel_1,10);		//得到ADC转换值	  
			temp=(float)adcx*(3.3/4096);			//得到ADC电压值
			adcx=temp;
            printf("Adc Voltage value integer : %d\n", adcx); 	//显示电压值整数部分
 			temp-=adcx;
			temp*=1000;
			printf("Adc Voltage value decimal: %d\n", temp); 	//显示电压值的小数部分 
			t=0;
		}	    
		delay_ms(10);
    }
}