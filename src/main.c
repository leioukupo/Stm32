#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"
#include "../HARDWARE/Dma/Dma.h"
const u8 TEXT_TO_SEND[]={"LY STM32 DMA 串口实验"};
#define TEXT_LENTH  sizeof(TEXT_TO_SEND)-1			//TEXT_TO_SEND字符串长度(不包含结束符)
u8 SendBuff[(TEXT_LENTH+2)*100];
int main(void)
{
    u16 i;
	u8 t=0; 
	float pro=0;			//进度 
	delay_init();	    	 //延时函数初始化	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    uart_init(115200); 	//串口初始化为115200
    MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)SendBuff,(TEXT_LENTH+2)*100);//DMA1通道4,外设为串口1,存储器为SendBuff,长(TEXT_LENTH+2)*100.
    for(i=0;i<(TEXT_LENTH+2)*100;i++)//填充ASCII字符集数据
    {
		if(t>=TEXT_LENTH)//加入换行符
		{ 
			SendBuff[i++]=0x0d; 
			SendBuff[i]=0x0a; 
			t=0;
		}
        else SendBuff[i]=TEXT_TO_SEND[t++];//复制TEXT_TO_SEND语句    
    }
    i=0;
	while(1)
	{
        printf("\r\nDMA DATA:\r\n "); 	    
        USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);         
        MYDMA_Enable(DMA1_Channel4);//开始一次DMA传输！	  
        //等待DMA传输完成，此时我们来做另外一些事，点灯
        //实际应用中，传输数据期间，可以执行另外的任务
        while(1)
        {
            if(DMA_GetFlagStatus(DMA1_FLAG_TC4)!=RESET)//等待通道4传输完成
            {
                DMA_ClearFlag(DMA1_FLAG_TC4);//清除通道4传输完成标志
                break; 
            }
            pro=DMA_GetCurrDataCounter(DMA1_Channel4);//得到当前还剩余多少个数据
            pro=1-pro/((TEXT_LENTH+2)*100);//得到百分比	  
            pro*=100;      //扩大100倍  
        }			    
		i++;
		delay_ms(10);		   
	}
}		