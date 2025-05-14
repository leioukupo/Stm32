#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "../HARDWARE/IIC/24cxx.h"
#include "../HARDWARE/IIC/iic.h"
const u8 TEXT_Buffer[]={"STM32 IIC TEST"};
#define SIZE sizeof(TEXT_Buffer)
int main(void)
 { 
    u8 t;
	u8 len;
	u8 datatemp[SIZE];
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	delay_init();	    	 //延时函数初始化	  
	uart_init(115200);	 	//串口初始化为9600		 	
	AT24CXX_Init();			//IIC初始化 
    printf("\r\nSTM32\r\n");
    printf("\r\nIIC TEST\r\n");
    printf("\r\nby leioukupo\r\n");	
 	while(AT24CXX_Check())//检测不到24c02
	{
		printf("\r\n24C02 Check Failed!\r\n");
        delay_ms(500);
		printf("\r\nPlease Check! \r\n");
		delay_ms(500);
	}
    printf("\r\n24C02 Ready!\r\n");  
	while(1)
	{
        if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
            printf("\r\nStart Write 24C02....\r\n");
            AT24CXX_Write(0,(u8*)USART_RX_BUF,sizeof(USART_RX_BUF));
            printf("\r\n24C02 Write Finished!\r\n");
            delay_ms(500);
            printf("\r\nStart Read 24C02....\r\n");
            AT24CXX_Read(0,datatemp,SIZE);
            printf("\r\nThe Data Readed Is:  \r\n");
			for(t=0;t<len;t++)
			{
				USART1->DR=datatemp[t];
				while((USART1->SR&0X40)==0);//等待发送结束
			}
			printf("\r\n\r\n");//插入换行
			USART_RX_STA=0;
		}
	}
}