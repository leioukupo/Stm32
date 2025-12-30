#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "../HARDWARE/Remote/remote.h"
#include "../HARDWARE/Oled/oled.h"
#include "../HARDWARE/Oled/bmp.h"
/*
硬件 SPI接线
D0(SCL)-->    PA5 | SPI1_SCK | PB3 |              
D1(sda) -->   PA7 | SPI1_MOSI | PB5 |        
CS      -->   PA4 | SPI1_NSS | PA15 |       
RES(复位,任选)   ---> PA3
DC(数据和命令选择)  ---> PA2
*/
/*
红外遥控数据脚  PA8
*/
int main(void){
    u8 key;
    u8 *str=0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    uart_init(115200);
    delay_init();
    Remote_Init();
    OLED_SPI_Init();
    OLED_SPI_ColorTurn(0);
    OLED_SPI_DisplayTurn(0);
    OLED_SPI_Refresh();
    while (1)
    {
        key=Remote_Scan();
        OLED_SPI_ShowString(8,0,"STM32F1",12);
        OLED_SPI_ShowString(8,12,"REMOTE TEST",12);
        OLED_SPI_ShowString(8,24,"KEYVAL:",12);
        OLED_SPI_ShowString(8,36,"KEYCNT:",12);
        OLED_SPI_ShowString(8,48,"SYMBOL:",12);	
        if(key)
		{	OLED_SPI_ShowNum(80,24,key,1,12); //显示键值
            OLED_SPI_ShowNum(80,36,RmtCnt,3,12);//显示按键次数
			switch(key)
			{
				case 0:str="ERROR";break;
				case 162:str="POWER";break;
				case 98:str="UP";break;
				case 2:str="PLAY";break;
				case 226:str="ALIENTEK";break;
				case 194:str="RIGHT";break;
				case 34:str="LEFT";break;
				case 224:str="VOL-";break;
				case 168:str="DOWN";break;
				case 144:str="VOL+";break;
				case 104:str="1";break;
				case 152:str="2";break;
				case 176:str="3";break;
				case 48:str="4";break;
				case 24:str="5";break;
				case 122:str="6";break;
				case 16:str="7";break;
				case 56:str="8";break;
				case 90:str="9";break;
				case 66:str="0";break;
				case 82:str="DELETE";break;
			}
            OLED_SPI_ShowString(80,48,str,12);//显示SYMBOL
		}else delay_ms(10);
        OLED_SPI_Refresh(); //刷新显示	  
    }
    
}