#include "delay.h"
#include "sys.h"
#include "usart.h"
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
int main(void){
    u8 t;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    uart_init(115200);
    delay_init();
    printf("oled spi init\n");
    OLED_SPI_Init();
    OLED_SPI_ColorTurn(0);
    OLED_SPI_DisplayTurn(0);
    OLED_SPI_Refresh();
    t = ' ';
    while (1)
    {
        OLED_SPI_ShowPicture(0,0,128,8,BMP1);
        delay_ms(500);
        OLED_SPI_Clear();
        OLED_SPI_ShowChinese(0,0,0,16);
        OLED_SPI_ShowChinese(18,0,1,16);
        OLED_SPI_ShowChinese(36,0,2,16);
        OLED_SPI_ShowChinese(54,0,3,16);
        OLED_SPI_ShowChinese(72,0,4,16);
        OLED_SPI_ShowChinese(90,0,5,16);
        OLED_SPI_ShowChinese(108,0,6,16);
        OLED_SPI_ShowString(8,16,"ZHONGJINGYUAN",16);
        OLED_SPI_ShowString(20,32,"2025/12/23",16);
        OLED_SPI_ShowString(0,48,"ASCII:",16);
        OLED_SPI_ShowString(63,48,"CODE:",16);
        OLED_SPI_ShowChar(48,48,t,16);
        t++;
        if (t>'~')
        {
            t = ' ';
        }
        OLED_SPI_ShowNum(103,48,t,3,16);
        OLED_SPI_Refresh();
        delay_ms(500);
        OLED_SPI_Clear();
        OLED_SPI_ShowChinese(0,0,0,16);//显示不同大小的中
        OLED_SPI_ShowChinese(16,0,0,24);
        OLED_SPI_ShowChinese(24,20,0,32);
        OLED_SPI_ShowChinese(64,0,0,64);
        OLED_SPI_Refresh();
        delay_ms(500);
        OLED_SPI_Clear();
        OLED_SPI_ShowString(0,0,"ABC",12);
        OLED_SPI_ShowString(0,12,"ABC",16);
        OLED_SPI_ShowString(0,28,"ABC",24);
        OLED_SPI_Refresh();
        delay_ms(500);
        OLED_SPI_ScrollDisplay(11,4);
    }
    
}