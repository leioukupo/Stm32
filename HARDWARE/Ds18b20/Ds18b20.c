#include "Ds18b20.h"
#include "delay.h"
// data_port PC15
void Data_Port_IN(void) // 让PC15为浮空输入模式
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Data_Port_OUT(void) // 让PC15为推挽输出模式
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOC, &GPIO_InitStruct);
}

int Ds18b20_Init(void)
{
    Data_Port_OUT();
    int data = 0;
    GPIO_SetBits(GPIOC, GPIO_Pin_15);   // 复位 高电平
    delay_us(10);                       // 稍做延时
    GPIO_ResetBits(GPIOC, GPIO_Pin_15); // 将DQ拉低
    delay_us(530);                      // 精确延时 大于 480us 小于960us
    GPIO_SetBits(GPIOC, GPIO_Pin_15);   // 拉高总线
    delay_us(50);                       // 15~60us 后 接收60-240us的存在脉冲
    Data_Port_IN();
    delay_us(250);
    data = GPIO_ReadInputDataBit(GPIOC, 15); // 如果0则初始化成功, 1则初始化失败
    delay_us(25);                            // 稍作延时返回
    return data;
}

void Write_Bit(int data)
{
    if (data == 0)
    {
        GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET);
        delay_us(70);
        GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET);
        delay_us(15);
    }
    else
    {
        GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET);
        delay_us(9);
        GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET);
        delay_us(70);
    }
}
unsigned char Read_Bit(void)
{
    Data_Port_OUT();
    GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET);
    delay_us(12);
    Data_Port_IN();
    delay_us(10);
    if (GPIO_ReadInputDataBit(GPIOC, 15) == Bit_SET)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
// 写入一个字节
void WriteOneChar(unsigned char data)
{
    unsigned char i = 0;
    for (i = 8; i > 0; i--)
    {
        GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET);
        Write_Bit(data & 0x01); // 0x01 = 0000 0001  data&0x01与完后就是写入data的最低位
        delay_us(25);
        GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET);
        data >>= 1; // 右移一位
    }
    delay_us(30);
}
unsigned char ReadOneChar(void)
{
    unsigned char i = 0;
    unsigned char data = 0;
    for (i = 8; i > 0; i--)
    {
        GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET); 
        data >>= 1;
        GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET);
        if (GPIO_ReadInputDataBit(GPIOC, 15)){
            data |= 0x80; // 0x80 = 1000 0000 第八位置1，也就是第八位接受到的数据是1
        }
        delay_us(30);
    }
    return (data);
}

unsigned int ReadTemperature(void){
    unsigned char a=0;
    unsigned int b=0;
    unsigned int t=0;
    Ds18b20_Init();
    WriteOneChar(0xCC); // 跳过读序号列号的操作
    WriteOneChar(0x44); // 启动温度转换
    delay_ms(10);
    Ds18b20_Init();
    WriteOneChar(0xCC); //跳过读序号列号的操作
    WriteOneChar(0xBE); //读取温度寄存器等（共可读9个寄存器） 前两个就是温度
    a = ReadOneChar(); //低位
    b = ReadOneChar(); //高位
    b <<= 8;
    t = a + b;
    t = (double)t*0.625;//转换 
    if(temp)return tem; //返回温度值
	else return -tem;
}






