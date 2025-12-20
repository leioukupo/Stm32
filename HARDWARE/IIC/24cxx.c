#include "24cxx.h"
#include "delay.h"

u8 AT24CXX_ADDR = 0xA0; //默认地址，运行时会被检测到的实际地址覆盖
//初始化IIC接口
void AT24CXX_Init(void)
{
	IIC_Init();
}
//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址
//返回值  :读到的数据
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{
	u8 temp=0;
    IIC_Start();
	//使用实际检测到的器件地址
	IIC_Send_Byte(AT24CXX_ADDR);	   //发送写命令
	if(IIC_Wait_Ack())
	{
	 printf("ACK failed after device address\r\n");
	 IIC_Stop();
	 return 0xFF;
	}
    IIC_Send_Byte(ReadAddr);   //发送地址（24c02是8位地址）
	if(IIC_Wait_Ack())
	{
	 printf("ACK failed after address\r\n");
	 IIC_Stop();
	 return 0xFF;
	}
	IIC_Start();
	IIC_Send_Byte(AT24CXX_ADDR+1);           //进入接收模式（读地址=写地址+1）
	if(IIC_Wait_Ack())
	{
	 printf("ACK failed after read command\r\n");
	 IIC_Stop();
	 return 0xFF;
	}
    temp=IIC_Read_Byte(0);
    IIC_Stop();//产生一个停止条件
	return temp;
}
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址
//DataToWrite:要写入的数据
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{
    IIC_Start();
	//使用实际检测到的器件地址
	IIC_Send_Byte(AT24CXX_ADDR);	    //发送写命令
	IIC_Wait_Ack();
    IIC_Send_Byte(WriteAddr);   //发送地址（24c02是8位地址）
	IIC_Wait_Ack();
	IIC_Send_Byte(DataToWrite);     //发送字节
	IIC_Wait_Ack();
    IIC_Stop();//产生一个停止条件
	delay_ms(10);	 //写入后需要等待写入完成
}
//在AT24CXX里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//在AT24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len)
{  	
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}
//检查AT24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
u8 AT24CXX_Check(void)
{
	u8 temp;
	//尝试多个可能的器件地址
	u8 addresses[] = {0xA0, 0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC, 0xAE};
	u8 addr_count = sizeof(addresses)/sizeof(addresses[0]);
	u8 found_device = 0;

	for(int i=0; i<addr_count; i++)
	{
		IIC_Start();
		IIC_Send_Byte(addresses[i]);
		if(IIC_Wait_Ack() == 0)
		{
		 printf("Found device at address 0x%02X\r\n", addresses[i]);
		 AT24CXX_ADDR = addresses[i]; //保存检测到的地址
		 found_device = 1;
		 IIC_Stop();
		 break;
		}
		IIC_Stop();
	}

	if(!found_device)
	{
	 printf("No device found on I2C bus!\r\n");
	 return 1;
	}

	temp=AT24CXX_ReadOneByte(255);//避免每次开机都写AT24CXX
	printf("Read value: 0x%02X\r\n", temp);
	if(temp==0X55)return 0;
	else//排除第一次初始化的情况
	{
		AT24CXX_WriteOneByte(255,0X55);
		delay_ms(100); //增加等待时间
	    temp=AT24CXX_ReadOneByte(255);
		printf("After write, read value: 0x%02X\r\n", temp);
		if(temp==0X55)return 0;
	}
	return 1;
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//在AT24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}

//简单的IIC总线测试
void IIC_Bus_Test(void)
{
 printf("\r\n=== IIC Bus Diagnostic ===\r\n");

 //测试1：检查上拉电阻
 printf("Test 1: Pull-up resistor check\r\n");
 IIC_Start();
 IIC_SDA=1;
 IIC_SCL=1;
 delay_us(10);

 SDA_IN();
 u8 sda_status = READ_SDA;
 IIC_SCL=0;

 if(sda_status)
 {
   printf("  SDA line is HIGH - pull-up resistor OK\r\n");
 }
 else
 {
   printf("  SDA line is LOW - NO pull-up resistor or short circuit!\r\n");
   printf("  SOLUTION: Add 4.7k resistor from SDA to VCC\r\n");
 }

 //测试2：检查时钟线
 SCL_OUT();
 IIC_SCL=0;
 delay_us(10);
 IIC_SCL=1;
 printf("  SCL line toggled successfully\r\n");

 //测试3：尝试设备扫描
 printf("\r\nTest 2: Device scanning\r\n");
 u8 found = 0;
 for(u8 addr = 0xA0; addr <= 0xAE; addr+=2)
 {
   IIC_Start();
   IIC_Send_Byte(addr);
   if(IIC_Wait_Ack() == 0)
   {
     printf("  Device found at address 0x%02X\r\n", addr);
     found = 1;
     AT24CXX_ADDR = addr; //保存地址
     IIC_Stop();
     break;
   }
   IIC_Stop();
 }

 if(!found)
 {
   printf("  No device responded!\r\n");
   printf("  POSSIBLE CAUSES:\r\n");
   printf("  1. Missing pull-up resistors on SDA and SCL\r\n");
   printf("  2. 24C02 not powered (check VCC and GND)\r\n");
   printf("  3. Write Protect pin is HIGH (connect to GND)\r\n");
   printf("  4. Wiring error (PC13=SCL, PC14=SDA)\r\n");
 }

 IIC_Stop();
 printf("=== End Diagnostic ===\r\n\r\n");
} 