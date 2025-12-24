# Stm32

## 关于stm32f103c8t6最小系统板的学习

1. pwm输出
2. oled屏幕(spi协议)
3. 输入捕获
4. 串口
5. LCD液晶屏
6. Usmart----通过串口调试函数
7. ADC----通过串口输出adc1的通道1(PA1)采集到的电压转化的数字量
8. DS18B20----通过DS18B20读取温度，在串口中打印
9. DMA进行串口一发送
> 通过 dma把数据直接送到串口一的dr寄存器   
> 中文要gbk编码
10. iic协议，与24c02通信
> 使用Pc13 ---> SCL    
> 使用Pc14 ---> SDA           
> 注意24c0地址，是0xAE  

11. spi 0.96pled屏幕       

| 引脚 | 功能 | Remap  
| :---: | :---: | :---: |              
| PA4 | SPI1_NSS | PA15 |     
| PA5 | SPI1_SCK | PB3 |      
| PA6 | SPI1_MISO | PB4 |       
| PA7 | SPI1_MOSI | PB5 | 
| PB12 | SPI2_NSS | / |
| PB13 | SPI2_SCK | / | 
| PB14 | SPI2_MISO | / |
| PB15 | SPI2_MOSI | / | 

SPI屏幕接线           
D0(SCL)-->    PA5 | SPI1_SCK | PB3 |              
D1(sda) -->   PA7 | SPI1_MOSI | PB5 |        
CS      -->   PA4 | SPI1_NSS | PA15 |       
RES(复位,任选)   ---> PA3
DC(数据和命令选择)  ---> PA2

初始化注意gpio 号
> stm32f103c8t6是中容量，没有dac