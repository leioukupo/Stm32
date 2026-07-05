# Stm32

## 关于stm32f103c8t6最小系统板的学习
> PB3 PB4当普通io使用需要禁止jtag
> ```c
> GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
> ```

1. pwm输出
2. oled屏幕(spi协议)
3. 输入捕获
4. 串口
5. LCD液晶屏
6. Usmart----通过串口调试函数
7. ADC----通过串口输出adc1的通道1(PA1)采集到的电压转化的数字量
> stm32f103c8t6是中容量，没有dac
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

11. can通信
> 1. can通信的五种帧
* 数据帧---   标准格式(11位)和扩展格式(29位)&emsp;&emsp;&emsp;  发送单元----->接收单元
* 遥控帧(远程帧)---   标准格式(11位)和扩展格式(29位)&emsp;&emsp;&emsp;  接收单元(**向发送单元请求数据**)<-----具有相同 ID 的发送单元
* 错误帧---   错误时向其它单元通知错误的帧&emsp;&emsp;&emsp;&emsp;         错误----->  其他单元
* 过载帧---   接收单元通知其尚未做好接收准备的&emsp;&emsp;   接收单元没有做好准备----->其他单元
* 帧间隔---   数据帧及遥控帧与前面的帧分离开来的帧
> 数据帧      
> 数据帧由 7 个段构成。      
> 数据帧的构成如图 16 所示。      
> (1) 帧起始     
> 表示数据帧开始的段。  表示帧开始的段。1 个位的显性位。    
> (2) 仲裁段(**标准格式和扩展格式不同**)      
> 表示该帧优先级的段。   数据的优先级   
> (3) 控制段(**标准格式和扩展格式不同**)     
> 表示数据的字节数及保留位的段。    数据段的字节数   
> (4) 数据段    
> 数据的内容，可发送 0～8 个字节的数据。   包含 0～8 个字节的数据。从 MSB（最高位）开始输出     
> (5) CRC 段    
> 检查帧的传输错误的段。    检查帧传输错误,15 个位的 CRC 顺序 *1 和 1 个位的 CRC 界定符           
> (6) ACK 段   
> 表示确认正常接收的段。 确认是否正常接收, ACK 槽(ACK Slot)和 ACK 界定符 2 个位   
> (7) 帧结束
> 表示数据帧结束的段。     表示该该帧的结束,7 个位的隐性   
![alt text](image.png)

12. 红外遥控收发
> 复习系统时钟和各外设时钟

13. MPU6050DMP库移植
> mpu6050.c和.h是移植文件，其他的是官方原有文件
> 1.在inv_mpu.c包含我们的移植文件包括延时
> 2.添加两个宏定义，注释msp430相关的，添加自己的宏定义,通过自己的函数实现原来固定的宏函数
> MPU6050_Write/MPU6050_Read（对应I2C读写协议，在MPU6050.c中实现）

DMP只使用i2c_write i2c_read delay_ms get_ms这4个固定的函数
所以只要实现自己的关于这个4个函数就是适配了DMP(前提是准确无误，返回正确的返回值)
最正确的做法应该是定义一个MOTION_DRIVER_TARGET_STM32F103的宏
然后
#elif defined MOTION_DRIVER_TARGET_MSP430
#define delay_ms    mpu_delay_ms
#define get_ms      mget_ms
#define log_i    printf
#define log_e    printf
暂时借用MOTION_DRIVER_TARGET_MSP430这个宏

核心驱动层：

inv_mpu.c：MPU6050 的底层驱动逻辑（复位、配置寄存器等）。

inv_mpu.h：核心驱动的头文件。

DMP 固件层：

inv_mpu_dmp_motion_driver.c：负责加载 DMP 固件和处理 DMP 特有的操作。

inv_mpu_dmp_motion_driver.h：DMP 驱动的头文件。

固件数据文件（被 .c 文件包含，通常不需要直接添加到工程列表中，但必须在同一目录下）：

dmpKey.h：包含 DMP 的密钥数据。

dmpmap.h：包含 DMP 的内存映射数据。

总结，移植库就是找到抽象的函数，在自己的硬件上实现抽象函数的底层操作    以人和车为例子，现在有一个控制人行动的库，有前进函数后退函数，要把它移植到车，那就要实现当库调用前进函数的时候,车接收到指令也前进到指定伪装     有点类似与中转站   上层是各种ai应用，然后可以通过自己写的chat2api也可以用上层的各种ai应用