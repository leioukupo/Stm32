#ifndef __MPU6050_H
#define __MPU6050_H

int MPU6050_Write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data);
int MPU6050_Read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data);
int mpu_delay_ms(unsigned long num_ms);
int mget_ms(unsigned long *count);

unsigned short inv_row_2_scale(const signed char *row);
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx); //自己的函数声明  反转矩阵
int MPU6050_DMPInit(void);
int MPU6500_run_self_test(void); //自我检测
uint8_t MPU6050_ReadDMP(float *Pitch, float *Roll, float *Yaw);
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_GetDataEx(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                       int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ,
                       int16_t *Temp);

// DMP FIFO 读取时填充的全局原始数据 (raw LSB)
extern short gyro[3];
extern short accel[3];

// 寄存器定义
#define MPU6050_ADDRESS 0xD0 //MPU6050器件地址
#define DEFAULT_MPU_HZ  (100) //默认采样率
#define q30 1073741824.0f //q30格式  long转float的除数
/* mpu6050寄存器地址定义 */
#define MPU6050_ACCEL_OFFS      0X06    // accel_offs寄存器,可读取版本号,寄存器手册未提到
#define MPU6050_PROD_ID         0X0C    // prod id寄存器,在寄存器手册未提到
#define MPU6050_SELF_TESTX      0X0D    // 自检寄存器X
#define MPU6050_SELF_TESTY      0X0E    // 自检寄存器Y
#define MPU6050_SELF_TESTZ      0X0F    // 自检寄存器Z
#define MPU6050_SELF_TESTA      0X10    // 自检寄存器A
#define MPU6050_SAMPLE_DIV      0X19    // 采样频率分频器
#define MPU6050_CFG             0X1A    // 配置寄存器
#define MPU6050_GYRO_CFG        0X1B    // 陀螺仪配置寄存器
#define MPU6050_ACCEL_CFG       0X1C    // 加速度计配置寄存器
#define MPU6050_MOTION_DET      0X1F    // 运动检测阀值设置寄存器
#define MPU6050_FIFO_EN         0X23    // FIFO使能寄存器
#define MPU6050_I2CMST_CTRL     0X24    // IIC主机控制寄存器
#define MPU6050_I2CSLV0_ADDR    0X25    // IIC从机0器件地址寄存器
#define MPU6050_I2CSLV0         0X26    // IIC从机0数据地址寄存器
#define MPU6050_I2CSLV0_CTRL    0X27    // IIC从机0控制寄存器
#define MPU6050_I2CSLV1_ADDR    0X28    // IIC从机1器件地址寄存器
#define MPU6050_I2CSLV1         0X29    // IIC从机1数据地址寄存器
#define MPU6050_I2CSLV1_CTRL    0X2A    // IIC从机1控制寄存器
#define MPU6050_I2CSLV2_ADDR    0X2B    // IIC从机2器件地址寄存器
#define MPU6050_I2CSLV2         0X2C    // IIC从机2数据地址寄存器
#define MPU6050_I2CSLV2_CTRL    0X2D    // IIC从机2控制寄存器
#define MPU6050_I2CSLV3_ADDR    0X2E    // IIC从机3器件地址寄存器
#define MPU6050_I2CSLV3         0X2F    // IIC从机3数据地址寄存器
#define MPU6050_I2CSLV3_CTRL    0X30    // IIC从机3控制寄存器
#define MPU6050_I2CSLV4_ADDR    0X31    // IIC从机4器件地址寄存器
#define MPU6050_I2CSLV4         0X32    // IIC从机4数据地址寄存器
#define MPU6050_I2CSLV4_DO      0X33    // IIC从机4写数据寄存器
#define MPU6050_I2CSLV4_CTRL    0X34    // IIC从机4控制寄存器
#define MPU6050_I2CSLV4_DI      0X35    // IIC从机4读数据寄存器
#define MPU6050_I2CMST_STA      0X36    // IIC主机状态寄存器
#define MPU6050_INTBP_CFG       0X37    // 中断/旁路设置寄存器
#define MPU6050_INT_EN          0X38    // 中断使能寄存器
#define MPU6050_INT_STA         0X3A    // 中断状态寄存器
#define MPU6050_ACCEL_XOUTH     0X3B    // 加速度值,X轴高8位寄存器
#define MPU6050_ACCEL_XOUTL     0X3C    // 加速度值,X轴低8位寄存器
#define MPU6050_ACCEL_YOUTH     0X3D    // 加速度值,Y轴高8位寄存器
#define MPU6050_ACCEL_YOUTL     0X3E    // 加速度值,Y轴低8位寄存器
#define MPU6050_ACCEL_ZOUTH     0X3F    // 加速度值,Z轴高8位寄存器
#define MPU6050_ACCEL_ZOUTL     0X40    // 加速度值,Z轴低8位寄存器
#define MPU6050_TEMP_OUTH       0X41    // 温度值高八位寄存器
#define MPU6050_TEMP_OUTL       0X42    // 温度值低8位寄存器
#define MPU6050_GYRO_XOUTH      0X43    // 陀螺仪值,X轴高8位寄存器
#define MPU6050_GYRO_XOUTL      0X44    // 陀螺仪值,X轴低8位寄存器
#define MPU6050_GYRO_YOUTH      0X45    // 陀螺仪值,Y轴高8位寄存器
#define MPU6050_GYRO_YOUTL      0X46    // 陀螺仪值,Y轴低8位寄存器
#define MPU6050_GYRO_ZOUTH      0X47    // 陀螺仪值,Z轴高8位寄存器
#define MPU6050_GYRO_ZOUTL      0X48    // 陀螺仪值,Z轴低8位寄存器
#define MPU6050_I2CSLV0_DO      0X63    // IIC从机0数据寄存器
#define MPU6050_I2CSLV1_DO      0X64    // IIC从机1数据寄存器
#define MPU6050_I2CSLV2_DO      0X65    // IIC从机2数据寄存器
#define MPU6050_I2CSLV3_DO      0X66    // IIC从机3数据寄存器
#define MPU6050_I2CMST_DELAY    0X67    // IIC主机延时管理寄存器
#define MPU6050_SIGPATH_RST     0X68    // 信号通道复位寄存器
#define MPU6050_MDETECT_CTRL    0X69    // 运动检测控制寄存器
#define MPU6050_USER_CTRL       0X6A    // 用户控制寄存器
#define MPU6050_PWR_MGMT1       0X6B    // 电源管理寄存器1
#define MPU6050_PWR_MGMT2       0X6C    // 电源管理寄存器2 
#define MPU6050_FIFO_CNTH       0X72    // FIFO计数寄存器高八位
#define MPU6050_FIFO_CNTL       0X73    // FIFO计数寄存器低八位
#define MPU6050_FIFO_RW         0X74    // FIFO读写寄存器
#define MPU6050_DEVICE_ID       0X75    // 器件ID寄存器

#endif // __MPU6050_H