#include "stm32f10x.h"
#include <math.h>
#include "mpu6050.h"
#include "../HARDWARE/IIC/iic.h"
#include "usart.h"
#include "stm32f10x_it.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "delay.h"
//陀螺仪方向设置
static signed char gyro_orientation[9] = { 1,  0,  0,
                                           0,  1,  0,
                                           0,  0,  1};
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];


/**
 *  @brief      获取系统当前运行的毫秒数。
 *  该函数读取全局变量 system_ms 并写入调用者提供的变量中。
 *  注意：主函数在启动时必须调用 SysTick_Config(SystemCoreClock / 1000) 
 *  配置系统滴答定时器，以每 1 毫秒产生一次中断并更新 system_ms。
 *  @param[out] count   指向无符号长整型变量的指针，用于接收当前毫秒值。
 *  @return     返回 0 表示成功；若 count 为空指针则返回 1（失败）。
 *  @note       计数范围约为 49.7 天（基于 32 位无符号整型溢出周期）。
 */
int mget_ms(unsigned long *count)// main函数一定要调用SysTick_Config(SystemCoreClock / 1000)来配置系统滴答定时器
{
    if (!count)
    return 1;           // 错误：空指针
    *count = system_ms;     // 读取当前毫秒数
    return 0;               // 成功  最大值约 49.7 天
}

/**
  * @brief  MPU6050指定地址写一个字节函数
  * @param  RegAddress 寄存器地址
  * @param  Data 要写入的字节数据
  * @retval 无
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    IIC_Start();
    IIC_Send_Byte(MPU6050_ADDRESS | 0x00);
    IIC_Wait_Ack();
    IIC_Send_Byte(RegAddress);
    IIC_Wait_Ack();
    IIC_Send_Byte(Data);
    IIC_Wait_Ack();
    IIC_Stop();   
}
 
/**
  * @brief  MPU6050指定地址读一个字节函数
  * @param  RegAddress 寄存器地址
  * @retval 返回读出的字节数据
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
 
    IIC_Start();
    IIC_Send_Byte(MPU6050_ADDRESS | 0x00);
    IIC_Wait_Ack();
    IIC_Send_Byte(RegAddress);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(MPU6050_ADDRESS | 0x01);
    IIC_Wait_Ack();
    // B1: IIC_Read_Byte(0) 内部已发 NACK, 删掉多余的 IIC_Ack()
    Data = IIC_Read_Byte(0);
    IIC_Stop(); 
    return Data;
}
/**
 *  @brief      从指定的 I2C 从机设备读取指定长度的数据。
 *  该函数用于 MPU6050 DMP 功能，通过 I2C 总线从从机设备的指定寄存器地址
 *  连续读取若干个字节。函数内部执行完整的 I2C 读写操作序列：
 *  起始条件 → 发送从机地址（写）→ 发送寄存器地址 → 重新起始条件
 *  → 发送从机地址（读）→ 连续读取数据 → 停止条件。
 *  @param[in]  slave_addr  从机设备地址（7 位地址，不含读写位）。
 *  @param[in]  reg_addr    寄存器起始地址（8 位）。
 *  @param[in]  length      要读取的字节数。
 *  @param[out] data        指向接收数据缓冲区的指针，长度至少为 length。
 *  @return     始终返回 0，表示操作成功（假定 I2C 硬件操作成功）。
 */
int MPU6050_Read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data)// 向指定从机指定地址读len个字节，用于dmp
{
    IIC_Start();
    IIC_Send_Byte(slave_addr << 1 | 0x00);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg_addr);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((slave_addr << 1) | 0x01);
    IIC_Wait_Ack();
    while (length--) {
        *data++ = IIC_Read_Byte(length ? 1 : 0);
    }
	IIC_Stop();
	return 0;
}
/**
 *  @brief      向指定的 I2C 从机设备写入指定长度的数据。
 *  该函数用于 MPU6050 DMP 功能，通过 I2C 总线将从机设备的指定寄存器地址
 *  开始连续写入若干个字节。函数内部执行完整的 I2C 写操作序列：
 *  起始条件 → 发送从机地址（写）→ 发送寄存器地址 → 连续发送数据 → 停止条件。
 *  @param[in]  slave_addr  从机设备地址（7 位地址，不含读写位）。
 *  @param[in]  reg_addr    寄存器起始地址（8 位）。
 *  @param[in]  length      要写入的字节数。
 *  @param[in]  data        指向待发送数据缓冲区的指针，长度至少为 length。
 *  @return     始终返回 0，表示操作成功（假定 I2C 硬件操作成功）。
 */

int MPU6050_Write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data) //向指定地址写len个字节,用于dmp
{
    IIC_Start();
    IIC_Send_Byte(slave_addr << 1 | 0x00); //发送器件地址和写命令
    IIC_Wait_Ack();
    IIC_Send_Byte(reg_addr);
    IIC_Wait_Ack();
	while (length--)
	{
        IIC_Send_Byte(*data++);
        IIC_Wait_Ack();
    }
    IIC_Stop();
	return 0;
}


/**
 *  @brief      初始化 MPU6050 传感器。
 *  该函数首先初始化 I2C 接口，然后对 MPU6050 的电源管理、采样率、数字滤波、
 *  陀螺仪和加速度计量程等关键寄存器进行配置，使其进入正常工作状态。
 *  具体配置包括：
 *  - 电源管理 1：解除睡眠状态，选择 X 轴陀螺仪作为时钟源；
 *  - 电源管理 2：所有轴均不待机；
 *  - 采样率分频：设置为 9，得到 100Hz 采样率（1kHz / (1+9)）；
 *  - 配置寄存器：数字低通滤波器模式 6；
 *  - 陀螺仪配置：量程 ±2000°/s，自测禁用；
 *  - 加速度计配置：量程 ±16g，高通滤波器禁用。
 *  @param      无。
 *  @return     无。
 */
void MPU6050_Init(void)
{
    IIC_Init();
    MPU6050_WriteReg(MPU6050_PWR_MGMT1, 0x01);      //不复位，解除睡眠，不循环，温度传感器不失能，选择X轴陀螺仪时钟
    MPU6050_WriteReg(MPU6050_PWR_MGMT2 , 0x00);      //不循环，不待机
    MPU6050_WriteReg(MPU6050_SAMPLE_DIV, 0x09);      //10分频，采样率1kHz / (1 + 9) = 100Hz
    MPU6050_WriteReg(MPU6050_CFG, 0x06);          //无外部同步，数字滤波模式6
    MPU6050_WriteReg(MPU6050_GYRO_CFG, 0x18);     //陀螺仪不自测，量程±2000°/s
    MPU6050_WriteReg(MPU6050_ACCEL_CFG, 0x08);    //S1: ±4g (0x08), DMP 自动调灵敏度
}
/**
 *  @brief      读取 MPU6050 的设备 ID。
 *  该函数通过读取 WHO_AM_I 寄存器（地址为 MPU6050_DEVICE_ID）获取
 *  芯片的标识符，通常用于验证 I2C 通信是否正常以及芯片型号是否正确。
 *  @param      无。
 *  @return     返回从 MPU6050 读取到的设备 ID（8 位无符号整型），
 *              正常情况下应为 0x68（MPU6050 的标准 ID）。
 */
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_DEVICE_ID);
}


/**
 *  @brief      从 MPU6050 读取加速度计和陀螺仪的原始数据。
 *  该函数通过连续读取加速度计和陀螺仪的 6 个 16 位寄存器（每个轴高低字节），
 *  将原始数据组合成完整的 16 位有符号整型值，并通过指针参数返回给调用者。
 *  @param[out] AccX   指向 int16_t 变量，用于存储 X 轴加速度值。
 *  @param[out] AccY   指向 int16_t 变量，用于存储 Y 轴加速度值。
 *  @param[out] AccZ   指向 int16_t 变量，用于存储 Z 轴加速度值。
 *  @param[out] GyroX  指向 int16_t 变量，用于存储 X 轴陀螺仪值。
 *  @param[out] GyroY  指向 int16_t 变量，用于存储 Y 轴陀螺仪值。
 *  @param[out] GyroZ  指向 int16_t 变量，用于存储 Z 轴陀螺仪值。
 *  @return      无。
 */
// S3: 一次 burst 读 14 字节 (ACCEL_XOUTH 起: accel6 + temp2 + gyro6),
// 替代原来的 12 次单字节 ReadReg.
void MPU6050_GetDataEx(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                       int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ,
                       int16_t *Temp)
{
    uint8_t buf[14];

    if (MPU6050_Read(0x68, MPU6050_ACCEL_XOUTH, 14, buf) != 0)
        return;   // 读失败时保持输出不变

    if (AccX) *AccX = (int16_t)((buf[0] << 8) | buf[1]);
    if (AccY) *AccY = (int16_t)((buf[2] << 8) | buf[3]);
    if (AccZ) *AccZ = (int16_t)((buf[4] << 8) | buf[5]);
    if (Temp) *Temp = (int16_t)((buf[6] << 8) | buf[7]);
    if (GyroX) *GyroX = (int16_t)((buf[8] << 8) | buf[9]);
    if (GyroY) *GyroY = (int16_t)((buf[10] << 8) | buf[11]);
    if (GyroZ) *GyroZ = (int16_t)((buf[12] << 8) | buf[13]);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    MPU6050_GetDataEx(AccX, AccY, AccZ, GyroX, GyroY, GyroZ, 0);
}



/**
 *  @brief      将矩阵的一行编码为 3 位标量值。
 *  该函数根据行向量中非零元素的位置和符号生成编码，用于表示该行对应哪个轴（X、Y、Z）
 *  以及该轴的正负方向。编码结果将被拼接到方向矩阵的标量表示中。
 *  @param[in]  row     指向 3 个有符号字符的数组，表示 3x3 矩阵的一行（即 X、Y、Z 分量）。
 *  @return     返回 3 位编码值（0~7）：
 *              - 低 2 位（bit0~1）表示轴：0=X，1=Y，2=Z；
 *              - 第 2 位（bit2）表示符号：0 为正，1 为负；
 *              - 若行向量全为零（错误情况），则返回 7。
 */
unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;
 
    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;		// error
    return b;
}


 /**
 *  @brief      将 3x3 方向矩阵转换为标量表示。
 *  矩阵以 9 元素数组形式按行主序给出。每一行编码为 3 位字段，
 *  指示该行映射到哪个轴（X、Y 或 Z），符号（正或负）另行编码。
 *  生成的标量将三行编码打包成一个 9 位值。
 *  @param[in]  mtx     指向 9 元素有符号字符数组的指针，表示按行主序
 *                      排列的 3x3 方向矩阵。
 *  @return     无符号短整型标量，编码方向信息。
 *              低 3 位对应第 0 行，第 3~5 位对应第 1 行，
 *              第 6~8 位对应第 2 行，由 inv_row_2_scale() 生成。
 */
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
 
    unsigned short scalar;
 
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
 
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
 
 
    return scalar;
}


/**
 *  @brief      运行 MPU6500 的自检程序并校准偏置。
 *  该函数调用底层自检函数 mpu_run_self_test() 获取陀螺仪和加速度计的
 *  自检偏差值。如果自检成功（返回值 0x07），则将这些偏差值转换为实际
 *  物理单位并写入 DMP 的偏置寄存器中，完成校准。
 *  @param      无。
 *  @return     返回 0 表示自检成功并已写入偏置；返回 1 表示自检失败。
 */
int MPU6500_run_self_test(void)
{
	int result;
	//char test_packet[4] = {0};
	long gyro[3], accel[3]; 
	result = mpu_run_self_test(gyro, accel);
	if (result == 0x07) 
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
		float gyro_sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long)(gyro[0] * gyro_sens);
		gyro[1] = (long)(gyro[1] * gyro_sens);
		gyro[2] = (long)(gyro[2] * gyro_sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		return 0;
	}else return 1;
}

/**
 *  @brief      初始化 MPU6050 的 DMP（数字运动处理器）功能。
 *  该函数执行完整的 DMP 初始化流程，包括：
 *  - 初始化 I2C 接口；
 *  - 调用 mpu_init() 初始化 MPU 底层；
 *  - 设置所需传感器（陀螺仪和加速度计）；
 *  - 配置 FIFO；
 *  - 设置采样率；
 *  - 加载 DMP 固件；
 *  - 设置陀螺仪方向矩阵；
 *  - 使能 DMP 功能（包括 6 轴低功耗四元数、敲击检测、Android 方向、原始加速度和校准陀螺仪等）；
 *  - 设置 DMP 输出速率；
 *  - 运行自检并校准偏置；
 *  - 使能 DMP 开始工作。
 *  每个步骤均会检查返回值，并通过串口打印相应的成功或错误信息。
 *  @param      无。
 *  @return     返回 0 表示整个 DMP 初始化成功；若 mpu_init() 失败则进入死循环；
 *              若使能 DMP 失败则返回 -1；其他步骤失败仅打印错误信息，但继续执行，
 *              最终若 dmp_set_fifo_rate 之后运行自检正常，且使能 DMP 成功则返回 0。
 *  @note       此函数依赖于串口 printf 输出调试信息，若需要无输出环境请修改。
 *              默认采样率由 DEFAULT_MPU_HZ 宏定义（通常为 200Hz 或 100Hz）。
 */
int MPU6050_DMPInit(void)
{
	uint8_t res = 0;

	// B4: 移除所有 printf, DMP 状态通过返回值上报 (STATUS 帧 0x13)
	IIC_Init();
    res = mpu_init();
    if(!res)
    {
		//设置所需要的传感器
        res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

        //设置FIFO
        res = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);

		//设置采样率
        res = mpu_set_sample_rate(DEFAULT_MPU_HZ);

		//加载DMP固件
        res = dmp_load_motion_driver_firmware();

		//设置陀螺仪方向
        res = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));

		//设置DMP功能
        res = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |	              
              DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
              DMP_FEATURE_GYRO_CAL);

		//设置DMP输出速率(最大不超过200Hz)
        res = dmp_set_fifo_rate(DEFAULT_MPU_HZ);

		//自检
        res = MPU6500_run_self_test();

		//使能DMP
        res = mpu_set_dmp_state(1);
        if(!res){
            return 0;
        }
        else{
            return -1;
        }
    }
    else
    {
        // B3: 初始化失败不再 while(1) 卡死, 返回 -1 让 main 决定降级
        return -1;
    }
}
/*
得到dmp处理后的数据
pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
返回值:0,正常
   其他,失败
*/
uint8_t MPU6050_ReadDMP(float *Pitch, float *Roll, float *Yaw)
{
    // 读取FIFO中的四元数(quat)
    if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more)) 
        return 1;
        
    if(sensors & INV_WXYZ_QUAT)
    {
        // 格式转换：q30格式 -> float
        q0 = quat[0] / q30;
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;
 
        // 姿态解算：四元数转欧拉角公式
        *Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; // 57.3 = 180/PI
        *Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
        *Yaw   = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;
    }
    return 0;
}

int mpu_delay_ms(unsigned long num_ms){
    delay_ms(num_ms);
    return 0;
}

