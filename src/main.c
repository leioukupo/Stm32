#include "delay.h"
#include "stm32f10x.h"
#include "stm32f10x_it.h"           // system_ms (1ms 时基)
#include "myusart.h"
#include "frame.h"
#include "mecanum.h"
#include "tb6612.h"
#include "ubtech_servo.h"
#include "../HARDWARE/MPU6050/mpu6050.h"
#include "../HARDWARE/Tim/timer2.h"
#include "../HARDWARE/Tim/timer.h"  // TIM3_Tick_Init (1ms 时基)
#include "../HARDWARE/LED/led.h"

//////////////////////////////////////////////////////////////////////////////////
// Talos rover STM32F103C8T6 主循环 (Phase 6 整合)
//
// 链路: K230(USART1) <-> STM32 <-> 14 路 UBTECH 舵机(USART2 单总线 PA2)
//                                <-> 4 轮麦轮(TIM4 PWM + 2×TB6612)
//                                <-> MPU6050(软件 I²C PC13/PC14, DMP)
//
// 功能:
//   - 解析 K230 §4 帧: 0x01 CMD_VEL / 0x02 JOINT_TARGET / 0x03 GIMBAL_MODE / 0x04 HEARTBEAT
//   - 300ms 心跳超时 -> 安全态: 4 路 PWM=0, 14 舵机广播强制中止, STATUS=FAULT
//   - 周期上报: STATUS(0x13)/IMU(0x10)/SERVO_FB(0x11 轮询)/WHEEL_FB(0x12)
//   - 心跳灯 PA0 500ms 翻转
//
// 注意: 不使用 OLED(其 SPI 引脚 PA4~7 已被 TB6612 右板占用), 不向 USART1 printf.
//////////////////////////////////////////////////////////////////////////////////

// ==================== 全局: DMP 姿态 (TIM2 IRQ 更新) ====================
float Pitch, Roll, Yaw;

// ==================== 常量 ====================
#define SERVO_COUNT          14
#define SERVO_ID_BASE        1
#define SERVO_ANGLE_UNIT     10          // 协议角度单位 0.1°
#define HEARTBEAT_TIMEOUT_MS 300
#define CYCLE_MS             10          // 主循环节拍
#define WHEEL_MAX_RPM        200         // 估算: 100% 占空比对应 RPM (无编码器)

// STATUS(0x13) 位定义
#define STATUS_HB_OK         0x01
#define STATUS_FAULT         0x02
#define STATUS_MPU_ERR       0x04
#define STATUS_LIDAR_STUB    0x08

// ==================== 状态 ====================
typedef struct {
    u8  id;
    u8  commanded;      // 1=收到过该舵机目标指令
    u16 target;         // 目标角 (0.1°)
    u16 target_sent;    // 已下发目标 (0.1°)
    u16 actual;         // 回读实际角 (0.1°)
    u8  stat;           // 上次回读状态 (AA/EE)
} servo_state_t;

static servo_state_t s_servos[SERVO_COUNT];
static volatile u32 s_last_hb_ms = 0;
static volatile u8  s_hb_ok = 0;
static volatile u8  s_fault = 0;
static volatile u8  s_mpu_ok = 0;
static u8  s_gimbal_mode = 0;       // 0=manual 1=visual (信息用)
static int s_cmd_vx = 0, s_cmd_vy = 0, s_cmd_wz = 0;
static int s_pwm[4] = {0, 0, 0, 0};
static u8  s_safety = 0;            // 1=心跳超时安全态
static u8  s_servo_poll_idx = 0;
static u32 s_tick = 0;

// ==================== 帧发送 ====================
static void send_frame(u8 msgid, const u8 *payload, u8 len)
{
    u8 buf[FRAME_MAX_LEN];
    u16 n = frame_pack(buf, sizeof(buf), msgid, payload, len);

    if (n)
        uart1_send(buf, n);
}

// ==================== K230 -> STM32 帧分发 ====================
static void on_frame(u8 msgid, const u8 *payload, u8 len)
{
    u8 i;

    switch (msgid) {
    case MSGID_CMD_VEL:                     // 0x01: 3×int16(vx,vy,wz) + SEQ
        if (len >= 7) {
            s_cmd_vx = (int16_t)(payload[0] | (payload[1] << 8));
            s_cmd_vy = (int16_t)(payload[2] | (payload[3] << 8));
            s_cmd_wz = (int16_t)(payload[4] | (payload[5] << 8));
        }
        break;

    case MSGID_JOINT_TARGET: {              // 0x02: n + n×{ID, uint16 0.1°}
        u8 n = (len >= 1) ? payload[0] : 0;
        if (n > SERVO_COUNT)
            n = SERVO_COUNT;
        for (i = 0; i < n; i++) {
            u8  id;
            u16 ang;
            if (1 + i * 3 + 2 > len)
                break;
            id = payload[1 + i * 3];
            ang = (u16)(payload[2 + i * 3] | (payload[3 + i * 3] << 8));
            if (id >= SERVO_ID_BASE && id < SERVO_ID_BASE + SERVO_COUNT) {
                s_servos[id - SERVO_ID_BASE].target = ang;
                s_servos[id - SERVO_ID_BASE].commanded = 1;
            }
        }
        break;
    }

    case MSGID_GIMBAL_MODE:                 // 0x03: 0=manual 1=visual
        if (len >= 1)
            s_gimbal_mode = payload[0];
        break;

    case MSGID_HEARTBEAT:                   // 0x04: uint32 seq
        s_last_hb_ms = system_ms;
        s_hb_ok = 1;
        break;

    default:
        break;
    }
}

// ==================== 周期上报 ====================
static void report_status(void)
{
    u8 st = STATUS_LIDAR_STUB;              // bit3: LiDAR stub 占位

    if (s_hb_ok)
        st |= STATUS_HB_OK;
    if (s_fault)
        st |= STATUS_FAULT;
    if (!s_mpu_ok)
        st |= STATUS_MPU_ERR;
    st |= (u8)(s_gimbal_mode << 4);         // 高半字节: 云台模式 (信息用)
    send_frame(MSGID_STATUS, &st, STATUS_LEN);
}

// gyro raw(±2000°/s, 16.4 LSB/°/s) -> 0.1°/s: raw*100/164
static int16_t gyro_to_01dps(int16_t raw)
{
    return (int16_t)(((int32_t)raw * 100) / 164);
}

static void report_imu(void)
{
    u8 payload[IMU_LEN];
    int16_t p, r, y, gx, gy, gz, temp, raw_t;
    u8 i;

    p = (int16_t)(Pitch * 10.0f);           // 0.1°
    r = (int16_t)(Roll * 10.0f);
    y = (int16_t)(Yaw * 10.0f);
    gx = gyro_to_01dps(gyro[0]);
    gy = gyro_to_01dps(gyro[1]);
    gz = gyro_to_01dps(gyro[2]);
    raw_t = (int16_t)((MPU6050_ReadReg(MPU6050_TEMP_OUTH) << 8) |
                      MPU6050_ReadReg(MPU6050_TEMP_OUTL));
    temp = (int16_t)(((int32_t)raw_t * 100) / 340 + 3653);     // 0.01°C

    {
        int16_t v[7] = {p, r, y, gx, gy, gz, temp};
        for (i = 0; i < 7; i++) {
            payload[i * 2]     = (u8)(v[i] & 0xFF);
            payload[i * 2 + 1] = (u8)((v[i] >> 8) & 0xFF);
        }
    }
    send_frame(MSGID_IMU, payload, IMU_LEN);
}

static void report_wheel_fb(void)
{
    u8 payload[WHEEL_FB_LEN];
    u8 i;

    // 无编码器: 按占空比估算 RPM, 装机后可换真实测速
    for (i = 0; i < 4; i++) {
        int16_t rpm = (int16_t)((s_pwm[i] * WHEEL_MAX_RPM) / 100);
        payload[i * 2]     = (u8)(rpm & 0xFF);
        payload[i * 2 + 1] = (u8)((rpm >> 8) & 0xFF);
    }
    send_frame(MSGID_WHEEL_FB, payload, WHEEL_FB_LEN);
}

static void report_servo_fb(u8 idx)
{
    u8 payload[SERVO_FB_LEN];
    servo_state_t *s = &s_servos[idx];

    payload[0] = s->id;
    payload[1] = s->stat;
    payload[2] = (u8)(s->target & 0xFF);
    payload[3] = (u8)((s->target >> 8) & 0xFF);
    payload[4] = (u8)(s->actual & 0xFF);
    payload[5] = (u8)((s->actual >> 8) & 0xFF);
    send_frame(MSGID_SERVO_FB, payload, SERVO_FB_LEN);
}

// ==================== 安全态 ====================
static void enter_safety(void)
{
    tb6612_stop();                          // 4 路 PWM=0
    servo_stop_all();                       // 广播强制中止, 14 舵机失电
    s_pwm[0] = s_pwm[1] = s_pwm[2] = s_pwm[3] = 0;
    s_fault |= STATUS_FAULT;
    s_safety = 1;
}

static void exit_safety(void)
{
    u8 i;

    s_safety = 0;
    s_fault &= (u8)~STATUS_FAULT;
    s_cmd_vx = s_cmd_vy = s_cmd_wz = 0;     // 要求新的速度指令, 防止残留速度恢复
    tb6612_stop();
    // 已下发过目标的舵机置 target_sent=0xFFFF, 主循环会重新上电驱动到最后目标
    for (i = 0; i < SERVO_COUNT; i++) {
        if (s_servos[i].commanded)
            s_servos[i].target_sent = 0xFFFF;
    }
}

// ==================== TIM2: DMP 定时读 (100Hz) ====================
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        if (s_mpu_ok)
            MPU6050_ReadDMP(&Pitch, &Roll, &Yaw);
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

// ==================== main ====================
int main(void)
{
    u8 i;

    // 时基: SysTick 归 delay 库独占 (delay_us/ms 会重配并关闭 SysTick),
    //        system_ms 改用空闲的 TIM3 提供精确 1ms 中断 (D2).
    delay_init();                           // 设置 delay 库倍乘系数
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    TIM3_Tick_Init();                       // 1ms -> system_ms (心跳/DMP)

    my_uart1_init();                        // K230 链路 USART1 (1,0)
    ubtech_servo_init(115200);              // 舵机单总线 USART2 (3,3)
    tb6612_init();                          // TIM4 4 路 PWM + TB6612 方向/STBY
    LED_Init();                             // 心跳灯 PA0

    // MPU6050 DMP: 失败降级不卡死 (B3), 状态经 STATUS 帧上报
    s_mpu_ok = (MPU6050_DMPInit() == 0) ? 1 : 0;
    TIM2_Init(1000 - 1, 720 - 1);           // 100Hz (B2), 抢占 (0,0) (B5)

    for (i = 0; i < SERVO_COUNT; i++) {
        s_servos[i].id = SERVO_ID_BASE + i;
        s_servos[i].commanded = 0;
        s_servos[i].target = 0;
        s_servos[i].target_sent = 0;
        s_servos[i].actual = 0;
        s_servos[i].stat = 0;
    }

    while (1) {
        // 1) 处理 K230 下行帧
        {
            u8 tmp[32];
            u16 n = 0;
            while (n < sizeof(tmp) && uart1_rx_available() > 0)
                tmp[n++] = uart1_rx_read();
            if (n)
                frame_unpack(tmp, n, on_frame);
        }

        // 2) 心跳超时 -> 安全态 (300ms 无 0x04)
        if (s_hb_ok && (u32)(system_ms - s_last_hb_ms) > HEARTBEAT_TIMEOUT_MS) {
            s_hb_ok = 0;
            enter_safety();
        }
        if (!s_hb_ok && !s_safety)
            enter_safety();                 // 开机首帧心跳未到也进入安全态
        else if (s_hb_ok && s_safety)
            exit_safety();                  // 链路恢复(仅一次)

        // 3) 正常控制
        if (s_hb_ok && !s_safety) {
            int pwm[4];

            mecanum_ik(&g_mecanum_default_cfg, s_cmd_vx, s_cmd_vy, s_cmd_wz, pwm);
            tb6612_set_wheels(pwm);
            for (i = 0; i < 4; i++)
                s_pwm[i] = pwm[i];

            // 关节目标变化才下发 (避免 10ms 一次刷总线)
            for (i = 0; i < SERVO_COUNT; i++) {
                if (s_servos[i].target != s_servos[i].target_sent) {
                    if (servo_set_angle(s_servos[i].id,
                                        s_servos[i].target / SERVO_ANGLE_UNIT, 0)
                        == UBTECH_OK)
                        s_servos[i].target_sent = s_servos[i].target;
                }
            }
        }

        // 4) 舵机回读轮询: 每 10ms 一个, 14 个一轮 (健康时约 150~280ms 全量)
        if (s_hb_ok && !s_safety) {
            u8 idx = s_servo_poll_idx;
            u16 actual = 0;
            int r = servo_read_angle_t(s_servos[idx].id, &actual, 10);
            if (r == UBTECH_OK) {
                s_servos[idx].actual = (u16)(actual * SERVO_ANGLE_UNIT);
                s_servos[idx].stat = UBTECH_STAT_OK;
            } else {
                s_servos[idx].actual = 0;
                s_servos[idx].stat = (r == UBTECH_ERR_TIMEOUT) ? 0xEE : 0x00;
            }
            report_servo_fb(idx);
            s_servo_poll_idx = (u8)((s_servo_poll_idx + 1) % SERVO_COUNT);
        }

        // 5) 周期上报 + 心跳灯
        s_tick++;
        if (s_tick % 10 == 0) {             // 100ms
            report_status();
            if (s_mpu_ok)
                report_imu();
        }
        if (s_tick % 20 == 0)               // 200ms
            report_wheel_fb();
        if (s_tick % 50 == 0)               // 500ms
            LED_Flip();

        delay_ms(CYCLE_MS);
    }
}
