#include "ubtech_servo.h"
#include "servo.h"
#include "delay.h"
#include <limits.h>

//////////////////////////////////////////////////////////////////////////////////
// UBTECH 单总线舵机驱动实现 (Phase 1)
// 底层: HARDWARE/SERVO (USART2 半双工单线 PA2)
// 半双工模式下自发自收, 发送后必须丢弃回显再等待舵机回应.
//////////////////////////////////////////////////////////////////////////////////

#define UBTECH_RSP_TIMEOUT_MS   50      // 单条命令等待舵机回应的默认超时

// ==================== 纯逻辑函数 ====================

u16 ubtech_checksum(u8 id, u8 cmd, u8 p1h, u8 p1l, u8 p2h, u8 p2l)
{
    // SUM = (ID + CMD + P1H + P1L + P2H + P2L) & 0xFF
    return (u16)(u8)((u32)id + cmd + p1h + p1l + p2h + p2l);
}

u8 ubtech_build_frame(u8 *buf, u8 id, u8 cmd,
                      u8 p1h, u8 p1l, u8 p2h, u8 p2l)
{
    buf[0] = UBTECH_HEAD0;
    buf[1] = UBTECH_HEAD1;
    buf[2] = id;
    buf[3] = cmd;
    buf[4] = p1h;
    buf[5] = p1l;
    buf[6] = p2h;
    buf[7] = p2l;
    buf[8] = (u8)ubtech_checksum(id, cmd, p1h, p1l, p2h, p2l);
    buf[9] = UBTECH_TAIL;
    return UBTECH_FRAME_LEN;
}

int ubtech_parse_response(const u8 *buf, u16 len, ubtech_rsp_t *rsp)
{
    u16 sum;

    if (buf == 0 || rsp == 0)
        return UBTECH_ERR_PARAM;

    // CMD 01 成功回应: 仅 [AA][ID] 两字节
    if (len == 2) {
        if (buf[0] != UBTECH_STAT_OK)
            return UBTECH_ERR_FRAME;
        rsp->id = buf[1];
        rsp->stat = UBTECH_STAT_OK;
        rsp->target_angle = 0;
        rsp->actual_angle = 0;
        rsp->offset = 0;
        rsp->version[0] = rsp->version[1] = 0;
        rsp->version[2] = rsp->version[3] = 0;
        return UBTECH_OK;
    }

    if (len != UBTECH_FRAME_LEN)
        return UBTECH_ERR_FRAME;
    if (buf[0] != UBTECH_HEAD0 || buf[1] != UBTECH_HEAD1 || buf[9] != UBTECH_TAIL)
        return UBTECH_ERR_FRAME;

    sum = ubtech_checksum(buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    if ((u8)sum != buf[8])
        return UBTECH_ERR_FRAME;

    rsp->id = buf[2];
    rsp->stat = buf[3];
    rsp->target_angle = (u16)((buf[4] << 8) | buf[5]);
    rsp->actual_angle = (u16)((buf[6] << 8) | buf[7]);
    rsp->offset = (s16)((buf[6] << 8) | buf[7]);
    rsp->version[0] = buf[4];
    rsp->version[1] = buf[5];
    rsp->version[2] = buf[6];
    rsp->version[3] = buf[7];
    return UBTECH_OK;
}

// ==================== 硬件相关 ====================

void ubtech_servo_init(u32 baud)
{
    // 复用现有 SERVO 底层 (USART2 半双工 + IDLE 帧完成检测)
    Servo_Init(baud);
}

// 发送一帧并丢弃半双工回显; 发送期间 IRQ 会把回显字节存入
// SERVO_RX_BUF, 等回显与 IDLE 处理完再整体清空, 之后收到的就是舵机回应.
static void ubtech_tx_frame(const u8 *buf, u8 len)
{
    u8 i;

    Servo_RX_Reset();
    for (i = 0; i < len; i++)
        Servo_SendByte(buf[i]);
    delay_us(300);          // 等最后一个回显字节被 IRQ 存入 + 线路 IDLE 置位
    Servo_RX_Reset();       // 丢弃自发自收回显
}

// 等待舵机回应帧: 以 100us 轮询 SERVO_RX_STA 的 bit15(帧完成);
// 完成后拷贝解析并复位, 防止处理期间被新数据覆盖.
int ubtech_wait_response(ubtech_rsp_t *rsp, u16 timeout_ms)
{
    u8 buf[UBTECH_FRAME_LEN];
    u32 loops = (u32)timeout_ms * 10;
    u16 len, copylen, i;

    while (loops-- > 0) {
        if (SERVO_RX_STA & 0x8000) {
            len = SERVO_RX_STA & 0x3FFF;
            if (len == 0) {
                Servo_RX_Reset();   // 空 IDLE(发完命令后线路空闲), 继续等回应
                continue;
            }
            copylen = (len > UBTECH_FRAME_LEN) ? UBTECH_FRAME_LEN : len;
            for (i = 0; i < copylen; i++)
                buf[i] = SERVO_RX_BUF[i];
            Servo_RX_Reset();
            return ubtech_parse_response(buf, copylen, rsp);
        }
        delay_us(100);
    }
    return UBTECH_ERR_TIMEOUT;
}

int servo_set_angle(u8 id, u16 angle, u16 lock_ms)
{
    u8 frame[UBTECH_FRAME_LEN];
    ubtech_rsp_t rsp;
    int r;

    if (id != UBTECH_BROADCAST_ID && (id < UBTECH_ID_MIN || id > UBTECH_ID_MAX))
        return UBTECH_ERR_PARAM;
    if (angle > UBTECH_ANGLE_MAX)
        angle = UBTECH_ANGLE_MAX;
    if (lock_ms > UBTECH_LOCK_MAX)
        lock_ms = UBTECH_LOCK_MAX;

    // [FA][AF][ID][01][目标角度][运动时间=0 全速][锁定时间高][锁定时间低][SUM][ED]
    ubtech_build_frame(frame, id, UBTECH_CMD_SET_ANGLE,
                       (u8)angle, 0x00,
                       (u8)((lock_ms >> 8) & 0xFF), (u8)(lock_ms & 0xFF));
    ubtech_tx_frame(frame, UBTECH_FRAME_LEN);

    // CMD 01 成功仅回 [AA][ID]; 广播(ID=0)或舵机失电等场景不回也属正常,
    // 因此超时不视为错误.
    r = ubtech_wait_response(&rsp, UBTECH_RSP_TIMEOUT_MS);
    if (r == UBTECH_OK && rsp.stat == UBTECH_STAT_OK)
        return UBTECH_OK;
    if (r == UBTECH_ERR_TIMEOUT)
        return UBTECH_OK;
    return r;
}

int servo_stop(u8 id)
{
    u8 frame[UBTECH_FRAME_LEN];

    if (id != UBTECH_BROADCAST_ID && (id < UBTECH_ID_MIN || id > UBTECH_ID_MAX))
        return UBTECH_ERR_PARAM;
    // [FA][AF][ID][01][FF][00][00][00][SUM][ED] 强制中止当前转动
    ubtech_build_frame(frame, id, UBTECH_CMD_SET_ANGLE,
                       0xFF, 0x00, 0x00, 0x00);
    ubtech_tx_frame(frame, UBTECH_FRAME_LEN);
    return UBTECH_OK;
}

int servo_stop_all(void)
{
    return servo_stop(UBTECH_BROADCAST_ID);
}

int servo_read_angle(u8 id, u16 *actual)
{
    return servo_read_angle_t(id, actual, UBTECH_RSP_TIMEOUT_MS);
}

int servo_read_angle_t(u8 id, u16 *actual, u16 timeout_ms)
{
    u8 frame[UBTECH_FRAME_LEN];
    ubtech_rsp_t rsp;
    int r;

    if (id < UBTECH_ID_MIN || id > UBTECH_ID_MAX || actual == 0)
        return UBTECH_ERR_PARAM;

    ubtech_build_frame(frame, id, UBTECH_CMD_READ_ANGLE, 0x00, 0x00, 0x00, 0x00);
    ubtech_tx_frame(frame, UBTECH_FRAME_LEN);

    r = ubtech_wait_response(&rsp, timeout_ms);
    if (r != UBTECH_OK)
        return r;
    if (rsp.stat == UBTECH_STAT_FAIL)
        return UBTECH_ERR_STATUS;
    *actual = rsp.actual_angle;
    return UBTECH_OK;
}

int servo_set_id(u8 old_id, u8 new_id)
{
    u8 frame[UBTECH_FRAME_LEN];
    ubtech_rsp_t rsp;
    int r;

    if (old_id < UBTECH_ID_MIN || old_id > UBTECH_ID_MAX ||
        new_id < UBTECH_ID_MIN || new_id > UBTECH_ID_MAX)
        return UBTECH_ERR_PARAM;

    // [FA][AF][旧ID][CD][00][新ID][00][00][SUM][ED], 回应 Byte2 为新 ID
    ubtech_build_frame(frame, old_id, UBTECH_CMD_SET_ID,
                       0x00, new_id, 0x00, 0x00);
    ubtech_tx_frame(frame, UBTECH_FRAME_LEN);

    r = ubtech_wait_response(&rsp, UBTECH_RSP_TIMEOUT_MS);
    if (r != UBTECH_OK)
        return r;
    if (rsp.stat == UBTECH_STAT_FAIL || rsp.id != new_id)
        return UBTECH_ERR_STATUS;
    return UBTECH_OK;
}

s16 servo_read_offset(u8 id)
{
    u8 frame[UBTECH_FRAME_LEN];
    ubtech_rsp_t rsp;
    int r;

    if (id < UBTECH_ID_MIN || id > UBTECH_ID_MAX)
        return INT16_MIN;

    ubtech_build_frame(frame, id, UBTECH_CMD_READ_OFFSET, 0x00, 0x00, 0x00, 0x00);
    ubtech_tx_frame(frame, UBTECH_FRAME_LEN);

    r = ubtech_wait_response(&rsp, UBTECH_RSP_TIMEOUT_MS);
    if (r != UBTECH_OK || rsp.stat == UBTECH_STAT_FAIL)
        return INT16_MIN;
    return rsp.offset;
}

int servo_set_offset(u8 id, s16 off)
{
    u8 frame[UBTECH_FRAME_LEN];
    ubtech_rsp_t rsp;
    int r;

    if (id < UBTECH_ID_MIN || id > UBTECH_ID_MAX)
        return UBTECH_ERR_PARAM;
    if (off < UBTECH_OFFSET_MIN || off > UBTECH_OFFSET_MAX)
        return UBTECH_ERR_PARAM;

    // [FA][AF][ID][D2][00][00][偏移高][偏移低][SUM][ED], 偏移 16 位有符号, 1/3 度
    ubtech_build_frame(frame, id, UBTECH_CMD_SET_OFFSET,
                       0x00, 0x00,
                       (u8)((off >> 8) & 0xFF), (u8)(off & 0xFF));
    ubtech_tx_frame(frame, UBTECH_FRAME_LEN);

    r = ubtech_wait_response(&rsp, UBTECH_RSP_TIMEOUT_MS);
    if (r != UBTECH_OK)
        return r;
    if (rsp.stat == UBTECH_STAT_FAIL)
        return UBTECH_ERR_STATUS;
    return UBTECH_OK;
}
