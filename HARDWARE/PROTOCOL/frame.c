#include "frame.h"

//////////////////////////////////////////////////////////////////////////////////
// K230 ↔ STM32 裸二进制帧协议层实现 (Phase 2)
//////////////////////////////////////////////////////////////////////////////////

// ---------------- 路由表 (计划 §4 MSGID 表) ----------------
static const frame_route_t s_route_table[] = {
    { MSGID_CMD_VEL,      "CMD_VEL",      CMD_VEL_LEN,     CMD_VEL_LEN },
    { MSGID_JOINT_TARGET, "JOINT_TARGET", 1,               FRAME_MAX_PAYLOAD },
    { MSGID_GIMBAL_MODE,  "GIMBAL_MODE",  GIMBAL_MODE_LEN, GIMBAL_MODE_LEN },
    { MSGID_HEARTBEAT,    "HEARTBEAT",    HEARTBEAT_LEN,   HEARTBEAT_LEN },
    { MSGID_IMU,          "IMU",          IMU_LEN,         IMU_LEN },
    { MSGID_SERVO_FB,     "SERVO_FB",     SERVO_FB_LEN,    SERVO_FB_LEN },
    { MSGID_WHEEL_FB,     "WHEEL_FB",     WHEEL_FB_LEN,    WHEEL_FB_LEN },
    { MSGID_STATUS,       "STATUS",       STATUS_LEN,      STATUS_LEN },
    { MSGID_LIDAR_REPORT, "LIDAR_REPORT", 0,               FRAME_MAX_PAYLOAD },
};
#define ROUTE_TABLE_SIZE (sizeof(s_route_table) / sizeof(s_route_table[0]))

// ---------------- CRC16-CCITT(FALSE) ----------------

u16 frame_crc16(const u8 *data, u16 len)
{
    u16 crc = 0xFFFF;
    u16 i, j;

    for (i = 0; i < len; i++) {
        crc ^= (u16)data[i] << 8;
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (u16)((crc << 1) ^ 0x1021);
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ---------------- 组帧 ----------------

u16 frame_pack(u8 *buf, u16 buf_size, u8 msgid, const u8 *payload, u8 len)
{
    u16 crc;
    u8 i;

    if (buf == 0 || (len > 0 && payload == 0) || len > FRAME_MAX_PAYLOAD)
        return 0;
    if (buf_size < (u16)len + FRAME_OVERHEAD)
        return 0;

    buf[0] = FRAME_HEAD0;
    buf[1] = FRAME_HEAD1;
    buf[2] = msgid;
    buf[3] = len;
    for (i = 0; i < len; i++)
        buf[4 + i] = payload[i];

    crc = frame_crc16(&buf[2], (u16)len + 2);   // 覆盖 MSGID + LEN + PAYLOAD
    buf[4 + len]     = (u8)(crc >> 8);
    buf[4 + len + 1] = (u8)(crc & 0xFF);
    return (u16)len + FRAME_OVERHEAD;
}

// ---------------- 流式解帧 ----------------

typedef enum {
    FRAME_ST_WAIT_H0 = 0,   // 等 0xAA
    FRAME_ST_WAIT_H1,       // 等 0x55
    FRAME_ST_MSGID,
    FRAME_ST_LEN,
    FRAME_ST_PAYLOAD,
    FRAME_ST_CRC_H,
    FRAME_ST_CRC_L
} frame_state_t;

typedef struct {
    frame_state_t state;
    u8  msgid;
    u8  len;
    u8  idx;
    u8  crc_h;
    u8  crc_l;
    u8  payload[FRAME_MAX_PAYLOAD];
} frame_parser_t;

// 喂入 1 字节; 返回 1=收到完整合法帧(已回调), 0=仍在解析/非法丢弃
static u8 frame_parser_feed(frame_parser_t *p, u8 b, frame_cb_t cb)
{
    u16 calc;

    switch (p->state) {
    case FRAME_ST_WAIT_H0:
        if (b == FRAME_HEAD0)
            p->state = FRAME_ST_WAIT_H1;
        break;

    case FRAME_ST_WAIT_H1:
        if (b == FRAME_HEAD1)
            p->state = FRAME_ST_MSGID;
        else if (b != FRAME_HEAD0)          // 连续 AA 只取最后一个是帧头
            p->state = FRAME_ST_WAIT_H0;
        break;

    case FRAME_ST_MSGID:
        p->msgid = b;
        p->state = FRAME_ST_LEN;
        break;

    case FRAME_ST_LEN:
        p->len = b;
        if (p->len > FRAME_MAX_PAYLOAD) {   // 非法长度, 丢弃并重新同步
            p->state = FRAME_ST_WAIT_H0;
            break;
        }
        p->idx = 0;
        p->state = (p->len == 0) ? FRAME_ST_CRC_H : FRAME_ST_PAYLOAD;
        break;

    case FRAME_ST_PAYLOAD:
        p->payload[p->idx++] = b;
        if (p->idx >= p->len)
            p->state = FRAME_ST_CRC_H;
        break;

    case FRAME_ST_CRC_H:
        p->crc_h = b;
        p->state = FRAME_ST_CRC_L;
        break;

    case FRAME_ST_CRC_L:
        p->crc_l = b;
        calc = frame_crc16(&p->msgid, (u16)p->len + 2);   // MSGID+LEN+PAYLOAD
        p->state = FRAME_ST_WAIT_H0;
        if ((u8)(calc >> 8) == p->crc_h && (u8)(calc & 0xFF) == p->crc_l) {
            if (cb)
                cb(p->msgid, p->payload, p->len);
            return 1;
        }
        break;                                          // CRC 错: 静默丢弃
    }
    return 0;
}

u16 frame_unpack(const u8 *buf, u16 len, frame_cb_t cb)
{
    static frame_parser_t s_parser = { FRAME_ST_WAIT_H0 };
    u16 i, frames = 0;

    for (i = 0; i < len; i++) {
        if (frame_parser_feed(&s_parser, buf[i], cb))
            frames++;
    }
    return frames;
}

// ---------------- 路由表查询 ----------------

const char *frame_msgid_str(u8 msgid)
{
    u8 i;
    for (i = 0; i < ROUTE_TABLE_SIZE; i++) {
        if (s_route_table[i].msgid == msgid)
            return s_route_table[i].name;
    }
    return "UNKNOWN";
}

u8 frame_route_valid(u8 msgid, u8 len)
{
    u8 i;
    for (i = 0; i < ROUTE_TABLE_SIZE; i++) {
        if (s_route_table[i].msgid == msgid) {
            return (len >= s_route_table[i].min_len && len <= s_route_table[i].max_len);
        }
    }
    return 0;   // 未注册的 MSGID 一律拒绝
}
