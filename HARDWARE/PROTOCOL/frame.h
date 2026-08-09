#ifndef __FRAME_H
#define __FRAME_H

#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////
// K230 ↔ STM32 裸二进制帧协议层 (Phase 2)
// 协议定义见 docs/PROJECT_PLAN.md §4
//
// 帧格式:
//   [0xAA][0x55][MSGID][LEN][ PAYLOAD(LEN) ][CRC16H][CRC16L]
//   CRC16-CCITT(FALSE): 多项式 0x1021, 初值 0xFFFF, 覆盖 MSGID+LEN+PAYLOAD
//   LEN ≤ 64, 整帧 ≤ 70 字节 (计划文档写 ≤69, 按 2+1+1+64+2=70 实现)
//////////////////////////////////////////////////////////////////////////////////

#define FRAME_HEAD0         0xAA
#define FRAME_HEAD1         0x55
#define FRAME_MAX_PAYLOAD   64
#define FRAME_OVERHEAD      6       // 帧头2 + MSGID1 + LEN1 + CRC16 2
#define FRAME_MAX_LEN       (FRAME_MAX_PAYLOAD + FRAME_OVERHEAD)

// ---------------- MSGID (路由表) ----------------
#define MSGID_CMD_VEL       0x01    // K→S  3×int16(vx,vy,wz mm/s) + SEQ
#define MSGID_JOINT_TARGET  0x02    // K→S  1×byte n + n×{byte ID, uint16 angle(0.1°)}
#define MSGID_GIMBAL_MODE   0x03    // K→S  1×byte (0=manual, 1=visual)
#define MSGID_HEARTBEAT     0x04    // K→S  uint32 seq
#define MSGID_IMU           0x10    // S→K  6×int16(Pitch,Roll,Yaw,gx,gy,gz) + int16 temp
#define MSGID_SERVO_FB      0x11    // S→K  byte ID + byte STAT + uint16 目标 + uint16 实际
#define MSGID_WHEEL_FB      0x12    // S→K  4×int16 (4 轮 RPM)
#define MSGID_STATUS        0x13    // S→K  byte(心跳状态/故障位/LiDAR stub)
#define MSGID_LIDAR_REPORT  0x20    // 预留 stub

// ---------------- 各消息 payload 长度 ----------------
#define CMD_VEL_LEN         7       // 3×int16 + 1×SEQ
#define GIMBAL_MODE_LEN     1
#define HEARTBEAT_LEN       4
#define IMU_LEN             14      // 6×int16 + int16 temp
#define SERVO_FB_LEN        6       // ID + STAT + 目标u16 + 实际u16
#define WHEEL_FB_LEN        8       // 4×int16
#define STATUS_LEN          1

// 帧解析回调: 每收到一个校验通过的完整帧调用一次
typedef void (*frame_cb_t)(u8 msgid, const u8 *payload, u8 len);

// 路由表项: 用于校验长度/打印消息名
typedef struct {
    u8  msgid;
    const char *name;
    u8  min_len;
    u8  max_len;
} frame_route_t;

// ---------------- API ----------------

// CRC16-CCITT(FALSE), poly=0x1021 init=0xFFFF, 覆盖 MSGID+LEN+PAYLOAD
u16 frame_crc16(const u8 *data, u16 len);

// 组帧: 写入 [AA][55][MSGID][LEN][PAYLOAD][CRCH][CRCL], 返回整帧长度;
// 参数非法或 buf 空间不足返回 0
u16 frame_pack(u8 *buf, u16 buf_size, u8 msgid, const u8 *payload, u8 len);

// 流式解帧: 处理一段字节流(可含部分帧/多帧), 每个完整且 CRC 正确的帧
// 调用一次 cb(msgid, payload, len); 返回处理到的完整帧数.
// 内部使用静态解析器, 状态跨调用保持, 适合配合串口环形缓冲增量喂入;
// 注意: 非重入, 仅主循环使用.
u16 frame_unpack(const u8 *buf, u16 len, frame_cb_t cb);

// 按 MSGID 查路由表返回消息名
const char *frame_msgid_str(u8 msgid);

// 校验 payload 长度是否符合路由表; 返回 1=合法 0=非法
u8 frame_route_valid(u8 msgid, u8 len);

#endif
