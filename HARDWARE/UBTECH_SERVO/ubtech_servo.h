#ifndef __UBTECH_SERVO_H
#define __UBTECH_SERVO_H

#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////
// UBTECH 单总线舵机驱动层 (Phase 1)
//
// 协议定义见 docs/PROJECT_PLAN.md §3 (FA-AF 帧, 已从 UBTECH_protocol_detail.pdf 抽取)
// 底层复用 HARDWARE/SERVO (USART2 半双工单线, PA2), 本模块只负责组帧/解析/收发.
//
// 主机命令帧 (10 字节定长):
//   [FA][AF][ID][CMD][P1H][P1L][P2H][P2L][SUM][ED]
//   SUM = (ID + CMD + P1H + P1L + P2H + P2L) & 0xFF
//   ID = 0 广播, 有效范围 1~240
//
// 舵机回应帧 (10 字节): [FA][AF][ID][STAT][P1H P1L][P2H P2L][SUM][ED]
//   STAT: AA=成功, EE=失败; CMD 01 成功时仅回 2 字节 [AA][ID]
//////////////////////////////////////////////////////////////////////////////////

#define UBTECH_HEAD0          0xFA
#define UBTECH_HEAD1          0xAF
#define UBTECH_TAIL           0xED
#define UBTECH_FRAME_LEN      10

#define UBTECH_BROADCAST_ID   0x00
#define UBTECH_ID_MIN         1
#define UBTECH_ID_MAX         240
#define UBTECH_ANGLE_MAX      240      // 目标角度最大有效值 (度)
#define UBTECH_LOCK_MAX       3270     // 锁定时间上限 (单位 20ms)

// 命令码
#define UBTECH_CMD_SET_ANGLE   0x01    // 转到指定角度 / 强制中止 / 读固件版本
#define UBTECH_CMD_READ_ANGLE  0x02    // 角度回读
#define UBTECH_CMD_SET_ID      0xCD    // 修改舵机 ID
#define UBTECH_CMD_SET_OFFSET  0xD2    // 设置角度偏移量
#define UBTECH_CMD_READ_OFFSET 0xD4    // 读取角度偏移量

// 状态码
#define UBTECH_STAT_OK         0xAA
#define UBTECH_STAT_FAIL       0xEE

// 角度偏移: 16 位有符号, 单位 1/3 度, 范围 ±90 (即 ±30 度)
#define UBTECH_OFFSET_MIN      (-90)
#define UBTECH_OFFSET_MAX      90

// 返回值约定
#define UBTECH_OK               0
#define UBTECH_ERR_PARAM       -1      // 参数非法
#define UBTECH_ERR_TIMEOUT     -2      // 等待回应超时
#define UBTECH_ERR_FRAME       -3      // 回应帧格式/校验错误
#define UBTECH_ERR_STATUS      -4      // 舵机回应 STAT=EE (失败)

// 舵机回应解析结果
typedef struct {
    u8      id;              // 回应帧中的舵机 ID
    u8      stat;            // AA=成功 / EE=失败
    u16     target_angle;    // CMD 02 回读: 目标角度 (度)
    u16     actual_angle;    // CMD 02 回读: 实际角度 (度)
    s16     offset;          // D4 回读: 角度偏移 (1/3 度)
    u8      version[4];      // 固件版本回读 (版本号 1~4)
} ubtech_rsp_t;

// ---------------- 纯逻辑函数 (与硬件无关, 便于调试) ----------------

// 计算帧校验和: SUM = (ID+CMD+P1H+P1L+P2H+P2L) & 0xFF
u16 ubtech_checksum(u8 id, u8 cmd, u8 p1h, u8 p1l, u8 p2h, u8 p2l);

// 组装 10 字节主机命令帧, 返回帧长 (恒为 UBTECH_FRAME_LEN)
u8 ubtech_build_frame(u8 *buf, u8 id, u8 cmd,
                      u8 p1h, u8 p1l, u8 p2h, u8 p2l);

// 解析舵机回应帧; len=2 时按 CMD 01 成功回应 [AA][ID] 处理
// 返回 UBTECH_OK 或 UBTECH_ERR_FRAME
int ubtech_parse_response(const u8 *buf, u16 len, ubtech_rsp_t *rsp);

// ---------------- 硬件相关 API (计划 §3) ----------------

// 初始化: 复用 HARDWARE/SERVO 的 USART2 半双工单线底层
void ubtech_servo_init(u32 baud);

// 转到指定角度: angle 0~240 度; lock_ms 单位 20ms (0=全速/不锁定, 最大 3270)
// 注: 运动时间固定为 0 (全速), 锁定期内舵机不再响应新的 01 命令
int servo_set_angle(u8 id, u16 angle, u16 lock_ms);

// 强制中止当前转动, 舵机失电仅靠齿轮阻尼保持
int servo_stop(u8 id);

// 广播强制中止全部舵机
int servo_stop_all(void);

// 角度回读, 实际角度写入 *actual (度); 返回 UBTECH_OK 或错误码
int servo_read_angle(u8 id, u16 *actual);

// 角度回读 (带超时参数, 供轮询使用; timeout_ms 建议 10~50)
int servo_read_angle_t(u8 id, u16 *actual, u16 timeout_ms);

// 修改舵机 ID, 成功后舵机立即用新 ID 回应
int servo_set_id(u8 old_id, u8 new_id);

// 读取角度偏移 (1/3 度), 返回偏移值; 出错返回 INT16_MIN
s16 servo_read_offset(u8 id);

// 设置角度偏移: off 单位 1/3 度, 范围 ±90 (即 ±30 度)
int servo_set_offset(u8 id, s16 off);

#endif
