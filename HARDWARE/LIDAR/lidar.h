#ifndef __LIDAR_H
#define __LIDAR_H

//////////////////////////////////////////////////////////////////////////////////
// LiDAR 预留 stub (PROJECT_PLAN.md Phase 7)
//
// 预留资源 (计划 §2):
//   USART3 PB10/PB11 + 1 GPIO; IO 不够时转 K230
// 协议: MSGID 0x20 LIDAR_REPORT 占位 (见 HARDWARE/PROTOCOL/frame.h)
// 本期不实现, 编译通过即可, 不破坏现有链路.
//////////////////////////////////////////////////////////////////////////////////

#include "sys.h"

// 初始化占位: 目前不做任何硬件操作
void lidar_init(void);

// 获取一帧雷达数据占位: 恒返回 0 (无数据), 不阻塞
u8 lidar_get_scan(u8 *buf, u8 max_len);

#endif
