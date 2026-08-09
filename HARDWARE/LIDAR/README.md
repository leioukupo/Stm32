# LiDAR stub (Phase 7)

本期不实现, 仅占位, 编译通过且不破坏现有链路。

## 预留资源 (PROJECT_PLAN.md §2)

| 资源 | 引脚 | 说明 |
|------|------|------|
| USART3 | PB10(TX) / PB11(RX) | 预留 LiDAR 串口 |
| GPIO | 1 个空闲脚 | 预留 (如雷达使能/复位) |

若后续 IO 不够, 计划允许把雷达接入 K230 而非 STM32。

## 协议占位

`HARDWARE/PROTOCOL/frame.h` 已定义 `MSGID_LIDAR_REPORT (0x20)` 并在路由表中
注册 (长度 0~64), STM32 端 `lidar_get_scan()` 恒返回 0。
