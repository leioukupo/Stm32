#ifndef __MYUSART_H
#define __MYUSART_H

#include "sys.h"

// USART1 (K230 链路): 115200 8N1, 收发中断 + 环形缓冲
void my_uart1_init(void);

// 接收环形缓冲中可读字节数
u16 uart1_rx_available(void);

// 从接收环形缓冲读出一个字节(先确认 available > 0)
u8  uart1_rx_read(void);

// 非阻塞发送: 返回入队字节数, 缓冲满返回 -1
int uart1_send(const u8 *buf, u16 len);

#endif
