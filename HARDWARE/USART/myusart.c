#include "stm32f10x.h"
#include "myusart.h"

//////////////////////////////////////////////////////////////////////////////////
// USART1 = K230 链路 (PA9 TX / PA10 RX), 115200 8N1 NONE (计划 §2)
// 收发均用中断 + 环形缓冲; 中断抢占优先级 (1,0) (计划 Phase 2)
// 注意: 本模块与 System/usart/usart.c 的 printf 重定向共用 USART1 外设,
//       协议帧收发请走 uart1_send/uart1_rx_read, 勿与 printf 混用同一条链路上层.
//////////////////////////////////////////////////////////////////////////////////

#define UART1_RX_BUF_SIZE   256
#define UART1_TX_BUF_SIZE   256

static u8 s_rx_buf[UART1_RX_BUF_SIZE];
static volatile u8 s_rx_head = 0;   // ISR 写入指针
static volatile u8 s_rx_tail = 0;   // 主循环读取指针
static u8 s_tx_buf[UART1_TX_BUF_SIZE];
static volatile u8 s_tx_head = 0;   // ISR 发送指针
static volatile u8 s_tx_tail = 0;   // 主循环写入指针

void my_uart1_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    // PA9 TX: 复用推挽
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    // PA10 RX: 浮空输入
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate = 115200;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &usart);

    s_rx_head = s_rx_tail = 0;
    s_tx_head = s_tx_tail = 0;

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  // 接收中断
    USART_Cmd(USART1, ENABLE);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;     // 抢占 (1,0)
    nvic.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic);
}

// 接收环形缓冲中可读字节数
u16 uart1_rx_available(void)
{
    return (u16)(u8)(s_rx_head - s_rx_tail);
}

// 从接收环形缓冲读出一个字节(调用前先确认 available > 0)
u8 uart1_rx_read(void)
{
    u8 b = s_rx_buf[s_rx_tail];
    s_rx_tail++;
    return b;
}

// 非阻塞发送: 写入 TX 环形缓冲并开启 TXE 中断;
// 返回已入队字节数, 缓冲满返回 -1
int uart1_send(const u8 *buf, u16 len)
{
    u16 i;

    for (i = 0; i < len; i++) {
        u8 next = (u8)(s_tx_tail + 1);
        if (next == s_tx_head)
            return -1;              // TX 缓冲满
        s_tx_buf[s_tx_tail] = buf[i];
        s_tx_tail = next;
    }
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    return (int)len;
}

void USART1_IRQHandler(void)
{
    u8 res;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        res = USART_ReceiveData(USART1);
        {
            u8 next = (u8)(s_rx_head + 1);
            if (next != s_rx_tail) {    // 缓冲未满
                s_rx_buf[s_rx_head] = res;
                s_rx_head = next;
            }
            // 满则丢弃(协议层靠 CRC 丢弃残缺帧)
        }
    }

    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
        if (s_tx_head != s_tx_tail) {
            USART_SendData(USART1, s_tx_buf[s_tx_head]);
            s_tx_head++;
        } else {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
    }
}
