#include "rs485.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"


UART_HandleTypeDef g_rs458_handler;     /* RS485控制句柄(串口) */

#ifdef RS485_EN_RX /* 如果使能了接收 */

uint8_t g_RS485_rx_buf[RS485_REC_LEN];  /* 接收缓冲, 最大 RS485_REC_LEN 个字节. */
uint8_t g_RS485_rx_cnt = 0;             /* 接收到的数据长度 */

void RS485_UX_IRQHandler(void)
{ //LED1(0);
    uint8_t res;
    if ((__HAL_UART_GET_FLAG(&g_rs458_handler, UART_FLAG_RXNE) != RESET)) /* 接收到数据 */
    {   
        HAL_UART_Receive(&g_rs458_handler, &res, 1, 1000);

        if (g_RS485_rx_cnt < RS485_REC_LEN)         /* 缓冲区未满 */
        {
            g_RS485_rx_buf[g_RS485_rx_cnt] = res;   /* 记录接收到的值 */
            g_RS485_rx_cnt++;                       /* 接收数据增加1 */
						
        }
    }
}

#endif

/**
 * @brief       RS485初始化函数
 *   @note      该函数主要是初始化串口
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @retval      无
 */
void rs485_init(uint32_t baudrate)
{
    /* IO 及 时钟配置 */
    RS485_RE_GPIO_CLK_ENABLE(); /* 使能 RS485_RE 脚时钟 */
    RS485_TX_GPIO_CLK_ENABLE(); /* 使能 串口TX脚 时钟 */
    RS485_RX_GPIO_CLK_ENABLE(); /* 使能 串口RX脚 时钟 */
    RS485_UX_CLK_ENABLE();      /* 使能 串口 时钟 */

    GPIO_InitTypeDef gpio_initure;
    gpio_initure.Pin = RS485_TX_GPIO_PIN;
    gpio_initure.Mode = GPIO_MODE_AF_PP;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_initure.Alternate = GPIO_AF7_USART3;               /* 复用为串口2 */
    HAL_GPIO_Init(RS485_TX_GPIO_PORT, &gpio_initure);       /* 串口TX 脚 模式设置 */

    gpio_initure.Pin = RS485_RX_GPIO_PIN;
    HAL_GPIO_Init(RS485_RX_GPIO_PORT, &gpio_initure);       /* 串口RX 脚 必须设置成输入模式 */

    gpio_initure.Pin = RS485_RE_GPIO_PIN;
    gpio_initure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_RE_GPIO_PORT, &gpio_initure);       /* RS485_RE 脚 模式设置 */

    /* USART 初始化设置 */
    g_rs458_handler.Instance = RS485_UX;                    /* 选择485对应的串口 */
    g_rs458_handler.Init.BaudRate = baudrate;               /* 波特率 */
    g_rs458_handler.Init.WordLength = UART_WORDLENGTH_8B;   /* 字长为8位数据格式 */
    g_rs458_handler.Init.StopBits = UART_STOPBITS_1;        /* 一个停止位 */
    g_rs458_handler.Init.Parity = UART_PARITY_NONE;         /* 无奇偶校验位 */
    g_rs458_handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;   /* 无硬件流控 */
    g_rs458_handler.Init.Mode = UART_MODE_TX_RX;            /* 收发模式 */
    HAL_UART_Init(&g_rs458_handler);                        /* 使能对应的串口, 但会调用MSp */
    __HAL_UART_DISABLE_IT(&g_rs458_handler, UART_IT_TC);

#if RS485_EN_RX /* 如果使能了接收 */
                /* 使能接收中断 */
    __HAL_UART_ENABLE_IT(&g_rs458_handler, UART_IT_RXNE);   /* 开启接收中断 */
    HAL_NVIC_EnableIRQ(RS485_UX_IRQn);                      /* 使能USART1中断 */
    HAL_NVIC_SetPriority(RS485_UX_IRQn, 0, 3);              /* 抢占优先级3，子优先级3 */
#endif

    RS485_RE(0); /* 默认为接收模式 */
}

/**
 * @brief       RS485发送len个字节
 * @param       buf     : 发送区首地址
 * @param       len     : 发送的字节数(为了和本代码的接收匹配,这里建议不要超过 RS485_REC_LEN 个字节)
 * @retval      无
 */
void rs485_send_data(uint8_t *buf, uint8_t len)
{
    RS485_RE(1);                                            /* 进入发送模式 */
    HAL_UART_Transmit(&g_rs458_handler, buf, len, 1000);    /* 串口2发送数据 */
    g_RS485_rx_cnt = 0;
    RS485_RE(0); /* 进入接收模式 */
}

/**
 * @brief       RS485查询接收到的数据
 * @param       buf     : 接收缓冲区首地址
 * @param       len     : 接收到的数据长度
 *   @arg               0   , 表示没有接收到任何数据
 *   @arg               其他, 表示接收到的数据长度
 * @retval      无
 */
void delay(int time)
{
while(time--);
}
void rs485_receive_data(uint8_t *buf, uint8_t *len)
{   RS485_RE(0);
    uint8_t rxlen = g_RS485_rx_cnt;
    uint8_t i = 0;
    *len = 0;     /* 默认为0 */
	//LED1(0);
   // delay_ms(10); /* 等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束 */
	 delay(500);
   // LED1(0);
    if (rxlen == g_RS485_rx_cnt && rxlen) /* 接收到了数据,且接收完成了 */
    {   //LED1(0);
        for (i = 0; i < rxlen; i++)
        {
            buf[i] = g_RS485_rx_buf[i];
        }

        *len = g_RS485_rx_cnt; /* 记录本次数据长度 */
        g_RS485_rx_cnt = 0;    /* 清零 */
    }
}