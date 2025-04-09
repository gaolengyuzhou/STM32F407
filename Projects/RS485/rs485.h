#ifndef __RS485_H
#define __RS485_H
#include "./SYSTEM/sys/sys.h"

/**
 ****************************************************************************************************
 * @file        rs485.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-23
 * @brief       RS485 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211023
 * 第一次发布
 *
 ****************************************************************************************************
 */
/******************************************************************************************/
/* RS485 引脚 和 串口 定义 
 * 默认是针对RS4852的.
 * 注意: 通过修改这10个宏定义, 可以支持UART1~UART7任意一个串口.
 */
#define RS485_RE_GPIO_PORT                  GPIOD
#define RS485_RE_GPIO_PIN                   GPIO_PIN_10
#define RS485_RE_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)   /* PD口时钟使能 */

#define RS485_TX_GPIO_PORT                  GPIOD
#define RS485_TX_GPIO_PIN                   GPIO_PIN_8
#define RS485_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)   /* PD口时钟使能 */

#define RS485_RX_GPIO_PORT                  GPIOD
#define RS485_RX_GPIO_PIN                   GPIO_PIN_9
#define RS485_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)   /* PD口时钟使能 */

#define RS485_UX                            USART3
#define RS485_UX_IRQn                       USART3_IRQn
#define RS485_UX_IRQHandler                 USART3_IRQHandler
#define RS485_UX_CLK_ENABLE()               do{ __HAL_RCC_USART3_CLK_ENABLE(); }while(0)  /* USART3 时钟使能 */


/******************************************************************************************/
/* 控制RS485_RE脚, 控制RS485发送/接收状态
 * RS485_RE = 0, 进入接收模式
 * RS485_RE = 1, 进入发送模式
 */
#define RS485_RE(x)   do{ x ? \
                          HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_SET) : \
                          HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_RESET); \
                      }while(0)


#define RS485_REC_LEN               64          /* 定义最大接收字节数 64 */
#define RS485_EN_RX                 1           /* 使能（1）/禁止（0）RS485接收 */


extern uint8_t g_RS485_rx_buf[RS485_REC_LEN];   /* 接收缓冲,最大RS485_REC_LEN个字节 */
											
extern uint8_t g_RS485_rx_cnt;                  /* 接收数据长度 */


void rs485_init( uint32_t baudrate);  /* RS485初始化 */
void rs485_send_data(uint8_t *buf, uint8_t len);    /* RS485发送数据 */
void rs485_receive_data(uint8_t *buf, uint8_t *len);/* RS485接收数据 */
void delay();

#endif