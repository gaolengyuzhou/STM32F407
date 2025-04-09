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

#ifndef __TIM_H
#define __TIM_H

#include "./SYSTEM/sys/sys.h"

extern TIM_HandleTypeDef g_tim2_handle; /* 定时器2句柄 */
extern TIM_HandleTypeDef g_tim3_handle; /* 定时器3句柄 */
extern TIM_HandleTypeDef g_tim4_handle; /* 定时器4句柄 */

//TIM2
#define GTIM_TIM2_INT                       TIM2
#define GTIM_TIM2_INT_IRQn                  TIM2_IRQn
#define GTIM_TIM2_INT_IRQHandler            TIM2_IRQHandler
#define GTIM_TIM2_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM2_CLK_ENABLE(); }while(0)  /* TIM2 时钟使能 */

//TIM3
#define GTIM_TIM3_INT                       TIM3
#define GTIM_TIM3_INT_IRQn                  TIM3_IRQn
#define GTIM_TIM3_INT_IRQHandler            TIM3_IRQHandler
#define GTIM_TIM3_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)  /* TIM3 时钟使能 */


/* TIM4 中断定义 ,输出PWN用
 * 默认是针对TIM2~TIM5
 * 注意: 通过修改这4个宏定义,可以支持TIM1~TIM8任意一个定时器. 
 *GD32的定时器从0开始计数，因此使用STM32的程序移植时需注意定时器对应关系
 */
#define GTIM_TIM4_INT                       TIM4
#define GTIM_TIM4_INT_IRQn                  TIM4_IRQn
#define GTIM_TIM4_INT_IRQHandler            TIM4_IRQHandler
#define GTIM_TIM4_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM4_CLK_ENABLE(); }while(0)   /* GD32 TIM3 时钟使能 */


/* TIM4 PWM输出定义 
 * 这里输出的PWM控制LED0(RED)的亮度
 * 默认是针对TIM2~TIM5
 * 注意: 通过修改这几个宏定义,可以支持TIM1~TIM8任意一个定时器,任意一个IO口输出PWM
 */
//PD12
#define GTIM_TIM4_PWM_CH1_GPIO_PORT         GPIOD
#define GTIM_TIM4_PWM_CH1_GPIO_PIN          GPIO_PIN_12
#define GTIM_TIM4_PWM_CH1_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)  /* PF口时钟使能 */
#define GTIM_TIM4_PWM_CH1_GPIO_AF           GPIO_AF2_TIM4                             /* 端口复用到TIM14 */

//PD13
#define GTIM_TIM4_PWM_CH2_GPIO_PORT         GPIOD
#define GTIM_TIM4_PWM_CH2_GPIO_PIN          GPIO_PIN_13
#define GTIM_TIM4_PWM_CH2_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)  /* PF口时钟使能 */
#define GTIM_TIM4_PWM_CH2_GPIO_AF           GPIO_AF2_TIM4   

//PD14
#define GTIM_TIM4_PWM_CH3_GPIO_PORT         GPIOD
#define GTIM_TIM4_PWM_CH3_GPIO_PIN          GPIO_PIN_14
#define GTIM_TIM4_PWM_CH3_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)  /* PF口时钟使能 */
#define GTIM_TIM4_PWM_CH3_GPIO_AF           GPIO_AF2_TIM4   

//PD15
#define GTIM_TIM4_PWM_CH4_GPIO_PORT         GPIOD
#define GTIM_TIM4_PWM_CH4_GPIO_PIN          GPIO_PIN_15
#define GTIM_TIM4_PWM_CH4_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)  /* PF口时钟使能 */
#define GTIM_TIM4_PWM_CH4_GPIO_AF           GPIO_AF2_TIM4   

/* TIM4 REMAP设置
 * 开启TIM4的部分重映射功能, 才能将TIM14_CH1输出到PD12上
 */
#define GTIM_TIM4_PWM                       TIM4                                       /* TIM4 */
//CH1
#define GTIM_TIM4_PWM_CH1                   TIM_CHANNEL_1                                /* 通道1,  1<= Y <=4 */
#define GTIM_TIM4_PWM_CH1_CCR1              TIM4->CCR1                                  /* 通道Y的输出比较寄存器 */
#define GTIM_TIM4_PWM_CH1_CLK_ENABLE()      do{ __HAL_RCC_TIM4_CLK_ENABLE(); }while(0)  /* TIM4 时钟使能 */

//CH2
#define GTIM_TIM4_PWM_CH2                   TIM_CHANNEL_2                                /* 通道2,  1<= Y <=4 */
#define GTIM_TIM4_PWM_CH2_CCR2              TIM4->CCR2                                  /* 通道Y的输出比较寄存器 */
#define GTIM_TIM4_PWM_CH2_CLK_ENABLE()      do{ __HAL_RCC_TIM4_CLK_ENABLE(); }while(0)  /* TIM4 时钟使能 */

//CH3
#define GTIM_TIM4_PWM_CH3                   TIM_CHANNEL_3                                /* 通道3,  1<= Y <=4 */
#define GTIM_TIM4_PWM_CH3_CCR3              TIM4->CCR3                                  /* 通道Y的输出比较寄存器 */
#define GTIM_TIM4_PWM_CH3_CLK_ENABLE()      do{ __HAL_RCC_TIM4_CLK_ENABLE(); }while(0)  /* TIM4 时钟使能 */

//CH4
#define GTIM_TIM4_PWM_CH4                   TIM_CHANNEL_4                                /* 通道4,  1<= Y <=4 */
#define GTIM_TIM4_PWM_CH4_CCR4              TIM4->CCR4                                  /* 通道Y的输出比较寄存器 */
#define GTIM_TIM4_PWM_CH4_CLK_ENABLE()      do{ __HAL_RCC_TIM4_CLK_ENABLE(); }while(0)  /* TIM4 时钟使能 */


void gtim_tim2_int_init(uint16_t arr, uint16_t psc);        /* 通用定时器 定时中断初始化函数 */
void gtim_tim3_int_init(uint16_t arr, uint16_t psc);        /* 通用定时器 定时中断初始化函数 */
void gtim_tim4_pwm_init(uint16_t arr, uint16_t psc);        /* 通用定时器 PWM初始化函数 */








#endif







