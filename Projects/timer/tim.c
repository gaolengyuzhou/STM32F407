/**
 ****************************************************************************************************
 * @file        rs485.c
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
#include "./SYSTEM/sys/sys.h"
#include "tim.h"
#include "./BSP/LED/led.h"
#include "./BSP/CAN/can.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "DHT22.h"

TIM_HandleTypeDef g_tim2_handle; /* 定时器2句柄 */
TIM_HandleTypeDef g_tim3_handle; /* 定时器3句柄 */
TIM_HandleTypeDef g_tim4_handle; /* 定时器4句柄 */

/**
 * @brief       通用定时器TIMX定时中断初始化函数
 * @note
 *              通用定时器的时钟来自APB1,当PPRE1 ≥ 2分频的时候
 *              通用定时器的时钟为APB1时钟的2倍, 而APB1为42M, 所以定时器时钟 = 84Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值
 * @param       psc: 预分频系数
 * @retval      无
 */



/*********************************以下是通用定时器PWM输出实验程序*************************************/

//TIM_HandleTypeDef g_tim4_handle;     /* 定时器x句柄 */

/**
 * @brief       通用定时器TIMX 通道Y PWM输出 初始化函数（使用PWM模式1）
 * @note
 *              通用定时器的时钟来自APB1,当PPRE1 ≥ 2分频的时候
 *              通用定时器的时钟为APB1时钟的2倍, 而APB1为42M, 所以定时器时钟 = 84Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft = 定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值
 * @param       psc: 预分频系数
 * @retval      无
 */
 //TIM2
 void gtim_tim2_int_init(uint16_t arr, uint16_t psc)
{
    
	  GTIM_TIM2_INT_CLK_ENABLE();                             /* 使能TIM2时钟 */

    g_tim2_handle.Instance = GTIM_TIM2_INT;                 /* 通用定时器x */
    g_tim2_handle.Init.Prescaler = psc;                     /* 预分频系数 */
    g_tim2_handle.Init.CounterMode = TIM_COUNTERMODE_UP;    /* 递增计数模式 */
    g_tim2_handle.Init.Period = arr;                        /* 自动装载值 */
    HAL_TIM_Base_Init(&g_tim2_handle);
    
    HAL_NVIC_SetPriority(GTIM_TIM2_INT_IRQn, 3, 3);         /* 设置中断优先级，抢占优先级1，子优先级3 */
    HAL_NVIC_EnableIRQ(GTIM_TIM2_INT_IRQn);                 /* 开启ITMx中断 */

    HAL_TIM_Base_Start_IT(&g_tim2_handle);                  /* 使能定时器x和定时器x更新中断 */
}
 //TIM3
void gtim_tim3_int_init(uint16_t arr, uint16_t psc)
{
    GTIM_TIM3_INT_CLK_ENABLE();                             /* 使能TIM1时钟 */

    g_tim3_handle.Instance = GTIM_TIM3_INT;                 /* 通用定时器x */
    g_tim3_handle.Init.Prescaler = psc;                     /* 预分频系数 */
    g_tim3_handle.Init.CounterMode = TIM_COUNTERMODE_UP;    /* 递增计数模式 */
    g_tim3_handle.Init.Period = arr;                        /* 自动装载值 */
    HAL_TIM_Base_Init(&g_tim3_handle);
    
    HAL_NVIC_SetPriority(GTIM_TIM3_INT_IRQn, 2, 2);         /* 设置中断优先级，抢占优先级1，子优先级3 */
    HAL_NVIC_EnableIRQ(GTIM_TIM3_INT_IRQn);                 /* 开启ITMx中断 */

    HAL_TIM_Base_Start_IT(&g_tim3_handle);                  /* 使能定时器x和定时器x更新中断 */
}

 //TIM4
void gtim_tim4_pwm_init(uint16_t arr, uint16_t psc)
{
    TIM_OC_InitTypeDef timx_oc_pwm_chy = {0};                       /* 定时器输出句柄 */
    
    g_tim4_handle.Instance = GTIM_TIM4_PWM;                 /* 定时器x */
    g_tim4_handle.Init.Prescaler = psc;                     /* 预分频系数 */
    g_tim4_handle.Init.CounterMode = TIM_COUNTERMODE_UP;    /* 递增计数模式 */
    g_tim4_handle.Init.Period = arr;                        /* 自动重装载值 */
    HAL_TIM_PWM_Init(&g_tim4_handle);                       /* 初始化PWM */

    timx_oc_pwm_chy.OCMode = TIM_OCMODE_PWM1;                       /* 模式选择PWM1 */
    timx_oc_pwm_chy.Pulse = arr / 2;                                /* 设置比较值,此值用来确定占空比 */

    timx_oc_pwm_chy.OCPolarity = TIM_OCPOLARITY_LOW;                                        /* 输出比较极性为低 */
    HAL_TIM_PWM_ConfigChannel(&g_tim4_handle, &timx_oc_pwm_chy, GTIM_TIM4_PWM_CH1); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_tim4_handle, GTIM_TIM4_PWM_CH1);                           /* 开启对应PWM通道 */
		
		HAL_TIM_PWM_ConfigChannel(&g_tim4_handle, &timx_oc_pwm_chy, GTIM_TIM4_PWM_CH2); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_tim4_handle, GTIM_TIM4_PWM_CH2);                           /* 开启对应PWM通道 */
		
		HAL_TIM_PWM_ConfigChannel(&g_tim4_handle, &timx_oc_pwm_chy, GTIM_TIM4_PWM_CH3); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_tim4_handle, GTIM_TIM4_PWM_CH3);                           /* 开启对应PWM通道 */
		
		HAL_TIM_PWM_ConfigChannel(&g_tim4_handle, &timx_oc_pwm_chy, GTIM_TIM4_PWM_CH4); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_tim4_handle, GTIM_TIM4_PWM_CH4);                           /* 开启对应PWM通道 */
}
/**
 * @brief       定时器底层驱动，时钟使能，引脚配置
                此函数会被HAL_TIM_PWM_Init()调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == GTIM_TIM4_PWM)
    {
        GPIO_InitTypeDef gpio_init_struct1;
        GTIM_TIM4_PWM_CH1_GPIO_CLK_ENABLE();                            /* 开启通道y的CPIO时钟 */
        GTIM_TIM4_PWM_CH1_CLK_ENABLE();                                 /* 使能定时器时钟 */

        gpio_init_struct1.Pin = GTIM_TIM4_PWM_CH1_GPIO_PIN;              /* 通道y的CPIO口 */
        gpio_init_struct1.Mode = GPIO_MODE_AF_PP;                        /* 复用推完输出 */
        gpio_init_struct1.Pull = GPIO_PULLUP;                            /* 上拉 */
        gpio_init_struct1.Speed = GPIO_SPEED_FREQ_HIGH;                  /* 高速 */
        gpio_init_struct1.Alternate = GTIM_TIM4_PWM_CH1_GPIO_AF;         /* IO口REMAP设置, 是否必要查看头文件配置的说明! */
        HAL_GPIO_Init(GTIM_TIM4_PWM_CH1_GPIO_PORT, &gpio_init_struct1);
			
				GPIO_InitTypeDef gpio_init_struct2;
				GTIM_TIM4_PWM_CH2_GPIO_CLK_ENABLE();                            /* 开启通道y的CPIO时钟 */
        GTIM_TIM4_PWM_CH2_CLK_ENABLE();                                 /* 使能定时器时钟 */
        gpio_init_struct2.Pin = GTIM_TIM4_PWM_CH2_GPIO_PIN;              /* 通道y的CPIO口 */
        gpio_init_struct2.Mode = GPIO_MODE_AF_PP;                        /* 复用推完输出 */
        gpio_init_struct2.Pull = GPIO_PULLUP;                            /* 上拉 */
        gpio_init_struct2.Speed = GPIO_SPEED_FREQ_HIGH;                  /* 高速 */
        gpio_init_struct2.Alternate = GTIM_TIM4_PWM_CH1_GPIO_AF;         /* IO口REMAP设置, 是否必要查看头文件配置的说明! */
        HAL_GPIO_Init(GTIM_TIM4_PWM_CH2_GPIO_PORT, &gpio_init_struct2);
			
			  GPIO_InitTypeDef gpio_init_struct3;
				GTIM_TIM4_PWM_CH3_GPIO_CLK_ENABLE();                            /* 开启通道y的CPIO时钟 */
        GTIM_TIM4_PWM_CH3_CLK_ENABLE();                                 /* 使能定时器时钟 */
        gpio_init_struct3.Pin = GTIM_TIM4_PWM_CH3_GPIO_PIN;              /* 通道y的CPIO口 */
        gpio_init_struct3.Mode = GPIO_MODE_AF_PP;                        /* 复用推完输出 */
        gpio_init_struct3.Pull = GPIO_PULLUP;                            /* 上拉 */
        gpio_init_struct3.Speed = GPIO_SPEED_FREQ_HIGH;                  /* 高速 */
        gpio_init_struct3.Alternate = GTIM_TIM4_PWM_CH3_GPIO_AF;         /* IO口REMAP设置, 是否必要查看头文件配置的说明! */
        HAL_GPIO_Init(GTIM_TIM4_PWM_CH3_GPIO_PORT, &gpio_init_struct3);
				
				GPIO_InitTypeDef gpio_init_struct4;
				GTIM_TIM4_PWM_CH4_GPIO_CLK_ENABLE();                            /* 开启通道y的CPIO时钟 */
        GTIM_TIM4_PWM_CH4_CLK_ENABLE();                                 /* 使能定时器时钟 */
        gpio_init_struct4.Pin = GTIM_TIM4_PWM_CH4_GPIO_PIN;              /* 通道y的CPIO口 */
        gpio_init_struct4.Mode = GPIO_MODE_AF_PP;                        /* 复用推完输出 */
        gpio_init_struct4.Pull = GPIO_PULLUP;                            /* 上拉 */
        gpio_init_struct4.Speed = GPIO_SPEED_FREQ_HIGH;                  /* 高速 */
        gpio_init_struct4.Alternate = GTIM_TIM4_PWM_CH4_GPIO_AF;         /* IO口REMAP设置, 是否必要查看头文件配置的说明! */
        HAL_GPIO_Init(GTIM_TIM4_PWM_CH4_GPIO_PORT, &gpio_init_struct4);
    }
}


