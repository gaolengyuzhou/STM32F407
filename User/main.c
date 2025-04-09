/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-25
 * @brief       CAN通信 实验
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
 ****************************************************************************************************
 */
 #define canbusID  0x13
 #define RK3588ScanID  0x100
 /*
 注意在下载0x18板子时（6号舱），记得更改TTL串口为RS232，在usart.h里面
 */
 /* 
0X12:做串口上位机，发送数据给串口，串口收到后发送到CAN总线上，加入了高度计和深度计的读取以及莱科CAN电机的调试
0X13：LED测试
0X14：推进器测试
0X15：舵机测试
0X16：高度计测试
0X17: 单波束避障声纳

*/

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./USMART/usmart.h"
#include "./BSP/KEY/key.h"
#include "./BSP/CAN/can.h"
#include "tim.h"
#include "DHT22.h"
#include "can_uc.h"

//extern TIM_HandleTypeDef g_tim4_handle;
//void thruster();
//extern
uint8_t    Hum_Tem[8]={0};//存放温湿度值，5位，第一位为01，第2、3位为温度，第4、5位为湿度
uint8_t    data[5];//存放单波束避障声纳数值，第一位为04
uint8_t    heightdata[5];//存放高度计数值，第一位为03
int main(void)
{
	uint8_t canbuf[8];
	uint8_t rxlen = 0;
	uint8_t i = 0;
	
		sys_stm32_clock_init(336, 8, 2, 7); /* 设置时钟,168Mhz */

    HAL_Init();  /* 初始化HAL库 */
		gtim_tim2_int_init(2000-1, 8400);//1s   /* 84 000 000 / 84 00 = 10 000 10Khz的计数频率，计数10000次为1000ms */10000-1		
	  gtim_tim3_int_init(5000-1, 8400);//1/4s  /* 84 000 000 / 84 00 = 10 000 10Khz的计数频率，计数100次为10ms */100-1 	   
    delay_init(168);                        /* 延时初始化 */
    usart_init(115200);                     /* 串口初始化为115200 */
    usmart_dev.init(84);                    /* 初始化USMART */
    led_init();                             /* 初始化LED */
		
//		HAL_CAN_MspInit(&g_canx_handler);
		can_init(CAN_SJW_1TQ, CAN_BS2_1TQ, CAN_BS1_4TQ, 7, CAN_MODE_NORMAL); //1M比特率     CAN_MODE_LOOPBACK[自发自收]
	
//		MotorSetting(1,3);
//	MotorIDReset();
		while(1){
			  uint8_t rxlen = can_receive_msg(0x01, canbuf);  /* CAN ID = 0x12, 接收数据查询 */

        if (rxlen) /* 接收到有数据 */
        {
					printf("id:%d\r\n", g_canx_rxheader.StdId);
					printf("ide:%d\r\n", g_canx_rxheader.IDE);
					printf("rtr:%d\r\n", g_canx_rxheader.RTR);
					printf("len:%d\r\n", g_canx_rxheader.DLC);

					printf("rxbuf[0]:%d\r\n", canbuf[0]);
					printf("rxbuf[1]:%d\r\n", canbuf[1]);
					printf("rxbuf[2]:%d\r\n", canbuf[2]);
					printf("rxbuf[3]:%d\r\n", canbuf[3]);
					printf("rxbuf[4]:%d\r\n", canbuf[4]);
					printf("rxbuf[5]:%d\r\n", canbuf[5]);
					printf("rxbuf[6]:%d\r\n", canbuf[6]);
					printf("rxbuf[7]:%d\r\n", canbuf[7]);
					
          RV_can_data_repack(&g_canx_rxheader, canbuf, 0x00);//00响应01自反馈
        }
//			delay_ms(100);
//			set_motor_speed(1,100,100,1);
		}
}

void GTIM_TIM2_INT_IRQHandler(void)//(GTIM_TIM2_INT_IRQn, 3, 2); 
{      
    /* 以下代码没有使用定时器HAL库共用处理函数来处理，而是直接通过判断中断标志位的方式 */
    if(__HAL_TIM_GET_FLAG(&g_tim2_handle, TIM_FLAG_UPDATE) != RESET)
    {
//				LED1_TOGGLE();
			
      __HAL_TIM_CLEAR_IT(&g_tim2_handle, TIM_IT_UPDATE);  /* 清除定时器溢出中断标志位 */
    }
}
void GTIM_TIM3_INT_IRQHandler(void)//(GTIM_TIM3_INT_IRQn, 0, 1);
{
    /* 以下代码没有使用定时器HAL库共用处理函数来处理，而是直接通过判断中断标志位的方式 */
    if(__HAL_TIM_GET_FLAG(&g_tim3_handle, TIM_FLAG_UPDATE) != RESET)
    { 
//				LED0_TOGGLE();
//				set_motor_speed(1,10.01,100,1);//10.0是速度RPM
//			set_motor_speed_36000(1,0x412028F6,100,1);
			/*
			0.0=0.0
			1.0472=60
			2.0944=120
			3.1416=180:06 66 cc a0 2a c7 1b ff:a0 2a=41002
			4.1888=240
			5.236=300
			6.2832=360
			*/
//			send_motor_ctrl_cmd(1,100.0,2.0,3.1416,10.0,15.0);//位置-12.5rad-12.5rad
			/*
			kp：比例系数，范围:0-4095 --> 0.0f-500.0f
			kd：微分系数，范围:0-50,0-511 --> 0.0f-5.0f
			pos：位置，范围: 0-6553 --> -12.5rad-12.5rad
			spd：速度，范围: 0-4095 --> -18.0rad/s-18.0rad/s
			tor：扭矩，范围: 0-4095 --> -30.0Nm-30.0Nm
			*/
			/*
			-12.5=-716.1972=0
			0.0=0.0		=		32767			[2745]
			1.0472=60	=		35512
			2.0944=120=		38257
			3.1416=180=		41002
			4.1888=240=		43748
			5.236=300	=		46493
			6.2832=360=		49238
			12.5=716.1972=65535
			*/
//			send_motor_ctrl_cmd_65536(1, 819, 204, 41002, 3185, 3071);//0-65536
//			set_motor_position(1, 120.0, 100, 1000, 1);//缺少PID参数
        __HAL_TIM_CLEAR_IT(&g_tim3_handle, TIM_IT_UPDATE);  /* 清除定时器溢出中断标志位 */
    }
}

	