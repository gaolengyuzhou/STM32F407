#include "tim.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LED/led.h"
#include "./BSP/CAN/can.h"
#include "./SYSTEM/delay/delay.h"
#include "tim.h"

#include "actuators.h"

//速度模式，目标速度转化值=（目标速度（度每秒）*减速比101*100）/360 ，此时5位，第一位1D，后四位为目标速度转化值，此时为36度每秒，目标速度转化值1010，3f2
void Set_velocity_mode()
{
 uint8_t key=1;
 uint8_t temp[5]={0x1D,0XF2,0X03,0X00,0X00};
key=can_send_msg(0x01,temp,5);
if(key==0)
{
	LED0(0);
}
}
//速度为0
void Set_0_mode()
{
 uint8_t key=1;
 uint8_t temp[5]={0x1D,0X00,0X00,0X00,0X00};
key=can_send_msg(0x01,temp,5);
if(key==0)
{
	LED0(0);
}
}	
void Set_position_mode()//相对于原点转90°
{
 uint8_t key=1;
 uint8_t temp[5]={0x1E,0X00,0X40,0X19,0X00};
key=can_send_msg(0x01,temp,5);
if(key==0)
{
	LED0(0);
}
	
}
void Set_0position_mode()//回到原点
{
 uint8_t key=1;
 uint8_t temp[5]={0x1E,0X00,0X00,0X00,0X00};
key=can_send_msg(0x01,temp,5);
if(key==0)
{
	LED0(0);
}
}

void CAN_Motor_Control()
{	
	Set_0_mode();
	delay_ms(5000);
	Set_velocity_mode();
	delay_ms(50000);
	Set_0_mode();
}
int  shenduji;//调试CAN深度计用
void read_depth()
{
 uint8_t key=1;
 uint8_t temp[8]={0x00,0X00,0X00,0X00,0X00,0X00,0X00,0X00};
 uint8_t data[8]={0};
 uint8_t data1[2]={0};
 key=can_send_msg(0x301,temp,8);
 //LED1(0);
 if(key==0)
 {
//	 LED1(0);
key=can_receive_msg(0x301,data);
if (key==8)
 {LED1(0);
	 data1[0]=data[2];
	 data1[1]=data[3];
	 delay_ms(5000);
 HAL_UART_Transmit(&g_uart1_handle,(uint8_t*)data1, 2, 1000);    /* 发送接收到的数据 */
 while(__HAL_UART_GET_FLAG(&g_uart1_handle,UART_FLAG_TC) != SET);           /* 等待发送结束 */
 g_usart_rx_sta = 0;
 shenduji=0;
}
}
 }
void uart5T_CAN(canbusID)//this uart is uart5,and port:under theJlink port
{     uint8_t key=1;
	    uint8_t len;
	    LED0(1);
			if (g_usart_rx_sta & 0x8000)        /* 接收到了数据 */
  {
      len = g_usart_rx_sta & 0x3fff;  /* 得到此次接收到的数据长度 */
		if(g_usart_rx_buf[0]!=0x08)
		{ key=can_send_msg(canbusID,g_usart_rx_buf,8);
		  g_usart_rx_sta = 0;
		  delay_ms(10);
			if(key==0)
	    {LED0(0);delay_ms(5);}
			if(g_usart_rx_buf[0]==0x20&&g_usart_rx_buf[1]==0x01)//测试CAN电机
			{CAN_Motor_Control();LED1(0);}
			if(g_usart_rx_buf[0]==0x20&&g_usart_rx_buf[1]==0x00)
			{Set_0_mode();}
//      HAL_UART_Transmit(&g_uart1_handle,(uint8_t*)g_usart_rx_buf, len, 1000);    /* 发送接收到的数据 */
//      while(__HAL_UART_GET_FLAG(&g_uart1_handle,UART_FLAG_TC) != SET);           /* 等待发送结束 */
//      g_usart_rx_sta = 0;
		}
			if(g_usart_rx_buf[0]==0x08&&g_usart_rx_buf[1]==0x01&&g_usart_rx_buf[2]==0x01)//测试CAN深度计
			{shenduji=1;LED0(0);}
			g_usart_rx_sta = 0;
  }
		while(shenduji==1){read_depth();}
}

void uart5R()
{
{     uint8_t len;
    	uint8_t key=1;
	    uint8_t data[4]={0};
//			if (g_usart_rx_sta & 0x8000)        /* 接收到了数据 */
//      len = g_usart_rx_sta & 0x3fff;  /* 得到此次接收到的数据长度 */
			data[0]=g_usart_rx_buf[11];
		  data[1]=g_usart_rx_buf[10];
		  data[2]=g_usart_rx_buf[9];
		  data[3]=g_usart_rx_buf[8];
      key=can_send_msg(0X16,data,4);
			if(key==0)
	    {LED0(0);}
//			HAL_UART_Transmit(&g_uart1_handle,(uint8_t*)data,4, 1000);    /* 发送接收到的数据 */
//      while(__HAL_UART_GET_FLAG(&g_uart1_handle,UART_FLAG_TC) != SET);           /* 等待发送结束 */
//      g_usart_rx_sta = 0;
  }
}
void read_height()
{
 LED1(1);
 uint8_t key=1;
 uint8_t data[4]={0};
 key=can_receive_msg(0x14,data);
 if(key==4)
 {LED1(0);
 HAL_UART_Transmit(&g_uart1_handle,(uint8_t*)data, 4, 1000);    /* 发送接收到的数据 */
 while(__HAL_UART_GET_FLAG(&g_uart1_handle,UART_FLAG_TC) != SET);           /* 等待发送结束 */
 g_usart_rx_sta = 0;
}
}

void read_sonar()//读取单波束避障声纳
{
 LED1(1);
 uint8_t key=1;
 uint8_t data[4]={0};
 key=can_receive_msg(0x13,data);
 if(key==4)
 {LED1(0);
 HAL_UART_Transmit(&g_uart1_handle,(uint8_t*)data, 4, 1000);    /* 发送接收到的数据 */
 while(__HAL_UART_GET_FLAG(&g_uart1_handle,UART_FLAG_TC) != SET);           /* 等待发送结束 */
 g_usart_rx_sta = 0;
// delay_ms(300);	 
}
}
void read_Tem_hum()
{
 LED1(1);
 uint8_t key=1;
 uint8_t data[4]={0};
 key=can_receive_msg(0x15,data);
 if(key==4)
 {LED1(0);
 HAL_UART_Transmit(&g_uart1_handle,(uint8_t*)data, 4, 1000);    /* 发送接收到的数据 */
 while(__HAL_UART_GET_FLAG(&g_uart1_handle,UART_FLAG_TC) != SET);           /* 等待发送结束 */
 g_usart_rx_sta = 0;
// delay_ms(300);	 
 }
}
void read_GPS()
{
 LED1(1);
 uint8_t key=1;
 uint8_t data[8]={0};
 key=can_receive_msg(0x18,data);
 if(key==8)
 {LED1(0);
 HAL_UART_Transmit(&g_uart1_handle,(uint8_t*)data, 8, 1000);    /* 发送接收到的数据 */
 while(__HAL_UART_GET_FLAG(&g_uart1_handle,UART_FLAG_TC) != SET);           /* 等待发送结束 */
 g_usart_rx_sta = 0;
 delay_ms(300);	 
}
}

void thruster()
{
  	//delay_ms(5000);
		__HAL_TIM_SET_COMPARE(&g_tim4_handle, GTIM_TIM4_PWM_CH1, 3000-1535);
		delay_ms(5000);
		__HAL_TIM_SET_COMPARE(&g_tim4_handle, GTIM_TIM4_PWM_CH1, 3000-1550);
		delay_ms(5000);
		__HAL_TIM_SET_COMPARE(&g_tim4_handle, GTIM_TIM4_PWM_CH1, 3000-1580);
		delay_ms(5000);
		__HAL_TIM_SET_COMPARE(&g_tim4_handle, GTIM_TIM4_PWM_CH1, 3000-1600);
		delay_ms(5000);

}
void servos()
{
  	//delay_ms(1000);
		__HAL_TIM_SET_COMPARE(&g_tim4_handle, GTIM_TIM4_PWM_CH1, 3000-1500);
		delay_ms(1000);
		__HAL_TIM_SET_COMPARE(&g_tim4_handle, GTIM_TIM4_PWM_CH1, 3000-1400);
		delay_ms(1000);
		__HAL_TIM_SET_COMPARE(&g_tim4_handle, GTIM_TIM4_PWM_CH1, 3000-1300);
		delay_ms(1000);
		__HAL_TIM_SET_COMPARE(&g_tim4_handle, GTIM_TIM4_PWM_CH1, 3000-1200);
		delay_ms(1000);
		__HAL_TIM_SET_COMPARE(&g_tim4_handle, GTIM_TIM4_PWM_CH1, 3000-1500);
		delay_ms(1000);

}
