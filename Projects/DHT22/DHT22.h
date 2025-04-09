#ifndef __DHT22_H_
#define __DHT22_H_
 
//#include "gd32f4xx.h"
#include "./SYSTEM/sys/sys.h"
 
 /**************引脚修改此处****************/
//#define RCU_DHT11   RCU_GPIOG
//#define PORT_DHT11  GPIOG
//#define GPIO_DHT11  GPIO_PIN_9

#define DHT22_GPIO_PORT                  GPIOG
#define DHT22_GPIO_PIN                   GPIO_PIN_9
#define DHT22_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)             /* PF口时钟使能 */


//设置DHT11输出高或低电平
#define DATA_GPIO_OUT(x)    HAL_GPIO_WritePin(DHT22_GPIO_PORT, DHT22_GPIO_PIN, x?GPIO_PIN_SET:GPIO_PIN_RESET)
//获取DHT11数据引脚高低电平状态
#define DATA_GPIO_IN        HAL_GPIO_ReadPin(DHT22_GPIO_PORT, DHT22_GPIO_PIN)

extern float temperature;
extern float humidity;


void DHT11_GPIO_Init(void);//引脚初始化
void delay_us1();//引脚初始化
void delay_ms1();//引脚初始化
unsigned int DHT11_Read_Data(void);//读取模块数据
float Get_temperature(void);//返回读取模块后的温度数据
float Get_humidity(void);//返回读取模块后的湿度数据

#endif

