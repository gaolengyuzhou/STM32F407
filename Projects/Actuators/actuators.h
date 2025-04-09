#ifndef __ACTUATORS_H
#define __ACTUATORS_H

#include "./SYSTEM/sys/sys.h"

void uart5T_CAN();//串口做上位机
void thruster();//推进器
void servos();//舵机
void uart5R();//串口5接收，串口读取高度值，发送到CAN线上
void read_height();//读取高度
void read_depth();//读取深度
void read_sonar();//读取单波束避障声纳
void read_Tem_hum();
void CAN_Motor_Control();
/*
关于莱科CAN电机的数据说明：
速度模式计算方法：(目标转速（度每秒）*减速比(101)*100)/360
算出数值后，通过五位16进制{0x1D,0XF2,0X03,0X00,0X00}发送，
其中03 F2为目标转速36°/s的值1010，其中1010的16进制为03 F2
关于位置模式：
下发参数为：(减速机目标角度/360)*减速比*65536
举例说明：90° 值为1654784 16进制为19 40 00 故发出指令 1E 00 40 19 00
*/
void Set_velocity_mode();//莱科CAN电机设置速度模式
void Set_position_mode();//莱科CAN电机的位置模式
void Set_0_mode();//莱科CAN电机的停止
void Set_0position_mode();//莱科CAN电机的停止


void read_GPS();














#endif
