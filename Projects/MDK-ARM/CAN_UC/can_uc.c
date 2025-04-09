/*
 * can.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */
#include "stm32f4xx_hal.h"
#include "./SYSTEM/usart/usart.h"
#include "can_uc.h"
#include "./BSP/CAN/can.h"
#include "./SYSTEM/delay/delay.h"

#include "math_ops.h"

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -30.0f
#define T_MAX 30.0f
#define I_MIN -30.0f
#define I_MAX 30.0f

union RV_TypeConvert
{
	float to_float;
	int to_int;
	unsigned int to_uint;
	uint8_t buf[4];
}rv_type_convert;

union RV_TypeConvert2
{
	int16_t to_int16;
	uint16_t to_uint16;
	uint8_t buf[2];
}rv_type_convert2;

/*
////////////////////////////////////7.2电机设置指令、7.3电机零点设置指令////////////////////////////////////
cmd:
0x00:NON（通信模式：问答模式（默认）、自动报文模式）
0x01:set the communication mode to automatic feedback.
0x02:set the communication mode to response.
0x03:set the current position to zero.
*/
uint8_t MotorSetting(uint16_t motor_id,uint8_t cmd)
{
	uint8_t data[4];
	
	if(cmd==0) return 1;
	
  data[0]=motor_id>>8;
	data[1]=motor_id&0xff;
	data[2]=0x00;
	data[3]=cmd;
	
  return can_send_msg(0x7FF, data, 4);
}
/*
////////////////////////////////////7.4电机CAN 通信ID 设置指令////////////////////////////////////
*/
uint8_t MotorIDSetting(uint16_t motor_id,uint16_t motor_id_new)
{
	uint8_t data[6];
	
  data[0]=motor_id>>8;
	data[1]=motor_id&0xff;
	data[2]=0x00;
	data[3]=0x04;
	data[4]=motor_id_new>>8;
	data[5]=motor_id_new&0xff;
	
  return can_send_msg(0x7FF, data, 6);
}
/*
////////////////////////////////////7.5电机CAN 通信ID 重置指令////////////////////////////////////
*/
uint8_t MotorIDReset(void)
{
	uint8_t data[6];
	
  data[0]=0x7F;
	data[1]=0x7F;
	data[2]=0x00;
	data[3]=0x05;
	data[4]=0x7F;
	data[5]=0x7F;
	
  return can_send_msg(0x7FF, data, 6);
}
/*
////////////////////////////////////8.1查询电机通信模式////////////////////////////////////
返回报文4字节：ID高|ID低|01|**
0x01：自动报文模式
0x02：问答模式
0x80：错误
*/
uint8_t MotorCommModeReading(uint16_t motor_id)
{
	uint8_t data[4];
	
	data[0]=motor_id>>8;
	data[1]=motor_id&0xff;
	data[2]=0x00;
	data[3]=0x81;
	
  return can_send_msg(0x7FF, data, 4);
}
/*
////////////////////////////////////8.2查询CAN 通信ID////////////////////////////////////
返回报文成功5字节 ID高|ID低、失败4字节0x08
*/
uint8_t MotorIDReading(void)
{
	uint8_t data[4];
	
  data[0]=0xFF;
	data[1]=0xFF;
	data[2]=0x00;
	data[3]=0x82;
	
  return can_send_msg(0x7FF, data, 4);
}
/*
////////////////////////////////////9.1.1 力位混控模式指令////////////////////////////////////
motor_id：电机的ID，范围是1-0x7FE
kp：比例系数，范围:0-4095 --> 0.0f-500.0f
kd：微分系数，范围:0-50,0-511 --> 0.0f-5.0f
pos：位置，范围: 0-65536 --> -12.5rad-12.5rad
spd：速度，范围: 0-4095 --> -18.0rad/s-18.0rad/s
tor：扭矩，范围: 0-4095 --> -30.0Nm-30.0Nm
send_motor_ctrl_cmd(1,100.0,20.0,5.0,10.0,15.0);
*/
uint8_t send_motor_ctrl_cmd(uint16_t motor_id,float kp,float kd,float pos,float spd,float tor)
{
	int kp_int;
	int kd_int;
	int pos_int;            
	int spd_int;
	int tor_int;
	
	uint8_t data[8];
		
	if(kp>KP_MAX) kp=KP_MAX;
		else if(kp<KP_MIN) kp=KP_MIN;
	if(kd>KD_MAX ) kd=KD_MAX;
		else if(kd<KD_MIN) kd=KD_MIN;	
	if(pos>POS_MAX)	pos=POS_MAX;
		else if(pos<POS_MIN) pos=POS_MIN;
	if(spd>SPD_MAX)	spd=SPD_MAX;
		else if(spd<SPD_MIN) spd=SPD_MIN;
	if(tor>T_MAX)	tor=T_MAX;
		else if(tor<T_MIN) tor=T_MIN;

  kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
	pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);            
  spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
  tor_int = float_to_uint(tor, T_MIN, T_MAX, 12);	
	
  data[0]=0x00|(kp_int>>7);//kp5
	data[1]=((kp_int&0x7F)<<1)|((kd_int&0x100)>>8);//kp7+kd1
	data[2]=kd_int&0xFF;
	data[3]=pos_int>>8;
	data[4]=pos_int&0xFF;
	data[5]=spd_int>>4;
	data[6]=(spd_int&0x0F)<<4|(tor_int>>8);
	data[7]=tor_int&0xff;
	
  return can_send_msg(motor_id, data, 8);
	
}
/*电机模式 uint3 参数 KP uint12 参数 KD uint9 期望位置 uint16 期望速度 uint12 前馈扭矩 uint12
motor_id：电机的ID，范围是1-0x7FE
kp：比例系数，范围:0-4095 --> 0.0f-500.0f
kd：微分系数，范围:0-50,0-511 --> 0.0f-5.0f
pos：位置，范围: 0-65536 --> -12.5rad-12.5rad
spd：速度，范围: 0-4095 --> -18.0rad/s-18.0rad/s
tor：扭矩，范围: 0-4095 --> -30.0Nm-30.0Nm
send_motor_pos(1,2048,256,*****,2048,2048)
*/
uint8_t send_motor_ctrl_cmd_65536(uint16_t motor_id, uint16_t kp, uint16_t kd, uint16_t pos, uint16_t spd, uint16_t tor)
{
    uint8_t data[8];

    data[0] = 0x00 | (kp >> 7); // kp5
    data[1] = ((kp & 0x7F) << 1) | ((kd & 0x100) >> 8); // kp7+kd1
    data[2] = kd & 0xFF;
    data[3] = pos >> 8;
    data[4] = pos & 0xFF;
    data[5] = spd >> 4;
    data[6] = (spd & 0x0F) << 4 | (tor >> 8);
    data[7] = tor & 0xff;

    return can_send_msg(motor_id, data, 8);
}
/*
////////////////////////////////////9.1.2 伺服位置控制模式指令////////////////////////////////////
函数功能：设置电机的位置，同时指定速度、电流和确认状态
motor_id：电机的ID，范围是1-0x7FE
pos：电机要设置的位置，为浮点数类型，120.5 就是120.5°
spd：电机的速度，0-32767 --> 0-3276.7rpm，比例为10！！！范围：0-18000
cur：电机的电流，0-4095 --> 0-409.5A，比例为10！！！！！范围：0-3000
ack_status：返回报文类型，是0-3
set_motor_position(1,120.0,1000,1500,1);
*/
uint8_t set_motor_position(uint16_t motor_id, float pos, uint16_t spd, uint16_t cur, uint8_t ack_status)
{
    uint8_t data[8];

    if (motor_id < 1 || motor_id > 0x7FE) return 1;
    if (spd > 18000) spd = 18000;
    if (cur > 3000) cur = 3000;
    if (ack_status > 3) return 1;

    rv_type_convert.to_float = pos;
    data[0] = 0x20 | (rv_type_convert.buf[3] >> 3);
    data[1] = (rv_type_convert.buf[3] << 5) | (rv_type_convert.buf[2] >> 3);
    data[2] = (rv_type_convert.buf[2] << 5) | (rv_type_convert.buf[1] >> 3);
    data[3] = (rv_type_convert.buf[1] << 5) | (rv_type_convert.buf[0] >> 3);
    data[4] = (rv_type_convert.buf[0] << 5) | (spd >> 10);
    data[5] = (spd & 0x3FC) >> 2;
    data[6] = (spd & 0x03) << 6 | (cur >> 6);
    data[7] = (cur & 0x3F) << 2 | ack_status;

    return can_send_msg(motor_id, data, 8);
}
/*
pos： 0-65535 对应 0-360°
spd：0-18000
cur：0-3000
set_motor_position(1, 65535 / 3, 1000, 1500, 1); // 假设设置到 120° 位置
*/
uint8_t set_motor_position_65535(uint16_t motor_id, uint16_t pos, uint16_t spd, uint16_t cur, uint8_t ack_status)
{
    uint8_t data[8];

    if (motor_id < 1 || motor_id > 0x7FE) return 1;
    if (spd > 18000) spd = 18000;
    if (cur > 3000) cur = 3000;
    if (ack_status > 3) return 1;

    // 这里假设 pos 是 16 位无符号整数，需要将其拆分为 4 个字节
    data[0] = 0x20 | (pos >> 13);
    data[1] = (pos >> 5) & 0xFF;
    data[2] = (pos << 3) & 0xFF;
    data[3] = 0;  // 由于 pos 是 16 位，这里补 0

    data[4] = (pos << 5) | (spd >> 10);
    data[5] = (spd & 0x3FC) >> 2;
    data[6] = (spd & 0x03) << 6 | (cur >> 6);
    data[7] = (cur & 0x3F) << 2 | ack_status;

    return can_send_msg(motor_id, data, 8);
}    
/*
////////////////////////////////////9.1.3 伺服速度控制模式指令////////////////////////////////////
set_motor_speed(1,100.0,100,1)
motor_id:1-0x7FE
spd:，CAN 数值与实际转速一一对应（RPM），！！！范围：-18000-18000
cur:0-65536 --> 0-6553.6A，比例为10，！！！！！范围：0-3000
ack_status:0-3
*/
uint8_t set_motor_speed(uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status)
{
    uint8_t data[7];

    if (spd > 18000) spd = 18000;
    else if (spd < -18000) spd = -18000;
    if (cur > 3000) cur = 3000;
    if (ack_status > 3) return 1;

    rv_type_convert.to_float = spd;
    data[0] = 0x40 | ack_status;
    data[1] = rv_type_convert.buf[3];
    data[2] = rv_type_convert.buf[2];
    data[3] = rv_type_convert.buf[1];
    data[4] = rv_type_convert.buf[0];
    data[5] = cur >> 8;
    data[6] = cur & 0xff;

    return can_send_msg(motor_id, data, 7);
}
/*
set_motor_speed(1,18000,100,1)
spd:
cur:0-3000
ack_status:0-3
*/
uint8_t set_motor_speed_36000(uint16_t motor_id, uint32_t spd_32, uint16_t cur, uint8_t ack_status)
{
    uint8_t data[7];

    if (motor_id < 1 || motor_id > 0x7FE) return 1;
    if (cur > 3000) cur = 3000;
    if (ack_status > 3) return 1;

    data[0] = 0x40 | ack_status;
    data[1] = (spd_32 >> 24) & 0xFF;
    data[2] = (spd_32 >> 16) & 0xFF;
    data[3] = (spd_32 >> 8) & 0xFF;
    data[4] = spd_32 & 0xFF;
    data[5] = cur >> 8;
    data[6] = cur & 0xff;

		for(uint8_t i=10;i>1;i--){can_send_msg(motor_id, data, 7);delay_ms(400);}
		
    return can_send_msg(motor_id, data, 7);
}
/*
////////////////////////////////////9.1.4 电流、力矩控制模式和刹车控制指令////////////////////////////////////
motor_id:1-0x7FE
期望电流:-32768-32767 --> -327.68-327.67A，！！！！！范围：-3000-3000
或者期望力矩：-32768-32767 --> -327.68-327.67Nm，！！范围：-3000-3000
（力矩=电流*转矩常数，转矩常数参考产品手册）
ctrl_status:
					0:正常电流控制
					1:力矩控制模式
					2:变阻尼制动控制模式，此时其实电流数据自动忽略
					3:能耗制动控制模式，此时期望电流数据自动忽略
					4:再生制动控制模式，此时期望电流为设置的刹车电流阈值
					5:NON
					6:NON
					7:NON
ack_status:0-3
*/
uint8_t set_motor_cur_tor(uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status)
{
    uint8_t data[3];

    if (ack_status > 3) 
        return 1;
    if (ctrl_status > 7)
        return 1;

    if (ctrl_status) // 进入扭矩控制模式或制动模式
    {
        if (cur_tor > 3000) cur_tor = 3000;
        else if (cur_tor < -3000) cur_tor = -3000;
    }
    else
    {
        if (cur_tor > 2000) cur_tor = 2000;
        else if (cur_tor < -2000) cur_tor = -2000;
    }

    data[0] = 0x60 | ctrl_status << 2 | ack_status;
    data[1] = cur_tor >> 8;
    data[2] = cur_tor & 0xff;

    return can_send_msg(motor_id, data, 3);
}
/*
////////////////////////////////////9.2.1 电机加速度配置指令////////////////////////////////////
motor_id:1-0x7FE
acc:0-2000
加速度数值：0-2000 --> 0-20rad/s，内部已限幅。加速度减小，运行会更柔和，但是
过小易超调，系统默认值为2000。
ack_status:0-3
*/
uint8_t set_motor_acceleration(uint16_t motor_id,uint16_t acc,uint8_t ack_status)
{
	 uint8_t data[4];
	
	if(ack_status>2) 
		return 1;
	if(acc>2000) acc=2000;
	
  data[0]=0xC0|ack_status;
	data[1]=0x01;
	data[2]=acc>>8;
	data[3]=acc&0xff;
	
  return can_send_msg(motor_id, data, 4);
}
/*
////////////////////////////////////9.2.2 电机扭矩系数配置指令////////////////////////////////////
motor_id:1-0x7FE
torconfig电机扭矩系数：0-65536。250 对应 2.5，比例 100。
*/
uint8_t set_motor_TorqueConfig(uint16_t motor_id,uint16_t torconfig)
{
		uint8_t data[4];

		data[0]=0xC0;
		data[1]=0x04;
		data[2]=torconfig>>8;
		data[3]=torconfig&0xff;

		return can_send_msg(motor_id, data, 4);
}
/*
////////////////////////////////////9.2.3 电机力位混控协议 KP 配置指令////////////////////////////////////
motor_id:1-0x7FE
kp_min和kp_max比例 1，1就是1，0就是0，500就是500
*/
uint8_t set_motor_FP_kpConfig(uint16_t motor_id,uint16_t kp_min,uint16_t kp_max)
{
		uint8_t data[6];

		data[0]=0xC0;
		data[1]=0x05;
		data[2]=kp_min>>8;
		data[3]=kp_min&0xff;
		data[4]=kp_max>>8;
		data[5]=kp_max&0xff;

		return can_send_msg(motor_id, data, 6);
}
/*
////////////////////////////////////9.2.4 电机力位混控协议 KD 配置指令////////////////////////////////////
motor_id:1-0x7FE
kd_min和kd_max比例 1，1就是1，0就是0，500就是500
*/
uint8_t set_motor_FP_kdConfig(uint16_t motor_id,uint16_t kd_min,uint16_t kd_max)
{
		uint8_t data[6];

		data[0]=0xC0;
		data[1]=0x06;
		data[2]=kd_min>>8;
		data[3]=kd_min&0xff;
		data[4]=kd_max>>8;
		data[5]=kd_max&0xff;

		return can_send_msg(motor_id, data, 6);
}
/*
////////////////////////////////////9.2.5 电机力位混控协议位置配置指令////////////////////////////////////
motor_id:1-0x7FE
pos_min和pos_max
-1250 对应-12.5（rad），比例 100。
*/
uint8_t set_motor_FP_posConfig(uint16_t motor_id,int16_t pos_min,int16_t pos_max)
{
		uint8_t data[6];

		data[0]=0xC0;
		data[1]=0x07;
		data[2]=pos_min>>8;
		data[3]=pos_min&0xff;
		data[4]=pos_max>>8;
		data[5]=pos_max&0xff;

		return can_send_msg(motor_id, data, 6);
}
/*
////////////////////////////////////9.2.6 电机力位混控协议速度配置指令////////////////////////////////////
motor_id:1-0x7FE
spd_min和spd_max
SPD MIN：-1800 对应-18（rad/s），比例 100。
*/
uint8_t set_motor_FP_spdConfig(uint16_t motor_id,int16_t spd_min,int16_t spd_max)
{
		uint8_t data[6];

		data[0]=0xC0;
		data[1]=0x08;
		data[2]=spd_min>>8;
		data[3]=spd_min&0xff;
		data[4]=spd_max>>8;
		data[5]=spd_max&0xff;

		return can_send_msg(motor_id, data, 6);
}
/*
////////////////////////////////////9.2.7 电机力位混控协议扭矩配置指令////////////////////////////////////
motor_id:1-0x7FE
tor_min和tor_max
-900 对应-90（Nm），比例 10。
*/
uint8_t set_motor_FP_torConfig(uint16_t motor_id,int16_t tor_min,int16_t tor_max)
{
		uint8_t data[6];

		data[0]=0xC0;
		data[1]=0x09;
		data[2]=tor_min>>8;
		data[3]=tor_min&0xff;
		data[4]=tor_max>>8;
		data[5]=tor_max&0xff;

		return can_send_msg(motor_id, data, 6);
}
/*
////////////////////////////////////9.2.8 电机力位混控协议电流配置指令////////////////////////////////////
motor_id:1-0x7FE
cur_min和cur_max
CUR MIN：-900 对应-90（A），比例 10。
*/
uint8_t set_motor_FP_curConfig(uint16_t motor_id,int16_t cur_min,int16_t cur_max)
{
		uint8_t data[6];

		data[0]=0xC0;
		data[1]=0x0A;
		data[2]=cur_min>>8;
		data[3]=cur_min&0xff;
		data[4]=cur_max>>8;
		data[5]=cur_max&0xff;

		return can_send_msg(motor_id, data, 6);
}
/*
////////////////////////////////////9.2.9 电机 CAN 超时时间配置指令////////////////////////////////////
motor_id:1-0x7FE
电机电机 CAN 超时时间：500 对应 500（ms），比例 1。
时间设置为 0 时即关闭 CAN 超时保护。
*/
uint8_t set_motor_can_timeConfig(uint16_t motor_id,uint16_t timeout)
{
		uint8_t data[4];

		data[0]=0xC0;
		data[1]=0x0B;
		data[2]=timeout>>8;
		data[3]=timeout&0xff;

		return can_send_msg(motor_id, data, 4);
}
/*
////////////////////////////////////9.2.10 电机电流环 PI 配置指令////////////////////////////////////
motor_id:1-0x7FE
电流环 Current KP：100 对应 0.01，比例 10000。
电流环 Current KI：500 对应 50，比例 10。
*/
uint8_t set_motor_currentloop_piConfig(uint16_t motor_id,uint16_t cur_kp,uint16_t cur_ki)
{
		uint8_t data[6];

		data[0]=0xC0;
		data[1]=0x0C;
		data[2]=cur_kp>>8;
		data[3]=cur_kp&0xff;
		data[4]=cur_ki>>8;
		data[5]=cur_ki&0xff;

		return can_send_msg(motor_id, data, 6);
}
/*
////////////////////////////////////9.2.11 电机速度环 PI 配置指令////////////////////////////////////
motor_id:1-0x7FE
电流环 Speed KP：600 对应 0.006，比例 100000。
电流环 Speed KI：10000 对应 0.1，比例 100000。
*/
uint8_t set_motor_speedloop_piConfig(uint16_t motor_id,uint16_t spd_kp,uint16_t spd_ki)
{
		uint8_t data[6];

		data[0]=0xC0;
		data[1]=0x0D;
		data[2]=spd_kp>>8;
		data[3]=spd_kp&0xff;
		data[4]=spd_ki>>8;
		data[5]=spd_ki&0xff;

		return can_send_msg(motor_id, data, 6);
}
/*
////////////////////////////////////9.2.12 电机位置环 PD 配置指令////////////////////////////////////
motor_id:1-0x7FE
位置环 Position KP：600 对应 0.006，比例 100000。
位置环 Position KD：4 对应 0.00004，比例 100000。
*/
uint8_t set_motor_position_pdConfig(uint16_t motor_id,uint16_t pos_kp,uint16_t pos_kd)
{
		uint8_t data[6];

		data[0]=0xC0;
		data[1]=0x0E;
		data[2]=pos_kp>>8;
		data[3]=pos_kp&0xff;
		data[4]=pos_kd>>8;
		data[5]=pos_kd&0xff;

		return can_send_msg(motor_id, data, 6);
}
/*
////////////////////////////////////9.2.13 电机扭矩系数 Kt 校准使能配置指令////////////////////////////////////
motor_id:1-0x7FE
0（关闭），1（打开）。
*/
uint8_t set_motor_torque_ktConfig(uint16_t motor_id,uint8_t tor_kt)
{
		uint8_t data[3];

		data[0]=0xC0;
		data[1]=0x0F;
		data[2]=tor_kt;

		return can_send_msg(motor_id, data, 3);
}
/*
////////////////////////////////////9.3电机控制参数查询指令////////////////////////////////////
motor_id:1-0x7FE
param_cmd:
0：保留，目前无效		1：查询当前位置		2：查询当前速度		3：查询当前电流
4：查询当前功率			5：查询当前加速度	6：查询当前磁链观测增益	7：查询当前扰动补偿系数
8：查询反馈补偿系数	9：查询阻尼系数		22：查询扭矩系数		23：查询力位混控协议KP 范围
24：查询力位混控协议KD 范围	25：查询力位混控协议POS 范围	26：查询力位混控协议SPD 范围
27：查询力位混控协议TOR 范围	28：查询力位混控协议CUR 范围	29：查询MCU_UUID
30：查询软件版本号与硬件版本号					31：查询CAN 超时保护时间
32：查询电流环KP 和KI				33：查询速度环KP 和KI				34：查询位置环KP 和KD
35：查询扭矩系数Kt校准使能状态
*/
uint8_t get_motor_parameter(uint16_t motor_id,uint8_t param_cmd)
{
	 uint8_t data[2];
	
	 data[0]=0xE0;
	 data[1]=param_cmd;
	
	 return can_send_msg(motor_id, data, 2);
}





////////////////*******************************////////////////////////////
////////////////*******************************////////////////////////////
MotorCommFbd motor_comm_fbd;
OD_Motor_Msg rv_motor_msg[8]; 
uint16_t motor_id_check=0;
// This function use in auto feedback communication mode.
/*
motor_quantity:1~8
*/
// 函数功能：在自动反馈通信模式下设置多个电机的电流
// 参数：
// rv_MotorMsg：包含电机信息的结构体数组，长度为 8
// motor_quantity：要设置电流的电机数量，范围 1~8
// 返回值：无
void set_motors_current(OD_Motor_Msg rv_MotorMsg[8], uint8_t motor_quantity)
{
    uint16_t i;
    uint8_t data[8];  // 用于存储要发送的数据

    // 检查电机数量是否合法
    if (motor_quantity < 1)
        return;

    // 发送前 4 个电机的电流设置信息
    for (i = 0; i < 4; i++)
    {
        data[2 * i] = rv_MotorMsg[i].current_desired_int >> 8;
        data[2 * i + 1] = rv_MotorMsg[i].current_desired_int & 0xff;
    }
    // 调用 can_send_msg 函数发送数据
    can_send_msg(0x1FF, data, 8);

    // 如果电机数量大于 4，发送后 4 个电机的电流设置信息
    if (motor_quantity >= 5)
    {
        for (i = 0; i < 4; i++)
        {
            data[2 * i] = rv_MotorMsg[i + 4].current_desired_int >> 8;
            data[2 * i + 1] = rv_MotorMsg[i + 4].current_desired_int & 0xff;
        }
        // 调用 can_send_msg 函数发送数据
        can_send_msg(0x2FF, data, 8);
    }
}
// 函数功能：在询问通信模式下设置电机的联动系数和速度积分系数
// 参数：
// motor_id：电机的 ID，范围 1~0x7FE
// linkage：联动系数，范围 0~10000
// speedKI：速度积分系数，范围 0~10000
// ack_status：确认状态，范围 0/1
// 返回值：无
void set_motor_linkage_speedKI(uint16_t motor_id, uint16_t linkage, uint16_t speedKI, uint8_t ack_status)
{
    uint8_t data[6];  // 用于存储要发送的数据

    // 检查确认状态、联动系数和速度积分系数是否合法
    if (ack_status > 2)
        return;
    if (linkage > 10000)
        linkage = 10000;
    if (speedKI > 10000)
        speedKI = 10000;

    // 填充数据数组
    data[0] = 0xC0 | ack_status;
    data[1] = 0x02;
    data[2] = linkage >> 8;
    data[3] = linkage & 0xff;
    data[4] = speedKI >> 8;
    data[5] = speedKI & 0xff;

    // 调用 can_send_msg 函数发送数据
    can_send_msg(motor_id, data, 6);
}
// 函数功能：在询问通信模式下设置电机的反馈比例系数和反馈微分系数
// 参数：
// motor_id：电机的 ID，范围是 1~0x7FE
// fdbKP：反馈比例系数，范围是 0~10000
// fdbKD：反馈微分系数，范围是 0~10000
// ack_status：确认状态，范围是 0/1
// 返回值：无
void set_motor_feedbackKP_KD(uint16_t motor_id, uint16_t fdbKP, uint16_t fdbKD, uint8_t ack_status)
{
    // 定义一个长度为 6 的数组，用于存储要发送的数据
    uint8_t data[6];

    // 检查确认状态、反馈比例系数和反馈微分系数是否合法
    if (ack_status > 2)
        return;
    if (fdbKP > 10000)
        fdbKP = 10000;
    if (fdbKD > 10000)
        fdbKD = 10000;

    // 填充数据数组
    data[0] = 0xC0 | ack_status;
    data[1] = 0x03;
    data[2] = fdbKP >> 8;
    data[3] = fdbKP & 0xff;
    data[4] = fdbKD >> 8;
    data[5] = fdbKD & 0xff;

    // 调用 can_send_msg 函数发送数据
    can_send_msg(motor_id, data, 6);
}
// 函数功能：根据CAN接收消息重新打包数据，处理不同通信模式下的电机反馈信息
// 参数：
// RxMessage：指向接收到的CAN消息结构体指针
// comm_mode：通信模式，0x00为响应模式，0x01为自动反馈模式
// 返回值：无
void RV_can_data_repack(CAN_RxHeaderTypeDef *RxHeader, uint8_t *data, uint8_t comm_mode)
{
    uint8_t motor_id_t = 0;
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;

    if (RxHeader->StdId == 0x7FF)
    {
        if (data[2] != 0x01){
						printf("Not a motor feedback instruction. StdId: 0x%X\n", RxHeader->StdId);
            return;
				}
        if ((data[0] == 0xff) && (data[1] == 0xFF))
        {
						motor_comm_fbd.motor_id = data[3] << 8 | data[4];
            motor_comm_fbd.motor_fbd = 0x01;
						printf("Motor ID set: 0x%X, Motor FBD: 0x%X\n", motor_comm_fbd.motor_id, motor_comm_fbd.motor_fbd);
        }
        else if ((data[0] == 0x80) && (data[1] == 0x80))
        {
            motor_comm_fbd.motor_id = 0;
            motor_comm_fbd.motor_fbd = 0x80;
						printf("Inquire failed. Motor ID: 0x%X, Motor FBD: 0x%X\n", motor_comm_fbd.motor_id, motor_comm_fbd.motor_fbd);
        }
        else if ((data[0] == 0x7F) && (data[1] == 0x7F))
        {
            motor_comm_fbd.motor_id = 1;
            motor_comm_fbd.motor_fbd = 0x05;
						printf("Reset ID succeed. Motor ID: 0x%X, Motor FBD: 0x%X\n", motor_comm_fbd.motor_id, motor_comm_fbd.motor_fbd);
        }
        else
        {
            motor_comm_fbd.motor_id = data[0] << 8 | data[1];
            motor_comm_fbd.motor_fbd = data[3];
						printf("Motor ID set: 0x%X, Motor FBD: 0x%X\n", motor_comm_fbd.motor_id, motor_comm_fbd.motor_fbd);
        }
    }
    else if (comm_mode == 0x00)
    {
        ack_status = data[0] >> 5;
        motor_id_t = RxHeader->StdId - 1;
        motor_id_check = RxHeader->StdId;

        rv_motor_msg[motor_id_t].motor_id = motor_id_t;
        rv_motor_msg[motor_id_t].error = data[0] & 0x1F;

        if (ack_status == 1)
        {
            pos_int = data[1] << 8 | data[2];
            spd_int = data[3] << 4 | (data[4] & 0xF0) >> 4;
            cur_int = (data[4] & 0x0F) << 8 | data[5];

            rv_motor_msg[motor_id_t].angle_actual_rad = uint_to_float(pos_int, -12.5, 12.5, 16);  // 假设范围
            rv_motor_msg[motor_id_t].speed_actual_rad = uint_to_float(spd_int, -18, 18, 12);    // 假设范围
            rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, -30, 30, 12); // 假设范围
            rv_motor_msg[motor_id_t].temperature = (data[6] - 50) / 2;
						printf("Response frame 1 - Motor ID: 0x%X, Angle: %f rad, Speed: %f rad/s, Current: %f A, Temperature: %d °C\n", 
                   motor_id_t, rv_motor_msg[motor_id_t].angle_actual_rad, rv_motor_msg[motor_id_t].speed_actual_rad, 
                   rv_motor_msg[motor_id_t].current_actual_float, rv_motor_msg[motor_id_t].temperature);
        }
        else if (ack_status == 2)
        {
            rv_type_convert.buf[0] = data[4];
            rv_type_convert.buf[1] = data[3];
            rv_type_convert.buf[2] = data[2];
            rv_type_convert.buf[3] = data[1];
            rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
            rv_motor_msg[motor_id_t].current_actual_int = data[5] << 8 | data[6];
            rv_motor_msg[motor_id_t].temperature = (data[7] - 50) / 2;
            rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
						printf("Response frame 2 - Motor ID: 0x%X, Angle: %f, Current: %f A, Temperature: %d °C\n", 
                   motor_id_t, rv_motor_msg[motor_id_t].angle_actual_float, rv_motor_msg[motor_id_t].current_actual_float, 
                   rv_motor_msg[motor_id_t].temperature);
        }
        else if (ack_status == 3)
        {
            rv_type_convert.buf[0] = data[4];
            rv_type_convert.buf[1] = data[3];
            rv_type_convert.buf[2] = data[2];
            rv_type_convert.buf[3] = data[1];
            rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
            rv_motor_msg[motor_id_t].current_actual_int = data[5] << 8 | data[6];
            rv_motor_msg[motor_id_t].temperature = (data[7] - 50) / 2;
            rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
						printf("Response frame 3 - Motor ID: 0x%X, Speed: %f, Current: %f A, Temperature: %d °C\n", 
                   motor_id_t, rv_motor_msg[motor_id_t].speed_actual_float, rv_motor_msg[motor_id_t].current_actual_float, 
                   rv_motor_msg[motor_id_t].temperature);
        }
        else if (ack_status == 4)
        {
            if (RxHeader->DLC != 3)
                return;
            motor_comm_fbd.INS_code = data[1];
            motor_comm_fbd.motor_fbd = data[2];
						printf("Response frame 4 - Motor ID: 0x%X, INS code: 0x%X, Motor FBD: 0x%X\n", 
                   motor_id_t, motor_comm_fbd.INS_code, motor_comm_fbd.motor_fbd);
        }
        else if (ack_status == 5)
        {
            motor_comm_fbd.INS_code = data[1];
            if (motor_comm_fbd.INS_code == 1 && RxHeader->DLC == 6)
            {
                rv_type_convert.buf[0] = data[5];
                rv_type_convert.buf[1] = data[4];
                rv_type_convert.buf[2] = data[3];
                rv_type_convert.buf[3] = data[2];
                rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
								printf("Response frame 5 - Get position. Motor ID: 0x%X, Angle: %f\n", motor_id_t, rv_motor_msg[motor_id_t].angle_actual_float);
            }
            else if (motor_comm_fbd.INS_code == 2 && RxHeader->DLC == 6)
            {
                rv_type_convert.buf[0] = data[5];
                rv_type_convert.buf[1] = data[4];
                rv_type_convert.buf[2] = data[3];
                rv_type_convert.buf[3] = data[2];
                rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
								printf("Response frame 5 - Get speed. Motor ID: 0x%X, Speed: %f\n", motor_id_t, rv_motor_msg[motor_id_t].speed_actual_float);
            }
            else if (motor_comm_fbd.INS_code == 3 && RxHeader->DLC == 6)
            {
                rv_type_convert.buf[0] = data[5];
                rv_type_convert.buf[1] = data[4];
                rv_type_convert.buf[2] = data[3];
                rv_type_convert.buf[3] = data[2];
                rv_motor_msg[motor_id_t].current_actual_float = rv_type_convert.to_float;
            }
            else if (motor_comm_fbd.INS_code == 4 && RxHeader->DLC == 6)
            {
                rv_type_convert.buf[0] = data[5];
                rv_type_convert.buf[1] = data[4];
                rv_type_convert.buf[2] = data[3];
                rv_type_convert.buf[3] = data[2];
                rv_motor_msg[motor_id_t].power = rv_type_convert.to_float;
								printf("Response frame 5 - Get current. Motor ID: 0x%X, Current: %f A\n", motor_id_t, rv_motor_msg[motor_id_t].current_actual_float);
            }
            else if (motor_comm_fbd.INS_code == 5 && RxHeader->DLC == 4)
            {
                rv_motor_msg[motor_id_t].acceleration = data[2] << 8 | data[3];
								printf("Response frame 5 - Get acceleration. Motor ID: 0x%X, Acceleration: %d\n", motor_id_t, rv_motor_msg[motor_id_t].acceleration);
            }
            else if (motor_comm_fbd.INS_code == 6 && RxHeader->DLC == 4)
            {
                rv_motor_msg[motor_id_t].linkage_KP = data[2] << 8 | data[3];
								printf("Response frame 5 - Get linkage_KP. Motor ID: 0x%X, Linkage KP: %d\n", motor_id_t, rv_motor_msg[motor_id_t].linkage_KP);
            }
            else if (motor_comm_fbd.INS_code == 7 && RxHeader->DLC == 4)
            {
                rv_motor_msg[motor_id_t].speed_KI = data[2] << 8 | data[3];
								printf("Response frame 5 - Get speed_KI. Motor ID: 0x%X, Speed KI: %d\n", motor_id_t, rv_motor_msg[motor_id_t].speed_KI);
            }
            else if (motor_comm_fbd.INS_code == 8 && RxHeader->DLC == 4)
            {
                rv_motor_msg[motor_id_t].feedback_KP = data[2] << 8 | data[3];
								printf("Response frame 5 - Get feedback_KP. Motor ID: 0x%X, Feedback KP: %d\n", motor_id_t, rv_motor_msg[motor_id_t].feedback_KP);
            }
            else if (motor_comm_fbd.INS_code == 9 && RxHeader->DLC == 4)
            {
                rv_motor_msg[motor_id_t].feedback_KD = data[2] << 8 | data[3];
								printf("Response frame 5 - Get feedback_KD. Motor ID: 0x%X, Feedback KD: %d\n", motor_id_t, rv_motor_msg[motor_id_t].feedback_KD);
            }
        }
    }
    else if (comm_mode == 0x01)
    {
        motor_id_t = RxHeader->StdId - 0x205;
        rv_motor_msg[motor_id_t].angle_actual_int = (uint16_t)(data[0] << 8 | data[1]);
        rv_motor_msg[motor_id_t].speed_actual_int = (int16_t)(data[2] << 8 | data[3]);
        rv_motor_msg[motor_id_t].current_actual_int = (data[4] << 8 | data[5]);
        rv_motor_msg[motor_id_t].temperature = data[6];
        rv_motor_msg[motor_id_t].error = data[7];
				printf("Automatic feedback mode - Motor ID: 0x%X, Angle: 0x%X, Speed: 0x%X, Current: 0x%X, Temperature: %d, Error: 0x%X\n", 
               motor_id_t, rv_motor_msg[motor_id_t].angle_actual_int, rv_motor_msg[motor_id_t].speed_actual_int, 
               rv_motor_msg[motor_id_t].current_actual_int, rv_motor_msg[motor_id_t].temperature, rv_motor_msg[motor_id_t].error);
    }
}

