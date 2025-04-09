/*
 * can.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#ifndef CAN_UC_H_
#define CAN_UC_H_


#define param_get_pos	0x01
#define param_get_spd	0x02
#define param_get_cur	0x03
#define param_get_pwr	0x04
#define param_get_acc	0x05
#define param_get_lkgKP	0x06
#define param_get_spdKI	0x07
#define param_get_fdbKP	0x08
#define param_get_fdbKD	0x09

#define comm_ack	0x00
#define comm_auto	0x01

typedef struct 
{
	uint16_t motor_id;
	uint8_t INS_code;		//instruction code.
	uint8_t motor_fbd;	//motor CAN communication feedback.
}MotorCommFbd;

typedef struct 
{
	uint16_t angle_actual_int;
	uint16_t angle_desired_int;
	int16_t speed_actual_int;
	int16_t speed_desired_int;
	int16_t current_actual_int;
	int16_t current_desired_int;
	float 	speed_actual_rad;
	float 	speed_desired_rad;
	float 	angle_actual_rad;	
	float   angle_desired_rad;
	uint16_t	motor_id;
	uint8_t 	temperature;
	uint8_t		error;
	float     angle_actual_float;
	float 		speed_actual_float;
	float 		current_actual_float;
	float     angle_desired_float;
	float 		speed_desired_float;
	float 		current_desired_float;
	float			power;
	uint16_t	acceleration;
	uint16_t	linkage_KP;
	uint16_t 	speed_KI;
	uint16_t	feedback_KP;
	uint16_t	feedback_KD;
}OD_Motor_Msg;

extern OD_Motor_Msg rv_motor_msg[8];
extern uint16_t motor_id_check;


/*
uint16_t kp = 2048;     // 比例系数		0-4095
uint16_t kd = 256;      // 微分系数		0-511
uint16_t pos = 32768;   // 期望位置		0-65536  
uint16_t spd = 2048;    // 期望速度		0-4095 
uint16_t tor = 2048;    // 前馈扭矩		0-4095
send_motor_pos_65536(1,2048,256,32768,2048,2048);
*/
uint8_t send_motor_ctrl_cmd_65536(uint16_t motor_id, uint16_t kp, uint16_t kd, uint16_t pos, uint16_t spd, uint16_t tor);
/*
pos： 0-65535 对应 0-360°0-60000
spd：0-18000
cur：0-3000
set_motor_position(1, 65535 / 3, 1000, 1500, 1); // 假设设置到 120° 位置
*/
uint8_t set_motor_position_65535(uint16_t motor_id, uint16_t pos, uint16_t spd, uint16_t cur, uint8_t ack_status);
//set_motor_speed_36000(1,18000,100,1)
uint8_t set_motor_speed_36000(uint16_t motor_id, uint32_t spd_32, uint16_t cur, uint8_t ack_status);





/*
////////////////////////////////////7.2电机设置指令、7.3电机零点设置指令////////////////////////////////////
cmd:
0x00:NON（通信模式：问答模式（默认）、自动报文模式）
0x01:set the communication mode to automatic feedback.
0x02:set the communication mode to response.
0x03:set the current position to zero.
*/
uint8_t MotorSetting(uint16_t motor_id,uint8_t cmd);
/*
////////////////////////////////////7.4电机CAN 通信ID 设置指令////////////////////////////////////
*/
uint8_t MotorIDSetting(uint16_t motor_id,uint16_t motor_id_new);
/*
////////////////////////////////////7.5电机CAN 通信ID 重置指令////////////////////////////////////
*/
uint8_t MotorIDReset(void);
/*
////////////////////////////////////8.1查询电机通信模式////////////////////////////////////
返回报文4字节：ID高|ID低|01|**
0x01：自动报文模式
0x02：问答模式
0x80：错误
*/
uint8_t MotorCommModeReading(uint16_t motor_id);
/*
////////////////////////////////////8.2查询CAN 通信ID////////////////////////////////////
返回报文成功5字节 ID高|ID低、失败4字节0x08
*/
uint8_t MotorIDReading(void);
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
uint8_t send_motor_ctrl_cmd(uint16_t motor_id,float kp,float kd,float pos,float spd,float tor);
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
uint8_t set_motor_position(uint16_t motor_id, float pos, uint16_t spd, uint16_t cur, uint8_t ack_status);
/*
////////////////////////////////////9.1.3 伺服速度控制模式指令////////////////////////////////////
set_motor_speed(1,100.0,100,1)
motor_id:1-0x7FE
spd:，CAN 数值与实际转速一一对应（RPM），！！！范围：-18000-18000
cur:0-65536 --> 0-6553.6A，比例为10，！！！！！范围：0-3000
ack_status:0-3
*/
uint8_t set_motor_speed(uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status);
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
uint8_t set_motor_cur_tor(uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status);
/*
////////////////////////////////////9.2.1 电机加速度配置指令////////////////////////////////////
motor_id:1-0x7FE
acc:0-2000
加速度数值：0-2000 --> 0-20rad/s，内部已限幅。加速度减小，运行会更柔和，但是
过小易超调，系统默认值为2000。
ack_status:0-3
*/
uint8_t set_motor_acceleration(uint16_t motor_id,uint16_t acc,uint8_t ack_status);
/*
////////////////////////////////////9.2.2 电机扭矩系数配置指令////////////////////////////////////
motor_id:1-0x7FE
torconfig电机扭矩系数：0-65536。250 对应 2.5，比例 100。
*/
uint8_t set_motor_TorqueConfig(uint16_t motor_id,uint16_t torconfig);
/*
////////////////////////////////////9.2.3 电机力位混控协议 KP 配置指令////////////////////////////////////
motor_id:1-0x7FE
kp_min和kp_max比例 1，1就是1，0就是0，500就是500
*/
uint8_t set_motor_FP_kpConfig(uint16_t motor_id,uint16_t kp_min,uint16_t kp_max);
/*
////////////////////////////////////9.2.4 电机力位混控协议 KD 配置指令////////////////////////////////////
motor_id:1-0x7FE
kd_min和kd_max比例 1，1就是1，0就是0，500就是500
*/
uint8_t set_motor_FP_kdConfig(uint16_t motor_id,uint16_t kd_min,uint16_t kd_max);
/*
////////////////////////////////////9.2.5 电机力位混控协议位置配置指令////////////////////////////////////
motor_id:1-0x7FE
pos_min和pos_max
-1250 对应-12.5（rad），比例 100。
*/
uint8_t set_motor_FP_posConfig(uint16_t motor_id,int16_t pos_min,int16_t pos_max);
/*
////////////////////////////////////9.2.6 电机力位混控协议速度配置指令////////////////////////////////////
motor_id:1-0x7FE
spd_min和spd_max
SPD MIN：-1800 对应-18（rad/s），比例 100。
*/
uint8_t set_motor_FP_spdConfig(uint16_t motor_id,int16_t spd_min,int16_t spd_max);
/*
////////////////////////////////////9.2.7 电机力位混控协议扭矩配置指令////////////////////////////////////
motor_id:1-0x7FE
tor_min和tor_max
-900 对应-90（Nm），比例 10。
*/
uint8_t set_motor_FP_torConfig(uint16_t motor_id,int16_t tor_min,int16_t tor_max);
/*
////////////////////////////////////9.2.8 电机力位混控协议电流配置指令////////////////////////////////////
motor_id:1-0x7FE
cur_min和cur_max
CUR MIN：-900 对应-90（A），比例 10。
*/
uint8_t set_motor_FP_curConfig(uint16_t motor_id,int16_t cur_min,int16_t cur_max);
/*
////////////////////////////////////9.2.9 电机 CAN 超时时间配置指令////////////////////////////////////
motor_id:1-0x7FE
电机电机 CAN 超时时间：500 对应 500（ms），比例 1。
时间设置为 0 时即关闭 CAN 超时保护。
*/
uint8_t set_motor_can_timeConfig(uint16_t motor_id,uint16_t timeout);
/*
////////////////////////////////////9.2.10 电机电流环 PI 配置指令////////////////////////////////////
motor_id:1-0x7FE
电流环 Current KP：100 对应 0.01，比例 10000。
电流环 Current KI：500 对应 50，比例 10。
*/
uint8_t set_motor_currentloop_piConfig(uint16_t motor_id,uint16_t cur_kp,uint16_t cur_ki);
/*
////////////////////////////////////9.2.11 电机速度环 PI 配置指令////////////////////////////////////
motor_id:1-0x7FE
电流环 Speed KP：600 对应 0.006，比例 100000。
电流环 Speed KI：10000 对应 0.1，比例 100000。
*/
uint8_t set_motor_speedloop_piConfig(uint16_t motor_id,uint16_t spd_kp,uint16_t spd_ki);
/*
////////////////////////////////////9.2.12 电机位置环 PD 配置指令////////////////////////////////////
motor_id:1-0x7FE
位置环 Position KP：600 对应 0.006，比例 100000。
位置环 Position KD：4 对应 0.00004，比例 100000。
*/
uint8_t set_motor_position_pdConfig(uint16_t motor_id,uint16_t pos_kp,uint16_t pos_kd);
/*
////////////////////////////////////9.2.13 电机扭矩系数 Kt 校准使能配置指令////////////////////////////////////
motor_id:1-0x7FE
0（关闭），1（打开）。
*/
uint8_t set_motor_torque_ktConfig(uint16_t motor_id,uint8_t tor_kt);
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
uint8_t get_motor_parameter(uint16_t motor_id,uint8_t param_cmd);






void RV_can_data_repack(CAN_RxHeaderTypeDef *RxHeader, uint8_t *data, uint8_t comm_mode);


#endif /* CAN_UC_H_ */
