#include "./USMART/usmart.h"
#include "./USMART/usmart_str.h"

/******************************************************************************************/
/* 用户配置区
 * 这下面要包含所用到的函数所申明的头文件(用户自己添加)
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/CAN/can.h"
#include "DHT22.h"
#include "can_uc.h"

/* 函数名列表初始化(用户自己添加)
 * 用户直接在这里输入要执行的函数名及其查找串
 */
struct _m_usmart_nametab usmart_nametab[] =
{
#if USMART_USE_WRFUNS == 1      /* 如果使能了读写操作 */
    (void *)read_addr, "uint32_t read_addr(uint32_t addr)",
    (void *)write_addr, "void write_addr(uint32_t addr, uint32_t val)",
#endif
    (void *)delay_ms, "void delay_ms(uint16_t nms)",
    (void *)delay_us, "void delay_us(uint32_t nus)",
        
		(void *)Get_temperature,"float Get_temperature(void)",
    (void *)Get_humidity,"float Get_humidity(void)",
    (void *)can_send_msg, "uint8_t can_send_msg(uint32_t id, uint8_t *msg, uint8_t len)",
		(void *)send_motor_ctrl_cmd,"uint8_t send_motor_ctrl_cmd(uint16_t motor_id,float kp,float kd,float pos,float spd,float tor)",
		(void *)send_motor_ctrl_cmd_65536,"uint8_t send_motor_ctrl_cmd_65536(uint16_t motor_id, uint16_t kp, uint16_t kd, uint16_t pos, uint16_t spd, uint16_t tor)",
		(void *)set_motor_position_65535,"uint8_t set_motor_position_65535(uint16_t motor_id, uint16_t pos, uint16_t spd, uint16_t cur, uint8_t ack_status)",
		(void *)set_motor_speed_36000,"uint8_t set_motor_speed_36000(uint16_t motor_id, uint32_t spd_32, uint16_t cur, uint8_t ack_status)",
		(void *)MotorSetting,"uint8_t MotorSetting(uint16_t motor_id,uint8_t cmd)",
		(void *)set_motor_can_timeConfig,"uint8_t set_motor_can_timeConfig(uint16_t motor_id,uint16_t timeout)",
		
};

/******************************************************************************************/

/* 函数控制管理器初始化
 * 得到各个受控函数的名字
 * 得到函数总数量
 */
struct _m_usmart_dev usmart_dev =
{
    usmart_nametab,
    usmart_init,
    usmart_cmd_rec,
    usmart_exe,
    usmart_scan,
    sizeof(usmart_nametab) / sizeof(struct _m_usmart_nametab), /* 函数数量 */
    0,      /* 参数数量 */
    0,      /* 函数ID */
    1,      /* 参数显示类型,0,10进制;1,16进制 */
    0,      /* 参数类型.bitx:,0,数字;1,字符串 */
    0,      /* 每个参数的长度暂存表,需要MAX_PARM个0初始化 */
    0,      /* 函数的参数,需要PARM_LEN个0初始化 */
};



















