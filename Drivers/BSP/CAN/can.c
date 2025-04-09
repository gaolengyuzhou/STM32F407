/**
 ****************************************************************************************************
 * @file        can.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.1
 * @date        2023-06-06
 * @brief       CAN 驱动代码
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
 * V1.0 20211025
 * 第一次发布
 * V1.1 20230606
 * 1, 优化can_send_msg函数, 新增发送超时处理机制
 ****************************************************************************************************
 */

#include "./BSP/CAN/can.h"
#include "./BSP/LED/led.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"


CAN_HandleTypeDef   g_canx_handler;     /* CANx句柄 */
CAN_TxHeaderTypeDef g_canx_txheader;    /* 发送参数句柄 */
CAN_RxHeaderTypeDef g_canx_rxheader;    /* 接收参数句柄 */

/**
 * @brief       CAN初始化
 * @param       tsjw    : 重新同步跳跃时间单元.范围: 1~3;
 * @param       tbs2    : 时间段2的时间单元.范围: 1~8;
 * @param       tbs1    : 时间段1的时间单元.范围: 1~16;
 * @param       brp     : 波特率分频器.范围: 1~1024;
 *   @note      以上4个参数, 在函数内部会减1, 所以, 任何一个参数都不能等于0
 *              CAN挂在APB1上面, 其输入时钟频率为 Fpclk1 = PCLK1 = 42Mhz
 *              tq     = brp * tpclk1;
 *              波特率 = Fpclk1 / ((tbs1 + tbs2 + 1) * brp);
 *              我们设置 can_init(1, 6, 7, 6, 1), 则CAN波特率为:
 *              42M / ((6 + 7 + 1) * 6) = 500Kbps
 *
 * @param       mode    : CAN_MODE_NORMAL,  普通模式;
                          CAN_MODE_LOOPBACK,回环模式;
 * @retval      0,  初始化成功; 其他, 初始化失败;
 */
uint8_t can_init(uint32_t tsjw, uint32_t tbs2, uint32_t tbs1, uint16_t brp, uint32_t mode)
{
    g_canx_handler.Instance = CAN1;
    g_canx_handler.Init.Prescaler = brp;                /* 分频系数(Fdiv)为brp+1 */
    g_canx_handler.Init.Mode = mode;                    /* 模式设置 */
    g_canx_handler.Init.SyncJumpWidth = tsjw;           /* 重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1TQ~CAN_SJW_4TQ */
    g_canx_handler.Init.TimeSeg1 = tbs1;                /* tbs1范围CAN_BS1_1TQ~CAN_BS1_16TQ */
    g_canx_handler.Init.TimeSeg2 = tbs2;                /* tbs2范围CAN_BS2_1TQ~CAN_BS2_8TQ */
    g_canx_handler.Init.TimeTriggeredMode = DISABLE;    /* 非时间触发通信模式 */
    g_canx_handler.Init.AutoBusOff = DISABLE;           /* 软件自动离线管理 */
    g_canx_handler.Init.AutoWakeUp = DISABLE;           /* 睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位) */
    g_canx_handler.Init.AutoRetransmission = ENABLE;    /* ！！！没有影响！！！禁止报文自动传送 */
    g_canx_handler.Init.ReceiveFifoLocked = DISABLE;    /* 报文不锁定,新的覆盖旧的 */
    g_canx_handler.Init.TransmitFifoPriority = DISABLE; /* 优先级由报文标识符决定 */
    if (HAL_CAN_Init(&g_canx_handler) != HAL_OK)
    {
        return 1;
    }

    CAN_FilterTypeDef sFilterConfig;

    /* 配置CAN过滤器 */
    sFilterConfig.FilterBank = 0;                             /* 过滤器0 */
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;                      /* 32位ID */
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;                  /* 32位MASK */
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;    /* 过滤器0关联到FIFO0 */
    sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;       /* 激活滤波器0 */
//    sFilterConfig.SlaveStartFilterBank = 14;


    /* 过滤器配置 */
    if (HAL_CAN_ConfigFilter(&g_canx_handler, &sFilterConfig) != HAL_OK)
    {
        return 2;
    }

    /* 使用中断接收 */
//    __HAL_CAN_ENABLE_IT(&g_canx_handler, CAN_IT_RX_FIFO0_MSG_PENDING); /* FIFO0消息挂号中断允许 */
//    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);                                 /* 使能CAN中断 */
//    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);                         /* 抢占优先级1，子优先级0 */

    /* 启动CAN外围设备 */
    if (HAL_CAN_Start(&g_canx_handler) != HAL_OK)
    {
        return 3;
    }


    return 0;
}

/**
 * @brief       CAN底层驱动，引脚配置，时钟配置，中断配置
                此函数会被HAL_CAN_Init()调用
 * @param       hcan:CAN句柄
 * @retval      无
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    if (CAN1 == hcan->Instance)
    {
        CAN_RX_GPIO_CLK_ENABLE();       /* CAN_RX脚时钟使能 */
        CAN_TX_GPIO_CLK_ENABLE();       /* CAN_TX脚时钟使能 */
        __HAL_RCC_CAN1_CLK_ENABLE();    /* 使能CAN1时钟 */

        GPIO_InitTypeDef gpio_init_struct;

        gpio_init_struct.Pin = CAN_TX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(CAN_TX_GPIO_PORT, &gpio_init_struct); /* CAN_TX脚 模式设置 */

        gpio_init_struct.Pin = CAN_RX_GPIO_PIN;
        HAL_GPIO_Init(CAN_RX_GPIO_PORT, &gpio_init_struct); /* CAN_RX脚 必须设置成输入模式 */
    }
}


/**
 * @brief       CAN RX0 中断服务函数
 *   @note      处理CAN FIFO0的接收中断
 * @param       无
 * @retval      无
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
		LED0_TOGGLE();
//    can_receive_msg(id, rxbuf);
}
//void  HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  //接收回调函数（函数名不可改）
//{
//		uint8_t  Data[8];//接收缓存数组
//		HAL_StatusTypeDef HAL_RetVal;//判断状态的枚举
//		HAL_RetVal=HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&g_canx_rxheader,Data);//接收邮箱中的数据
//		if (HAL_RetVal==HAL_OK){//判断接收是否成功
//			//接收成功后的数据处理程序，写在此处。（数据在Data数组中）
//			//以下2行是采用简单的寄存器查寻方式处理接收数据，每次只接收1位。在实际项目中的复杂接收程序可自行编写。
//			LED1_TOGGLE();
//		}
//}


//void CAN1_RX0_IRQHandler(void)
//{
//	LED1_TOGGLE();
//}

/**
 * @brief       CAN 发送一组数据
 *   @note      发送格式固定为: 标准ID, 数据帧
 * @param       id      : 标准ID(11位)
 * @param       msg     : 数据指针
 * @param       len     : 数据长度
 * @retval      发送状态 0, 成功; 1, 失败;
 */
uint8_t can_send_msg(uint32_t id, uint8_t *msg, uint8_t len)
{
    uint16_t t = 0;
    uint32_t TxMailbox = CAN_TX_MAILBOX0;
    
    g_canx_txheader.StdId = id;         /* 标准标识符 */
    g_canx_txheader.ExtId = id;         /* 扩展标识符(29位) */
    g_canx_txheader.IDE = CAN_ID_STD;   /* 使用标准帧 */
    g_canx_txheader.RTR = CAN_RTR_DATA; /* 数据帧 */
    g_canx_txheader.DLC = len;

    if (HAL_CAN_AddTxMessage(&g_canx_handler, &g_canx_txheader, msg, &TxMailbox) != HAL_OK) /* 发送消息 */
    {
        return 1;
    }
    
    while (HAL_CAN_GetTxMailboxesFreeLevel(&g_canx_handler) != 3)   /* 等待发送完成,所有邮箱为空 */
    {
        t++;
        
        if (t > 0xFFF)
        {
            HAL_CAN_AbortTxRequest(&g_canx_handler, TxMailbox);     /* 超时，直接中止邮箱的发送请求 */
            return 1;
        }
    }
    
    return 0;
}

/**
 * @brief       CAN 接收数据查询
 *   @note      接收数据格式固定为: 标准ID, 数据帧
 * @param       id      : 要查询的 标准ID(11位)
 * @param       buf     : 数据缓存区
 * @retval      接收结果
 *   @arg       0   , 无数据被接收到;
 *   @arg       其他, 接收的数据长度
 */
uint8_t can_receive_msg(uint32_t id, uint8_t *buf)
{
    if (HAL_CAN_GetRxFifoFillLevel(&g_canx_handler, CAN_RX_FIFO0) == 0)     /* 没有接收到数据 */
    {
        return 0;
    }

    if (HAL_CAN_GetRxMessage(&g_canx_handler, CAN_RX_FIFO0, &g_canx_rxheader, buf) != HAL_OK)  /* 读取数据 */
    {
        return 0;
    }

//    if (g_canx_rxheader.StdId!= id || g_canx_rxheader.IDE != CAN_ID_STD || g_canx_rxheader.RTR != CAN_RTR_DATA)       /* 接收到的ID不对 / 不是标准帧 / 不是数据帧 */
//    {
//        return 0;    
//    }

    return g_canx_rxheader.DLC;

}
