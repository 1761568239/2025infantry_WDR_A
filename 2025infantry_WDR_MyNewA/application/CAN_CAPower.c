/**
  ****************************(C) COPYRIGHT 2023 Zhen****************************
  * @file       CAN_CAPower.c/h
  * @brief      这里是超级电容控制板CAN通信代码，负责C板和电容板数据的交互，通过头文件中的宏定义设置板子.
				使用时仅需将回调函数在CAN接收中断中调用，并调用CapDataTypedef *get_CAPower_measure_point(void)获取数据指针即可
				工作状态的数据目前暂未使用，可不予理会
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0       2023-03-16     	杨圳              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Zhen****************************
  */

#include "CAN_CAPower.h"

//电容控制器反馈数据 0x329 0x32a
//输入功率（16bit） 底盘功率（16bit） 电容电压（16bit） 电容电量（8bit） 效率（8bit） 工作状态（8bit） 
//设置限定功率	0x330
//限制功率（16bit） 设置模式（8bit）

CapDataTypedef CAP_CANData;
CAN_TxHeaderTypeDef CAP_TxHeader;	
uint8_t CAP_CAN_txbuf[8];

//CAN接收数据回调，解析数据
void CAP_CAN_RxCallback(CAN_RxHeaderTypeDef *prx_header, CapDataTypedef *pCAP_Data, uint8_t *rx_data)	
{
	if(prx_header->StdId == CAP_TX_ID)       //前8字节
	{		
		pCAP_Data->Pin  = (float)((int16_t)(rx_data[0]<<8 | rx_data[1]))/100.0f;  //输入功率
		pCAP_Data->Pout = (float)((int16_t)(rx_data[2]<<8 | rx_data[3]))/100.0f;  //底盘功率
		pCAP_Data->Ucap = (float)((int16_t)(rx_data[4]<<8 | rx_data[4]))/100.0f;  //电容电压
		pCAP_Data->cap_energy = rx_data[6];    //电容电量
		pCAP_Data->now_state = rx_data[7];    //效率
	}
}

void CAP_CAN_DataSend(CapDataTypedef *pCAP_Data, float lim_power, uint8_t set_mode)
{
	pCAP_Data->Plim = lim_power;
	pCAP_Data->set_mode = (uint8_t)((set_mode<<2)|((uint8_t)1<<1)|((uint8_t)0<<0));
	uint32_t send_mail_box;  //一般使用邮箱0
	uint16_t temp_data;
	CAP_TxHeader.StdId = CAP_RX_ID;
	CAP_TxHeader.IDE = CAN_ID_STD;
	CAP_TxHeader.RTR = CAN_RTR_DATA;
	CAP_TxHeader.DLC = 0x03;

	temp_data = pCAP_Data->Plim*100.0f;
	CAP_CAN_txbuf[0] = temp_data>>8;
	CAP_CAN_txbuf[1] = temp_data & 0xff;
	CAP_CAN_txbuf[2] = pCAP_Data->set_mode;

	HAL_CAN_AddTxMessage(&HCAN_CAPOWER, &CAP_TxHeader, CAP_CAN_txbuf, &send_mail_box);
}

//返回数据指针
CapDataTypedef *get_CAPower_measure_point(void)
{
	return &CAP_CANData;
}



