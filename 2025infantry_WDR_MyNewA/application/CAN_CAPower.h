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

#ifndef CAN_CAPOWER_H
#define CAN_CAPOWER_H

#include "main.h"
#include "can.h"

//定义电容通信CAN
#define HCAN_CAPOWER	 hcan1

//电容通信ID
#define CAP_TX_ID	    0X329
#define CAP_RX_ID	    0x330
//使能失能电容
#define CAP_ENABLE   	0
#define CAP_DISABNLE  	1


typedef struct{
	float Pin;				//输入功率
	float Pout;				//底盘功率
	float Ucap;				//电容电压
	uint8_t cap_energy;		//电容电量百分比
	uint8_t now_state;     //总效率 
	uint8_t now_mode;		//当前模式

	float	Plim;			//设置的限制功率
	uint8_t set_mode;		//设置的模式
}CapDataTypedef;

//变量申明
extern CapDataTypedef CAP_CANData;

//这个函数需要在CAN接收中断函数中调用，才可以完成电容控制板反馈回来的数据解析
void CAP_CAN_RxCallback(CAN_RxHeaderTypeDef *prx_header, CapDataTypedef *pCAP_Data, uint8_t *rx_data);  //CAN接收数据回调，解析数据
void CAP_CAN_DataSend(CapDataTypedef *pCAP_Data, float lim_power, uint8_t set_mode);		//C板设置控制板参数


CapDataTypedef *get_CAPower_measure_point(void);  //返回控制板数据指针

#endif
