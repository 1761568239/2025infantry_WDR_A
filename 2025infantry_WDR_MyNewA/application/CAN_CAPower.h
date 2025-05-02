/**
  ****************************(C) COPYRIGHT 2023 Zhen****************************
  * @file       CAN_CAPower.c/h
  * @brief      �����ǳ������ݿ��ư�CANͨ�Ŵ��룬����C��͵��ݰ����ݵĽ�����ͨ��ͷ�ļ��еĺ궨�����ð���.
				ʹ��ʱ���轫�ص�������CAN�����ж��е��ã�������CapDataTypedef *get_CAPower_measure_point(void)��ȡ����ָ�뼴��
				����״̬������Ŀǰ��δʹ�ã��ɲ������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0       2023-03-16     	����              1. done
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

//�������ͨ��CAN
#define HCAN_CAPOWER	 hcan1

//����ͨ��ID
#define CAP_TX_ID	    0X329
#define CAP_RX_ID	    0x330
//ʹ��ʧ�ܵ���
#define CAP_ENABLE   	0
#define CAP_DISABNLE  	1


typedef struct{
	float Pin;				//���빦��
	float Pout;				//���̹���
	float Ucap;				//���ݵ�ѹ
	uint8_t cap_energy;		//���ݵ����ٷֱ�
	uint8_t now_state;     //��Ч�� 
	uint8_t now_mode;		//��ǰģʽ

	float	Plim;			//���õ����ƹ���
	uint8_t set_mode;		//���õ�ģʽ
}CapDataTypedef;

//��������
extern CapDataTypedef CAP_CANData;

//���������Ҫ��CAN�����жϺ����е��ã��ſ�����ɵ��ݿ��ư巴�����������ݽ���
void CAP_CAN_RxCallback(CAN_RxHeaderTypeDef *prx_header, CapDataTypedef *pCAP_Data, uint8_t *rx_data);  //CAN�������ݻص�����������
void CAP_CAN_DataSend(CapDataTypedef *pCAP_Data, float lim_power, uint8_t set_mode);		//C�����ÿ��ư����


CapDataTypedef *get_CAPower_measure_point(void);  //���ؿ��ư�����ָ��

#endif
