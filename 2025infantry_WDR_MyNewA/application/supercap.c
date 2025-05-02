#include "supercap.h"
#include "referee.h"
#include "CAN_CAPower.h"

extern ext_game_robot_state_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
const supercap_data_t * supercap;
CapDataTypedef pCAP_Data;
float limit_;

/*
		���Ƴ������ݵĳ�繦�� 
		���̹��� = ��繦�� + ���̵������
*/
void supercap_task(void const *pvParameters)
{
	static fp32 chassis_power=0, buffer=0;
	static fp32 err=0, err_sum=0;
	static uint16_t power_limit=40;
	while(1)
	{		
//		limit_ = POWER_INIT + (robot_state.robot_level-1) * POWER_ADD;
//		CAP_CAN_DataSend(&pCAP_Data, (limit_-10.0f), 0);				//���ͳ�����������
				
		get_chassis_power_and_buffer(&chassis_power, &buffer, &power_limit);
		
		err = power_limit - chassis_power;
		err_sum += err;
		if(err_sum >= 10.0f) err_sum=10.0f;
		
		if(chassis_power < (0.9f*power_limit))
		{
			limit_ = -(0.7f*err + 0.01f*err_sum) + power_limit; //��bug
		}
		else 
		{
			limit_ = (0.7f*err + 0.01f*err_sum) + power_limit;  //��bug
		}
		CAP_CAN_DataSend(&pCAP_Data, limit_-5, 0);				//���ͳ�����������
		vTaskDelay(50);				//100HZ����һ���趨����
	}	
}

