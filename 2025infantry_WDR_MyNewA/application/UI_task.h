#ifndef UI_task_H
#define UI_task_H
#include "UI.h"
#include "chassis_task.h"

//����任
#define UI_POS_X(x)      (x)
#define UI_POS_Y(y)      (1080 - y)

typedef float float32_t;


typedef struct
{
	uint8_t fric_flag;	    //Ħ���ֱ�־
	uint8_t cap_remain_energy;	//���ݵ���
	uint8_t gyro_flag;  	//С���ݱ�־
	float   distance;			//�Ӿ����صľ���
	int16_t remain_bullet;  //��������
	uint8_t tracking_ID;    //׷�ٵ��ĳ���ID
}UI_data_t;


void UI_init(void);
void UI_update(void);

#endif
