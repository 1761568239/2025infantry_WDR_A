#ifndef UI_task_H
#define UI_task_H
#include "UI.h"
#include "chassis_task.h"

//坐标变换
#define UI_POS_X(x)      (x)
#define UI_POS_Y(y)      (1080 - y)

typedef float float32_t;


typedef struct
{
	uint8_t fric_flag;	    //摩擦轮标志
	uint8_t cap_remain_energy;	//电容电量
	uint8_t gyro_flag;  	//小陀螺标志
	float   distance;			//视觉读回的距离
	int16_t remain_bullet;  //允许发弹量
	uint8_t tracking_ID;    //追踪到的车的ID
}UI_data_t;


void UI_init(void);
void UI_update(void);

#endif
