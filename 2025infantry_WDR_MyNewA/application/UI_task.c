#include "UI_task.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "CAN_receive.h"
#include "chassis_task.h"
#include "shoot.h"
#include "auto_shoot.h"
#include "CAN_CAPower.h"
#include "arm_math.h"
#include "referee.h"

extern RC_ctrl_t rc_ctrl;
extern received_packed_t received_packed;
extern uint8_t cap_state;
extern chassis_move_t chassis_move;
const CapDataTypedef *supercap_data;
uint8_t *fric_data;
uint8_t *gyro_data;
UI_data_t UI_data;
int temp = 0;	
int temp_y = 30;
/****************************  UI数据 ******************************/
//字符传图形
String_Data fric_flag_data;
String_Data cap_vol_data;  //cap_vol_data
String_Data dis_auto_data; //自瞄传回距离
String_Data gyro_flag_data;
String_Data pitch_dat;
String_Data yaw_dat;
//图形
Graph_Data Circle_line;
Graph_Data Circle_line_v;
Graph_Data Car_line_L;    //车道线 左
Graph_Data Car_line_R;
Graph_Data Shoot_rectangle;  
Graph_Data AT_ID_rectangle;
//弹道线
Graph_Data Shoot_horizontal_line1,Shoot_horizontal_line2,Shoot_horizontal_line3,Shoot_horizontal_line4,Shoot_horizontal_line5;
Graph_Data Shoot_vertical_line1,Shoot_vertical_line2,Shoot_vertical_line3,Shoot_vertical_line4,Shoot_vertical_line5;
//自瞄范围横线
Graph_Data vision_horizontal_line1,vision_horizontal_line2,vision_horizontal_line3,vision_horizontal_line4; 
//自瞄范围竖线
Graph_Data vision_vertical_line1,vision_vertical_line2,vision_vertical_line3,vision_vertical_line4;
Graph_Data cap_energy_percent_line;    //电容电量图形
String_Data Allow_bullet; //允许发弹量
String_Data Tracking_id;  //识别到的机器人ID
//UI字符串数据
char Tracking_id_string[20]; 
char Allow_bullet_string[20];        //允许发弹量字符串
char fric_flag_data_string[20];
char cap_vol_data_string[20];
char dis_auto_data_string[20];
char gyro_flag_data_string[20];
char pitch_data_string[40];
char yaw_data_string[40];
char auto_mode_string[20];
char speed_dat_string[20];
/****************************  UI数据 ******************************/

/**
  * @brief    UI任务入口函数          
  * @param    无
  * @param    无
  * @retval   无        
  */
void UI_task_entry(void const *pvParameters)
{		
	UI_init();
	//分配内存
	memset(&cap_vol_data, 0 ,sizeof(cap_vol_data));
	memset(&gyro_flag_data, 0, sizeof(gyro_flag_data));
	memset(&fric_flag_data, 0, sizeof(fric_flag_data));
	memset(&Car_line_L, 0, sizeof(Car_line_L));    //车道线左右
	memset(&Car_line_R, 0, sizeof(Car_line_R));
	memset(&Circle_line, 0, sizeof(Circle_line));
	memset(&Circle_line_v, 0, sizeof(Circle_line_v));
	memset(&dis_auto_data, 0, sizeof(dis_auto_data));
	//弹道线
	memset(&Shoot_horizontal_line1, 0, sizeof(Shoot_horizontal_line1));  
	memset(&Shoot_horizontal_line2, 0, sizeof(Shoot_horizontal_line2));	
	memset(&Shoot_horizontal_line3, 0, sizeof(Shoot_horizontal_line4));
	memset(&Shoot_horizontal_line4, 0, sizeof(Shoot_horizontal_line3));
	memset(&Shoot_horizontal_line5, 0, sizeof(Shoot_horizontal_line5));
	memset(&Shoot_vertical_line1, 0, sizeof(Shoot_vertical_line1));   
	memset(&Shoot_vertical_line2, 0, sizeof(Shoot_vertical_line2)); 
	memset(&Shoot_vertical_line3, 0, sizeof(Shoot_vertical_line3)); 
	memset(&Shoot_vertical_line4, 0, sizeof(Shoot_vertical_line4)); 
	memset(&Shoot_vertical_line5, 0, sizeof(Shoot_vertical_line5)); 
	//自瞄ID框
	memset(&AT_ID_rectangle, 0, sizeof(AT_ID_rectangle)); 
	//允许发弹量，ID
	memset(&Allow_bullet, 0, sizeof(Allow_bullet)); 
	memset(&Tracking_id, 0, sizeof(Tracking_id)); 
	//自瞄范围横线
	memset(&vision_horizontal_line1, 0, sizeof(vision_horizontal_line1)); 
	memset(&vision_horizontal_line2, 0, sizeof(vision_horizontal_line2)); 
	memset(&vision_horizontal_line3, 0, sizeof(vision_horizontal_line3)); 
	memset(&vision_horizontal_line4, 0, sizeof(vision_horizontal_line4)); 
	//自瞄范围竖线
	memset(&vision_vertical_line1, 0, sizeof(vision_vertical_line1)); 
	memset(&vision_vertical_line2, 0, sizeof(vision_vertical_line2)); 
	memset(&vision_vertical_line3, 0, sizeof(vision_vertical_line3)); 
	memset(&vision_vertical_line4, 0, sizeof(vision_vertical_line4)); 
	//电容电量图形线
	memset(&cap_energy_percent_line, 0, sizeof(cap_energy_percent_line)); 
	
	//打印静态字符串或字符
	sprintf(cap_vol_data_string,"CAP_en:%3d",UI_data.cap_remain_energy);
	sprintf(dis_auto_data_string, "Dis: %.2f",UI_data.distance);	
	sprintf(fric_flag_data_string,"fric: off");
	sprintf(gyro_flag_data_string,"gyro: off");	
	sprintf(Tracking_id_string, " ");      //追踪到的机器人ID
	sprintf(Allow_bullet_string, "0000");      //允许发弹量

    //初始化动态字符串
	Char_Draw(&Allow_bullet,"bul",UI_Graph_ADD,9 ,UI_Color_Green,24,strlen(Allow_bullet_string),4,UI_POS_X(1150),UI_POS_Y(547),Allow_bullet_string);  //允许发弹量
	Char_Draw(&Tracking_id,"tra",UI_Graph_ADD,0 ,UI_Color_Green,36,strlen(Tracking_id_string),4,958,720,Tracking_id_string);   //追踪ID
	Char_Draw(&dis_auto_data, "dia", UI_Graph_ADD, 9, UI_Color_Orange, 24, strlen(dis_auto_data_string), 4, 1300, 400, dis_auto_data_string);   //这两行换位置
	Char_Draw(&fric_flag_data,"fri",UI_Graph_ADD,2 ,UI_Color_White,24,strlen(fric_flag_data_string),4,UI_POS_X(100), UI_POS_Y(380),fric_flag_data_string);
	Char_Draw(&gyro_flag_data,"gyr",UI_Graph_ADD,3 ,UI_Color_White,24,strlen(gyro_flag_data_string),4,UI_POS_X(100), UI_POS_Y(460),gyro_flag_data_string);
	Char_Draw(&cap_vol_data,"cap",UI_Graph_ADD,9 ,UI_Color_Orange,24,strlen(cap_vol_data_string),4,UI_POS_X(100), UI_POS_Y(220),cap_vol_data_string);	

	//初始化静态图形
	//车道线
	Line_Draw(&Car_line_L, "sh1", UI_Graph_ADD, 5, UI_Color_Yellow, 3, UI_POS_X(745-130), UI_POS_Y(1014), UI_POS_X(890-130), UI_POS_Y(793));
	Line_Draw(&Car_line_R, "sh2", UI_Graph_ADD, 5, UI_Color_Yellow, 3, UI_POS_X(1130+130), UI_POS_Y(1014), UI_POS_X(985+130), UI_POS_Y(793));	

	Rectangle_Draw(&AT_ID_rectangle, "re2", UI_Graph_ADD, 5, UI_Color_Cyan, 3, 936, 745, 1016, 675); 		

	//弹道线  上加下减  左加右减
	Line_Draw(&Shoot_horizontal_line1, "ho1", UI_Graph_ADD,4, UI_Color_Orange, 2, UI_POS_X(920+5+temp), UI_POS_Y(630-10+temp_y), UI_POS_X(940+5+temp),  UI_POS_Y(630-10+temp_y));  //左上水平线
	Line_Draw(&Shoot_horizontal_line2, "ho2", UI_Graph_ADD,4, UI_Color_Orange, 2, UI_POS_X(970+5+temp), UI_POS_Y(630-10+temp_y), UI_POS_X(990+5+temp), UI_POS_Y(630-10+temp_y)); //右上 
	Line_Draw(&Shoot_horizontal_line3, "ho3", UI_Graph_ADD,4, UI_Color_Green, 2, UI_POS_X(920+5+temp),  UI_POS_Y(590-10+temp_y), UI_POS_X(940+5+temp),  UI_POS_Y(590-10+temp_y));   //左下
	Line_Draw(&Shoot_horizontal_line4, "ho4", UI_Graph_ADD,4, UI_Color_Green, 2, UI_POS_X(970+5+temp),  UI_POS_Y(590-10+temp_y), UI_POS_X(990+5+temp), UI_POS_Y(590-10+temp_y));  //右下
	Line_Draw(&Shoot_horizontal_line5, "ho5", UI_Graph_ADD,4, UI_Color_Green, 2, UI_POS_X(920+5+temp),  UI_POS_Y(610-10+temp_y), UI_POS_X(990+5+temp), UI_POS_Y(610-10+temp_y));  //中间
	
	Line_Draw(&Shoot_vertical_line1, "ve1", UI_Graph_ADD,4, UI_Color_Yellow, 2, UI_POS_X(940+5+temp), UI_POS_Y(630-10+temp_y), UI_POS_X(940+5+temp), UI_POS_Y(675-10+temp_y));    //左上竖直线
	Line_Draw(&Shoot_vertical_line2, "ve2", UI_Graph_ADD,4, UI_Color_Yellow, 2, UI_POS_X(970+5+temp), UI_POS_Y(630-10+temp_y), UI_POS_X(970+5+temp), UI_POS_Y(675-10+temp_y));    //右上 
	Line_Draw(&Shoot_vertical_line3, "ve3", UI_Graph_ADD,4, UI_Color_Yellow, 2, UI_POS_X(940+5+temp), UI_POS_Y(590-10+temp_y), UI_POS_X(940+5+temp), UI_POS_Y(535-10+temp_y));    //左下
	Line_Draw(&Shoot_vertical_line4, "ve4", UI_Graph_ADD,4, UI_Color_Yellow, 2, UI_POS_X(970+5+temp), UI_POS_Y(590-10+temp_y), UI_POS_X(970+5+temp), UI_POS_Y(535-10+temp_y));    //右下
	Line_Draw(&Shoot_vertical_line5, "ve5", UI_Graph_ADD,4, UI_Color_Yellow, 2, UI_POS_X(955+5+temp), UI_POS_Y(675-10+temp_y), UI_POS_X(955+5+temp), UI_POS_Y(535-10+temp_y));    //中间


	//自瞄框
	Line_Draw(&vision_horizontal_line1, "hl1", UI_Graph_ADD, 0, UI_Color_Cyan, 3, 716, 735, 746, 735);
	Line_Draw(&vision_horizontal_line2, "hl2", UI_Graph_ADD, 0, UI_Color_Cyan, 3, 716, 331, 746, 331);
	Line_Draw(&vision_horizontal_line3, "hl3", UI_Graph_ADD, 0, UI_Color_Cyan, 3, 1236, 735, 1206, 735);
	Line_Draw(&vision_horizontal_line4, "hl4", UI_Graph_ADD, 0, UI_Color_Cyan, 3, 1236, 331, 1206, 331);

	Line_Draw(&vision_vertical_line1, "vl1", UI_Graph_ADD, 0, UI_Color_Cyan, 3, 716, 331, 716, 361);
	Line_Draw(&vision_vertical_line2, "vl2", UI_Graph_ADD, 0, UI_Color_Cyan, 3, 1236, 331, 1236, 361);
	Line_Draw(&vision_vertical_line3, "vl3", UI_Graph_ADD, 0, UI_Color_Cyan, 3, 716, 735, 716, 705);
	Line_Draw(&vision_vertical_line4, "vl4", UI_Graph_ADD, 0, UI_Color_Cyan, 3, 1236, 735, 1236, 705);
	//电容电量
	Line_Draw(&cap_energy_percent_line, "caV", UI_Graph_ADD, 0, UI_Color_Green, 20,UI_POS_X(10),UI_POS_Y(300), UI_POS_X(316), UI_POS_Y(300));//5m线横，画好


	for(int i=0;i<10;i++)
	{	
		//UI刷新10hz
		Char_ReFresh(cap_vol_data);
		vTaskDelay(100);
		Char_ReFresh(fric_flag_data);
		vTaskDelay(100);
		Char_ReFresh(gyro_flag_data);
		vTaskDelay(100);
		Char_ReFresh(dis_auto_data);
		vTaskDelay(100);			
		UI_ReFresh(2, Car_line_L, Car_line_R);  //画图形
		vTaskDelay(100);
		UI_ReFresh(2, Circle_line, Circle_line_v);
		vTaskDelay(100);
		UI_ReFresh(2, vision_horizontal_line4,AT_ID_rectangle); 
		vTaskDelay(100);
		UI_ReFresh(7, vision_vertical_line1,vision_vertical_line2,vision_vertical_line3,vision_vertical_line4,\
		vision_horizontal_line1,vision_horizontal_line2,vision_horizontal_line3); 
		vTaskDelay(100);
		//弹道线
		UI_ReFresh(5, Shoot_horizontal_line1,Shoot_horizontal_line2,Shoot_horizontal_line3,Shoot_horizontal_line4,Shoot_horizontal_line5); 
		vTaskDelay(100);
		UI_ReFresh(5, Shoot_vertical_line1,Shoot_vertical_line2,Shoot_vertical_line3,Shoot_vertical_line4,Shoot_vertical_line5); 
		
		Char_ReFresh(Allow_bullet);    //发弹量和追踪ID
		vTaskDelay(100);
		Char_ReFresh(Tracking_id);
		vTaskDelay(100);

		UI_ReFresh(1, cap_energy_percent_line);
		vTaskDelay(100);
	}
	while(1)
	{
		//更新显示数据
		UI_update();
		//打印动态字符串或者图形
		sprintf(cap_vol_data_string,"CAP_en:%3d",UI_data.cap_remain_energy);
		sprintf(dis_auto_data_string, "Dis: %.2f",UI_data.distance);
		sprintf(Tracking_id_string, "%0d",UI_data.tracking_ID);
		sprintf(Allow_bullet_string, "%4d",UI_data.remain_bullet);

		if(UI_data.fric_flag == 1)
			sprintf(fric_flag_data_string,"fric: on ");   //根据按键状态更新字符显示
		else		
			sprintf(fric_flag_data_string,"fric: off");
		if(UI_data.gyro_flag == 1)
			sprintf(gyro_flag_data_string,"gyro: on ");
		else
			sprintf(gyro_flag_data_string,"gyro: off");
				
		//动态显示识别到的装甲板ID
		Char_Draw(&Tracking_id,"tra",UI_Graph_Change,0 ,UI_Color_Green,36,strlen(Tracking_id_string),4,966,720,Tracking_id_string);   //追踪ID
		Char_ReFresh(Tracking_id);	
		vTaskDelay(50);
		
		//显示自瞄装甲板距离
		Char_Draw(&dis_auto_data, "dia", UI_Graph_Change, 9, UI_Color_Orange, 24, strlen(dis_auto_data_string), 4, 1300, 400, dis_auto_data_string);
		Char_ReFresh(dis_auto_data);
		vTaskDelay(50);
		
		//显示允许发弹量
		Char_Draw(&Allow_bullet,"bul",UI_Graph_Change,9 ,UI_Color_Green,24,strlen(Allow_bullet_string),4,UI_POS_X(1150),UI_POS_Y(547),Allow_bullet_string);  //允许发弹量	
		Char_ReFresh(Allow_bullet);	  
		vTaskDelay(50);	

		//摩擦轮颜色改变
		if(UI_data.fric_flag)
		{
			Char_Draw(&fric_flag_data,"fri",UI_Graph_Change,2 ,UI_Color_Purplish_red,24,strlen(fric_flag_data_string),4,UI_POS_X(100), UI_POS_Y(380),fric_flag_data_string);				
		}
		else
		{
			Char_Draw(&fric_flag_data,"fri",UI_Graph_Change,2 ,UI_Color_White,24,strlen(fric_flag_data_string),4,UI_POS_X(100), UI_POS_Y(380),fric_flag_data_string);				
		}
		Char_ReFresh(fric_flag_data);
		vTaskDelay(50);
		//小陀螺颜色改变
		if(UI_data.gyro_flag)
		{
			Char_Draw(&gyro_flag_data,"gyr",UI_Graph_Change,3 ,UI_Color_Purplish_red,24,strlen(gyro_flag_data_string),4,UI_POS_X(100), UI_POS_Y(460),gyro_flag_data_string);			
		}
		else
		{
			Char_Draw(&gyro_flag_data,"gyr",UI_Graph_Change,3 ,UI_Color_White,24,strlen(gyro_flag_data_string),4,UI_POS_X(100), UI_POS_Y(460),gyro_flag_data_string);
		}
		Char_ReFresh(gyro_flag_data);
		vTaskDelay(50);
		//动态显示电容电量
		if(chassis_move.cap_flag == 1)
		{
			Char_Draw(&cap_vol_data,"cap",UI_Graph_Change,9 ,UI_Color_Yellow,24,strlen(cap_vol_data_string),4,UI_POS_X(100), UI_POS_Y(220),cap_vol_data_string);			
			Line_Draw(&cap_energy_percent_line, "caV", UI_Graph_Change, 0, UI_Color_Yellow, 15,UI_POS_X(10),UI_POS_Y(300), UI_POS_X( UI_data.cap_remain_energy*3 +10), UI_POS_Y(300));
		}
		else
		{
			if(UI_data.cap_remain_energy >70)
			{
				Char_Draw(&cap_vol_data,"cap",UI_Graph_Change,9 ,UI_Color_Green,24,strlen(cap_vol_data_string),4,UI_POS_X(100), UI_POS_Y(220),cap_vol_data_string);
				Line_Draw(&cap_energy_percent_line, "caV", UI_Graph_Change, 0, UI_Color_Green, 15,UI_POS_X(10),UI_POS_Y(300), UI_POS_X( UI_data.cap_remain_energy*3 +10), UI_POS_Y(300));		
			}else if(UI_data.cap_remain_energy <= 70 && UI_data.cap_remain_energy >40)
			{
				Char_Draw(&cap_vol_data,"cap",UI_Graph_Change,9 ,UI_Color_Orange,24,strlen(cap_vol_data_string),4,UI_POS_X(100), UI_POS_Y(220),cap_vol_data_string);			
				Line_Draw(&cap_energy_percent_line, "caV", UI_Graph_Change, 0, UI_Color_Orange, 15,UI_POS_X(10),UI_POS_Y(300), UI_POS_X( UI_data.cap_remain_energy*3 +10), UI_POS_Y(300));
			}else if(UI_data.cap_remain_energy <= 50)
			{
				Char_Draw(&cap_vol_data,"cap",UI_Graph_Change,9 ,UI_Color_Purplish_red,24,strlen(cap_vol_data_string),4,UI_POS_X(100), UI_POS_Y(220),cap_vol_data_string);			
				Line_Draw(&cap_energy_percent_line, "caV", UI_Graph_Change, 0, UI_Color_Purplish_red, 15,UI_POS_X(10),UI_POS_Y(300), UI_POS_X( UI_data.cap_remain_energy*3 +10), UI_POS_Y(300));
			}			
		}
		//刷新	
		Char_ReFresh(cap_vol_data);
		vTaskDelay(50);		
		UI_ReFresh(1,cap_energy_percent_line);
		vTaskDelay(100);
		//动态显示自瞄框
		if(UI_data.tracking_ID)
		{
			Line_Draw(&vision_horizontal_line1, "hl1", UI_Graph_Change, 0, UI_Color_Purplish_red, 3, 716, 735, 746, 735);
			Line_Draw(&vision_horizontal_line2, "hl2", UI_Graph_Change, 0, UI_Color_Purplish_red, 3, 716, 331, 746, 331);
			Line_Draw(&vision_horizontal_line3, "hl3", UI_Graph_Change, 0, UI_Color_Purplish_red, 3, 1236, 735, 1206, 735);
			Line_Draw(&vision_horizontal_line4, "hl4", UI_Graph_Change, 0, UI_Color_Purplish_red, 3, 1236, 331, 1206, 331);

			Line_Draw(&vision_vertical_line1, "vl1", UI_Graph_Change, 0, UI_Color_Purplish_red, 3, 716, 331, 716, 361);
			Line_Draw(&vision_vertical_line2, "vl2", UI_Graph_Change, 0, UI_Color_Purplish_red, 3, 1236, 331, 1236, 361);
			Line_Draw(&vision_vertical_line3, "vl3", UI_Graph_Change, 0, UI_Color_Purplish_red, 3, 716, 735, 716, 705);
			Line_Draw(&vision_vertical_line4, "vl4", UI_Graph_Change, 0, UI_Color_Purplish_red, 3, 1236, 735, 1236, 705);
		}
		else 
		{
			Line_Draw(&vision_horizontal_line1, "hl1", UI_Graph_Change, 0, UI_Color_Cyan, 3, 716, 735, 746, 735);
			Line_Draw(&vision_horizontal_line2, "hl2", UI_Graph_Change, 0, UI_Color_Cyan, 3, 716, 331, 746, 331);
			Line_Draw(&vision_horizontal_line3, "hl3", UI_Graph_Change, 0, UI_Color_Cyan, 3, 1236, 735, 1206, 735);
			Line_Draw(&vision_horizontal_line4, "hl4", UI_Graph_Change, 0, UI_Color_Cyan, 3, 1236, 331, 1206, 331);

			Line_Draw(&vision_vertical_line1, "vl1", UI_Graph_Change, 0, UI_Color_Cyan, 3, 716, 331, 716, 361);
			Line_Draw(&vision_vertical_line2, "vl2", UI_Graph_Change, 0, UI_Color_Cyan, 3, 1236, 331, 1236, 361);
			Line_Draw(&vision_vertical_line3, "vl3", UI_Graph_Change, 0, UI_Color_Cyan, 3, 716, 735, 716, 705);
			Line_Draw(&vision_vertical_line4, "vl4", UI_Graph_Change, 0, UI_Color_Cyan, 3, 1236, 735, 1236, 705);
		}	
		UI_ReFresh(7, vision_vertical_line1,vision_vertical_line2,vision_vertical_line3,vision_vertical_line4,\
				   vision_horizontal_line1,vision_horizontal_line2,vision_horizontal_line3); 
		vTaskDelay(100);
		UI_ReFresh(1, vision_horizontal_line4);  
		vTaskDelay(100);
		
//		//系统复位
//		if((rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)&&(rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL))
//		{
//			NVIC_SystemReset();
//		}
	}	
}


/**
  * @brief    UI数据初始化，初始化第一次显示的数据              
  * @param    无
  * @param    无
  * @retval   无      
  */
void UI_init()  //取数据指针并数据更新
{
	supercap_data = get_CAPower_measure_point();
	fric_data = get_fric_flag();    //函数定义shoot.c
	gyro_data = get_gyro_flag();

	UI_data.cap_remain_energy = supercap_data->cap_energy;
	UI_data.fric_flag = *fric_data;
	UI_data.gyro_flag = *gyro_data;

	UI_data.remain_bullet = get_remain_bullet_num();  //读取允许发弹量
	UI_data.tracking_ID = received_packed.id;         //获取识别到的车的ID
	UI_data.distance = get_UI_dis_data();
}

/**
  * @brief  UI数据更新                 
  * @param  无  
  * @param  无
  * @retval 无          
  */
void UI_update(void)
{
	UI_data.remain_bullet = get_remain_bullet_num();   //更新允许发弹量
	UI_data.tracking_ID = received_packed.id;          //更新识别到的车的ID
	UI_data.distance = get_UI_dis_data();
	
	UI_data.cap_remain_energy = supercap_data->cap_energy;       //读取电容电量
	UI_data.fric_flag = *fric_data; 
	UI_data.gyro_flag = *gyro_data;	
}
