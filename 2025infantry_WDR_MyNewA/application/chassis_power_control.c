/**
  ****************************(C) COPYRIGHT 2023 jiewen_shen****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     16-4-2023     	jiewen_shen     1.This is a chassis control code used to limit the total output power of the chassis.
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 jiewen_shen****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "user_lib.h"
#include "CAN_receive.h"
#include "CAN_CAPower.h"


extern CapDataTypedef CAP_CANData; // capacitor data structure

float input_power = 50;	
//调试变量
float debug_chassic_buffer = 0.0f;
uint8_t cap_energy_cal = 0;
float debug_chassic_limit = 0.0f;

void chassis_power_control(chassis_move_t *chassis_power_control)
{
	uint16_t max_power_limit = 40;//最大功率限制 
	fp32 chassis_max_power = 0;
	float initial_give_power[4]; // initial power from PID calculation
	float initial_total_power = 0;
	fp32 scaled_give_power[4];

	fp32 chassis_power = 0.0f;
	fp32 chassis_power_buffer = 0.0f;
	
	fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
	fp32 a = 1.23e-07;						 // k1
	fp32 k2 = 1.453e-07;					 // k2
	fp32 constant = 4.081f;
	//获取功率缓冲及上限	
	get_chassis_power_limit(&max_power_limit);  
	//从电容控制板获取底盘功率		
	chassis_power = CAP_CANData.Pout;   
	//调试DEBUG显示变量
	debug_chassic_limit = max_power_limit;        

	//计算输入功率
	input_power = 0.5f*(input_power - chassis_power) + input_power; //修bug  0.5	
	
	if(chassis_power < 0.9f*max_power_limit)
	{
		input_power = -0.5f*(max_power_limit - chassis_power) + 0.9*max_power_limit; //修bug
	}
	else if(chassis_power <= max_power_limit) 
	{
		input_power = -0.5f*(max_power_limit - chassis_power) + 0.9f*max_power_limit; //修bug
	}
	else
	{
		input_power = 0.1f*(max_power_limit - chassis_power) + max_power_limit;
	}
	// 设置电容控制器的输入功率
	if(chassis_power_control->chassis_RC->key.v & KEY_PRESSED_OFFSET_F)
	{chassis_max_power = 200;
	 input_power = 0.8*max_power_limit;
	 chassis_power_control->cap_flag = 1;
	}
	else{
	//设置不同的需求功率，控制超电选择超电或者放电
	chassis_power_control->cap_flag = 0;

	if (CAP_CANData.cap_energy > 60)
	{
		chassis_max_power = input_power +  50; // 略大于最大功率，避免电容器一直充满，提高能量利用率 50
	}
	else if (CAP_CANData.cap_energy <= 60 && CAP_CANData.cap_energy > 40)
	{
		chassis_max_power = input_power + 30;    //当电容电量不足60%，但是大于40%时，限制底盘使用能量
	}
	else if (CAP_CANData.cap_energy <= 40 && CAP_CANData.cap_energy > 30)						//当电容电量不足40%，但是大于30%时，限制底盘使用能量
	{
		chassis_max_power = input_power + 10;	
	}
	else
	{
		chassis_max_power = input_power;
	}
	}
		CAP_CAN_DataSend(&CAP_CANData, input_power-5, CAP_ENABLE);

	for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
	{
		initial_give_power[i] = chassis_power_control->motor_speed_pid[i].out * toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
								k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
								a * chassis_power_control->motor_speed_pid[i].out * chassis_power_control->motor_speed_pid[i].out + constant;

		if (initial_give_power < 0) // negative power not included (transitory)
			continue;
		initial_total_power += initial_give_power[i];
	}

	if (initial_total_power > chassis_max_power) // determine if larger than max power
	{
		fp32 power_scale = chassis_max_power / initial_total_power;   //计算放缩比例
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
			if (scaled_give_power[i] < 0)
			{
				continue;
			}

			fp32 b = toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm;
			fp32 c = k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_give_power[i] + constant;

			if (chassis_power_control->motor_speed_pid[i].out > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp > 16000)
				{
					chassis_power_control->motor_speed_pid[i].out = 16000; 
				}
				else
					chassis_power_control->motor_speed_pid[i].out = temp;
			}
			else
			{
				fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp < -16000)
				{
					chassis_power_control->motor_speed_pid[i].out = -16000;
				}
				else
					chassis_power_control->motor_speed_pid[i].out = temp;
			}
		}
	}
}

