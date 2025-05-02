/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"
#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "auto_shoot.h"
#include "gimbal_task.h"
#include "SolveTrajectory.h"

//========================================== 静态函数申明 ==============================================    
/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);
//========================================== 静态函数申明 ==============================================


//========================================== 全局变量定义区域 ==============================================
//注意：任何任务文件全局变量定义区域只应存在一个变量，即该任务控制结构体！
//射击控制结构体
shoot_control_t shoot_control; 
//========================================== 全局变量定义区域 ==============================================




//========================================== 全局变量区域 ==============================================
extern gimbal_control_t gimbal_control;
//========================================== 全局变量区域 ==============================================
/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{
    //拨弹电机的PID
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
    //设计速度的PID
	static const fp32 Shoot_L_speed_pid[3] = {SHOOT_L_SPEED_PID_KP, SHOOT_L_SPEED_PID_KI, SHOOT_L_SPEED_PID_KD};
	static const fp32 Shoot_R_speed_pid[3] = {SHOOT_R_SPEED_PID_KP, SHOOT_R_SPEED_PID_KI, SHOOT_R_SPEED_PID_KD};
    //初始化不开启射击
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    //获取电机测量结构体指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();		
	shoot_control.shoot_3508_measure[0] = get_shoot_data_point1();
	shoot_control.shoot_3508_measure[1] = get_shoot_data_point2();		
    //初始化拨弹电机PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_BULLET_PID_MAX_OUT, TRIGGER_BULLET_PID_MAX_IOUT);
    //初始化摩擦轮PID
	PID_init(&shoot_control.shoot_pid[0], PID_POSITION, Shoot_L_speed_pid, SHOOT_L_SPEED_PID_MAX_OUT, SHOOT_L_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_control.shoot_pid[1], PID_POSITION, Shoot_R_speed_pid, SHOOT_R_SPEED_PID_MAX_OUT, SHOOT_R_SPEED_PID_MAX_IOUT);   
	//更新数据
    shoot_feedback_update();
	//初始化目标值和发送电流
    shoot_control.given_current = 0;
    shoot_control.trigger_speed = 0.0f;
    shoot_control.trigger_speed_set = 0.0f;
	shoot_control.shoot_speed_l = 0.0f;
	shoot_control.shoot_speed_r = 0.0f;
	shoot_control.shoot_speed_set_l = 0;
	shoot_control.shoot_speed_set_r = 0;
	shoot_control.shoot_current_l = 0.0f;
	shoot_control.shoot_current_r = 0.0f;
	//初始化发射相关标志位
	shoot_control.fric_flag = 0.0f;
	shoot_control.last_press_l = shoot_control.press_l = 0;      //鼠标点击状态
	shoot_control.press_l_time = shoot_control.press_r_time = 0; //鼠标按下时间
	shoot_control.mouse_state = MOUSE_NO_PRESS;          //鼠标长按状态          
	shoot_control.trigger_down_state = TRIGGER_DOWN_NO;  //拨弹遥感打下状态
	shoot_control.trigger_down_time = 0;                 //拨弹遥感打下计时
	//初始化射速限制和热量上限
	get_shoot_speed_limit (&shoot_control.shoot_speed_limit);	
	get_shoot_heat1_limit_and_heat1(&shoot_control.heat_limit, &shoot_control.heat);
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */

int16_t shoot_control_loop(void)
{
    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据 里面包括摩擦轮速度设定
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
		shoot_control.trigger_speed_set = 0.0f;
		shoot_control.shoot_speed_set = 0.0f;
    }
    else if(shoot_control.shoot_mode == OPEN_FRIC)
    {				
		shoot_control.trigger_speed_set = 0.0f;
    }
	else if (shoot_control.shoot_mode == OPEN_TRIGGER)   //设置射频
    {
		if(gimbal_behaviour == GIMBAL_AUTO)
		{
			if(shoot_control.heat_limit < 90.0f)  //总热量小于90时射频给低
				shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
			else
			  shoot_control.trigger_speed_set = AUTO_TRIGGER_SPEED;    //20自瞄射频高一点
		}
		else
		{ 
			if(shoot_control.mouse_state == MOUSE_SHORT_PRESS || shoot_control.trigger_down_state ==TRIGGER_DOWN_SHORT)
			{
				shoot_control.trigger_speed_set = TRIGGER_SPEED;   //5 鼠标短按或者遥杆短时间打下 
			}	
			else if(shoot_control.mouse_state == MOUSE_LONG_PRESS || shoot_control.trigger_down_state ==TRIGGER_DOWN_LONG)
			{
				shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED; //10 鼠标长按或者遥杆长时间打下
			}
		}
		trigger_motor_turn_back();
    }
	//由旋转切换至停止状态
    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.given_current = 0;	
		if(fabs(shoot_control.shoot_speed_l) < 1.0f && fabs(shoot_control.shoot_speed_r) < 1.0f)
		{
			shoot_control.shoot_current_l = 0;
			shoot_control.shoot_current_r = 0;
		}
		else
		{
			shoot_control.shoot_current_l = PID_calc(&shoot_control.shoot_pid[0], shoot_control.shoot_speed_l, shoot_control.shoot_speed_set);
			shoot_control.shoot_current_r = PID_calc(&shoot_control.shoot_pid[1], shoot_control.shoot_speed_r, shoot_control.shoot_speed_set);
		}
    }
    else
    {
		//拨弹PID计算
		PID_calc(&shoot_control.trigger_motor_pid, shoot_control.trigger_speed, shoot_control.trigger_speed_set);
        //保存拨弹电机电流
        shoot_control.given_current = shoot_control.trigger_motor_pid.out;			
	//旋转方向由电机屁股看向输出轴 电流为正对应逆时针旋转
#if FRIC_L_TURN
		shoot_control.shoot_speed_set_l =  -shoot_control.shoot_speed_set;
		shoot_control.shoot_speed_set_r =  shoot_control.shoot_speed_set;
#else
		shoot_control.shoot_speed_set_l =  shoot_control.shoot_speed_set;
		shoot_control.shoot_speed_set_r =  -shoot_control.shoot_speed_set;
#endif
        //保存摩擦轮电流
		shoot_control.shoot_current_l = PID_calc(&shoot_control.shoot_pid[0], shoot_control.shoot_speed_l, shoot_control.shoot_speed_set_l);
		shoot_control.shoot_current_r = PID_calc(&shoot_control.shoot_pid[1], shoot_control.shoot_speed_r, shoot_control.shoot_speed_set_r);
    }		
    return shoot_control.given_current;   
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
uint8_t shoot_flag;//未超热量射击标志位，如果为1，则可以射击，为0不可以射击
uint8_t auto_state = 0;
uint16_t shoot_time_1000ms_cnt = 0;
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;
	static uint16_t moust_press_cnt;	
    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = OPEN_FRIC;
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
    //处于中档， 可以使用键盘开启摩擦轮
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = OPEN_FRIC;			
    }
    //处于中档， 可以使用键盘关闭摩擦轮
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
	//获取热量限制和当前热量
	get_shoot_heat1_limit_and_heat1(&shoot_control.heat_limit, &shoot_control.heat);	
    //判断当前摩擦轮速度否达到所需速度
    if(shoot_control.shoot_mode >= OPEN_FRIC && fabs(shoot_control.shoot_speed_l - shoot_control.shoot_speed_set_l) < 0.4f 
	   && fabs(shoot_control.shoot_speed_r - shoot_control.shoot_speed_set_r) < 0.4f)
	{
#if REFEREE
//	//如果当前裁判系统没问题，且热量余量高于限制值      
//	if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE_AUTO_MAX <= shoot_control.heat_limit) && gimbal_behaviour == GIMBAL_AUTO)
//		shoot_flag = 1;
//	else if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE_MAX <= shoot_control.heat_limit))
//		shoot_flag = 1;
//	else00
//		shoot_flag = 0;
	//如果当前裁判系统没问题，且热量余量高于限制值
	if(shoot_control.heat_limit < 90.0f)  //针对1、2级冷却优先模式热量上限 <= 85时，预留49热量防超热量  此时自瞄弹频为10
	{
		if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE_MAX <= shoot_control.heat_limit))
			shoot_flag = 1;	
		else
			shoot_flag = 0;
	}
	else																	//热量 >= 85 时，热量优先或等级足够的冷却优先，预留90热量（主要给自瞄时的高射频留足余量）
	{
		if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE_AUTO_MAX <= shoot_control.heat_limit))
			shoot_flag = 1;	
		else
			shoot_flag = 0;
	}

#else
	shoot_flag = 1;
#endif
      //如果左拨杆在下面或鼠标左键点击
		if((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])||shoot_control.press_l) && shoot_flag == 1)
		{
			if((gimbal_behaviour == GIMBAL_AUTO) && get_auto_fire_state() == 2)   //火控
			{	
				shoot_control.shoot_mode = OPEN_TRIGGER;
			}
			else if(gimbal_behaviour!=GIMBAL_AUTO)
			{
				shoot_control.shoot_mode = OPEN_TRIGGER;
			}else //自瞄模式点击右键但未锁定目标
			{
				shoot_control.shoot_mode = OPEN_FRIC;
			}	
		}				
		else
			shoot_control.shoot_mode = OPEN_FRIC;
    }
	else
	{
		if(shoot_control.shoot_mode >= OPEN_FRIC)
		shoot_control.shoot_mode = OPEN_FRIC;
		else if(shoot_control.shoot_mode == SHOOT_STOP)
		shoot_control.shoot_mode = SHOOT_STOP;
	}
    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
	//只要摩擦轮处于开启状态，flag就置为一
	if(shoot_control.shoot_mode >= OPEN_FRIC)
	{
		shoot_control.fric_flag = 1;
	}
	else
	{
		shoot_control.fric_flag = 0;
	}
    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{
	static int8_t trigger_last_s = RC_SW_MID;    //记录遥控器拨弹遥感状态
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
	//定义长度为20的弹速历史数组及管理变量
	static fp32 bullet_speed_history[20] = {24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 
                                           24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f};
    static uint8_t history_index = 0;    //弹速计算历史索引
    static fp32 last_bullet_speed = 0.0f; //上次弹速
    fp32 current_bullet_speed = 0.0f;     //当前弹速
    fp32 average_speed = 0.0f;            //平均弹速
    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.trigger_speed = speed_fliter_3;   //
    //更新摩擦轮转速
    shoot_control.shoot_speed_l = shoot_control.shoot_3508_measure[0]->speed_rpm * FRICTION_ALL_COEFFICIENT;
    shoot_control.shoot_speed_r = shoot_control.shoot_3508_measure[1]->speed_rpm * FRICTION_ALL_COEFFICIENT;        
    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    //鼠标状态检测
    if(!shoot_control.last_press_l && shoot_control.press_l)  //上次未按下，这次按下--短按
    {
        shoot_control.mouse_state = MOUSE_SHORT_PRESS;
    }
    else if(shoot_control.last_press_l && shoot_control.press_l) //上次按下，这次按下，时间自加
    {
        shoot_control.press_l_time ++;
        if(shoot_control.press_l_time > PRESS_LONG_TIME)    //时间大于阈值，标记为长按状态
        {
            shoot_control.mouse_state = MOUSE_LONG_PRESS;   
        }
    }
    else if(shoot_control.last_press_l && !shoot_control.press_l)  //上次按下，这次未按下
    {
        shoot_control.press_l_time = 0;                   //清空计时，标记为没有按下
        shoot_control.mouse_state = MOUSE_NO_PRESS;
    }
	//遥控器拨弹状态检测
	if(switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(trigger_last_s) &&\
	   shoot_control.shoot_mode >=OPEN_FRIC)
	{
		shoot_control.trigger_down_state = TRIGGER_DOWN_SHORT;    //要控制短暂打下，用于单发
	}
	else if(switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_down(trigger_last_s) &&\
	        shoot_control.shoot_mode >=OPEN_FRIC)
	{
		shoot_control.trigger_down_time ++;
		if(shoot_control.trigger_down_time >= RC_S_LONG_TIME)
		{
			shoot_control.trigger_down_state = TRIGGER_DOWN_LONG;  //遥控器在开摩擦轮时长打下，持续退弹
		}
	}
	else if(!switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_down(trigger_last_s) &&\
	       shoot_control.shoot_mode >=OPEN_FRIC)
	{
		shoot_control.trigger_down_time =0;
		shoot_control.trigger_down_state = TRIGGER_DOWN_NO;   //遥控器在开摩擦轮时没有打下，不开拨弹
	}
    // 获取当前弹速
    get_current_bullet_speed(&current_bullet_speed);    
    // 当裁判系统读取到的速度与上一次不一致时，更新数组
    if(current_bullet_speed != last_bullet_speed && current_bullet_speed > 0.1f)
    {
        // 更新历史数组
        bullet_speed_history[history_index] = current_bullet_speed;       
        // 更新索引，循环使用数组
        history_index = (history_index + 1) % 20;        
        // 更新上一次速度记录
        last_bullet_speed = current_bullet_speed;
    }   
    // 计算数组平均值
    for(uint8_t i = 0; i < 20; i++)
    {
        average_speed += bullet_speed_history[i];
    }
    average_speed /= 20.0f;   
    // 设置弹速：2*22-数组的均值
    shoot_control.shoot_speed_set = 2 * 21.5f - average_speed;    
    // 设置弹速上下限，防止极端情况
    if(shoot_control.shoot_speed_set > 24.0f)
    {
        shoot_control.shoot_speed_set = 24.0f;
    }
    else if(shoot_control.shoot_speed_set < 20.0f)
    {
        shoot_control.shoot_speed_set = 20.0f;
    }
	trigger_last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}

/**
  * @brief   拨弹电机卡弹后反转函数，通过判断卡弹时间来判断是否够卡弹，进行反转处理                
  * @param   无
  * @param   无
  * @retval  无       
  */
static void trigger_motor_turn_back(void)
{
	if(shoot_control.shoot_mode == OPEN_TRIGGER && (!(switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
	&& (shoot_control.press_l ==0)))  
	{
		shoot_control.shoot_mode = OPEN_FRIC;
		return;
	}
	if(shoot_control.block_time < BLOCK_TIME)
	{
		shoot_control.trigger_speed_set = shoot_control.trigger_speed_set;
	}
	else
	{
		shoot_control.trigger_speed_set = -shoot_control.trigger_speed_set;
	}
	if(fabs(shoot_control.trigger_speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)   
	{
		shoot_control.block_time++;
		shoot_control.reverse_time = 0;
	}
	else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
	{
		shoot_control.reverse_time++;
	}
	else
	{
		shoot_control.block_time = 0;
	}
}

/**
  * @brief 获取摩擦轮标志位
  */
uint8_t * get_fric_flag()
{
	return &shoot_control.fric_flag;
}
/**
  * @brief		返回射击控制结构体指针
  * @retval		&chassis_move
  */
const shoot_control_t *get_shoot_control_point(void) 
{
	return &shoot_control;
}
