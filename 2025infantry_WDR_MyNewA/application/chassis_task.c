/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "CAN_CAPower.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "math.h"
#include "referee.h"


#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
	
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

//========================================== 静态函数申明区域 ==============================================
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);

/**
  * @brief          
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);

/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
//运动学正逆解函数
static void chassis_vector_to_omni_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4],chassic_configure_e chassic_configure);
static void omni_wheel_vector_to_chassis_speed(chassis_move_t *chassic_vector,chassic_configure_e chassic_configure);
//底盘三种控制模式的控制目标计算函数
static void chassic_follow_gimbal_control(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_move_t* chassis_move_control);
static void chassic_no_follow_gimbal_control(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_move_t* chassis_move_control,chassic_configure_e CHASSIC_CONFIG);
static void chassic_raw_control(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_move_t* chassis_move_control);
//========================================== 静态函数申明区域 ==============================================


//========================================== 全局变量定义区域 ==============================================
//注意：任何任务文件全局变量定义区域只应存在一个变量，即该任务控制结构体！
//底盘运动数据
chassis_move_t chassis_move;
//========================================== 全局变量定义区域 ==============================================


//========================================== 全局变量区域 ==============================================
extern uint8_t cap_state;
//========================================== 全局变量区域 ==============================================

/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //底盘初始化
    chassis_init(&chassis_move);
    //判断底盘电机是否都在线
    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
    while (1)
    {
        //设置底盘控制模式
        chassis_set_mode(&chassis_move);
        //模式切换数据保存
        chassis_mode_change_control_transit(&chassis_move);
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);
			
        //确保至少一个电机在线， 这样CAN控制包可以被接收到
        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
        {
            //当遥控器掉线的时候，发送给底盘电机零电流.
            if (toe_is_error(DBUS_TOE))
            {
                CAN_cmd_chassis(0, 0, 0, 0);
            }
            else
            {
				#if (CHASSIC_DISABLE == 1)      //选择是否失能底盘
				CAN_cmd_chassis(0, 0, 0, 0);
				#else
				CAN_cmd_chassis((chassis_move.motor_chassis[0].give_current), (chassis_move.motor_chassis[1].give_current),
                                (chassis_move.motor_chassis[2].give_current), (chassis_move.motor_chassis[3].give_current));
				#endif 
            }
        }
        //系统延时
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};    
    //底盘角度pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
	//底盘跟随慢速回中角度pid值
    const static fp32 chassis_yaw_slow_pid[3] = {CHASSIS_FOLLOW_GIMBAL_SLOW_PID_KP, CHASSIS_FOLLOW_GIMBAL_SLOW_PID_KI, CHASSIS_FOLLOW_GIMBAL_SLOW_PID_KD};
	//小陀螺旋转Pid值
	const static fp32 chassis_WZ_pid[3] = {CHASSIS_WZ_PID_KP,	CHASSIS_WZ_PID_KI,	CHASSIS_WZ_PID_KD};
	//**滤波参数数组，作为一阶低通滤波参数**//
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    //底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();  //将云台YAW轴电机指针给底盘YAW，便于计算相对角度
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();  //云台PITCH电机指针给底盘
	//获取云台指针
	chassis_move_init->gimbal_control_point = get_gimbal_control_point();
    //获取底盘电机数据指针，初始化PID 
	uint8_t i;
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	 //初始化角度PID，回中时降低速度，给定较小的PID参数
    PID_init(&chassis_move_init->chassis_angle_slow_pid, PID_POSITION, chassis_yaw_slow_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);	
	//初始化旋转角速度PID
	PID_init(&chassis_move_init->chassis_wz_pid, PID_POSITION, chassis_WZ_pid, CHASSIS_WZ_PID_MAX_OUT, CHASSIS_WZ_PID_MAX_IOUT);   
	//用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    //最大 最小速度
		chassis_move_init->vx_max_speed =  CHASSIS_MAX_SPEED_X;
		chassis_move_init->vy_max_speed =  CHASSIS_MAX_SPEED_Y;	
		chassis_move_init->vx_min_speed =  -CHASSIS_MAX_SPEED_X;
		chassis_move_init->vy_min_speed =  -CHASSIS_MAX_SPEED_Y;				
	    chassis_move_init->gyroscope_flag = 0;	
    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }
	//模式未改变
    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }
    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;    //相对角度为0，底盘前进方向回到云台中值
    }
    //切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw; //保持上一次的当前角度作为目标值防止车疯转
    }
    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;  
}

/**
  * @brief		底盘机体速度更新是根据四个轮毂电机的实际速度计算得出，
				该函数完成底盘正运动学解算，将电机的速度转化为车体速度--add by qiyin on 2024.12.16 
  * @param[in]  chassic_vector：底盘控制结构体
  * @param[in]  chassic_configure：选择底盘构型，可选全向轮或者麦克纳姆轮:
  * @retval     无        
  */
static void omni_wheel_vector_to_chassis_speed(chassis_move_t *chassic_vector,chassic_configure_e chassic_configure)
{
	switch(chassic_configure)    //底盘模式选择
	{
		//更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系	
		case CHASSIC_MACANUM_WHEEL:    //麦轮
		{
			chassic_vector->vx = (-chassic_vector->motor_chassis[0].speed + chassic_vector->motor_chassis[1].speed + chassic_vector->motor_chassis[2].speed - chassic_vector->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
			chassic_vector->vy = (-chassic_vector->motor_chassis[0].speed - chassic_vector->motor_chassis[1].speed + chassic_vector->motor_chassis[2].speed + chassic_vector->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
			chassic_vector->wz = (-chassic_vector->motor_chassis[0].speed - chassic_vector->motor_chassis[1].speed - chassic_vector->motor_chassis[2].speed - chassic_vector->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
			break;		
		}			
		case CHASSIC_OMNI_DIRECTION_WHEEL:   //全向轮
		{
			chassic_vector->vx = (chassic_vector->motor_chassis[0].speed + chassic_vector->motor_chassis[2].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX; 
			chassic_vector->vy = (chassic_vector->motor_chassis[1].speed + chassic_vector->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
			chassic_vector->wz = (chassic_vector->motor_chassis[0].speed + chassic_vector->motor_chassis[1].speed + chassic_vector->motor_chassis[2].speed + chassic_vector->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ; 
			break;
		}
		default:
			break;
	}
}

/**
 * @brief 底盘速度更新计算
 * @param chassis_move_t*，底盘控制信息结构体指针 
 * @param 无
 * @retvl 无
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if(chassis_move_update == NULL)
    {
      return;
    }
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }   
	//底盘正运动学解算更新车体速度
	omni_wheel_vector_to_chassis_speed(chassis_move_update,CHASSIC_MACANUM_WHEEL);
    //计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);  //作为上坡检测有用
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);              //能知道底盘横着在坡道上的角度
}

/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
	static fp32 vx_set_channel, vy_set_channel;
    int16_t vx_channel, vy_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	//将要控制乘以速度转化比例得到实际要设置的车体速度
    vx_set_channel = vx_channel *  CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
    //键盘操作
	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
	{
		vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
	{
		vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
	}
	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
	{
		vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
	{
		vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
	}				
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //停止信号，不需要缓慢加速，直接减速到零
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
	}
	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
	}
	*vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	*vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
	//由于联盟赛前小陀螺选手端方向相反故有此代码，若正常可去除
	if(chassis_move_rc_to_vector->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
	{
		*vx_set = -(*vx_set);
		*vy_set = -(*vy_set);
	}
}

/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f; 	
    //获取三个控制设置值---vx,vy,wz
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
	//通过判断控制模式选择底盘控制函数
	switch(chassis_move_control->chassis_mode)
	{
		case CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW:  //跟随云台模式
			chassic_follow_gimbal_control(vx_set,vy_set,angle_set,chassis_move_control);
			break;
		case CHASSIS_VECTOR_NO_FOLLOW_YAW:      //小陀螺模式
			chassic_no_follow_gimbal_control(vx_set,vy_set,angle_set,chassis_move_control,CHASSIC_MACANUM_WHEEL);
			break;
		case CHASSIS_VECTOR_RAW:                //原始模式
			chassic_raw_control(vx_set,vy_set,angle_set,chassis_move_control);
			break;
		default:
			break;
	}
}

/**
  * @brief		通过三个底盘运动的目标值计算底盘跟随云台时实际需要输出的目标值
  * @param[in]	vx_set:前进方向的目标值
  * @param[in]	vy_set:水平方向的目标值
  * @param[in]	angle_set:旋转方向的目标值
  * @param[out]	chassis_move_control:底盘控制结构体指针
  * @retval		none
  */
static void chassic_follow_gimbal_control(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_move_t* chassis_move_control)
{
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
	sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
	cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
	//计算速度分解在X和Y方向的速度
	chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
	chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
	//设置控制相对云台角度
	chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);   
	//计算旋转PID角速度		
	chassis_move_control->wz_set = -improve_PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, 
													 chassis_move_control->chassis_relative_angle_set, NO_I_SCOPE, NO_D_SCOPE, CHASSIS_FOLLOW_deadline);
	//速度限幅
	chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
	chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
}

/**
  * @brief		通过三个底盘运动的目标值计算底盘不跟随云台时实际需要输出的目标值
  * @param[in]	vx_set:前进方向的目标值
  * @param[in]	vy_set:水平方向的目标值
  * @param[in]	angle_set:旋转方向的目标值
  * @param[out]	chassis_move_control:底盘控制结构体指针
  * @retval		none
  */
static void chassic_no_follow_gimbal_control(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_move_t* chassis_move_control,chassic_configure_e CHASSIC_CONFIG)
{
	//中间变量
	static float relative_angle = 0.0f;  
	static float absolute_value = 0.0f;
	static fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
	switch(CHASSIC_CONFIG)
	{
		case CHASSIC_OMNI_DIRECTION_WHEEL:       //全向轮
		{
			static fp32 vx = 0.0f, vy= 0.0f;
			//1、全向轮小陀螺，检验两种写法效果一样，保留较简洁的方案
			sin_yaw = arm_sin_f32(ONE_PI+chassis_move_control->chassis_yaw_motor->relative_angle);
			cos_yaw = arm_cos_f32(ONE_PI+chassis_move_control->chassis_yaw_motor->relative_angle);
			vx = vx_set;
			vy = vy_set;
			vx_set = vx * cos_yaw - vy * sin_yaw;
			vy_set = vx * sin_yaw + vy * cos_yaw;
			break;
		}
		case CHASSIC_MACANUM_WHEEL:             //麦轮
		{
			//2、麦轮小陀螺
			vx_set = vx_set / CHASSIS_VX_RC_SEN;    //恢复原来的数值 
			vy_set = vy_set / (-CHASSIS_VY_RC_SEN);
			absolute_value = (float) sqrt((double)(vx_set * vx_set + vy_set * vy_set)); //取模长
			relative_angle = atan2(vx_set,vy_set); 										//取摇杆相角
			relative_angle += chassis_move_control->chassis_yaw_motor->relative_angle;	//加上此时底盘与云台的相对角度
			relative_angle = loop_fp32_constrain(relative_angle, 0, 2*PI) ;				//限定角度在0-2PI	
			if(absolute_value == 0)														//初始状态时为0
				relative_angle = 0;		
			vx_set = (float)(absolute_value * sin(relative_angle)) * CHASSIS_VX_RC_SEN;	//三角函数运算求折算后的VX VY
			vy_set = (float)(absolute_value * cos(relative_angle)) * (-CHASSIS_VY_RC_SEN);
			break;
		}
		default:
			break;
	}
	//设置三个目标值，限幅
	chassis_move_control->vy_set = -fp32_constrain(vy_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
	chassis_move_control->vx_set = -fp32_constrain(vx_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);	
	chassis_move_control->wz_set = angle_set;
}

/**
  * @brief		通过三个底盘运动的目标值计算底盘原始模式下实际需要输出的目标值
  * @param[in]	vx_set:前进方向的目标值
  * @param[in]	vy_set:水平方向的目标值
  * @param[in]	angle_set:旋转方向的目标值
  * @param[out]	chassis_move_control:底盘控制结构体指针
  * @retval		none
  */
static void chassic_raw_control(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_move_t* chassis_move_control)
{
	//在原始模式，设置值是发送到CAN总线
	chassis_move_control->vx_set = vx_set;
	chassis_move_control->vy_set = vy_set;
	chassis_move_control->wz_set = angle_set;
	chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
	chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
}

/**
  * @brief	四个麦轮速度是通过三个参数计算出来的，
			该函数是底盘逆运动学解算，将车体速度解算得到每个电机的转速。--update by qiyin on 2024.12.16 
  * @param[in]  vx_set: 纵向速度
  * @param[in]  vy_set: 横向速度
  * @param[in]  wz_set: 旋转速度
  * @param[out] wheel_speed: 四个麦轮速度
  * @param[in]  chassic_configure：选择底盘构型，可选全向轮或者麦克纳姆轮
  * @retval     无        
  */
static void chassis_vector_to_omni_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4],chassic_configure_e chassic_configure)
{
	switch(chassic_configure)        //选择底盘构型
	{
		case CHASSIC_MACANUM_WHEEL:   //麦轮
		{
			wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			break;
		} 
		case CHASSIC_OMNI_DIRECTION_WHEEL:  //全向轮
		{
			wheel_speed[0] = -vx_set - vy_set + (-1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[1] =  vx_set - vy_set + (-1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[2] =  vx_set + vy_set + (-1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[3] = -vx_set + vy_set + (-1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			break;
		}
		default:
			break;
	}	
}

/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //底盘逆运动解算，根据车体速度解算出每个轮毂电机的转速
    chassis_vector_to_omni_wheel_speed(chassis_move_control_loop->vx_set,chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed,CHASSIC_MACANUM_WHEEL);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW && !chassis_move_control_loop->auto_flag)
    {    
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        //raw控制直接返回
        return;
    }
    //计算轮子控制最大速度，并限制其最大速度
	//1、获取速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i]; 
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }
	//限制速度
    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;   //得到一个小于1的数
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;  //轮子速度乘上个小于1的数字
        }
    }
    //计算pid
    for (i = 0; i < 4; i++)
    {
				PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
		}
		
    //功率控制---通过功率控制限制功率
	chassis_power_control(chassis_move_control_loop);
    //赋值电流值
    for (i = 0; i < 4; i++)
    {
			if(chassis_move.chassis_pitch > -0.1f)
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
			else
			{
				if(i < 2)
					chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
				else
					chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out * 1.7f);
			}
    }
}

/**
  * @brief    获取小陀螺标志          
  * @retval   gyroscope_flag      
  */
uint8_t *get_gyro_flag(void)
{
	return &chassis_move.gyroscope_flag;
}

/**
  * @brief          返回底盘控制结构体指针数据
  * @retval         &chassis_move
  */
const chassis_move_t *get_chassic_control_point(void)
{
	return &chassis_move;
}
