/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"


#define ONE_PI (3.14159265f)
#define angle_to_radian = 0.01745f    // PI/180
/**/
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
	
/**********************************  底盘相关PID参数 *************************************/
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 20000.0f   //20000
#define M3505_MOTOR_SPEED_PID_KI 15.0f      //20.0f       30
#define M3505_MOTOR_SPEED_PID_KD 0.0f	
#define M3505_MOTOR_SPEED_PID_MAX_OUT    MAX_MOTOR_CAN_CURRENT   //16000
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 	2000.0f                  //2000.0f

//底盘电机速度环PID
//第四个电机响应慢故单给一个PID进行控制
#define M3505_MOTOR3_SPEED_PID_KP 25000.0f   //20000
#define M3505_MOTOR3_SPEED_PID_KI 15.0f      //20.0f       30
#define M3505_MOTOR3_SPEED_PID_KD 0.0f	
#define M3505_MOTOR3_SPEED_PID_MAX_OUT    MAX_MOTOR_CAN_CURRENT   //16000
#define M3505_MOTOR3_SPEED_PID_MAX_IOUT 	2000.0f                  //2000.0f
	
//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 16.0f      //7.5     15
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.01f      //0      0.01
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.10f  	//0.0
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT    12.0f   //10
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT   0.5f    //0.5
#define CHASSIS_FOLLOW_deadline      0.02f  			 //底盘跟随死区//0.1
//回中时缓慢回中，防止过冲
#define CHASSIS_FOLLOW_GIMBAL_SLOW_PID_KP 4.0f      //7.5 
#define CHASSIS_FOLLOW_GIMBAL_SLOW_PID_KI 0.01f     //0
#define CHASSIS_FOLLOW_GIMBAL_SLOW_PID_KD 0.10f  	//0.0

//小陀螺旋转速度PID
#define CHASSIS_WZ_PID_KP 			80.0f  //80
#define CHASSIS_WZ_PID_KI			0.0f   //0
#define CHASSIS_WZ_PID_KD			0.0f   //10
#define CHASSIS_WZ_PID_MAX_OUT		10.0f  //10
#define CHASSIS_WZ_PID_MAX_IOUT   	3.0f   //0
/**********************************  底盘相关PID参数 *************************************/


/**********************************  其他宏定义常量 *************************************/
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
//调试时可选择失能底盘，直接发送0电流，1：失能底盘  0：使能底盘 
#define CHASSIC_DISABLE   0
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0
//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 0
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.008f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.004f     //0.004
//加速度滤波系数
#define CHASSIS_ACCEL_X_NUM (0.1666666667f * 1.0f)  //0.1666666667f 
#define CHASSIS_ACCEL_Y_NUM (0.1666666667f * 1.0f)	//0.3333333333f
//摇杆死区
#define CHASSIS_RC_DEADLINE 10
//系数用于底盘解算
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.5f   
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.5f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.1f
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.004f
//底盘任务控制频率
#define CHASSIS_CONTROL_FREQUENCE 500.0f   //用于计算加速度，dx/dt->dx*(f频率)
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY 	KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY 	KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY 	KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY   KEY_PRESSED_OFFSET_D
//底盘电机中心到YAW轴中心的距离
#define MOTOR_DISTANCE_TO_CENTER 0.252f 
#define CHASSIS_WZ_SET_SCALE 0.1f    
//m3508转化成底盘速度(m/s)的比例
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN  0.000299f  
//单个底盘电机最大速度
#define MAX_WHEEL_SPEED         8.0f   
//底盘运动过程最大前进速度   			
#define CHASSIS_MAX_SPEED_X 	2.8f  //2.5   
//底盘运动过程最大平移速度
#define CHASSIS_MAX_SPEED_Y     2.5f  //2.0  
//小陀螺转速
#define CHASSIS_MAX_SPEED_WZ	4.3f  //1.8  2.5   3.0       4.3
/**********************************  其他宏定义常量 *************************************/

/**********************************  枚举和结构体定义 *************************************/
typedef enum
{
	CHASSIC_MACANUM_WHEEL,       //麦克纳姆轮
	CHASSIC_OMNI_DIRECTION_WHEEL,//全向轮
}chassic_configure_e;        //底盘构型选择枚举，add by qiyin 2024.12.l6

typedef enum
{
	/*底盘移动速度由遥控器和键盘决定，旋转速度由云台角度差
	计算出，是CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW 选择的
	控制模式*/
	CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,
	/*底盘移动速度和旋转速度由遥控器决定，无角度环控制，是
	CHASSIS_NO_FOLLOW_YAW 和 CHASSIS_NO_MOVE
	选择的控制模式*/
	CHASSIS_VECTOR_NO_FOLLOW_YAW,
	/*底盘电机电流控制值是直接由遥控器通道值计算出来的，将
	直接发送到 CAN 总线上，是 CHASSIS_OPEN 和 
	CHASSIS_ZERO_FORCE 选择的控制模式。*/
	CHASSIS_VECTOR_RAW,               
} chassis_mode_e;

typedef struct
{
	const motor_measure_t *chassis_motor_measure; //底盘电机信息指针
	fp32 accel;		//电机加速度
	fp32 speed;		//电机速度
	fp32 speed_set; //电机速度设置值
	int16_t give_current; //电机电流发送值
} chassis_motor_t;

typedef struct
{
	const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
	const gimbal_motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角.
	const gimbal_motor_t *chassis_pitch_motor; //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
	const fp32 *chassis_INS_angle;             //获取陀螺仪解算出的欧拉角指针
	const gimbal_control_t *gimbal_control_point;  //云台控制指针
	chassis_mode_e chassis_mode;               //底盘控制状态机
	chassis_mode_e last_chassis_mode;          //底盘上次控制状态机
	chassis_motor_t motor_chassis[4];          //底盘电机数据
	pid_type_def motor_speed_pid[4];           //底盘电机速度pid
	pid_type_def chassis_angle_pid;            //底盘跟随角度pid
	pid_type_def chassis_angle_slow_pid;       //底盘跟随角度pid
	pid_type_def chassis_wz_pid;			   //底盘旋转角速度pid

	first_order_filter_type_t chassis_cmd_slow_set_vx;  //使用一阶低通滤波减缓设定值
	first_order_filter_type_t chassis_cmd_slow_set_vy;  //使用一阶低通滤波减缓设定值

	fp32 vx;                          //底盘速度 前进方向 前为正，单位 m/s
	fp32 vy;                          //底盘速度 左右方向 左为正  单位 m/s
	fp32 wz;                          //底盘旋转角速度，逆时针为正 单位 rad/s
	fp32 vx_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
	fp32 vy_set;                      //底盘设定速度 左右方向 左为正，单位 m/s
	fp32 wz_set;                      //底盘设定旋转角速度，逆时针为正 单位 rad/s
	fp32 chassis_relative_angle;      //底盘与云台的相对角度，单位 rad
	fp32 chassis_relative_angle_set;  //设置相对云台控制角度
	fp32 chassis_yaw_set;             

	fp32 vx_max_speed; 		 //前进方向最大速度 单位m/s
	fp32 vx_min_speed; 		 //后退方向最大速度 单位m/s
	fp32 vy_max_speed;  	 //左方向最大速度 单位m/s
	fp32 vy_min_speed; 		 //右方向最大速度 单位m/s
	fp32 chassis_yaw;  		 //陀螺仪和云台电机叠加的yaw角度
	fp32 chassis_pitch; 	 //陀螺仪和云台电机叠加的pitch角度
	fp32 chassis_roll; 		 //陀螺仪和云台电机叠加的roll角度

	uint8_t gyroscope_flag;  //小陀螺旋转标志
	uint8_t auto_flag;		 //自瞄标志
	uint8_t cap_flag;        
} chassis_move_t;
/**********************************  枚举和结构体定义 *************************************/


/**********************************  变量申明 *************************************/
extern chassis_move_t chassis_move;
extern fp32 vx_set_channel, vy_set_channel;

/**********************************  变量申明 *************************************/

/**********************************  函数申明 *************************************/
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

extern uint8_t *get_gyro_flag(void);
extern uint8_t *get_cap_state(void);
extern const chassis_move_t *get_chassic_control_point(void);
/**********************************  函数申明 *************************************/

#endif
