/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
      

/**********************************  云台PID参数 *************************************/
//===================1、PITCH轴电机PID参数，分为速度，绝对角度，相对角度===============//
//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP        11000.0f        //7000
#define PITCH_SPEED_PID_KI        30.0f           //60
#define PITCH_SPEED_PID_KD        0.0f			  //0
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  20000.0f

//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 	16.0f  		  //15
#define PITCH_GYRO_ABSOLUTE_PID_KI 	0.0f	      //0
#define PITCH_GYRO_ABSOLUTE_PID_KD 	0.3f		  //0
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 12.0f	  //10
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 4.0f	  //3

//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 20.0f         //  15
#define PITCH_ENCODE_RELATIVE_PID_KI 0.0f          //0
#define PITCH_ENCODE_RELATIVE_PID_KD 0.60f         //0
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 12.0f    //10
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f    //0

//===================2、YAW轴电机PID参数，分为速度，绝对角度，相对角度===============//
//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP        25500.0f		   //20000
#define YAW_SPEED_PID_KI        25.0f			   //20
#define YAW_SPEED_PID_KD        0.0f			   //0
#define YAW_SPEED_PID_MAX_OUT   30000.0f		   //30000
#define YAW_SPEED_PID_MAX_IOUT  20000.0f		   //20000

//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP        18.0f      //10
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f       //0
#define YAW_GYRO_ABSOLUTE_PID_KD        0.375      //0.3
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   15.0f      //15
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f       //0

//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP        15.0f     //10
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f	    //0
#define YAW_ENCODE_RELATIVE_PID_KD        0.2f      //0.1
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f     //10
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f      //0
/**********************************  云台PID参数 *************************************/

/**********************************  其他宏定义常量 *************************************/
//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0
//小陀螺和自瞄切换通道号
#define GIMBAL_AUTO_CHANGE_CHANNEL 4
//遥杆切换自瞄和小陀螺范围值
#define GIMBAL_CHANNEL_CHANGE_MAX    600
#define GIMBAL_CHANNEL_CHANGE_MIN    0
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   10
//遥控器转化为云台转动角度的灵敏度
#define YAW_RC_SEN    -0.000006f
#define PITCH_RC_SEN  -0.000004f //
//鼠标转化为云台转动角度的灵敏度
#define YAW_MOUSE_SEN   0.000025f 
#define PITCH_MOUSE_SEN 0.000025f // 0.00015
//云台阻塞时间，控制周期1毫秒
#define GIMBAL_CONTROL_TIME 1
//云台电机是否装反条件编译
#define PITCH_TURN  0
#define YAW_TURN    0
//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.005f
//初始化时云台电机的目标值
#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f
//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET   12000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f
//校准步骤宏定义
#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4
#define GIMBAL_CALI_START_STEP      GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP        5
//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif
//自瞄重置追踪器按键
#define AUTO_SHOOT_RESET KEY_PRESSED_OFFSET_R
/**********************************  其他宏定义常量 *************************************/


/**********************************  枚举和结构体定义 *************************************/
typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef enum
{
	GIMBAL_UP_AUTO = 0,  //自瞄
	GIMBAL_UP_GYRO = 1,  //小陀螺
}auto_gyro_mode_e;

typedef struct
{
	fp32 kp;
	fp32 ki;
	fp32 kd;

	fp32 set;
	fp32 get;
	fp32 err;

	fp32 max_out;
	fp32 max_iout;

	fp32 Pout;
	fp32 Iout;
	fp32 Dout;

	fp32 out;

	fp32 Dbuf[3];
	fp32 error[3];
} gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;   //电机测量值指针
    gimbal_PID_t gimbal_motor_absolute_angle_pid;  //绝对角度PID计算结构体指针
    gimbal_PID_t gimbal_motor_relative_angle_pid;  //相对角度PID计算结构体指针
    pid_type_def gimbal_motor_gyro_pid;            //角速度PID结构体指针
    gimbal_motor_mode_e gimbal_motor_mode;         //云台电机模式
    gimbal_motor_mode_e last_gimbal_motor_mode;    //云台电机模式-上一次
    uint16_t offset_ecd;     //电机编码值中值
    fp32 max_relative_angle; //rad 最大相对角度
    fp32 min_relative_angle; //rad 最小相对角度

	fp32 relative_angle;     //rad 相对角度
	fp32 relative_angle_set; //rad 相对角度设置值
	fp32 absolute_angle;     //rad 绝对角度
	fp32 absolute_angle_set; //rad 绝对角度设置值
	fp32 motor_gyro;         //rad/s 电机角速度
	fp32 motor_gyro_set;     //rad/s 电机角速度设置值
	fp32 raw_cmd_current;    //计算增量
	fp32 current_set;        //原始模式下电流设置值
	int16_t given_current;   //电流最终发送值
} gimbal_motor_t;

typedef struct
{
    fp32 max_yaw;      //最大YAW角度    
    fp32 min_yaw;      //最小YAW角度
    fp32 max_pitch;    //最大PITCH角度
    fp32 min_pitch;	   //最小PITCH角度
    uint16_t max_yaw_ecd;   //最大YAW编码值
    uint16_t min_yaw_ecd;   //最小YAW编码值
    uint16_t max_pitch_ecd; //最大PITCH编码值
    uint16_t min_pitch_ecd; //最小PITCH编码值
    uint8_t step;           //校准步骤
} gimbal_step_cali_t;

typedef struct
{
	const RC_ctrl_t *gimbal_rc_ctrl;      //遥控器指针
	const fp32 *gimbal_INT_angle_point;   //姿态角度指针
	const fp32 *gimbal_INT_gyro_point;    //陀螺仪角速度指针
	gimbal_motor_t gimbal_yaw_motor;      //YAW轴电机数据结构体
	gimbal_motor_t gimbal_pitch_motor;    //PITCH轴电机数据结构体
	gimbal_step_cali_t gimbal_cali;       //校准管理结构体
	auto_gyro_mode_e auto_gyro_mode;      //自瞄和小陀螺切换枚举
	int16_t gyro_change_channel;          //遥控器滚轮状态
	int16_t last_gyro_change_channel;	  //上一次遥控器滚轮状态
} gimbal_control_t;
/**********************************  枚举和结构体定义 *************************************/


/**********************************  函数申明 *************************************/
/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */

extern void gimbal_task(void const *pvParameters);

/**
  * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     yaw 中值 指针
  * @param[out]     pitch 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     pitch 最大相对角度 指针
  * @param[out]     pitch 最小相对角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

/**
  * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
  * @param[in]      yaw_offse:yaw 中值
  * @param[in]      pitch_offset:pitch 中值
  * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
  * @param[in]      min_yaw:yaw 最小相对角度
  * @param[in]      max_yaw:pitch 最大相对角度
  * @param[in]      min_yaw:pitch 最小相对角度
  * @retval         返回空
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);

extern const gimbal_control_t *get_gimbal_control_point(void);
/**********************************  函数申明 *************************************/

#endif
