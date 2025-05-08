/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
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

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"

/**********************************  发射机构PID参数 *************************************/
//1、拨弹轮电机PID
#define TRIGGER_SPEED_PID_KP        800.0f   //1000.0f
#define TRIGGER_SPEED_PID_KI        5.0f
#define TRIGGER_SPEED_PID_KD        0.0f
#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 4000.0f

//2、左摩擦轮射击电机速度环PID，单位：m/s
#define SHOOT_L_SPEED_PID_KP 8000.0f   //15000
#define SHOOT_L_SPEED_PID_KI 5.0f      //10
#define SHOOT_L_SPEED_PID_KD 0.0f
#define SHOOT_L_SPEED_PID_MAX_OUT  16000.0f
#define SHOOT_L_SPEED_PID_MAX_IOUT 1000.0f

//3、右摩擦轮射击电机速度环PID，单位：m/s
#define SHOOT_R_SPEED_PID_KP 8000.0f   //15000
#define SHOOT_R_SPEED_PID_KI 5.0f      //10
#define SHOOT_R_SPEED_PID_KD 0.0f
#define SHOOT_R_SPEED_PID_MAX_OUT  16000.0f
#define SHOOT_R_SPEED_PID_MAX_IOUT 1000.0f
/**********************************  发射机构PID参数 *************************************/

/**********************************  其他宏定义常量 *************************************/
//摩擦轮是对称安装，必然有一边是反的，更改它即可改变旋转方向
#define FRIC_L_TURN  1   
 //是否裁判系统使能，安装裁判系统请置1，否则置0
#define REFEREE      1    
//摩擦轮电机速度转化m/s
#define FRICTION_RADIUS    0.03f   //摩擦轮半径值，换摩擦轮记得更改该值   
#ifndef PI
#define PI   3.14159265358979323846f
#endif

//速度转化公式：v = PI*radius*n/30    线速度 = PI * 半径*n /30
#define FRICTION_COEFFICIENT   (PI*FRICTION_RADIUS/30.0f)                //速度转化为m/s总的系数，用于快速替换
#define FRICTION_ALL_COEFFICIENT   0.00314159265358979323846264338328f   //速度系数结果，减少计算          
//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
//云台模式使用的开关通道
#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME
#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f  //100
//射击摩擦轮打开 关闭
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E
//鼠标长按判断
#define PRESS_LONG_TIME             500
//遥控器射击开关打下档一段时间后加快拨弹----需要再写逻辑
#define RC_S_LONG_TIME              800    //2000
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f 
//拨弹速度
#define TRIGGER_SPEED               -5.0f     //-5
#define CONTINUE_TRIGGER_SPEED      -10.0f		//10.0
#define AUTO_TRIGGER_SPEED          -20.0f	  //20.0
//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         2.0f
#define BLOCK_TIME                  1000   //1000
#define REVERSE_TIME                800    //500
//定义PI/4等角度，如果有角度环可以使用
#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_FIVE 					0.628318530717958647692528676655f
#define PI_TEN                      0.314f
//热量限制保留，防止操作手超热量
#define SHOOT_HEAT_REMAIN_VALUE_MAX     49  //50
#define SHOOT_HEAT_REMAIN_VALUE_AUTO_MAX 80
//3508转换为速度
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f 
/**********************************  其他宏定义常量 *************************************/

/**********************************  枚举和结构体定义 *************************************/
typedef enum
{
  SHOOT_STOP  = 0,  //停止射击，待开
  OPEN_FRIC,        //打开摩擦轮
  OPEN_TRIGGER,     //打开拨弹电机
}shoot_mode_e;


typedef enum
{
	MOUSE_NO_PRESS = 0,    //未按下
	MOUSE_SHORT_PRESS = 1, //短按
	MOUSE_LONG_PRESS = 2,  //长按
}mouse_state_e;    //鼠标点击状态

typedef enum 
{
	TRIGGER_DOWN_NO = 0,     //没有打下拨弹
	TRIGGER_DOWN_SHORT = 1,  //短时间打下，单发
	TRIGGER_DOWN_LONG = 2,   //长时间打下，连续拨弹
}trigger_down_state_e;   //拨弹遥感状态

typedef struct
{
    shoot_mode_e shoot_mode;		//射击模式
    const RC_ctrl_t *shoot_rc;		//遥控器指针
    const motor_measure_t *shoot_motor_measure;		//拨弹电机测回数据
	const motor_measure_t *shoot_3508_measure[2];	//摩擦轮电机测量数据
	fp32 shoot_speed_l;   	//左摩擦轮速度
	fp32 shoot_speed_r;   	//右摩擦轮速度
	fp32 shoot_speed_set; 	//摩擦轮设定速度
	fp32 shoot_speed_set_l;	//左摩擦轮速度设置值
	fp32 shoot_speed_set_r;	//右摩擦轮速度设置值
	
    pid_type_def trigger_motor_pid;  //拨弹电机PID
	pid_type_def shoot_pid[2];       //摩擦轮闭环射击Pid

    fp32 trigger_speed_set; //拨弹电机速度设定 ，这个是用来更改的设定速度
    fp32 trigger_speed;     //拨弹电机速度

    int16_t given_current;  //记录拨弹电机电流
    bool_t press_l;         //鼠标左键点击状态
    bool_t press_r;         //鼠标右键点击状态
    bool_t last_press_l;    //上一次鼠标左键点击状态   
    bool_t last_press_r;    //上一次鼠标右键点击状态  
    uint16_t press_l_time;  //鼠标左键长按计时
    uint16_t press_r_time;  //鼠标右键长按计时
	mouse_state_e mouse_state;  //鼠标状态
	uint16_t trigger_down_time; //拨弹遥感打下时间
	trigger_down_state_e trigger_down_state;  //拨弹遥感打下状态
	
    uint16_t block_time;       //堵转时间
    uint16_t reverse_time;     //堵转保持时间
    bool_t key;                //设计键盘按下判断     
    uint16_t heat_limit;       //枪口热量上限
    uint16_t heat;	           //当前枪口热量
	uint16_t shoot_speed_limit;  //设计速度上限，没什么用
	uint8_t fric_flag;           //摩擦轮打开标志位
	int16_t shoot_current_l;     //左射击电流
	int16_t shoot_current_r;     //右射击电流
} shoot_control_t;
/**********************************  枚举和结构体定义 *************************************/


//===============================  函数申明 =========================================//
//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行，当然也可以再开一个任务
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);
extern uint8_t * get_fric_flag(void);
extern const shoot_control_t *get_shoot_control_point(void);

//===============================  函数申明 =========================================//
#endif
