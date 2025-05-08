/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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

/**********************************  �������PID���� *************************************/
//1�������ֵ��PID
#define TRIGGER_SPEED_PID_KP        800.0f   //1000.0f
#define TRIGGER_SPEED_PID_KI        5.0f
#define TRIGGER_SPEED_PID_KD        0.0f
#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 4000.0f

//2����Ħ�����������ٶȻ�PID����λ��m/s
#define SHOOT_L_SPEED_PID_KP 8000.0f   //15000
#define SHOOT_L_SPEED_PID_KI 5.0f      //10
#define SHOOT_L_SPEED_PID_KD 0.0f
#define SHOOT_L_SPEED_PID_MAX_OUT  16000.0f
#define SHOOT_L_SPEED_PID_MAX_IOUT 1000.0f

//3����Ħ�����������ٶȻ�PID����λ��m/s
#define SHOOT_R_SPEED_PID_KP 8000.0f   //15000
#define SHOOT_R_SPEED_PID_KI 5.0f      //10
#define SHOOT_R_SPEED_PID_KD 0.0f
#define SHOOT_R_SPEED_PID_MAX_OUT  16000.0f
#define SHOOT_R_SPEED_PID_MAX_IOUT 1000.0f
/**********************************  �������PID���� *************************************/

/**********************************  �����궨�峣�� *************************************/
//Ħ�����ǶԳư�װ����Ȼ��һ���Ƿ��ģ����������ɸı���ת����
#define FRIC_L_TURN  1   
 //�Ƿ����ϵͳʹ�ܣ���װ����ϵͳ����1��������0
#define REFEREE      1    
//Ħ���ֵ���ٶ�ת��m/s
#define FRICTION_RADIUS    0.03f   //Ħ���ְ뾶ֵ����Ħ���ּǵø��ĸ�ֵ   
#ifndef PI
#define PI   3.14159265358979323846f
#endif

//�ٶ�ת����ʽ��v = PI*radius*n/30    ���ٶ� = PI * �뾶*n /30
#define FRICTION_COEFFICIENT   (PI*FRICTION_RADIUS/30.0f)                //�ٶ�ת��Ϊm/s�ܵ�ϵ�������ڿ����滻
#define FRICTION_ALL_COEFFICIENT   0.00314159265358979323846264338328f   //�ٶ�ϵ����������ټ���          
//������俪��ͨ������
#define SHOOT_RC_MODE_CHANNEL       1
//��̨ģʽʹ�õĿ���ͨ��
#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME
#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f  //100
//���Ħ���ִ� �ر�
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E
//��곤���ж�
#define PRESS_LONG_TIME             500
//ң����������ش��µ�һ��ʱ���ӿ첦��----��Ҫ��д�߼�
#define RC_S_LONG_TIME              800    //2000
//�����������ֵ��Χ
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//���rmp �仯�� ��ת�ٶȵı���
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f 
//�����ٶ�
#define TRIGGER_SPEED               -5.0f     //-5
#define CONTINUE_TRIGGER_SPEED      -10.0f		//10.0
#define AUTO_TRIGGER_SPEED          -20.0f	  //20.0
//����ʱ�� �Լ���תʱ��
#define BLOCK_TRIGGER_SPEED         2.0f
#define BLOCK_TIME                  1000   //1000
#define REVERSE_TIME                800    //500
//����PI/4�ȽǶȣ�����нǶȻ�����ʹ��
#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_FIVE 					0.628318530717958647692528676655f
#define PI_TEN                      0.314f
//�������Ʊ�������ֹ�����ֳ�����
#define SHOOT_HEAT_REMAIN_VALUE_MAX     49  //50
#define SHOOT_HEAT_REMAIN_VALUE_AUTO_MAX 80
//3508ת��Ϊ�ٶ�
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f 
/**********************************  �����궨�峣�� *************************************/

/**********************************  ö�ٺͽṹ�嶨�� *************************************/
typedef enum
{
  SHOOT_STOP  = 0,  //ֹͣ���������
  OPEN_FRIC,        //��Ħ����
  OPEN_TRIGGER,     //�򿪲������
}shoot_mode_e;


typedef enum
{
	MOUSE_NO_PRESS = 0,    //δ����
	MOUSE_SHORT_PRESS = 1, //�̰�
	MOUSE_LONG_PRESS = 2,  //����
}mouse_state_e;    //�����״̬

typedef enum 
{
	TRIGGER_DOWN_NO = 0,     //û�д��²���
	TRIGGER_DOWN_SHORT = 1,  //��ʱ����£�����
	TRIGGER_DOWN_LONG = 2,   //��ʱ����£���������
}trigger_down_state_e;   //����ң��״̬

typedef struct
{
    shoot_mode_e shoot_mode;		//���ģʽ
    const RC_ctrl_t *shoot_rc;		//ң����ָ��
    const motor_measure_t *shoot_motor_measure;		//��������������
	const motor_measure_t *shoot_3508_measure[2];	//Ħ���ֵ����������
	fp32 shoot_speed_l;   	//��Ħ�����ٶ�
	fp32 shoot_speed_r;   	//��Ħ�����ٶ�
	fp32 shoot_speed_set; 	//Ħ�����趨�ٶ�
	fp32 shoot_speed_set_l;	//��Ħ�����ٶ�����ֵ
	fp32 shoot_speed_set_r;	//��Ħ�����ٶ�����ֵ
	
    pid_type_def trigger_motor_pid;  //�������PID
	pid_type_def shoot_pid[2];       //Ħ���ֱջ����Pid

    fp32 trigger_speed_set; //��������ٶ��趨 ��������������ĵ��趨�ٶ�
    fp32 trigger_speed;     //��������ٶ�

    int16_t given_current;  //��¼�����������
    bool_t press_l;         //���������״̬
    bool_t press_r;         //����Ҽ����״̬
    bool_t last_press_l;    //��һ�����������״̬   
    bool_t last_press_r;    //��һ������Ҽ����״̬  
    uint16_t press_l_time;  //������������ʱ
    uint16_t press_r_time;  //����Ҽ�������ʱ
	mouse_state_e mouse_state;  //���״̬
	uint16_t trigger_down_time; //����ң�д���ʱ��
	trigger_down_state_e trigger_down_state;  //����ң�д���״̬
	
    uint16_t block_time;       //��תʱ��
    uint16_t reverse_time;     //��ת����ʱ��
    bool_t key;                //��Ƽ��̰����ж�     
    uint16_t heat_limit;       //ǹ����������
    uint16_t heat;	           //��ǰǹ������
	uint16_t shoot_speed_limit;  //����ٶ����ޣ�ûʲô��
	uint8_t fric_flag;           //Ħ���ִ򿪱�־λ
	int16_t shoot_current_l;     //���������
	int16_t shoot_current_r;     //���������
} shoot_control_t;
/**********************************  ö�ٺͽṹ�嶨�� *************************************/


//===============================  �������� =========================================//
//�����������̨ʹ��ͬһ��can��id��Ҳ�����������̨������ִ�У���ȻҲ�����ٿ�һ������
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);
extern uint8_t * get_fric_flag(void);
extern const shoot_control_t *get_shoot_control_point(void);

//===============================  �������� =========================================//
#endif
