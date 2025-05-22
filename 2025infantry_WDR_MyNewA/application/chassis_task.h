/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
	
/**********************************  �������PID���� *************************************/
//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 20000.0f   //20000
#define M3505_MOTOR_SPEED_PID_KI 15.0f      //20.0f       30
#define M3505_MOTOR_SPEED_PID_KD 0.0f	
#define M3505_MOTOR_SPEED_PID_MAX_OUT    MAX_MOTOR_CAN_CURRENT   //16000
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 	2000.0f                  //2000.0f

//���̵���ٶȻ�PID
//���ĸ������Ӧ���ʵ���һ��PID���п���
#define M3505_MOTOR3_SPEED_PID_KP 25000.0f   //20000
#define M3505_MOTOR3_SPEED_PID_KI 15.0f      //20.0f       30
#define M3505_MOTOR3_SPEED_PID_KD 0.0f	
#define M3505_MOTOR3_SPEED_PID_MAX_OUT    MAX_MOTOR_CAN_CURRENT   //16000
#define M3505_MOTOR3_SPEED_PID_MAX_IOUT 	2000.0f                  //2000.0f
	
//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 16.0f      //7.5     15
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.01f      //0      0.01
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.10f  	//0.0
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT    12.0f   //10
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT   0.5f    //0.5
#define CHASSIS_FOLLOW_deadline      0.02f  			 //���̸�������//0.1
//����ʱ�������У���ֹ����
#define CHASSIS_FOLLOW_GIMBAL_SLOW_PID_KP 4.0f      //7.5 
#define CHASSIS_FOLLOW_GIMBAL_SLOW_PID_KI 0.01f     //0
#define CHASSIS_FOLLOW_GIMBAL_SLOW_PID_KD 0.10f  	//0.0

//С������ת�ٶ�PID
#define CHASSIS_WZ_PID_KP 			80.0f  //80
#define CHASSIS_WZ_PID_KI			0.0f   //0
#define CHASSIS_WZ_PID_KD			0.0f   //10
#define CHASSIS_WZ_PID_MAX_OUT		10.0f  //10
#define CHASSIS_WZ_PID_MAX_IOUT   	3.0f   //0
/**********************************  �������PID���� *************************************/


/**********************************  �����궨�峣�� *************************************/
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357
//����ʱ��ѡ��ʧ�ܵ��̣�ֱ�ӷ���0������1��ʧ�ܵ���  0��ʹ�ܵ��� 
#define CHASSIC_DISABLE   0
//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
//ѡ�����״̬ ����ͨ����
#define CHASSIS_MODE_CHANNEL 0
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.008f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.004f     //0.004
//���ٶ��˲�ϵ��
#define CHASSIS_ACCEL_X_NUM (0.1666666667f * 1.0f)  //0.1666666667f 
#define CHASSIS_ACCEL_Y_NUM (0.1666666667f * 1.0f)	//0.3333333333f
//ҡ������
#define CHASSIS_RC_DEADLINE 10
//ϵ�����ڵ��̽���
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.5f   
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.5f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.1f
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.004f
//�����������Ƶ��
#define CHASSIS_CONTROL_FREQUENCE 500.0f   //���ڼ�����ٶȣ�dx/dt->dx*(fƵ��)
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY 	KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY 	KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY 	KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY   KEY_PRESSED_OFFSET_D
//���̵�����ĵ�YAW�����ĵľ���
#define MOTOR_DISTANCE_TO_CENTER 0.252f 
#define CHASSIS_WZ_SET_SCALE 0.1f    
//m3508ת���ɵ����ٶ�(m/s)�ı���
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN  0.000299f  
//�������̵������ٶ�
#define MAX_WHEEL_SPEED         8.0f   
//�����˶��������ǰ���ٶ�   			
#define CHASSIS_MAX_SPEED_X 	2.8f  //2.5   
//�����˶��������ƽ���ٶ�
#define CHASSIS_MAX_SPEED_Y     2.5f  //2.0  
//С����ת��
#define CHASSIS_MAX_SPEED_WZ	4.3f  //1.8  2.5   3.0       4.3
/**********************************  �����궨�峣�� *************************************/

/**********************************  ö�ٺͽṹ�嶨�� *************************************/
typedef enum
{
	CHASSIC_MACANUM_WHEEL,       //�����ķ��
	CHASSIC_OMNI_DIRECTION_WHEEL,//ȫ����
}chassic_configure_e;        //���̹���ѡ��ö�٣�add by qiyin 2024.12.l6

typedef enum
{
	/*�����ƶ��ٶ���ң�����ͼ��̾�������ת�ٶ�����̨�ǶȲ�
	���������CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW ѡ���
	����ģʽ*/
	CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,
	/*�����ƶ��ٶȺ���ת�ٶ���ң�����������޽ǶȻ����ƣ���
	CHASSIS_NO_FOLLOW_YAW �� CHASSIS_NO_MOVE
	ѡ��Ŀ���ģʽ*/
	CHASSIS_VECTOR_NO_FOLLOW_YAW,
	/*���̵����������ֵ��ֱ����ң����ͨ��ֵ��������ģ���
	ֱ�ӷ��͵� CAN �����ϣ��� CHASSIS_OPEN �� 
	CHASSIS_ZERO_FORCE ѡ��Ŀ���ģʽ��*/
	CHASSIS_VECTOR_RAW,               
} chassis_mode_e;

typedef struct
{
	const motor_measure_t *chassis_motor_measure; //���̵����Ϣָ��
	fp32 accel;		//������ٶ�
	fp32 speed;		//����ٶ�
	fp32 speed_set; //����ٶ�����ֵ
	int16_t give_current; //�����������ֵ
} chassis_motor_t;

typedef struct
{
	const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��
	const gimbal_motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����.
	const gimbal_motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
	const fp32 *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
	const gimbal_control_t *gimbal_control_point;  //��̨����ָ��
	chassis_mode_e chassis_mode;               //���̿���״̬��
	chassis_mode_e last_chassis_mode;          //�����ϴο���״̬��
	chassis_motor_t motor_chassis[4];          //���̵������
	pid_type_def motor_speed_pid[4];           //���̵���ٶ�pid
	pid_type_def chassis_angle_pid;            //���̸���Ƕ�pid
	pid_type_def chassis_angle_slow_pid;       //���̸���Ƕ�pid
	pid_type_def chassis_wz_pid;			   //������ת���ٶ�pid

	first_order_filter_type_t chassis_cmd_slow_set_vx;  //ʹ��һ�׵�ͨ�˲������趨ֵ
	first_order_filter_type_t chassis_cmd_slow_set_vy;  //ʹ��һ�׵�ͨ�˲������趨ֵ

	fp32 vx;                          //�����ٶ� ǰ������ ǰΪ������λ m/s
	fp32 vy;                          //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
	fp32 wz;                          //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	fp32 vx_set;                      //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
	fp32 vy_set;                      //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
	fp32 wz_set;                      //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	fp32 chassis_relative_angle;      //��������̨����ԽǶȣ���λ rad
	fp32 chassis_relative_angle_set;  //���������̨���ƽǶ�
	fp32 chassis_yaw_set;             

	fp32 vx_max_speed; 		 //ǰ����������ٶ� ��λm/s
	fp32 vx_min_speed; 		 //���˷�������ٶ� ��λm/s
	fp32 vy_max_speed;  	 //��������ٶ� ��λm/s
	fp32 vy_min_speed; 		 //�ҷ�������ٶ� ��λm/s
	fp32 chassis_yaw;  		 //�����Ǻ���̨������ӵ�yaw�Ƕ�
	fp32 chassis_pitch; 	 //�����Ǻ���̨������ӵ�pitch�Ƕ�
	fp32 chassis_roll; 		 //�����Ǻ���̨������ӵ�roll�Ƕ�

	uint8_t gyroscope_flag;  //С������ת��־
	uint8_t auto_flag;		 //�����־
	uint8_t cap_flag;        
} chassis_move_t;
/**********************************  ö�ٺͽṹ�嶨�� *************************************/


/**********************************  �������� *************************************/
extern chassis_move_t chassis_move;
extern fp32 vx_set_channel, vy_set_channel;

/**********************************  �������� *************************************/

/**********************************  �������� *************************************/
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

extern uint8_t *get_gyro_flag(void);
extern uint8_t *get_cap_state(void);
extern const chassis_move_t *get_chassic_control_point(void);
/**********************************  �������� *************************************/

#endif
