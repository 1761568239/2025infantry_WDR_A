/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
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

//========================================== ��̬������������ ==============================================
/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);

/**
  * @brief          
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
//�˶�ѧ����⺯��
static void chassis_vector_to_omni_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4],chassic_configure_e chassic_configure);
static void omni_wheel_vector_to_chassis_speed(chassis_move_t *chassic_vector,chassic_configure_e chassic_configure);
//�������ֿ���ģʽ�Ŀ���Ŀ����㺯��
static void chassic_follow_gimbal_control(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_move_t* chassis_move_control);
static void chassic_no_follow_gimbal_control(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_move_t* chassis_move_control,chassic_configure_e CHASSIC_CONFIG);
static void chassic_raw_control(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_move_t* chassis_move_control);
//========================================== ��̬������������ ==============================================


//========================================== ȫ�ֱ����������� ==============================================
//ע�⣺�κ������ļ�ȫ�ֱ�����������ֻӦ����һ������������������ƽṹ�壡
//�����˶�����
chassis_move_t chassis_move;
//========================================== ȫ�ֱ����������� ==============================================


//========================================== ȫ�ֱ������� ==============================================
extern uint8_t cap_state;
//========================================== ȫ�ֱ������� ==============================================

/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //���̳�ʼ��
    chassis_init(&chassis_move);
    //�жϵ��̵���Ƿ�����
    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
    while (1)
    {
        //���õ��̿���ģʽ
        chassis_set_mode(&chassis_move);
        //ģʽ�л����ݱ���
        chassis_mode_change_control_transit(&chassis_move);
        //�������ݸ���
        chassis_feedback_update(&chassis_move);
        //���̿���������
        chassis_set_contorl(&chassis_move);
        //���̿���PID����
        chassis_control_loop(&chassis_move);
			
        //ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
        {
            //��ң�������ߵ�ʱ�򣬷��͸����̵�������.
            if (toe_is_error(DBUS_TOE))
            {
                CAN_cmd_chassis(0, 0, 0, 0);
            }
            else
            {
				#if (CHASSIC_DISABLE == 1)      //ѡ���Ƿ�ʧ�ܵ���
				CAN_cmd_chassis(0, 0, 0, 0);
				#else
				CAN_cmd_chassis((chassis_move.motor_chassis[0].give_current), (chassis_move.motor_chassis[1].give_current),
                                (chassis_move.motor_chassis[2].give_current), (chassis_move.motor_chassis[3].give_current));
				#endif 
            }
        }
        //ϵͳ��ʱ
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    //�����ٶȻ�pidֵ
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};    
    //���̽Ƕ�pidֵ
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
	//���̸������ٻ��нǶ�pidֵ
    const static fp32 chassis_yaw_slow_pid[3] = {CHASSIS_FOLLOW_GIMBAL_SLOW_PID_KP, CHASSIS_FOLLOW_GIMBAL_SLOW_PID_KI, CHASSIS_FOLLOW_GIMBAL_SLOW_PID_KD};
	//С������תPidֵ
	const static fp32 chassis_WZ_pid[3] = {CHASSIS_WZ_PID_KP,	CHASSIS_WZ_PID_KI,	CHASSIS_WZ_PID_KD};
	//**�˲��������飬��Ϊһ�׵�ͨ�˲�����**//
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    //���̿���״̬Ϊԭʼ
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();
    //��ȡ��������̬��ָ��
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //��ȡ��̨�������ָ��
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();  //����̨YAW����ָ�������YAW�����ڼ�����ԽǶ�
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();  //��̨PITCH���ָ�������
	//��ȡ��ָ̨��
	chassis_move_init->gimbal_control_point = get_gimbal_control_point();
    //��ȡ���̵������ָ�룬��ʼ��PID 
	uint8_t i;
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //��ʼ���Ƕ�PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	 //��ʼ���Ƕ�PID������ʱ�����ٶȣ�������С��PID����
    PID_init(&chassis_move_init->chassis_angle_slow_pid, PID_POSITION, chassis_yaw_slow_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);	
	//��ʼ����ת���ٶ�PID
	PID_init(&chassis_move_init->chassis_wz_pid, PID_POSITION, chassis_WZ_pid, CHASSIS_WZ_PID_MAX_OUT, CHASSIS_WZ_PID_MAX_IOUT);   
	//��һ���˲�����б����������
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    //��� ��С�ٶ�
		chassis_move_init->vx_max_speed =  CHASSIS_MAX_SPEED_X;
		chassis_move_init->vy_max_speed =  CHASSIS_MAX_SPEED_Y;	
		chassis_move_init->vx_min_speed =  -CHASSIS_MAX_SPEED_X;
		chassis_move_init->vy_min_speed =  -CHASSIS_MAX_SPEED_Y;				
	    chassis_move_init->gyroscope_flag = 0;	
    //����һ������
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
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
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }
	//ģʽδ�ı�
    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }
    //���������̨ģʽ
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;    //��ԽǶ�Ϊ0������ǰ������ص���̨��ֵ
    }
    //���벻������̨ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw; //������һ�εĵ�ǰ�Ƕ���ΪĿ��ֵ��ֹ����ת
    }
    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;  
}

/**
  * @brief		���̻����ٶȸ����Ǹ����ĸ���챵����ʵ���ٶȼ���ó���
				�ú�����ɵ������˶�ѧ���㣬��������ٶ�ת��Ϊ�����ٶ�--add by qiyin on 2024.12.16 
  * @param[in]  chassic_vector�����̿��ƽṹ��
  * @param[in]  chassic_configure��ѡ����̹��ͣ���ѡȫ���ֻ��������ķ��:
  * @retval     ��        
  */
static void omni_wheel_vector_to_chassis_speed(chassis_move_t *chassic_vector,chassic_configure_e chassic_configure)
{
	switch(chassic_configure)    //����ģʽѡ��
	{
		//���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ	
		case CHASSIC_MACANUM_WHEEL:    //����
		{
			chassic_vector->vx = (-chassic_vector->motor_chassis[0].speed + chassic_vector->motor_chassis[1].speed + chassic_vector->motor_chassis[2].speed - chassic_vector->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
			chassic_vector->vy = (-chassic_vector->motor_chassis[0].speed - chassic_vector->motor_chassis[1].speed + chassic_vector->motor_chassis[2].speed + chassic_vector->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
			chassic_vector->wz = (-chassic_vector->motor_chassis[0].speed - chassic_vector->motor_chassis[1].speed - chassic_vector->motor_chassis[2].speed - chassic_vector->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
			break;		
		}			
		case CHASSIC_OMNI_DIRECTION_WHEEL:   //ȫ����
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
 * @brief �����ٶȸ��¼���
 * @param chassis_move_t*�����̿�����Ϣ�ṹ��ָ�� 
 * @param ��
 * @retvl ��
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
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }   
	//�������˶�ѧ������³����ٶ�
	omni_wheel_vector_to_chassis_speed(chassis_move_update,CHASSIC_MACANUM_WHEEL);
    //���������̬�Ƕ�, �����������������������ⲿ�ִ���
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);  //��Ϊ���¼������
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);              //��֪�����̺������µ��ϵĽǶ�
}

/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
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
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	//��Ҫ���Ƴ����ٶ�ת�������õ�ʵ��Ҫ���õĳ����ٶ�
    vx_set_channel = vx_channel *  CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
    //���̲���
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
    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
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
	//����������ǰС����ѡ�ֶ˷����෴���д˴��룬��������ȥ��
	if(chassis_move_rc_to_vector->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
	{
		*vx_set = -(*vx_set);
		*vy_set = -(*vy_set);
	}
}

/**
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f; 	
    //��ȡ������������ֵ---vx,vy,wz
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
	//ͨ���жϿ���ģʽѡ����̿��ƺ���
	switch(chassis_move_control->chassis_mode)
	{
		case CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW:  //������̨ģʽ
			chassic_follow_gimbal_control(vx_set,vy_set,angle_set,chassis_move_control);
			break;
		case CHASSIS_VECTOR_NO_FOLLOW_YAW:      //С����ģʽ
			chassic_no_follow_gimbal_control(vx_set,vy_set,angle_set,chassis_move_control,CHASSIC_MACANUM_WHEEL);
			break;
		case CHASSIS_VECTOR_RAW:                //ԭʼģʽ
			chassic_raw_control(vx_set,vy_set,angle_set,chassis_move_control);
			break;
		default:
			break;
	}
}

/**
  * @brief		ͨ�����������˶���Ŀ��ֵ������̸�����̨ʱʵ����Ҫ�����Ŀ��ֵ
  * @param[in]	vx_set:ǰ�������Ŀ��ֵ
  * @param[in]	vy_set:ˮƽ�����Ŀ��ֵ
  * @param[in]	angle_set:��ת�����Ŀ��ֵ
  * @param[out]	chassis_move_control:���̿��ƽṹ��ָ��
  * @retval		none
  */
static void chassic_follow_gimbal_control(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_move_t* chassis_move_control)
{
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	//��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
	sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
	cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
	//�����ٶȷֽ���X��Y������ٶ�
	chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
	chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
	//���ÿ��������̨�Ƕ�
	chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);   
	//������תPID���ٶ�		
	chassis_move_control->wz_set = -improve_PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, 
													 chassis_move_control->chassis_relative_angle_set, NO_I_SCOPE, NO_D_SCOPE, CHASSIS_FOLLOW_deadline);
	//�ٶ��޷�
	chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
	chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
}

/**
  * @brief		ͨ�����������˶���Ŀ��ֵ������̲�������̨ʱʵ����Ҫ�����Ŀ��ֵ
  * @param[in]	vx_set:ǰ�������Ŀ��ֵ
  * @param[in]	vy_set:ˮƽ�����Ŀ��ֵ
  * @param[in]	angle_set:��ת�����Ŀ��ֵ
  * @param[out]	chassis_move_control:���̿��ƽṹ��ָ��
  * @retval		none
  */
static void chassic_no_follow_gimbal_control(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_move_t* chassis_move_control,chassic_configure_e CHASSIC_CONFIG)
{
	//�м����
	static float relative_angle = 0.0f;  
	static float absolute_value = 0.0f;
	static fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	//��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
	switch(CHASSIC_CONFIG)
	{
		case CHASSIC_OMNI_DIRECTION_WHEEL:       //ȫ����
		{
			static fp32 vx = 0.0f, vy= 0.0f;
			//1��ȫ����С���ݣ���������д��Ч��һ���������ϼ��ķ���
			sin_yaw = arm_sin_f32(ONE_PI+chassis_move_control->chassis_yaw_motor->relative_angle);
			cos_yaw = arm_cos_f32(ONE_PI+chassis_move_control->chassis_yaw_motor->relative_angle);
			vx = vx_set;
			vy = vy_set;
			vx_set = vx * cos_yaw - vy * sin_yaw;
			vy_set = vx * sin_yaw + vy * cos_yaw;
			break;
		}
		case CHASSIC_MACANUM_WHEEL:             //����
		{
			//2������С����
			vx_set = vx_set / CHASSIS_VX_RC_SEN;    //�ָ�ԭ������ֵ 
			vy_set = vy_set / (-CHASSIS_VY_RC_SEN);
			absolute_value = (float) sqrt((double)(vx_set * vx_set + vy_set * vy_set)); //ȡģ��
			relative_angle = atan2(vx_set,vy_set); 										//ȡҡ�����
			relative_angle += chassis_move_control->chassis_yaw_motor->relative_angle;	//���ϴ�ʱ��������̨����ԽǶ�
			relative_angle = loop_fp32_constrain(relative_angle, 0, 2*PI) ;				//�޶��Ƕ���0-2PI	
			if(absolute_value == 0)														//��ʼ״̬ʱΪ0
				relative_angle = 0;		
			vx_set = (float)(absolute_value * sin(relative_angle)) * CHASSIS_VX_RC_SEN;	//���Ǻ���������������VX VY
			vy_set = (float)(absolute_value * cos(relative_angle)) * (-CHASSIS_VY_RC_SEN);
			break;
		}
		default:
			break;
	}
	//��������Ŀ��ֵ���޷�
	chassis_move_control->vy_set = -fp32_constrain(vy_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
	chassis_move_control->vx_set = -fp32_constrain(vx_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);	
	chassis_move_control->wz_set = angle_set;
}

/**
  * @brief		ͨ�����������˶���Ŀ��ֵ�������ԭʼģʽ��ʵ����Ҫ�����Ŀ��ֵ
  * @param[in]	vx_set:ǰ�������Ŀ��ֵ
  * @param[in]	vy_set:ˮƽ�����Ŀ��ֵ
  * @param[in]	angle_set:��ת�����Ŀ��ֵ
  * @param[out]	chassis_move_control:���̿��ƽṹ��ָ��
  * @retval		none
  */
static void chassic_raw_control(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_move_t* chassis_move_control)
{
	//��ԭʼģʽ������ֵ�Ƿ��͵�CAN����
	chassis_move_control->vx_set = vx_set;
	chassis_move_control->vy_set = vy_set;
	chassis_move_control->wz_set = angle_set;
	chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
	chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
}

/**
  * @brief	�ĸ������ٶ���ͨ������������������ģ�
			�ú����ǵ������˶�ѧ���㣬�������ٶȽ���õ�ÿ�������ת�١�--update by qiyin on 2024.12.16 
  * @param[in]  vx_set: �����ٶ�
  * @param[in]  vy_set: �����ٶ�
  * @param[in]  wz_set: ��ת�ٶ�
  * @param[out] wheel_speed: �ĸ������ٶ�
  * @param[in]  chassic_configure��ѡ����̹��ͣ���ѡȫ���ֻ��������ķ��
  * @retval     ��        
  */
static void chassis_vector_to_omni_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4],chassic_configure_e chassic_configure)
{
	switch(chassic_configure)        //ѡ����̹���
	{
		case CHASSIC_MACANUM_WHEEL:   //����
		{
			wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			break;
		} 
		case CHASSIC_OMNI_DIRECTION_WHEEL:  //ȫ����
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
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //�������˶����㣬���ݳ����ٶȽ����ÿ����챵����ת��
    chassis_vector_to_omni_wheel_speed(chassis_move_control_loop->vx_set,chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed,CHASSIC_MACANUM_WHEEL);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW && !chassis_move_control_loop->auto_flag)
    {    
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        //raw����ֱ�ӷ���
        return;
    }
    //�������ӿ�������ٶȣ�������������ٶ�
	//1����ȡ�ٶ�
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i]; 
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }
	//�����ٶ�
    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;   //�õ�һ��С��1����
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;  //�����ٶȳ��ϸ�С��1������
        }
    }
    //����pid
    for (i = 0; i < 4; i++)
    {
				PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
		}
		
    //���ʿ���---ͨ�����ʿ������ƹ���
	chassis_power_control(chassis_move_control_loop);
    //��ֵ����ֵ
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
  * @brief    ��ȡС���ݱ�־          
  * @retval   gyroscope_flag      
  */
uint8_t *get_gyro_flag(void)
{
	return &chassis_move.gyroscope_flag;
}

/**
  * @brief          ���ص��̿��ƽṹ��ָ������
  * @retval         &chassis_move
  */
const chassis_move_t *get_chassic_control_point(void)
{
	return &chassis_move;
}
