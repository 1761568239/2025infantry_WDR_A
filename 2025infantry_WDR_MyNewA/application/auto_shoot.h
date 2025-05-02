#ifndef AUTO_SHOOT_H
#define AUTO_SHOOT_H
#include "struct_typedef.h"

#include "string.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "GafProjectileSolver.h"
#include "usbd_cdc_if.h"


#ifndef PI
#define PI					3.14159265358979f
#endif

#define RX_BUF_NUM                  128u    //�����յ�����
#define YAW_AUTO_SEN                0.05
#define AUTO_SHOOT_FIFO_BUF_LENGTH  1024     //�����յ�����
#define DATE_LENGTH    sizeof(received_packed_t) //��Ч����

//���鴮��ѡ�� 1��ʹ�����⴮��USB     0��ʹ�ô���1
#define AUTO_UART_SELECT         1
//��ʼ��CRCУ��ֵ
#define CRC_INIT_AUTO            0xFFFF

typedef enum{
	RED = 0,
	BLUE
}ROBOT_TEAM;

typedef __packed struct
{
	uint8_t header;						//֡ͷ 0xA5
	ROBOT_TEAM detect_color : 1; 		//��������ɫ�췽�������� 0-red,1-blue
	bool_t reset_tracker : 1;			//�Ƿ�׷�ٵ����� Whether to reset the tracker
	uint8_t reserved : 6;				//����λ Reserved bit
 
	float roll;		//ROLL��Ƕ�
	float pitch;    //PITCH��Ƕ�
	float yaw;		//YAW�ܽǶ�
	
	float aim_x;	//Ŀ��X 
	float aim_y;	//Ŀ��Y
	float aim_z;	//Ŀ��Z
	uint16_t checksum;	//CRCУ��ֵ CRC checksum
}send_packed_t;


typedef __packed struct
{
	uint8_t header;				//֡ͷ Frame header 0xA5
	bool_t tracking : 1;		//׷�ٵ����˱�־ Whether the target is tracked.
	uint8_t id : 3;				//׷�ٵ���Ŀ��װ�װ�ID Target id
	bool_t armors_num : 3;		//Ŀ��װ�װ����� Number of decks
	uint8_t reserved : 1;		//����λ Reserved bit
    
	float x;     //Ŀ��X��������
	float y;	 //Ŀ��Y��������
	float z;	 //Ŀ��Z��������
	float yaw;   //Ŀ��YAW
    
	float vx;	 //Ŀ��VX
	float vy;	 //Ŀ��VY
	float vz;	 //Ŀ��VZ
	float v_yaw; //Ŀ��v_yaw
	
	float r1;    //Ŀ�공�峤��
	float r2;	 //Ŀ�공����
	float dz;	 //Ŀ�공����װ�װ�߶Ȳ� 
	uint16_t checksum;	//CRCУ��ֵ CRC checksum	
}received_packed_t;

typedef struct
{
  received_packed_t *p_header;     //ͷָ��
  uint16_t       data_len;         //���ݳ���
  uint8_t        protocol_packet[sizeof(received_packed_t)];  //���������
  uint8_t  		 unpack_step;      //�������
  uint16_t       index;            //�������
} unpack_autoshoot_data_t;


typedef struct
{
	fp32 pitch_lower_tar;	//Ԥ����PITCH
	fp32 yaw_lower_tar;     //Ԥ����YAW
	fp32 aim_x;		//Ԥ����Ŀ��X
	fp32 aim_y;		//Ԥ����Ŀ��Y
	fp32 aim_z;		//Ԥ����Ŀ��Z
}lower_target_t;

typedef struct
{
	float range_yaw;   //YAW�ɿ��������Ĳ�����Χ
	float range_pitch; //PITCH�ɿ��������Ĳ�����Χ
}fire_control_t;       //���ƿ���YAW��PITCH��Χ



void auto_task(void const * argument);
void AUTO_init(void);
void Auto_Track(gimbal_control_t *gimbal_control_set);
void send_data_to_upper(send_packed_t *send_packed,lower_target_t *lower_target);
uint16_t Get_CRC16_Check_Sum_Auto(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
float lowPassFilter(float input, float prevOutput);
void UART1_receive_IDE(void);
extern float get_UI_dis_data(void);
extern const received_packed_t *get_auto_shoot_point(void);

#endif

