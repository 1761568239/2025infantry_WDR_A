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

#define RX_BUF_NUM                  128u    //最大接收的数据
#define YAW_AUTO_SEN                0.05
#define AUTO_SHOOT_FIFO_BUF_LENGTH  1024     //最大接收的数据
#define DATE_LENGTH    sizeof(received_packed_t) //有效数据

//自瞄串口选择 1：使用虚拟串口USB     0：使用串口1
#define AUTO_UART_SELECT         1
//初始化CRC校验值
#define CRC_INIT_AUTO            0xFFFF

typedef enum{
	RED = 0,
	BLUE
}ROBOT_TEAM;

typedef __packed struct
{
	uint8_t header;						//帧头 0xA5
	ROBOT_TEAM detect_color : 1; 		//机器人颜色红方或者蓝方 0-red,1-blue
	bool_t reset_tracker : 1;			//是否追踪到敌人 Whether to reset the tracker
	uint8_t reserved : 6;				//保留位 Reserved bit
 
	float roll;		//ROLL轴角度
	float pitch;    //PITCH轴角度
	float yaw;		//YAW周角度
	
	float aim_x;	//目标X 
	float aim_y;	//目标Y
	float aim_z;	//目标Z
	uint16_t checksum;	//CRC校验值 CRC checksum
}send_packed_t;


typedef __packed struct
{
	uint8_t header;				//帧头 Frame header 0xA5
	bool_t tracking : 1;		//追踪到敌人标志 Whether the target is tracked.
	uint8_t id : 3;				//追踪到的目标装甲板ID Target id
	bool_t armors_num : 3;		//目标装甲板数量 Number of decks
	uint8_t reserved : 1;		//保留位 Reserved bit
    
	float x;     //目标X中心坐标
	float y;	 //目标Y中心坐标
	float z;	 //目标Z中心坐标
	float yaw;   //目标YAW
    
	float vx;	 //目标VX
	float vy;	 //目标VY
	float vz;	 //目标VZ
	float v_yaw; //目标v_yaw
	
	float r1;    //目标车体长度
	float r2;	 //目标车体宽度
	float dz;	 //目标车两对装甲板高度差 
	uint16_t checksum;	//CRC校验值 CRC checksum	
}received_packed_t;

typedef struct
{
  received_packed_t *p_header;     //头指针
  uint16_t       data_len;         //数据长度
  uint8_t        protocol_packet[sizeof(received_packed_t)];  //解包缓冲区
  uint8_t  		 unpack_step;      //解包步骤
  uint16_t       index;            //解包索引
} unpack_autoshoot_data_t;


typedef struct
{
	fp32 pitch_lower_tar;	//预测后的PITCH
	fp32 yaw_lower_tar;     //预测后的YAW
	fp32 aim_x;		//预测后的目标X
	fp32 aim_y;		//预测后的目标Y
	fp32 aim_z;		//预测后的目标Z
}lower_target_t;

typedef struct
{
	float range_yaw;   //YAW可开火允许将的波动范围
	float range_pitch; //PITCH可开火允许将的波动范围
}fire_control_t;       //控制开火YAW，PITCH范围



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

