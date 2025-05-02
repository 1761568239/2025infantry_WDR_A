#include "auto_shoot.h"
#include "referee.h"
#include "arm_math.h"
#include "user_lib.h"
#include "tim.h"
#include "SolveTrajectory.h"
#include "fifo.h"

//================================  静态函数申明 =====================================//
static void autoshoot_unpack_fifo_data(void);
static void auto_uart_transmit(send_packed_t *send_packed);
//================================  静态函数申明 =====================================//


//================================  全局变量定义区域 =====================================//
extern UART_HandleTypeDef huart1; 						//串口1的句柄
extern DMA_HandleTypeDef  hdma_usart1_rx;				//串口1的DMA的句柄
extern fp32 INS_angle[3];			 					//陀螺仪数据
uint8_t frame,frame_count = 0;							//当前接收数据的帧率和帧率计数值	
uint8_t RX_buf[RX_BUF_NUM];                             //DMA接收不缓冲区
received_packed_t received_packed;                      //解包完成接收数据结构体
send_packed_t send_packed;                              //发送数据结构体
lower_target_t lower_target;                            //落点，传回上位机用于可视化
fifo_s_t auto_shoot_fifo;                               //解包FIFO
uint8_t auto_shoot_fifo_buf[AUTO_SHOOT_FIFO_BUF_LENGTH];  //FIFO缓冲区
unpack_autoshoot_data_t autoshoot_unpack_obj;             //解包管理结构体
fire_control_t fire_control;                              //火控结构体
//================================  全局变量区域 =====================================//

/**
  * @brief    自瞄任务，解包自瞄数据，并发送感到上位机              
  * @param    无
  * @param    无
  * @retval   无      
  */
void auto_task(void const * argument)
{
	AUTO_init();
	st_Data_Iint();
	fifo_s_init(&auto_shoot_fifo, auto_shoot_fifo_buf, AUTO_SHOOT_FIFO_BUF_LENGTH);
	memset(&received_packed, 0, sizeof(received_packed_t));
	memset(&send_packed, 0, sizeof(send_packed_t));
	
	while(1)
	{
		autoshoot_unpack_fifo_data();
		send_data_to_upper(&send_packed,&lower_target);
		vTaskDelay(10);
	}
}

/**
  * @brief    自瞄任务，解包自瞄数据，并发送感到上位机              
  * @retval   无      
  */
void AUTO_init(void)            //初始化串口与DMA，设置数据长度
{
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,RX_buf,AUTO_SHOOT_FIFO_BUF_LENGTH);
}

/**
  * @brief          对中断接收完的数据进行解包和校验后放入自瞄接收结构体
  * @param[in]      无
  * @retval         无
  */
static void autoshoot_unpack_fifo_data(void)
{
	uint16_t check_sum = 0;
    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    unpack_autoshoot_data_t *p_obj = &autoshoot_unpack_obj;
    p_obj->data_len = sizeof(received_packed_t);
    while(fifo_s_used(&auto_shoot_fifo))
    {
        byte = fifo_s_get(&auto_shoot_fifo);
        switch (p_obj->unpack_step)
        {
            //包头检验
            case 0:
            {
                if(byte == sof)
                {
                    p_obj->unpack_step = 1;
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }else
                {
                    p_obj->index = 0;
                }
            }break;
            case 1:
            {
                if(p_obj->index < p_obj->data_len)
                {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                if(p_obj->index >= p_obj->data_len)
                {
                    p_obj->unpack_step = 0;
                    p_obj->index = 0;
                    check_sum = (p_obj->protocol_packet[sizeof(received_packed) - 1] << 8) | p_obj->protocol_packet[sizeof(received_packed) - 2];
                    if(Get_CRC16_Check_Sum_Auto(p_obj->protocol_packet, sizeof(received_packed) - sizeof(check_sum), CRC_INIT_AUTO) == check_sum)
                    {
                        //复制至结构体中
                        memcpy(&received_packed, p_obj->protocol_packet, sizeof(received_packed)); 
                        SolveDataUnpack(&received_packed);   //自瞄装甲板位置更新
                    }
                }
            }break;
            default:
            {
                p_obj->unpack_step = 0;
                p_obj->index = 0;
            }break;
        }
    }
}

/**
  * @brief          发送自瞄落点数据回NUC用于视觉可视化观测
  * @param[in]      send_packed:待发送的结构体指针，在该函数将赋值
  * @param[in]      lower_target: 存储预测完成后数据的结构体指针
  * @retval         无
  */
void send_data_to_upper(send_packed_t *send_packed,lower_target_t *lower_target){
	//帧头
	send_packed->header = 0x5A;
	
	//获取当前机器人颜色
	if(get_robot_id() <= RED_SENTRY){  //红色
		send_packed->detect_color =BLUE;   
	}
	else if(get_robot_id() >= BLUE_HERO){  
		send_packed->detect_color =RED;
	}
	
	//是否重置追踪器
	if(gimbal_behaviour == GIMBAL_AUTO&&(get_gimbal_control_point()->gimbal_rc_ctrl->key.v & AUTO_SHOOT_RESET))  //R键重置追踪
	{
		send_packed->reset_tracker = 1;
	}else
	{
		send_packed->reset_tracker = 0;
	}
	
	//获取云台角度数据
	send_packed->roll  = INS_angle[2];
	send_packed->pitch = INS_angle[1];
	send_packed->yaw   = INS_angle[0];
	//检测到装甲板
	if(received_packed.tracking)
	{
		lower_target->yaw_lower_tar = send_packed->yaw;
		//自瞄预测
		autoSolveTrajectory(&lower_target->pitch_lower_tar, &lower_target->yaw_lower_tar, &lower_target->aim_x, &lower_target->aim_y ,&lower_target->aim_z);		
		send_packed->aim_x = lower_target->aim_x;
		send_packed->aim_y = lower_target->aim_y;
		send_packed->aim_z = lower_target->aim_z;
	}
	//获取校验码
	send_packed->checksum = Get_CRC16_Check_Sum_Auto((uint8_t *)(send_packed),sizeof(send_packed_t) - 2,CRC_INIT_AUTO);
	
	//串口发送数据
	auto_uart_transmit(send_packed);
}

/**
  * @brief          自瞄数据发送函数，将虚拟串口发送和串口发送进行封装
  * @param[in]      send_packed:发送数据结构体指针
  * @param[in]      无
  * @retval         无
  */
static void auto_uart_transmit(send_packed_t *send_packed)
{
	#if(AUTO_UART_SELECT == 1)      //是否使用虚拟串口发送
	CDC_Transmit_FS((uint8_t *)send_packed,sizeof(send_packed_t));
	#else
	HAL_UART_Transmit(&huart1,(uint8_t *)send_packed,sizeof(send_packed_t),5);
	#endif
}

/**
  * @brief          设置自瞄模式下云台角度目标值
  * @param[out]      gimbal_control_set:云台结构体指针
  * @param[in]      无
  * @retval         无
  */
void Auto_Track(gimbal_control_t *gimbal_control_set)
{
	if (gimbal_control_set == NULL)
    {
        return;
    }
	if(received_packed.tracking)
	{
		gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = lower_target.pitch_lower_tar;		
		gimbal_control_set->gimbal_yaw_motor.absolute_angle_set   = rad_format(lower_target.yaw_lower_tar);//yaw_lower_tar;
	}   
}

/**
  * @brief    串口中断函数          
  * @retval   无      
  */
void USART1_IRQHandler(void)  
{
	if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		UART1_receive_IDE();
	}
	HAL_UART_IRQHandler(&huart1);
}

/**
  * @brief    串口接收函数          
  * @retval   无      
  */
void UART1_receive_IDE(void)
{
	static uint16_t check_sum = 0;
	HAL_UART_DMAStop(&huart1);  //停止DMA传输
//	fifo_s_puts(&auto_shoot_fifo, (char*)RX_buf, DATE_LENGTH);    //现在放在虚拟串口接收里面
	HAL_UART_Receive_DMA(&huart1,RX_buf, RX_BUF_NUM);  //打开DMA继续接收
}


/**
  * @brief    返回自瞄接收完成后的结构体指针          
  * @retval   received_packed      
  */
const received_packed_t *get_auto_shoot_point(void){
	return &received_packed;	
}

/**
  * @brief    返回视觉距离          
  * @retval   dis      
  */
float get_UI_dis_data(void)  
{
	return (sqrt(received_packed.x * received_packed.x + received_packed.y * received_packed.y));
}

/**
  * @brief    此定时器用于计算视觉发送数据的帧率          
  * @retval   无      
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int i = 0;
	if(htim->Instance == TIM1)
	{
		//定时发送数据/50Hz
		//发送云台数据给上位机,显示自瞄数据波形
		//用户代码
		i++;
		if(i == 50)
		{
			i = 0;
			frame = frame_count;
			frame_count = 0;
		}
	}
}

/**
  * @brief    低通滤波函数          
  * @retval   无      
  */
float lowPassFilter(float input, float prevOutput) 
{
	static float alpha = 0.8;
    // 低通滤波公式：output = (1 - ALPHA) * input + ALPHA * prevOutput
    float output = (1 - alpha) * input + alpha * prevOutput;
    return output;
}


//==============================  CRC校验表和函数 =================================//
const uint16_t wCRC_Table_Auto[256] = 
{ 
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
  * @brief          计算CRC校验值
  * @param[in]      pchMessage:云台结构体指针
  * @param[in]      dwLength：待校验数据的长度，单位：字节
  * @param[in]      wCRC：校验时数据的保存
  * @retval         wCRC：校验完成发后的校验值
  */
uint16_t Get_CRC16_Check_Sum_Auto(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) 
{
	uint8_t chData;
	if (pchMessage == NULL)
	{
		return 0xFFFF;
	}
	while(dwLength--) 
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table_Auto[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff]; 
	}
	return wCRC; 
}


