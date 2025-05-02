/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/

// 近点只考虑水平方向的空气阻力
//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算

#include <math.h>
#include <stdio.h>
#include "SolveTrajectory.h"
#include "GafProjectileSolver.h"
#include "auto_shoot.h"
#include "user_lib.h"

struct SolveTrajectoryParams st;
extern received_packed_t received_packed;
extern gimbal_control_t gimbal_control;
float  t = 0.12f;    // 飞行时间
uint8_t control_status = 0;
float ALLOW_ERROR_DISTANCE = 0.09;
float fly_time;
float pre_xc;
float pre_yc;
float pre_yaw;
float smoothing_factor = 0;

/**
  * @brief   初始化自动但刀片解算相关参数               
  * @param   无
  * @param   无
  * @retval  无         
  */
void st_Data_Iint(void)
{
	//定义参数
	st.k = 0.092f;                   //弹道系数
	st.bullet_type =  BULLET_17;     //弹丸类型
	st.current_v = 18.0;               //弹丸出射速度    17.5   26
	st.current_pitch = 0;            
	st.current_yaw = 0;
	st.xw = 3.0f;    //3.0f
	// st.yw = 0.0159;
	st.yw = 0;
	// st.zw = -0.2898;
	st.zw = 1.5f;

	st.vxw = 0;
	st.vyw = 0;
	st.vzw = 0;
	st.v_yaw = 0;
	st.tar_yaw = 0.0f;  //0.09131f;

	st.r1 = 0.5f;       //目标车的宽度
	st.r2 = 0.5f;       //目标车的长度
 
	st.dz = 0.1f;              //目标车两对装甲板高度差
	st.bias_time = 150.0f;       //50.0f
	st.s_bias = 0.19133f;      //枪口前推距离
	st.z_bias  = 0.21265f;     //YAW电机到枪口高度差
	st.armor_id = ARMOR_INFANTRY3;
	st.armor_num = ARMOR_NUM_NORMAL;
}

/**
 *@brief 动态预测权重计算函数，添加前哨站特殊处理
 *@param[in] v_yaw:rad  传入v_yaw
 *@param[in] armor_id   传入armor_id
 *@retval factor		返回factor
 */

//float smoothing_factor = 1.0f; // 平滑系数，可以根据实际情况调整 越大越相信预测结果
float calculate_dynamic_smoothing_factor(float v_yaw, uint8_t armor_id) 
{
    // 前哨站特殊处理
    if (armor_id == ARMOR_OUTPOST) {
        // 为前哨站提供更高的预测权重，提高对预测结果的信任度
        return 0.35f;  // 可以根据实际测试调整这个值
    }
    float factor = 0.5f * fabs(v_yaw) - 0.4f;//0.4  0.2
		
		if(factor <= 0.2f)    // 确保平滑系数不会低于最小值，防止负值或过小值
			return 0.2f;
		else if (factor > 0.2f && factor <= 1.5f)
			return factor;
		else
			return 1.0f;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/

float watch_debug_1 = 0;
float watch_debug_2 = 0;
float watch_debug_3 = 0;
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{
    float xc = st.xw;
    float yc = st.yw;
    float z = st.zw;
    float armor_yaw = st.tar_yaw;
    float v_yaw = st.v_yaw;
    float r1 = st.r1;
    float r2 = st.r2;
		float linear_speed = sqrt(st.vxw*st.vxw + st.vyw*st.vyw);//目标平移速度
	  smoothing_factor = calculate_dynamic_smoothing_factor(v_yaw, st.armor_id);//预测权重，可以根据实际情况调整 越大越相信预测结果
        st.current_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        st.current_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
	  // 调试
	
//    filtered_v_yaw = filtered_v_yaw * (1-BASE_YAW_FILTER_COEFF) + v_yaw * BASE_YAW_FILTER_COEFF;
//		if (linear_speed > 0.3f)// 避免静止目标跟随慢
//		{
//				filtered_v_yaw += step_size; // 增量更新
//		}
//		// 调试
//		watch_debug_1 = st.v_yaw;
//		filtered_v_yaw += step_size; // 增量更新
//		watch_debug_2 = filtered_v_yaw;
//		smoothing_factor = calculate_dynamic_smoothing_factor(filtered_v_yaw, st.armor_id);//预测权重，可以根据实际情况调整 越大越相信预测结果
	  watch_debug_3 = smoothing_factor;
    //计算装甲板坐标
    float xa = xc - r1*cosf(armor_yaw);
    float ya = yc - r1*sinf(armor_yaw);
//    //计算延迟 
//    float latency = 
    //计算飞行时间
    float distance = sqrt(xa*xa + ya*ya);
    float horizontal_speed = st.current_v * cosf(st.current_pitch);

//    fly_time = calculate_fly_time_optimized(xa, ya, st.vxw, st.vyw, horizontal_speed, st.bias_time);
    fly_time = distance / horizontal_speed + st.bias_time/1000.0f;        

    //计算飞行时间后的装甲板位置
    pre_xc = xc + st.vxw * fly_time;
    pre_yc = yc + st.vyw * fly_time;
    pre_yaw = armor_yaw + st.v_yaw * fly_time;
   
    control_status = 0;
    if(fabsf(st.v_yaw) < 2.0f && received_packed.tracking == 1)
    {
        *aim_x = pre_xc - r1 * cosf(pre_yaw);
        *aim_y = pre_yc - r1 * sinf(pre_yaw);
        *aim_z = z;
        control_status = 2;
    }
        else
    {
        // 选择最优板
        float a_n = st.armor_num;
        bool_t is_current_pair = 1;
        float r = 0;
        float center_yaw = atan2(pre_yc, pre_xc);
			  float best_distance = 9999.0f;
        int best_armor_index = -1;
			
        for (size_t i = 0; i < a_n; i++) {
            float tmp_yaw = pre_yaw + i * (2 * PI / a_n);
            float yaw_diff = loop_fp32_constrain(center_yaw - tmp_yaw, -PI, PI);
            float yaw_diff_offset = signbit(v_yaw) ? -ARMOR_YAW_LIMIT_OFFSET : ARMOR_YAW_LIMIT_OFFSET;
            
            if (-ARMOR_YAW_LIMIT + yaw_diff_offset < yaw_diff &&
                yaw_diff < ARMOR_YAW_LIMIT + yaw_diff_offset) {

              if (a_n == 4) {
                r = is_current_pair ? r1 : r2;
                *aim_z = z + (is_current_pair ? 0 : st.dz);
              } else {
                r = r1;
                *aim_z = z;
              }
							// 新的装甲板选择策略：综合考虑角度和距离
							float armor_x = pre_xc - r * cosf(tmp_yaw);
              float armor_y = pre_yc - r * sinf(tmp_yaw);
              float current_distance = sqrt(armor_x*armor_x + armor_y*armor_y);
                
                // 优先选择角度和距离都最优的装甲板   下面系数可以改变权重
							float score = fabsf(yaw_diff) + 0.1f * current_distance;
              if (score < best_distance) {
                    best_distance = score;
                    best_armor_index = i;
                    
                    *aim_x = pre_xc * (1 - smoothing_factor) + (pre_xc - r * cosf(tmp_yaw)) * smoothing_factor;
                    *aim_y = pre_yc * (1 - smoothing_factor) + (pre_yc - r * sinf(tmp_yaw)) * smoothing_factor;
                    control_status = 1;
              }
              
            }
            is_current_pair = !is_current_pair;
        }
    }
    if( control_status != 0)
    {        
        float temp_pitch = 0.0f;
        solver(DEFAULT_VEL, st.k, sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias, *aim_z, &temp_pitch);
        if(temp_pitch)
            *pitch = -temp_pitch;
        
        if(*aim_x || *aim_y)
                    {
                        *yaw = (float)(atan2(*aim_y, *aim_x));
                    }
        if( control_status == 1 && received_packed.tracking == 1)
        {
            // Fire control
            ALLOW_ERROR_DISTANCE = ( st.armor_num == 2 || st.armor_id == ARMOR_HERO) ? 0.1f : ALLOW_ERROR_DISTANCE;
            ALLOW_ERROR_DISTANCE= (st.armor_id == ARMOR_OUTPOST) ? 0.05f :  ALLOW_ERROR_DISTANCE;
            float allow_error_angle_y = ALLOW_ERROR_DISTANCE / distance;
            float allow_error_angle_p = 0.02f / distance;
                    
            if (fabs(loop_fp32_constrain( st.current_yaw - (*yaw), -PI, PI)) < allow_error_angle_y &&\
                            fabs(loop_fp32_constrain( st.current_pitch - (*pitch), -PI, PI)) < allow_error_angle_p) {
              control_status = 2;
            }
        }
    }
}

/**
  * @brief    更新下位机自瞄选板数据，从视觉接收数据结构体搬运，从坐标轴正向看向原点，逆时针方向为正                
  * @param    自瞄串口接收数据结构体
  * @param	  无
  * @retval   无       
  */
void SolveDataUnpack(received_packed_t* received_data)
{   
    st.armor_id = received_data->id;
    st.armor_num = received_data->armors_num;
    
    st.xw = received_data->x;
    st.yw = received_data->y;
    st.zw = received_data->z;
    st.tar_yaw = received_data->yaw;
    st.vxw = received_data->vx;
    st.vyw = received_data->vy;
    st.vzw = received_data->vz;
    st.v_yaw = received_data->v_yaw;
    st.r1 = received_data->r1;
    st.r2 = received_data->r2;
    st.dz = received_data->dz;
}

/**
  * @brief     获取自瞄开火状态，计时判断是否云台角度已经到可以击中的范围，是则允许开火             
  * @retval    无       
  */
uint8_t get_auto_fire_state(void)
{
	return control_status;
}
