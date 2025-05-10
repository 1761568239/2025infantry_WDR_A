
//最新的
#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
ext_referee_warning_t referee_warning_t;


ext_game_robot_state_t robot_state;
ext_power_heat_data_t power_heat_data_t;  //热量 280
robot_pos_t game_robot_pos_t;
buff_t buff_musk_t;
air_support_data_t robot_energy_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;          //射速 30m/s

ext_bullet_remaining_t bullet_remaining_t;   
robot_interaction_data_t robot_interaction_data;

//RFID_STATUS_CMD_ID                = 0x0209
rfid_status_t  rfid_status;
//DART_CMD_ID                    = 0x020A
dart_client_cmd_t dart_client_cmd;
//GROUND_ROBOT_POSITION_CMD_ID            = 0x020B
ground_robot_position_t ground_robot_position;
//LIDAR_MARK_PROGRESS_CMD_ID        = 0x020C
radar_mark_data_t radar_mark_data;
//SENTRY_DECISION_CMD_ID            = 0x020D
sentry_info_t sentry_info;
//LIDAR_DECISION_CMD_ID             = 0x020E
radar_info_t radar_info;
//COSTUM_ROBOT_CMD_ID               = 0x0302
custom_robot_data_t custom_robot_data;
//MAP_COMMAND_CMD_ID                = 0x0303
map_command_t map_command;
//REMOTE_CONTROL_CMD_ID                  = 0x0304
remote_control_t remote_control;


void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));

    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));


    memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(buff_t));
    memset(&robot_energy_t, 0, sizeof(air_support_data_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));

    memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));
}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(ext_game_result_t));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;

        //?????????λ???
        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(ext_event_data_t));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, frame + index, sizeof(ext_supply_projectile_action_t));
        }
        break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:      //??????????????????20240310
        {
            memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(ext_game_robot_state_t));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t, frame + index, sizeof(ext_power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index, sizeof(robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID:                  //0x0204
        {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:        //0x0205
        {
            memcpy(&robot_energy_t, frame + index, sizeof(air_support_data_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:                 //0x0206
        {
            memcpy(&robot_hurt_t, frame + index, sizeof(ext_robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:                 //0x0207
        {
            memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:           //0x0208
        {
            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;
        
        //??????????? 0x0209-0x020E
        case RFID_STATUS_CMD_ID:           //0x0209
        {
            memcpy(&rfid_status, frame + index, sizeof(rfid_status_t));
        }
        break;
        case DART_CMD_ID:
        {
            memcpy(&dart_client_cmd,frame + index, sizeof(dart_client_cmd_t));
        }
        break;
        case GROUND_ROBOT_POSITION_CMD_ID:
        {
            memcpy(&ground_robot_position,frame + index, sizeof(ground_robot_position_t));
        }
        break;
        case LIDAR_MARK_PROGRESS_CMD_ID:
        {
            memcpy(&radar_mark_data,frame + index, sizeof(radar_mark_data_t));
        }
        break;
        case SENTRY_DECISION_CMD_ID:
        {
            memcpy(&sentry_info,frame + index, sizeof(sentry_info_t));
        }
        break;
        case LIDAR_DECISION_CMD_ID:
        {
            memcpy(&radar_info,frame + index, sizeof(radar_info_t));
        }
        break;

        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data_t));
        }
        break;

        case COSTUM_ROBOT_CMD_ID:
        {
            memcpy(&custom_robot_data, frame + index, sizeof(custom_robot_data_t));
        }
        break;
        case MAP_COMMAND_CMD_ID:
        {
            memcpy(&map_command, frame + index, sizeof(map_command_t));
        }
        break;
        case REMOTE_CONTROL_CMD_ID:
        {
            memcpy(&remote_control, frame + index, sizeof(remote_control_t));
        }
        break;
        default:
        {
            break;
        }
    }
}


/**
  * @brief          获取底盘功率上限
  * @param[out]     power_limit:底盘功率上限指针
  * @param[out]     none
  * @retval         none
  */
void get_chassis_power_limit(uint16_t *power_limit)
{
	*power_limit = robot_state.chassis_power_limit;
}

/**
  * @brief		获取机器人ID
  * @retval		robot_state.robot_id
  */
uint8_t get_robot_id(void)
{
    return (robot_state.robot_id);
}

/**
  * @brief		获取被击打装甲板ID
  * @retval		&chassis_move
  */
uint8_t get_Armor_Attacked_ID(void)
{
    if(robot_hurt_t.hurt_type == 0)
    {
        return (robot_hurt_t.armor_type);
    }
}

/**
  * @brief          获取枪口1热量和热量上限
  * @param[out]     heat1_limit:热量上限指针
  * @param[out]     heat1:当前热量指针
  * @retval         none
  */
void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_state.shooter_barrel_heat_limit;
    *heat1 = power_heat_data_t.shooter_id1_17mm_cooling_heat;
}

/**
  * @brief          获取枪口2热量和热量上限
  * @param[out]     heat2_limit:热量上限指针
  * @param[out]     heat2:当前热量指针
  * @retval         none
  */
void get_shoot_heat2_limit_and_heat2(uint16_t *heat2_limit, uint16_t *heat2)
{
    *heat2_limit = robot_state.shooter_barrel_heat_limit;
    *heat2 = power_heat_data_t.shooter_id2_17mm_cooling_heat;
}

/**
  * @brief          获取枪口2热量和热量上限
  * @param[out]     shoot_limit:射速上限指针
  * @param[out]     none
  * @retval         none
  */
void get_shoot_speed_limit (uint16_t *shoot_limit)
{
	*shoot_limit = 25;  
}

/**
  * @brief		获取允许发弹量值
  * @retval		&chassis_move
  */
uint16_t get_remain_bullet_num(void)
{
	return bullet_remaining_t.bullet_remaining_num_17mm;
}

/**
  * @brief		获取机器人当前等级
  * @retval		&shoot
  */
uint16_t get_robot_level(void)
{
	return robot_state.robot_level;
}

/**
  * @brief          读取当前弹速
  * @param[out]     bullet_speed:当前弹速指针
  * @param[out]     none
  * @retval         none
  */
void get_current_bullet_speed(fp32 *bullet_speed)
{
	*bullet_speed = shoot_data_t.bullet_speed;
}

/**
  * @brief		获取机器人剩余血量
  * @retval		none
  */
uint16_t get_robot_remain_HP(void)
{
	return robot_state.current_HP;
}
