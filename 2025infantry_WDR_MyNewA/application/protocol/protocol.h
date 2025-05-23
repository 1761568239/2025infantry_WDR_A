/*
 * @Date: 2024-03-09 20:28:13
 * @LastEditors: Towers
 * @LastEditTime: 2024-03-10 15:36:16
 */
#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001,
    GAME_RESULT_CMD_ID                = 0x0002,
    GAME_ROBOT_HP_CMD_ID              = 0x0003,
    FIELD_EVENTS_CMD_ID               = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,
    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,
    REFEREE_WARNING_CMD_ID            = 0x0104,
    ROBOT_STATE_CMD_ID                = 0x0201,
    POWER_HEAT_DATA_CMD_ID            = 0x0202,
    ROBOT_POS_CMD_ID                  = 0x0203,
    BUFF_MUSK_CMD_ID                  = 0x0204,
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,
    ROBOT_HURT_CMD_ID                 = 0x0206,
    SHOOT_DATA_CMD_ID                 = 0x0207,
    BULLET_REMAINING_CMD_ID           = 0x0208,
    //新增的指令码 0x0209-0x020E
    RFID_STATUS_CMD_ID                = 0x0209,
    DART_CMD_ID                       = 0x020A,
    GROUND_ROBOT_POSITION_CMD_ID      = 0x020B,
    LIDAR_MARK_PROGRESS_CMD_ID        = 0x020C,
    SENTRY_DECISION_CMD_ID            = 0x020D,
    LIDAR_DECISION_CMD_ID             = 0x020E,
    //需要额外的数据处理子命令
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,
    //新增的指令码 0x0302-0x0308
    COSTUM_ROBOT_CMD_ID               = 0x0302,
    MAP_COMMAND_CMD_ID                = 0x0303,
    REMOTE_CONTROL_CMD_ID                  = 0x0304,
    // RFID_STATUS_CMD_ID                = 0x0305,  雷达发送
    // RFID_STATUS_CMD_ID                = 0x0306,  自定义控制器发送
    // RFID_STATUS_CMD_ID                = 0x0307,  半/全自动机器人发送
    // RFID_STATUS_CMD_ID                = 0x0308,  机器人发送
    IDCustomData,
}referee_cmd_id_t;
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
