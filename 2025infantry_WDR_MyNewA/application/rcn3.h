//
// Created by X. yu on 2025/4/21.
//

#ifndef RCN3_H
#define RCN3_H

#include "struct_typedef.h"

#define RCN3_FRAME_LENGTH 21u
#define JOYSTICK_MID_VALUE 1024
#define WHEEL_MID_VALUE 1024

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_LEFT                ((uint16_t)0)
#define RC_SW_MIDDLE               ((uint16_t)1)
#define RC_SW_RIGHT              ((uint16_t)2)
#define switch_is_left(s)       (s == RC_SW_LEFT)
#define switch_is_middle(s)        (s == RC_SW_MIDDLE)
#define switch_is_right(s)         (s == RC_SW_RIGHT)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

/* ----------------------- Data Struct ------------------------------------- */
typedef struct {
    int16_t ch[5];          //5��ҡ��ͨ��
    uint8_t sw;     //��λ���أ�CNS(012)
    char fn[3];         //����������1��2�м�0
    uint8_t trigger;     //�����

    int16_t mouse_x;        //���X�ᣨ�����ٶȣ�
    int16_t mouse_y;        //���Y�ᣨǰ���ٶȣ�
    int16_t mouse_z;        //���Z�ᣨ�����ٶȣ�
    uint8_t mouse_left;   //������
    uint8_t mouse_right;  //����Ҽ�
    uint8_t mouse_middle; //����м�
    uint16_t key;           //���̰���

} __attribute__((packed)) RCN3_data_t;

void uart_to_rcn3(RCN3_data_t *rcn3, uint8_t *rx_buf);

RCN3_data_t *get_rcn3_point(void);

#endif //RCN3_H
