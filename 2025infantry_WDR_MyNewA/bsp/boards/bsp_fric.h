#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#define FRIC_UP 1600					// ��ΧΪ 1000-1800 1800���Ϊ28m/s �Ҽ��������
#define FRIC_DOWN 1460        // �������������ٶȣ� 1500 -14.8
#define FRIC_OFF 1000        // ��Сֵ

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
