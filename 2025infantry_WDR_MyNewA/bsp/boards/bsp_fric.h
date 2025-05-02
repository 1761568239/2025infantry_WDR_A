#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#define FRIC_UP 1600					// 范围为 1000-1800 1800大概为28m/s 右键提速射击
#define FRIC_DOWN 1460        // 左键射击（正常速度） 1500 -14.8
#define FRIC_OFF 1000        // 最小值

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
