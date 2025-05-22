#ifndef _GafProjectileSolver_H_
#define _GafProjectileSolver_H_
#include <stdio.h>
#include "arm_math.h"

#define GRAVITY 	9.7947f
#define MAX_ITER 	100           //100

//用于计算pitch轴角度
#define DEFAULT_VEL 25

#ifndef PI
#define PI 3.141592653
#endif

unsigned char solver(float vel, float coeff, float target_x, float target_h, float* angle);
void forward_motion(float vel, float coeff, float given_angle, float given_x, float* h, float* t);

#endif // !_GafProjectileSolver_H_


