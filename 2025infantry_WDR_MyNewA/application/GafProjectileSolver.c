#include "GafProjectileSolver.h"

int watch_flag = 0;

/**
 * @brief 弹道求解器
 * 函数类型: unsigned char solver(float vel,float coeff,float target_x, float target_h, float *angle)
 * @param vel 出射速度 (m/s)
 * @param coeff 空气阻力系数 
 * @param target_x 目标水平距离 (m)
 * @param target_h 目标竖直高度 (m)
 * @param angle 云台pitch角度 (rad)
 * @return 1 or 0
 */
unsigned char solver(float vel,float coeff,float target_x, float target_h, float *angle) {
    float aimed_h, h;//目标高度,当前高度
    float dh = 0;//高度差
    float tmp_angle = 0;//临时角度
    float t = 0;//时间
    aimed_h = target_h;//初始化目标高度

    //迭代求解
    for (int i = 0; i < MAX_ITER; i++) {
        //计算临时角度
        tmp_angle = atan2(aimed_h,target_x);

        //检查角度范围是否在(-80d,80d)之间
        if (tmp_angle > 80 * PI / 180 || tmp_angle < -80 * PI / 180) {
            //迭代角度超过正负80度
            return 0;
        }

        //使用向前计算函数计算高度和时间
        forward_motion(vel, coeff, tmp_angle, target_x, &h, &t);

        //检查运动时间是否超过10s
        if (t > 10) {
            //运动时间太长了,误差过于大,不予计算
            return 0;
        }

        //计算高度差
        dh = target_h - h;

        //更新目标高度
        aimed_h = aimed_h + dh;

        //打印迭代信息
        //printf("第%d次迭代:仰角:%f,临时目标点y值:%f,高度误差:%f\n", i, (tmp_angle / PI) * 180, aimed_h, dh);

        //如果高度差小于0.001,则迭代结束
        if (fabs(dh) < 0.001) {
            break;
        }
    }

    //如果高度差大于0.01,则返回错误
    if (fabs(dh) > 0.01) {
        return 0;
    }

    //将最终计算的角度赋值给参数angle
    *angle = tmp_angle;

    return 1;
}

/**
 * @brief 设置弹道前向运动解算函数:向前运动模型:考虑重力和空气摩擦模型
 * 函数类型: void forward_motion(float vel, float coeff, float given_angle, float given_x, float* h, float* t)
 * 在水平坐标系下
 * @param vel: input,出射速度 / m/s
 * @param coeff: input,空气阻力系数
 * @param given_x: input,射击距离 / m
 * @param given_angle: input,出射角度 / rad
 * @param h: output,射击的落点高度 / m
 * @param t: output,射击飞行时间 / s
 * @return void
 */
void forward_motion(float vel, float coeff, float given_angle, float given_x, float* h, float* t) {
    //如果存在上升阶段
    if (given_angle > 0.01) {
			watch_flag = 0;
        float t0, x0, y0;//上升阶段最高点的坐标和到达时间
        t0 = vel * sin(given_angle) / GRAVITY;
        x0 = vel * cos(given_angle) * t0;
        y0 = (GRAVITY * t0 * t0) / 2;
        //如果只存在上升阶段,退化为抛物线模型
        if (given_x < x0) {
            *t = given_x / (vel * cos(given_angle));
            *h = vel * sin(given_angle) * (*t) - (GRAVITY * (*t) * (*t)) / 2;
        }
        //先上升,后下降
        else
        {
            float t1, x1;
            x1 = given_x - x0;
            t1 = (exp(coeff * x1) - 1) / (coeff * vel * cos(given_angle));
            *t = t0 + t1;
            *h = y0 - (GRAVITY * t1 * t1) / 2;
        }
    }
    //只有下降阶段
    else
    {
				watch_flag = 1;
        *t = (exp(coeff * given_x) - 1) / (coeff * vel * cos(given_angle));
        *h = vel * sin(given_angle) * (*t) - (GRAVITY * (*t) * (*t)) / 2;
    }
}