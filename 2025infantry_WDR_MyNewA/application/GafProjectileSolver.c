#include "GafProjectileSolver.h"

// 定义速度偏差表的最大距离（米），修改为7米
#define MAX_DISTANCE_TABLE 7   //15

// 速度偏差查找表（每米一个数据点），修改为最大15米
// 可以根据实际测试结果调整这些值
float vel_bias_table[MAX_DISTANCE_TABLE + 1] = {
    -20.0f,     // 0米
    -15.0f,     // 1米
    -10.0f,     // 2米
    -3.0f,      // 3米 
    -2.0f,       // 4米
    -2.0f,       // 5米
    0.0f,      // 6米,
    -1.0f,       // 7米 只到这里
};


/**
 * @brief 根据距离获取速度偏差值
 * 		  使用线性插值计算两个距离点之间的速度偏差
 * @param distance 目标距离（米）
 * @return float 对应的速度偏差值
 */
float get_velocity_bias(float distance) {
    // 如果距离超出表格范围，使用表格中的最大值
    if (distance >= MAX_DISTANCE_TABLE) {
        return vel_bias_table[MAX_DISTANCE_TABLE];
    }

    // 如果距离为负或为0，返回0偏差
    if (distance <= 0) {
        return 0.0f;
    }

    // 获取距离的整数部分和小数部分
    int lower_idx = (int)distance;
    float fraction = distance - lower_idx;

    // 计算上下边界索引
    int upper_idx = lower_idx + 1;
    if (upper_idx > MAX_DISTANCE_TABLE) {
        upper_idx = MAX_DISTANCE_TABLE;
    }

    // 使用线性插值计算速度偏差
    float bias = vel_bias_table[lower_idx] * (1.0f - fraction) + vel_bias_table[upper_idx] * fraction;

    return bias;
}

/**
 * @brief 弹道求解器
 * 根据目标距离、高度、发射速度和空气阻力系数计算所需的发射角度
 *
 * @param vel 子弹出射速度 (m/s)
 * @param coeff 空气阻力系数
 * @param target_x 目标水平距离 (m)
 * @param target_h 目标垂直高度 (m)
 * @param angle 计算得到的云台pitch角度 (rad)，作为输出参数
 * @return unsigned char 1表示计算成功，0表示计算失败
 */
unsigned char solver(float vel, float coeff, float target_x, float target_h, float *angle) {
    float aimed_h, h;   // aimed_h: 迭代中的目标高度，h: 当前计算得到的高度
    float dh = 0;       // 高度误差
    float tmp_angle = 0;  // 临时角度值
    float t = 0;        // 飞行时间

    aimed_h = target_h;  // 初始化目标高度为实际目标高度

    // 迭代求解最优发射角度
    for (int i = 0; i < MAX_ITER; i++) {
        // 计算当前迭代的角度
        tmp_angle = atan2(aimed_h, target_x);

        // 检查角度是否在有效范围内 (-80°到80°)
        if (tmp_angle > 80.0f * PI / 180.0f || tmp_angle < -80.0f * PI / 180.0f) {
            // 如果角度超出范围，返回失败
            return 0;
        }

        // 使用前向运动模型计算在当前角度下子弹的实际落点高度和飞行时间
        forward_motion(vel, coeff, tmp_angle, target_x, &h, &t);

        // 检查飞行时间是否合理（不超过10秒）
        if (t > 10) {
            // 飞行时间过长，误差过大，返回失败
            return 0;
        }

        // 计算高度误差
        dh = target_h - h;

        // 更新目标高度，用于下一次迭代
        aimed_h = aimed_h + dh;

        // 调试信息（已注释）
        // printf("第%d次迭代:俯仰角:%f,临时目标点y值:%f,高度误差:%f\n", i, (tmp_angle / PI) * 180, aimed_h, dh);

        // 如果高度误差小于阈值，迭代结束
        if (fabs(dh) < 0.001f) {
            break;
        }
    }

    // 如果最终高度误差仍然较大，返回失败
    if (fabs(dh) > 0.01f) {
        return 0;
    }

    // 将最终计算的角度赋值给输出参数
    *angle = tmp_angle;
    return 1;
}

/**
 * @brief 设置弹道前向运动解算函数：使用抛物线模型
 * 根据给定的初始速度、角度和水平距离，计算子弹的落点高度和飞行时间
 *
 * @param vel 子弹出射速度 (m/s)
 * @param coeff 空气阻力系数 (不再使用)
 * @param given_angle 出射角度 (rad)
 * @param given_x 指定的水平射击距离 (m)
 * @param h 输出参数，计算得到的落点高度 (m)
 * @param t 输出参数，计算得到的飞行时间 (s)
 */
void forward_motion(float vel, float coeff, float given_angle, float given_x, float* h, float* t) {
    // 应用速度偏差，所有阶段都应用
    float adjusted_vel = vel;
    float vel_bias = get_velocity_bias(given_x); // 获取速度偏差值
    adjusted_vel = vel + vel_bias; // 应用速度偏差

    // 统一使用抛物线模型进行解算（忽略空气阻力）
    *t = given_x / (adjusted_vel * arm_cos_f32(given_angle));
    *h = adjusted_vel * arm_sin_f32(given_angle) * (*t) - (GRAVITY * (*t) * (*t)) / 2.0f;
}
