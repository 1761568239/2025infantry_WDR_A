#include "GafProjectileSolver.h"

// �����ٶ�ƫ���������루�ף����޸�Ϊ7��
#define MAX_DISTANCE_TABLE 7   //15

// �ٶ�ƫ����ұ�ÿ��һ�����ݵ㣩���޸�Ϊ���15��
// ���Ը���ʵ�ʲ��Խ��������Щֵ
float vel_bias_table[MAX_DISTANCE_TABLE + 1] = {
    -20.0f,     // 0��
    -15.0f,     // 1��
    -10.0f,     // 2��
    -3.0f,      // 3�� 
    -2.0f,       // 4��
    -2.0f,       // 5��
    0.0f,      // 6��,
    -1.0f,       // 7�� ֻ������
};


/**
 * @brief ���ݾ����ȡ�ٶ�ƫ��ֵ
 * 		  ʹ�����Բ�ֵ�������������֮����ٶ�ƫ��
 * @param distance Ŀ����루�ף�
 * @return float ��Ӧ���ٶ�ƫ��ֵ
 */
float get_velocity_bias(float distance) {
    // ������볬�����Χ��ʹ�ñ���е����ֵ
    if (distance >= MAX_DISTANCE_TABLE) {
        return vel_bias_table[MAX_DISTANCE_TABLE];
    }

    // �������Ϊ����Ϊ0������0ƫ��
    if (distance <= 0) {
        return 0.0f;
    }

    // ��ȡ������������ֺ�С������
    int lower_idx = (int)distance;
    float fraction = distance - lower_idx;

    // �������±߽�����
    int upper_idx = lower_idx + 1;
    if (upper_idx > MAX_DISTANCE_TABLE) {
        upper_idx = MAX_DISTANCE_TABLE;
    }

    // ʹ�����Բ�ֵ�����ٶ�ƫ��
    float bias = vel_bias_table[lower_idx] * (1.0f - fraction) + vel_bias_table[upper_idx] * fraction;

    return bias;
}

/**
 * @brief ���������
 * ����Ŀ����롢�߶ȡ������ٶȺͿ�������ϵ����������ķ���Ƕ�
 *
 * @param vel �ӵ������ٶ� (m/s)
 * @param coeff ��������ϵ��
 * @param target_x Ŀ��ˮƽ���� (m)
 * @param target_h Ŀ�괹ֱ�߶� (m)
 * @param angle ����õ�����̨pitch�Ƕ� (rad)����Ϊ�������
 * @return unsigned char 1��ʾ����ɹ���0��ʾ����ʧ��
 */
unsigned char solver(float vel, float coeff, float target_x, float target_h, float *angle) {
    float aimed_h, h;   // aimed_h: �����е�Ŀ��߶ȣ�h: ��ǰ����õ��ĸ߶�
    float dh = 0;       // �߶����
    float tmp_angle = 0;  // ��ʱ�Ƕ�ֵ
    float t = 0;        // ����ʱ��

    aimed_h = target_h;  // ��ʼ��Ŀ��߶�Ϊʵ��Ŀ��߶�

    // ����������ŷ���Ƕ�
    for (int i = 0; i < MAX_ITER; i++) {
        // ���㵱ǰ�����ĽǶ�
        tmp_angle = atan2(aimed_h, target_x);

        // ���Ƕ��Ƿ�����Ч��Χ�� (-80�㵽80��)
        if (tmp_angle > 80.0f * PI / 180.0f || tmp_angle < -80.0f * PI / 180.0f) {
            // ����Ƕȳ�����Χ������ʧ��
            return 0;
        }

        // ʹ��ǰ���˶�ģ�ͼ����ڵ�ǰ�Ƕ����ӵ���ʵ�����߶Ⱥͷ���ʱ��
        forward_motion(vel, coeff, tmp_angle, target_x, &h, &t);

        // ������ʱ���Ƿ����������10�룩
        if (t > 10) {
            // ����ʱ������������󣬷���ʧ��
            return 0;
        }

        // ����߶����
        dh = target_h - h;

        // ����Ŀ��߶ȣ�������һ�ε���
        aimed_h = aimed_h + dh;

        // ������Ϣ����ע�ͣ�
        // printf("��%d�ε���:������:%f,��ʱĿ���yֵ:%f,�߶����:%f\n", i, (tmp_angle / PI) * 180, aimed_h, dh);

        // ����߶����С����ֵ����������
        if (fabs(dh) < 0.001f) {
            break;
        }
    }

    // ������ո߶������Ȼ�ϴ󣬷���ʧ��
    if (fabs(dh) > 0.01f) {
        return 0;
    }

    // �����ռ���ĽǶȸ�ֵ���������
    *angle = tmp_angle;
    return 1;
}

/**
 * @brief ���õ���ǰ���˶����㺯����ʹ��������ģ��
 * ���ݸ����ĳ�ʼ�ٶȡ��ǶȺ�ˮƽ���룬�����ӵ������߶Ⱥͷ���ʱ��
 *
 * @param vel �ӵ������ٶ� (m/s)
 * @param coeff ��������ϵ�� (����ʹ��)
 * @param given_angle ����Ƕ� (rad)
 * @param given_x ָ����ˮƽ������� (m)
 * @param h �������������õ������߶� (m)
 * @param t �������������õ��ķ���ʱ�� (s)
 */
void forward_motion(float vel, float coeff, float given_angle, float given_x, float* h, float* t) {
    // Ӧ���ٶ�ƫ����н׶ζ�Ӧ��
    float adjusted_vel = vel;
    float vel_bias = get_velocity_bias(given_x); // ��ȡ�ٶ�ƫ��ֵ
    adjusted_vel = vel + vel_bias; // Ӧ���ٶ�ƫ��

    // ͳһʹ��������ģ�ͽ��н��㣨���Կ���������
    *t = given_x / (adjusted_vel * arm_cos_f32(given_angle));
    *h = adjusted_vel * arm_sin_f32(given_angle) * (*t) - (GRAVITY * (*t) * (*t)) / 2.0f;
}
