#include "GafProjectileSolver.h"

int watch_flag = 0;

/**
 * @brief ���������
 * ��������: unsigned char solver(float vel,float coeff,float target_x, float target_h, float *angle)
 * @param vel �����ٶ� (m/s)
 * @param coeff ��������ϵ�� 
 * @param target_x Ŀ��ˮƽ���� (m)
 * @param target_h Ŀ����ֱ�߶� (m)
 * @param angle ��̨pitch�Ƕ� (rad)
 * @return 1 or 0
 */
unsigned char solver(float vel,float coeff,float target_x, float target_h, float *angle) {
    float aimed_h, h;//Ŀ��߶�,��ǰ�߶�
    float dh = 0;//�߶Ȳ�
    float tmp_angle = 0;//��ʱ�Ƕ�
    float t = 0;//ʱ��
    aimed_h = target_h;//��ʼ��Ŀ��߶�

    //�������
    for (int i = 0; i < MAX_ITER; i++) {
        //������ʱ�Ƕ�
        tmp_angle = atan2(aimed_h,target_x);

        //���Ƕȷ�Χ�Ƿ���(-80d,80d)֮��
        if (tmp_angle > 80 * PI / 180 || tmp_angle < -80 * PI / 180) {
            //�����Ƕȳ�������80��
            return 0;
        }

        //ʹ����ǰ���㺯������߶Ⱥ�ʱ��
        forward_motion(vel, coeff, tmp_angle, target_x, &h, &t);

        //����˶�ʱ���Ƿ񳬹�10s
        if (t > 10) {
            //�˶�ʱ��̫����,�����ڴ�,�������
            return 0;
        }

        //����߶Ȳ�
        dh = target_h - h;

        //����Ŀ��߶�
        aimed_h = aimed_h + dh;

        //��ӡ������Ϣ
        //printf("��%d�ε���:����:%f,��ʱĿ���yֵ:%f,�߶����:%f\n", i, (tmp_angle / PI) * 180, aimed_h, dh);

        //����߶Ȳ�С��0.001,���������
        if (fabs(dh) < 0.001) {
            break;
        }
    }

    //����߶Ȳ����0.01,�򷵻ش���
    if (fabs(dh) > 0.01) {
        return 0;
    }

    //�����ռ���ĽǶȸ�ֵ������angle
    *angle = tmp_angle;

    return 1;
}

/**
 * @brief ���õ���ǰ���˶����㺯��:��ǰ�˶�ģ��:���������Ϳ���Ħ��ģ��
 * ��������: void forward_motion(float vel, float coeff, float given_angle, float given_x, float* h, float* t)
 * ��ˮƽ����ϵ��
 * @param vel: input,�����ٶ� / m/s
 * @param coeff: input,��������ϵ��
 * @param given_x: input,������� / m
 * @param given_angle: input,����Ƕ� / rad
 * @param h: output,��������߶� / m
 * @param t: output,�������ʱ�� / s
 * @return void
 */
void forward_motion(float vel, float coeff, float given_angle, float given_x, float* h, float* t) {
    //������������׶�
    if (given_angle > 0.01) {
			watch_flag = 0;
        float t0, x0, y0;//�����׶���ߵ������͵���ʱ��
        t0 = vel * sin(given_angle) / GRAVITY;
        x0 = vel * cos(given_angle) * t0;
        y0 = (GRAVITY * t0 * t0) / 2;
        //���ֻ���������׶�,�˻�Ϊ������ģ��
        if (given_x < x0) {
            *t = given_x / (vel * cos(given_angle));
            *h = vel * sin(given_angle) * (*t) - (GRAVITY * (*t) * (*t)) / 2;
        }
        //������,���½�
        else
        {
            float t1, x1;
            x1 = given_x - x0;
            t1 = (exp(coeff * x1) - 1) / (coeff * vel * cos(given_angle));
            *t = t0 + t1;
            *h = y0 - (GRAVITY * t1 * t1) / 2;
        }
    }
    //ֻ���½��׶�
    else
    {
				watch_flag = 1;
        *t = (exp(coeff * given_x) - 1) / (coeff * vel * cos(given_angle));
        *h = vel * sin(given_angle) * (*t) - (GRAVITY * (*t) * (*t)) / 2;
    }
}