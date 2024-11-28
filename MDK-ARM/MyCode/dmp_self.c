#include "dmp_self.h"
#include <math.h>

#define ACCEL_RANGE 16.0f
#define GYRO_RANGE  2000.0f
#define ALPHA 0.98f
#define FIXED_DELTA_TIME 0.040f  //��������Ϊ 40ms
#define NUM_SAMPLES 100


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ȫ�ֱ�������
float pitch = 1.0f, roll = 0.0f, yaw = 0.0f;
short aacx = 0, aacy = 0, aacz = 0;
short gyrox = 0, gyroy = 0, gyroz = 0;


void calculate_angles(void) {
    // ��һ�����ٶ�����
    float ax = aacx / 32768.0f * ACCEL_RANGE;
    float ay = aacy / 32768.0f * ACCEL_RANGE;
    float az = aacz / 32768.0f * ACCEL_RANGE;

    // ���� Pitch �� Roll
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
    roll = atan2(ay, az) * 180.0f / M_PI;

    // ��һ������������
    float gz = gyroz / 32768.0f * GYRO_RANGE;

    // ���������ݣ����У����㾲̬ Yaw��
    float yaw_mag = 0.0f; // �滻Ϊ�����ƽ��㹫ʽ���輯�ɴ����ƣ�

    // ������Ư��У��
    static float yaw_gyro = 0.0f;
    yaw_gyro += gz * FIXED_DELTA_TIME;

    // ������Χ
    yaw_gyro = fmod(yaw_gyro, 360.0f);
    if (yaw_gyro < 0) yaw_gyro += 360.0f;

    // �����˲�
    yaw = ALPHA * yaw_gyro + (1.0f - ALPHA) * yaw_mag;

    // �˲�ƽ�����򵥻���ƽ��ʾ����
    static float yaw_history[5] = {0};
    static int index = 0;
    yaw_history[index] = yaw;
    index = (index + 1) % 5;
    float smoothed_yaw = 0.0f;
    for (int i = 0; i < 5; i++) {
        smoothed_yaw += yaw_history[i];
    }
    yaw = smoothed_yaw / 5.0f;

    // ���Ʒ�Χ [-180, 180]
    if (yaw > 180.0f) yaw -= 360.0f;
    
}
