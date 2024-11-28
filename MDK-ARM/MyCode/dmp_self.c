#include "dmp_self.h"
#include <math.h>

#define ACCEL_RANGE 16.0f
#define GYRO_RANGE  2000.0f
#define ALPHA 0.98f
#define FIXED_DELTA_TIME 0.040f  //采样周期为 40ms
#define NUM_SAMPLES 100


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 全局变量定义
float pitch = 1.0f, roll = 0.0f, yaw = 0.0f;
short aacx = 0, aacy = 0, aacz = 0;
short gyrox = 0, gyroy = 0, gyroz = 0;


void calculate_angles(void) {
    // 归一化加速度数据
    float ax = aacx / 32768.0f * ACCEL_RANGE;
    float ay = aacy / 32768.0f * ACCEL_RANGE;
    float az = aacz / 32768.0f * ACCEL_RANGE;

    // 计算 Pitch 和 Roll
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
    roll = atan2(ay, az) * 180.0f / M_PI;

    // 归一化陀螺仪数据
    float gz = gyroz / 32768.0f * GYRO_RANGE;

    // 磁力计数据（如有，计算静态 Yaw）
    float yaw_mag = 0.0f; // 替换为磁力计解算公式（需集成磁力计）

    // 陀螺仪漂移校正
    static float yaw_gyro = 0.0f;
    yaw_gyro += gz * FIXED_DELTA_TIME;

    // 修正范围
    yaw_gyro = fmod(yaw_gyro, 360.0f);
    if (yaw_gyro < 0) yaw_gyro += 360.0f;

    // 互补滤波
    yaw = ALPHA * yaw_gyro + (1.0f - ALPHA) * yaw_mag;

    // 滤波平滑（简单滑动平均示例）
    static float yaw_history[5] = {0};
    static int index = 0;
    yaw_history[index] = yaw;
    index = (index + 1) % 5;
    float smoothed_yaw = 0.0f;
    for (int i = 0; i < 5; i++) {
        smoothed_yaw += yaw_history[i];
    }
    yaw = smoothed_yaw / 5.0f;

    // 限制范围 [-180, 180]
    if (yaw > 180.0f) yaw -= 360.0f;
    
}
