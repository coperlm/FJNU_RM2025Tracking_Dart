#ifndef DMP_SELF_H
#define DMP_SELF_H

// 全局变量声明
extern float pitch, roll, yaw;  // 欧拉角
extern short aacx, aacy, aacz;  // 加速度传感器原始数据
extern short gyrox, gyroy, gyroz;  // 陀螺仪原始数据

// 函数声明
void calculate_angles(void);

#endif // DMP_SELF_H
