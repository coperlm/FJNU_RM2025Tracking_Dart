#ifndef PID_H
#define PID_H

typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float error;        // 当前误差
    float lastError;    // 上一次误差
    float integral;     // 积分项
    float maxIntegral;  // 积分最大值
    float maxOutput;    // 输出最大值
    float output;       // 输出值
} PID;

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(PID *pid, float reference, float feedback);

#endif // PID_H
