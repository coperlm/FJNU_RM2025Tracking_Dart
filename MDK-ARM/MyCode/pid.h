#ifndef PID_H
#define PID_H

// PID控制器结构体定义
typedef struct {
  float kp, ki, kd;
  float error, lastError;
  float integral, maxIntegral;
  float output, maxOutput;
} PID;

// PID控制器初始化函数原型
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);

// PID控制器计算函数原型
void PID_Calc(PID *pid, float reference, float feedback);

#endif
