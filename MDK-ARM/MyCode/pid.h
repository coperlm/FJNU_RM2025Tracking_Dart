#ifndef PID_H
#define PID_H

// PID�������ṹ�嶨��
typedef struct {
  float kp, ki, kd;
  float error, lastError;
  float integral, maxIntegral;
  float output, maxOutput;
} PID;

// PID��������ʼ������ԭ��
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);

// PID���������㺯��ԭ��
void PID_Calc(PID *pid, float reference, float feedback);

#endif
