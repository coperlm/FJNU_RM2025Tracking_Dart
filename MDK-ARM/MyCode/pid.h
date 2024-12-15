#ifndef PID_H
#define PID_H

typedef struct {
    float kp;           // ����ϵ��
    float ki;           // ����ϵ��
    float kd;           // ΢��ϵ��
    float error;        // ��ǰ���
    float lastError;    // ��һ�����
    float integral;     // ������
    float maxIntegral;  // �������ֵ
    float maxOutput;    // ������ֵ
    float output;       // ���ֵ
} PID;

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(PID *pid, float reference, float feedback);

#endif // PID_H
