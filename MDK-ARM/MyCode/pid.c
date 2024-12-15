#include "pid.h"

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut) {
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
    pid->integral = 0.0f;
    pid->error = 0.0f;
    pid->lastError = 0.0f;
    pid->output = 0.0f;
}

void PID_Calc(PID *pid, float reference, float feedback) {
    pid->lastError = pid->error;
    pid->error = reference - feedback;

    // ���������
    float pout = pid->kp * pid->error;

    // ����������ֹ���ֱ���
    pid->integral += pid->error;
    if(pid->integral > pid->maxIntegral) 
        pid->integral = pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) 
        pid->integral = -pid->maxIntegral;
    float iout = pid->ki * pid->integral;

    // ����΢����
    float dout = pid->kd * (pid->error - pid->lastError);

    // �������������ֹ�������
    pid->output = pout + iout + dout;
    if(pid->output > pid->maxOutput) 
        pid->output = pid->maxOutput;
    else if(pid->output < -pid->maxOutput) 
        pid->output = -pid->maxOutput;
}
