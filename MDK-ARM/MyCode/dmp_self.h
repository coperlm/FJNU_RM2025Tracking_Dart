#ifndef DMP_SELF_H
#define DMP_SELF_H

// ȫ�ֱ�������
extern float pitch, roll, yaw;  // ŷ����
extern short aacx, aacy, aacz;  // ���ٶȴ�����ԭʼ����
extern short gyrox, gyroy, gyroz;  // ������ԭʼ����

// ��������
void calculate_angles(void);

#endif // DMP_SELF_H
