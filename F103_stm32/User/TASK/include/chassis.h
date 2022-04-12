#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "main.h"

struct chassis
{
	int leftSpeedNow;//���ҵ����ǰ�ٶ�
	int rightSpeedNow;
	int angle;//��ǰ����Ƕ�,100������
	int controlFlag;//����ָ��
	int speed_error;//ʧ���ۼ�
};
typedef struct chassis Chassis;
enum motorID{
right,left
};
void CHASSIC_task(void *pvParameters);

//������λ���������ٶ�ָ������˶�
void Chassis_CurrentPid_Cal(void);

//pid��ʼ���Ͷ�̬����
void PID_Param_Init(void);
//���pid������̬ѡ��
void pid_motor_chose(Pid_Typedef *P, int speed);

//ѡ����ת��
int motor_direction(int ID,int Value);//����ռ�ձ�


//ֱ����1m�궨
void goto_1m(void);

//���Ʋ���
void control(void);
void SoftReset(void);
void SpeedReset(void);

#endif

