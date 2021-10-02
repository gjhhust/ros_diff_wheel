#ifndef __PID_H
#define __PID_H
typedef struct PID{
		float SetPoint;			//�趨Ŀ��ֵ
	  float SetPointLast;
		
		float P;						//��������
		float I;						//���ֳ���
		float D;						//΢�ֳ���
		
		float LastError;		//ǰ�����
		float PreError;			//��ǰ���
		float SumError;			//�������
		float dError;
		
		float ErrorMax;			//ƫ������ ����ƫ���򲻼����������
		float IMax;					//��������
		
		float POut;					//�������
		float IOut;					//�������
		float DOut;					//΢�����
	  float OutMax;       //�޷�
}Pid_Typedef;

struct _pid{
	float target_val; 
	float actual_val; 
	float err;
	float err_next; 
	float err_last; 
	float Kp, Ki, Kd;
	float integral; //�������ֵ
	int out_Max;//������ֵ
};

typedef struct _pid pid;
float PID_realize(pid * P,float actual_val);
float PID_Calc(Pid_Typedef *P, float ActualValue);
float speed_PID_Calc(Pid_Typedef *P, float ActualValue);
float position_PID_Calc(Pid_Typedef *P, float ActualValue);
#endif
