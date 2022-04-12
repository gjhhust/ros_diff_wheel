#include "main.h"
/**
  * @brief  PID�����㷨
  * @param  PID_Struct *P  PID�����ṹ��
  *         ActualValue    PID���㷴��������ǰ��ʵ���ֵ��
  * @retval PID�����������ֵ
  */
	
	//ֻ�������
float speed_PID_Calc(Pid_Typedef *P, float ActualValue)
{
		P->PreError = P->SetPoint - ActualValue;
		P->dError = P->PreError - P->LastError;
	
	  P->SetPointLast = P->SetPoint;
	
		if(P->PreError > -P->ErrorMax && P->PreError < P->ErrorMax)
		{
			P->SumError += P->PreError; 
		}
		
		P->LastError = P->PreError;
		
		if(P->SumError >= P->IMax)
			P->SumError = P->IMax;
		else if(P->SumError <= -P->IMax)
			P->SumError = -P->IMax;
		
		P->POut = P->P * P->PreError;
		P->IOut = P->I * P->SumError;
		P->DOut = P->D * P->dError;
		
		
		return LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax); 
}
	//ֻ�����������
float position_PID_Calc(Pid_Typedef *P, float ActualValue)
{
		P->PreError = P->SetPoint - ActualValue;
		P->dError = P->PreError - P->LastError;
	
	  P->SetPointLast = P->SetPoint;
	
		if(P->PreError > -P->ErrorMax && P->PreError < P->ErrorMax)
		{
			P->SumError += P->PreError; 
		}
		
		P->LastError = P->PreError;
		
		if(P->SumError >= P->IMax)
			P->SumError = P->IMax;
		else if(P->SumError <= -P->IMax)
			P->SumError = -P->IMax;
		
		P->POut = P->P * P->PreError;
		P->IOut = P->I * P->SumError;
		P->DOut = P->D * P->dError;
		
		
		return LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax);
}


float PID_realize(pid * P,float actual_val) {
/* ����Ŀ��ֵ��ʵ��ֵ����� */
	P->err=P->target_val-actual_val; 
	/* ����ۻ� */
	P->integral+=P->err; 
	/*PID �㷨ʵ�� */
	P->actual_val=P->Kp*P->err+P->Ki*P->integral+P->Kd*(P->err-P->err_last); 
	
	/* ���� */
	P->err_last=P->err; 
	
	/* ���ص�ǰʵ��ֵ */
	return LIMIT_MAX_MIN(P->actual_val,P->out_Max,-P->out_Max);
}

