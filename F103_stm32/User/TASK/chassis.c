#include "chassis.h"

Pid_Typedef Chassis_speed_L;
Pid_Typedef Chassis_speed_R;
Pid_Typedef Chassis_position;
pid speed_L;
Chassis F103RC_chassis={100,200,20,1,0};//����ʵʱ����


extern Ctrl_information chassis_ctrl;//��λ������ָ��
extern float Input[2];
extern float Output[2];

int led0pwmval=20;  
int led0pwmval2=0;  
int i=0;
int flag=1;
double positionNow=0;

/**********************************************************************************************************
*�� �� ��: Chassis_task
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Chassis_high_water;
void CHASSIC_task(void *pvParameters){
	portTickType xLastWakeTime;
	const portTickType xFrequency = 1;	

	PID_Param_Init();


	
	while(1)
	{
 
		Chassis_CurrentPid_Cal();

		//ESP8266_CheckRecvDataTest();
		
		//vTaskDelay(1); /* ��ʱ 500 �� tick */
	
		xLastWakeTime = xTaskGetTickCount();
				
		vTaskDelayUntil(&xLastWakeTime,xFrequency); 
	
//	#if INCLUDE_uxTaskGetStackHighWaterMark
//					Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
//	#endif
	}
}


/**********************************************************************************************************
*�� �� ��: Chassis_CurrentPid_Cal
*����˵��: ���̲���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_CurrentPid_Cal(void)
{
	//�ٶȸ�ֵ 
	Chassis_speed_L.SetPoint = LIMIT_MAX_MIN(chassis_ctrl.leftSpeedSet,35,-35);
	Chassis_speed_R.SetPoint = LIMIT_MAX_MIN(chassis_ctrl.rightSpeedSet,35,-35);
	//TIM_SetCompare1(TIM1,led0pwmval);
	//TIM_SetCompare2(TIM1,led0pwmval);

	//ֱ�߱궨
	//goto_1m();
	
	//��λ������λ
	control();
	
	//��ú����
	GYRO();

	//ѡ��pid
	pid_motor_chose(&Chassis_speed_L,Chassis_speed_L.SetPoint);
	pid_motor_chose(&Chassis_speed_R,Chassis_speed_R.SetPoint);
	
	//�ٶȹ������λ
	//if( F103RC_chassis.speed_error >500) SpeedReset(), F103RC_chassis.speed_error=0;		
}

/**********************************************************************************************************
*�� �� ��: goto_1m
*����˵��: ֱ����һ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static int set_des_flag=0;
void goto_1m(void){
	if(set_des_flag == 0){
		Chassis_position.SetPoint = 100;//1m
		set_des_flag = 1;
	}	
	Chassis_speed_L.SetPoint = LIMIT_MAX_MIN(position_PID_Calc(&Chassis_position, positionNow),35,-35);
	Chassis_speed_R.SetPoint = Chassis_speed_L.SetPoint;
	
}
/**********************************************************************************************************
*�� �� ��: motor_direction
*����˵��: �������ѡ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
int motor_direction(int ID,int Value)
{
	
	switch(ID){
		case left:
			if(Value>0) motor_L_move();
		  else  motor_L_back();
			break;
		case right:
			if(Value>0) motor_R_move();
		  else  motor_R_back();
			break;
	}
	if(Chassis_speed_R.SetPoint==0)motor_R_stop();
	if(Chassis_speed_L.SetPoint==0)motor_L_stop();
		
	return ABS(Value);
}
/**********************************************************************************************************
*�� �� ��: PID_Param_Init
*����˵��: ���PID��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void PID_Param_Init(void)
{
			//�ٶȻ�����
		Chassis_speed_L.P = 28;
		Chassis_speed_L.I = 2.5;
		Chassis_speed_L.D = 0;
		Chassis_speed_L.ErrorMax = 1000.0f;
		Chassis_speed_L.IMax = 200;
		Chassis_speed_L.SetPoint = 0.0f;	
		Chassis_speed_L.SumError = 0.0f;	
		Chassis_speed_L.OutMax = 800;	//���ռ�� �ٶ�Ϊ370-590
	
		//�ٶȻ��ҵ��
		Chassis_speed_R.P = 28;
		Chassis_speed_R.I = 2.5;
		Chassis_speed_R.D = 0;
		Chassis_speed_R.ErrorMax = 1000.0f;
		Chassis_speed_R.IMax = 200;
		Chassis_speed_R.SetPoint = 0.0f;
		Chassis_speed_R.SumError = 0.0f;		
		Chassis_speed_R.OutMax = 800;	//���ռ�� �ٶ�Ϊ370-590
	
		//λ�û�
		Chassis_position.P = 2.0f;
		Chassis_position.I = 0;
		Chassis_position.D = 2;
		Chassis_position.ErrorMax = 1000.0f;
		Chassis_position.IMax = 200.0f;
		Chassis_position.SetPoint = 0.0f;	
		Chassis_position.OutMax = 40;	
		
		/* ��ʼ������ */
	speed_L.target_val=100.0; 
	speed_L.actual_val=0.0; 
	speed_L.err=0.0;
	speed_L.err_last=0.0; 
	speed_L.integral=0.0;
	
	speed_L.Kp = 13; speed_L.Ki = 3.5; speed_L.Kd = 0.04;
}

/**********************************************************************************************************
*�� �� ��: pid_motor_chose
*����˵��: ���PIDѡ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/

void pid_motor_chose(Pid_Typedef *P, int speed)
{
	speed = ABS(speed);
	//���pid
	if(speed<50)
	P->I = 0.0005* speed * speed - 0.0199*speed + 3.0;

	
}

/**********************************************************************************************************
*�� �� ��: control
*����˵��: ִ�п���λ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
char temp=0;
void control(void)
{
	if(chassis_ctrl.crtlFlag>>7){
		SoftReset();
	}
	
	temp = chassis_ctrl.crtlFlag << 1;
	if(temp>>7){
		chassis_ctrl.left_move = 1;
	}else{
		chassis_ctrl.left_move = 0;
	}
	
	temp = chassis_ctrl.crtlFlag << 2;
	if(temp>>7){
		chassis_ctrl.right_move = 1;
	}else{
		chassis_ctrl.right_move = 0;
	}
}

/**********************************************************************************************************
*�� �� ��: SoftReset
*����˵��: �����λ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void SoftReset(void)
{
    __set_FAULTMASK(1); // �ر������ж�
    NVIC_SystemReset(); // ��λ
}

/**********************************************************************************************************
*�� �� ��: SpeedReset
*����˵��: �����λ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void SpeedReset(void)
{
  Chassis_speed_L.SetPoint = 0;
	Chassis_speed_L.I = 0;
	Chassis_speed_L.SetPointLast = 0;
	Chassis_speed_L.SumError = 0;
	Chassis_speed_L.LastError = 0;
	Chassis_speed_L.PreError = 0;
	motor_R_stop();
	
	Chassis_speed_R.SetPoint = 0;
	Chassis_speed_R.I = 0;
	Chassis_speed_R.SetPointLast = 0;
	Chassis_speed_R.SumError = 0;
	Chassis_speed_R.LastError = 0;
	Chassis_speed_R.PreError = 0;
	motor_R_stop();
}



/**********************************************************************************************************
*�� �� ��: TIM6_IRQHandler
*����˵��: ������������ 
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
//5ms��һ���жϲ���
static short pid_flag=0; 
static short data_send_flag=0;
static short ESP8266_send_flag=0;
void TIM6_IRQHandler(void)   //TIM3�ж�
{
		if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
		{
			
			pid_flag++;
			data_send_flag++;
			
			
			TIM_ClearITPendingBit(TIM6, TIM_IT_Update);   //���TIMx���жϴ�����λ:TIM �ж�Դ
					 
			Get_Motor_Speed(&F103RC_chassis.leftSpeedNow,&F103RC_chassis.rightSpeedNow);
			
	
			F103RC_chassis.rightSpeedNow = IIR_TICK_d_R(F103RC_chassis.rightSpeedNow);
			F103RC_chassis.leftSpeedNow = IIR_TICK_d_L(F103RC_chassis.leftSpeedNow);
			
			
			//�������
			if(pid_flag == 2){
					//ѡ��������
//					if(Chassis_speed_L.SetPoint>0) motor_L_move();
//					else if(Chassis_speed_L.SetPoint<0) motor_L_back();
//					else   motor_L_stop();
//				
//					if(Chassis_speed_R.SetPoint>0) motor_R_move();
//					else  if(Chassis_speed_R.SetPoint<0) motor_R_back();
//					else  motor_R_stop();
				TIM_SetCompare2(TIM1,motor_direction(left,speed_PID_Calc(&Chassis_speed_L, F103RC_chassis.leftSpeedNow)));	//led0pwmval);//
				TIM_SetCompare1(TIM1,motor_direction(right,speed_PID_Calc(&Chassis_speed_R, F103RC_chassis.rightSpeedNow)));	
//				TIM_SetCompare2(TIM1,speed_PID_Calc(&Chassis_speed_L, F103RC_chassis.leftSpeedNow));	//led0pwmval);//
//				TIM_SetCompare1(TIM1,speed_PID_Calc(&Chassis_speed_R, F103RC_chassis.rightSpeedNow));
				pid_flag = 0;
			}
			
			
			//����Ҫ���͵�ROS�����ݣ��Ӹú���������ǰ�������ݷ�Χ��-32768 - +32767�������ĸ����ݵķ�Χ(0 - 255)
			if(data_send_flag == 4){
				usartSendData(F103RC_chassis.leftSpeedNow,F103RC_chassis.rightSpeedNow,F103RC_chassis.angle,F103RC_chassis.controlFlag);
				data_send_flag = 0;
			}
			
			
			//�궨ʹ��
			positionNow +=  (F103RC_chassis.rightSpeedNow*0.005 + F103RC_chassis.leftSpeedNow*0.005 ) / 2;
			
				//�ٶ�ʧ��
			//if(F103RC_chassis.rightSpeedNow>42 || F103RC_chassis.rightSpeedNow > 42) F103RC_chassis.speed_error++;
			
			
//			ESP8266_send_flag++;
//			if(ESP8266_send_flag == 400)
//			{
//				ESP8266_SendInf();
//				ESP8266_send_flag=0;
//			}

		}
}
