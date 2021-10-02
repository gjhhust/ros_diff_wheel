#include "main.h" 

/*************************************************
��������˵��
1.����� TIM1_pwm:PA10 �߼�����PA10 PC10  ������:TIM3 :PA7  PA6
2.����� TIM1_pwm:PA9 �߼�����PC7 PC9	 ������:TIM5 :PA0  PA 1
3.MPU6050(Gyro) IIC: 	SCL_PB10
											SDA_PB11
  
4.����ͨ�� UART2 	TX  PA2
									RX  PA3


******************************************************/

void BSP_init(void);

int main(void)
{


 	 BSP_init();  
   delay_ms(500);
	
	
	 startTast();
   vTaskStartScheduler();
   while (1){}
		 
}


void BSP_init(void)
{
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	TIM2_Init();
	TIM4_Init();
	
	
		/**************************���**************************/
	Encoder_Init_TIM3();				//TIM3������ģʽ��ʼ����A6��A7�ֱ���ΪA���B�����������
	TIM1_PWM_Init(899,0);//����Ƶ��PWMƵ��=72000/(899+1)=80Khz 
	Encoder_Init_TIM5();				//TIM5������ģʽ��ʼ����A1��A2�ֱ���ΪA���B�����������
	TIM6_Init();//��Ϊ����ת�ٶ�ʱ��ʹ��
	
	mpu_dmp_init();
	
	UART2_DMA_Init();
	
	
	//ESP8266_Init ();                                                        //��ʼ��WiFiģ��ʹ�õĽӿں�����
	//ESP8266_StaTcpClient_Unvarnish_ConfigTest();       //ģʽ����
}

