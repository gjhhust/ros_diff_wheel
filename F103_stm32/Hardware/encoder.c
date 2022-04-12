#include "encoder.h"

extern int leftSpeedNow;//���ҵ����ǰ�ٶ�
extern int rightSpeedNow;
extern Chassis F103RC_chassis;
extern Pid_Typedef Chassis_speed_L;
extern Pid_Typedef Chassis_speed_R;
/**************************************************************************
�������ܣ���TIM2��ʼ��Ϊ�������ӿ�ģʽ ����
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM3(void)
{
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
        TIM_ICInitTypeDef TIM_ICInitStructure;  
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    //ʹ�ܶ�ʱ��2��ʱ��
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //ʹ��PA�˿�ʱ��

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;        //�˿�����
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //��������
        GPIO_Init(GPIOA, &GPIO_InitStructure);                                        //�����趨������ʼ��GPIOA

        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
        TIM_TimeBaseStructure.TIM_Prescaler = 0x0;              //Ԥ��Ƶ�� 
        TIM_TimeBaseStructure.TIM_Period = 65535;  //�趨�������Զ���װֵ
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ѡ��ʱ�ӷ�Ƶ������Ƶ
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���  
        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
        
        TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
        
        TIM_ICStructInit(&TIM_ICInitStructure);
        TIM_ICInitStructure.TIM_ICFilter = 10;
        TIM_ICInit(TIM3, &TIM_ICInitStructure);
        TIM_ClearFlag(TIM3, TIM_FLAG_Update);                   //���TIM�ĸ��±�־λ
        TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
        //Reset counter
        TIM_SetCounter(TIM3,0);
        //===============================================
        TIM3->CNT = 0x7fff;
        //===============================================
        TIM_Cmd(TIM3, ENABLE); 
}
/**************************************************************************
�������ܣ���TIM5��ʼ��Ϊ�������ӿ�ģʽ ����
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM5(void)
{
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
        TIM_ICInitTypeDef TIM_ICInitStructure;  
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);    //ʹ�ܶ�ʱ��4��ʱ��
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //ʹ��PB�˿�ʱ��

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;        //�˿�����
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //��������
        GPIO_Init(GPIOA, &GPIO_InitStructure);                                        //�����趨������ʼ��GPIOA

        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
        TIM_TimeBaseStructure.TIM_Prescaler = 0x0;              // Ԥ��Ƶ�� 
        TIM_TimeBaseStructure.TIM_Period = 65535;  //�趨�������Զ���װֵ
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ѡ��ʱ�ӷ�Ƶ������Ƶ
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���  
        TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
        TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	
        TIM_ICStructInit(&TIM_ICInitStructure);
        TIM_ICInitStructure.TIM_ICFilter = 10;
        TIM_ICInit(TIM5, &TIM_ICInitStructure);
        TIM_ClearFlag(TIM5, TIM_FLAG_Update);                   //���TIM�ĸ��±�־λ
        TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
        //Reset counter
        TIM_SetCounter(TIM5,0);
        //===============================================
        TIM5->CNT = 0x7fff;
        //===============================================
        TIM_Cmd(TIM5, ENABLE); 
}

/**************************************************************************
�������ܣ���ȡ�����������ֵ
��ڲ�����TIM_TypeDef * TIMx
����  ֵ����
**************************************************************************/
s16 getTIMx_DetaCnt(TIM_TypeDef * TIMx)
{
        s16 cnt;
        cnt = TIMx->CNT-0x7fff;     //��һ��Ĭ����Ϊ����λʱ���ڱ������������0x7fff������仯����ʵ��Ҳ������
        TIMx->CNT = 0x7fff;
        return cnt;
}

/**************************************************************************
�������ܣ�������������
��ڲ�����int *leftSpeed,int *rightSpeed
����  ֵ����
                //�������ҳ������ٶȣ������ٶ�Ϊ��ֵ �������ٶ�Ϊ��ֵ
                //һ��ʱ���ڵı������仯ֵ*ת���ʣ�ת��Ϊֱ���ϵľ���m��*200s��5ms����һ�Σ� �õ� m/s *1000ת��Ϊint����
								
							 �� 3300 = 11 4 75
							�� 1980 = 11 * 4 * 45
                һȦ����������
                        ��  11
                        �ң�  11
                �����ܳ���13cm
                һ������ľ��룺
                        �� 0.13/17
                        �ң� 0.13/17
**************************************************************************/

void Get_Motor_Speed(int *leftSpeed,int *rightSpeed)
{
        static int leftWheelEncoderNow    = 0;
        static int rightWheelEncoderNow   = 0;
        static int leftWheelEncoderLast   = 0;
        static int rightWheelEncoderLast  = 0;        
        
        //��¼�������ұ���������
        leftWheelEncoderNow += getTIMx_DetaCnt(TIM5);
        rightWheelEncoderNow+= getTIMx_DetaCnt(TIM3);
                
        //5ms����            
				//�����ٻ�ʹ�ø���������
        *leftSpeed   =    13 * (leftWheelEncoderNow - leftWheelEncoderLast) * 200 /  1980;  //�ٶ�Ϊcm/s
        *rightSpeed  =    13 * (rightWheelEncoderNow - rightWheelEncoderLast)* 200 / 1980;

        //��¼�ϴα���������
        leftWheelEncoderLast  = leftWheelEncoderNow;                    
        rightWheelEncoderLast = rightWheelEncoderNow;                   
}


/**************************************************************************
�������ܣ�����TIM6���Զ�ʱ����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM6_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); 

    TIM_TimeBaseInitStructure.TIM_Period = 50-1; 	
    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200-1;  
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);
		TIM_ARRPreloadConfig(TIM6, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���

    

    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
		TIM_Cmd(TIM6, ENABLE); 		
}



