#include "Data_Dispose.h"

//ͨ��Э�鳣��
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};
/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ�������Ŀ���������չ������size������
--------------------------------------------------------------------------*/

/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ�������Ŀ���������չ������size������
--------------------------------------------------------------------------*/

/*--------------------------------����λ-----------------------------------

8λ�ɸߵ��׷ֱ���
�����λ ���ַ��� ���ַ��� 
--------------------------------------------------------------------------*/

/**************************************************************************
ͨ�ŵķ��ͺ����ͽ��պ��������һЩ���������������������
**************************************************************************/

extern unsigned char JudgeSend[SendBiggestSize];//��������ڴ�

unsigned char SaveBuffer[200];//����˫������

Ctrl_information chassis_ctrl ={0,0,0,1,1}; //������λ��������Ϣ
extern Chassis F103RC_chassis;//����ʵʱ����
         


//�������ݣ������١������١��Ƕȣ������壨-32767 - +32768��
union sendData
{
	short d;
	unsigned char data[2];
}leftVelNow,rightVelNow,angleNow;

//�������ٿ����ٶȹ�����
union receiveData
{
	short d;
	unsigned char data[2];
}leftVelSet,rightVelSet;



/**********************************************************************************************************
*�� �� ��: JudgeBuffReceive
*����˵��: ���պ���
*��    ��: ReceiveBuffer[]
*�� �� ֵ: ��
**********************************************************************************************************/
void JudgeBuffReceive(unsigned char ReceiveBuffer[])
{
	uint16_t cmd_id;//ʶ����Ϣ���ݣ���δ����
	short k=0;
	short PackPoint;
	memcpy(&SaveBuffer[reBiggestSize],&ReceiveBuffer[0],reBiggestSize);		//��ReceiveBuffer[0]��ַ������SaveBuffer[13], ���ο���13��, ����һ�ν��յĴ浽�����
	for(PackPoint=0;PackPoint<reBiggestSize;PackPoint++)		//�ȴ���ǰ�������(����һ�����ѽ������)
	{
		if(SaveBuffer[PackPoint]==header[0] && SaveBuffer[PackPoint + 1]== header[1]) //��ͷ���
		{	
			short dataLength  = SaveBuffer[PackPoint + 2]    ;
			unsigned char checkSum = getCrc8(&SaveBuffer[PackPoint], 3 + dataLength);
				// �����ϢУ��ֵ
			if (checkSum == SaveBuffer[PackPoint +3 + dataLength]) //SaveBuffer[PackPoint��ʼ��У��λ]
			{
				//˵�����ݺ˶Գɹ�����ʼ��ȡ����
				 for(k = 0; k < 2; k++)
					{
						leftVelSet.data[k]  = SaveBuffer[PackPoint + k + 3]; //SaveBuffer[3]  SaveBuffer[4]
						rightVelSet.data[k] = SaveBuffer[PackPoint + k + 5]; //SaveBuffer[5]  SaveBuffer[6]
					}				
					
					//�ٶȸ�ֵ����
					chassis_ctrl.leftSpeedSet  = (int)leftVelSet.d;
					chassis_ctrl.rightSpeedSet = (int)rightVelSet.d;
					
					//ctrlFlag
					chassis_ctrl.crtlFlag = SaveBuffer[PackPoint + 7];                //SaveBuffer[7]
				
			}
		}	
	memcpy(&SaveBuffer[0],&SaveBuffer[reBiggestSize],reBiggestSize);		//��SaveBuffer[13]��ַ������SaveBuffer[0], ���ο���13������֮ǰ�浽����������ᵽǰ�棬׼������
	}
}
/**************************************************************************
�������ܣ����������ٺͽǶ����ݡ������źŽ��д����ͨ�����ڷ��͸�Linux
��ڲ�����ʵʱ�������١�ʵʱ�������١�ʵʱ�Ƕȡ������źţ����û�нǶ�Ҳ���Բ�����
����  ֵ����
**************************************************************************/
void usartSendData(short leftVel, short rightVel,short angle,unsigned char ctrlFlag)
{
	int i, length = 0;

	// ���������������ٶ�
	leftVelNow.d  = leftVel;
	rightVelNow.d = rightVel;
	angleNow.d    = angle;
	
	// ������Ϣͷ
	for(i = 0; i < 2; i++)
		JudgeSend[i] = header[i];                      // buf[0] buf[1] 
	
	// ���û������������ٶȡ��Ƕ�
	length = 7;
	JudgeSend[2] = length;                             // buf[2]
	for(i = 0; i < 2; i++)
	{
		JudgeSend[i + 3] = leftVelNow.data[i];         // buf[3] buf[4]
		JudgeSend[i + 5] = rightVelNow.data[i];        // buf[5] buf[6]
		JudgeSend[i + 7] = angleNow.data[i];           // buf[7] buf[8]
	}
	// Ԥ������ָ��
	JudgeSend[3 + length - 1] = ctrlFlag;              // buf[9]
	
	// ����У��ֵ����Ϣβ
	JudgeSend[3 + length] = getCrc8(JudgeSend, 3 + length);  // buf[10]
	JudgeSend[3 + length + 1] = ender[0];              // buf[11]
	JudgeSend[3 + length + 2] = ender[1];              // buf[12]
	
	//�����ַ�������
		/*****�����ϴ�*****/

//	USART_ClearFlag(USART2,USART_FLAG_TC);
//	for(i=0;i<13;i++)
//	{
//	  USART_SendData(USART2,JudgeSend[i]);
//	  while (USART_GetFlagStatus(USART2,USART_FLAG_TC) == RESET); //�ȴ�֮ǰ���ַ��������
//	}
	DMA1_Channel7->CNDTR = SendBiggestSize; 
	DMA_Cmd(DMA1_Channel7,ENABLE);
}

/**************************************************************************
�������ܣ������λѭ������У�飬��usartSendData��usartReceiveOneData��������
��ڲ����������ַ�������С
����  ֵ����
**************************************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}
/**********************************END***************************************/







/**********************************************************************************************************
*�� �� ��: Gyro_task
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
//void DATA_DISPOSE_task(void *pvParameters){

//	
//	while(1)
//	{
// 
//		
//		
//		
//		//IMUupdate(gyrox, gyroy, gyroz, aacx, aacy,aacz);
//			
//		vTaskDelay(20); 

//	}

//}
