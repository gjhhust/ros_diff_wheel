#include "main.h"

float Input[2] = {0};
float Output[2] = {0};

extern Chassis F103RC_chassis;
const float Gains[6] = {
 0.01016679592,   0.1177062541,   0.3721269369,   0.3721269369,   0.1177062541,
    0.01016679592
};

float Buffer[2][ORDER+1];        //������ʷ����
/**
  * @brief  ���뵱ǰ�����ͣ�����FIR�˲�����
  * @param  Input: �˲�����ĵ�ǰֵ     
  * @retval �˲�������ֵ
  */
void Fir(float Input[],float Output[2])
{
		unsigned int Index;                //�±�����
		for(int i = 0;i < 2;i ++)
		{
			for(Index=ORDER;Index>0;Index--) Buffer[i][Index]=Buffer[i][Index-1];
			Buffer[i][0]=Input[i];
			//�������
			for(Index=0;Index<ORDER+1;Index++)
			{
				Output[i]+=Gains[Index]*Buffer[i][Index];
			}
			if (i==0) F103RC_chassis.leftSpeedNow = Output[i];
			if (i==1) F103RC_chassis.rightSpeedNow = Output[i];
			Output[i] = 0;
		}
        
}
