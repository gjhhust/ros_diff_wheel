#include "Gyro.h"
extern Chassis F103RC_chassis;



float Yaw,Pitch,Roll;  //ƫ���ǣ������ǣ�������
/**********************************************************************************************************
*�� �� ��: Gyro_task
*����˵��: ����������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
//uint32_t GYRO_high_water;
//void GYRO_task(void *pvParameters){

//	
//	
//	while(1)
//	{
// 
//		

//		mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
//		
//		if(Yaw==-180) Yaw = 180.0;
//		
//		if(Yaw== 180) Yaw = -180.0;
//			
//		
//		F103RC_chassis.angle = 100 * Yaw;
//		
//		vTaskDelay(1);

//	}

//}
void GYRO(void)
{
		mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
		

	
		F103RC_chassis.angle = 88 * Yaw;
		
}

