#include "start_task.h"

uint32_t CPU_high_water;

#define START_TASK_PRIO 1  //�������ȼ�
#define START_STK_SIZE 256 //�����ջ
static TaskHandle_t StartTask_Handler; //������

////��������
#define CHASSIS_TASK_PRIO 18  //�������ȼ�
#define CHASSIS_STK_SIZE 512 //�����ջ
static TaskHandle_t chassis_Handler; //������

////����������
//#define GYRO_TASK_PRIO 15  //�������ȼ�
//#define GYRO_STK_SIZE 512 //�����ջ
//static TaskHandle_t Gyro_Handler; //������

////���ݷ�������
//#define DATA_DISPOSE_TASK_PRIO 20  //�������ȼ�
//#define DATA_DISPOSE_STK_SIZE 256 //�����ջ
//static TaskHandle_t Data_Dispose_Handler; //������


#define CPU_TASK_PRIO 5  //�������ȼ�
#define CPU_STK_SIZE 256 //�����ջ
static TaskHandle_t CPUTask_Handler; //������

uint8_t CPU_RunInfo1[200]; //������������ʱ����Ϣ �ֱ��ǣ������� ����״̬ ���ȼ� ʣ��ջ �������
uint8_t CPU_RunInfo2[200]; //������������ʱ����Ϣ �ֱ��ǣ������� ���м���  ʹ����


void start_task(void *pvParameters)
{
  taskENTER_CRITICAL();
	
	xTaskCreate((TaskFunction_t)CPU_task,          //������
                (const char *)"CPU_task",          //��������
                (uint16_t)CPU_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)CPU_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&CPUTask_Handler); //������
	
	xTaskCreate((TaskFunction_t)CHASSIC_task,          //������
                (const char *)"CHASSIC_task",          //��������
                (uint16_t)CHASSIS_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)CHASSIS_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&chassis_Handler); //������
		
							
//	xTaskCreate((TaskFunction_t)DATA_DISPOSE_task,          //������
//					(const char *)"DATA_DISPOSE_task",          //��������
//					(uint16_t)DATA_DISPOSE_STK_SIZE,            //�����ջ��С
//					(void *)NULL,                        //���ݸ��������Ĳ���
//					(UBaseType_t)DATA_DISPOSE_TASK_PRIO,        //�������ȼ�
//					(TaskHandle_t *)&Data_Dispose_Handler); //������
//					
//	xTaskCreate((TaskFunction_t)GYRO_task,          //������
//					(const char *)"GYRO_task",          //��������
//					(uint16_t)GYRO_STK_SIZE,            //�����ջ��С
//					(void *)NULL,                        //���ݸ��������Ĳ���
//					(UBaseType_t)GYRO_TASK_PRIO,        //�������ȼ�
//					(TaskHandle_t *)&Gyro_Handler); //������
//							
								
								
	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
  taskEXIT_CRITICAL();            //�˳��ٽ���
}



void CPU_task(void *pvParameters)
{
	

	
   while (1) {
    memset(CPU_RunInfo1,0,200); //��Ϣ����������
 
    vTaskList((char *)&CPU_RunInfo1); //��ȡ��������ʱ����Ϣ
    
    memset(CPU_RunInfo2,0,200); //��Ϣ����������
 
    vTaskGetRunTimeStats((char *)&CPU_RunInfo2);
    vTaskDelay(10); /* ��ʱ 500 �� tick */
		 
//#if INCLUDE_uxTaskGetStackHighWaterMark
//        CPU_high_water = uxTaskGetStackHighWaterMark(NULL);
//#endif
    }
}
void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //������
                (const char *)"start_task",          //��������
                (uint16_t)START_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler); //������
}

