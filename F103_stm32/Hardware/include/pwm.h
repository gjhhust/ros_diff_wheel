#ifndef __PWM_H
#define __PWM_H
#include "main.h"

#define IN1  GPIO_Pin_10  //PA10 �ҵ��  PC6�ҵ��
#define IN2  GPIO_Pin_12  //PA12

#define IN3  GPIO_Pin_11  //PA11
#define IN4  GPIO_Pin_15  //PA15

#define R1	PAout(10)
#define R2	PAout(12)

#define L1	PAout(11)
#define L2	PAout(15)

void TIM1_PWM_Init(u16 arr,u16 psc);

void motor_R_move(void);//�ҵ������ת
void motor_R_back(void);

void motor_L_move(void);//��������ת
void motor_L_back(void);

void motor_R_stop(void);//�ƶ�
void motor_L_stop(void);
#endif
