#ifndef IAT_PUBLISH_H
#define IAT_PUBLISH_H
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"
#include <termio.h>

#include <iostream>

#include <sys/ioctl.h>  

#include "ros/ros.h"

#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"//use data struct of std_msgs/String  
#include <math.h>

#include "key.h"
#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))
typedef struct PID{
		float SetPoint;			//设定目标值
	  float SetPointLast;
		
		float P;						//比例常数
		float I;						//积分常数
		float D;						//微分常数
		
		float LastError;		//前次误差
		float PreError;			//当前误差
		float SumError;			//积分误差
		float dError;
		
		float ErrorMax;			//偏差上限 超出偏差则不计算积分作用
		float IMax;					//积分限制
		
		float POut;					//比例输出
		float IOut;					//积分输出
		float DOut;					//微分输出
	  float OutMax;       //限幅
}Pid_Typedef;


enum Voice_AT{
go,back,left,right,stop    
};

typedef union{
    int setpoint;
    char data[5];
}Setpoint_Type;

typedef struct{
    double position_x;
    double position_y;
    double yaw;
    double position_now;
}carNOW_Type;

int get_shuzi(char shiwei,char gewei);
float position_PID_Calc(Pid_Typedef *P, float ActualValue);
double anglar_to_r(double anglar);
void odom_Callback(const nav_msgs::Odometry msg);


#endif