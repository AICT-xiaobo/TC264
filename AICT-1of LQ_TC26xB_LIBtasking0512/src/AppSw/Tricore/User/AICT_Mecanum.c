/*
 * AICT_Mecanum.c
 *
 *  Created on: 2020年5月13日
 *      Author: 小波666
 */
#include <AICT_Mecanum.h>
#include <LQ_Atom_Motor.h>
#include <LQ_GTM.h>
#include <LQ_STM.h>
#include "AICT_SimpleFunction.h"
#include "AICT_SineTable.h"






PID pid;


/*************************************************************************
*  函数名称：void PID_realize(sint16 Speed,float ang ,sint16 w)
*  功能说明：对四个电机分别闭环
*  参数说明：Speed:正前方速度 ang:逆时针方向偏移角度 W:逆时针方向角速度
*  函数返回：无
*  修改时间：2020年5月13日
*  备    注：驱动单个电机单个方向
*************************************************************************/
sint16 Wheel_speed[4]; //cm/ms
void PID_realize(sint16 Speed,float ang ,sint16 w,sint16 encSpeed[4])
{
if(myfabs(ang)>3.5)
return;
  int incrementSpeed[4];
  float Cos_ang=0,Sin_ang=0;
 Cos_ang=lookup_cos(ang);
 Sin_ang=lookup_sin(ang);
    pid.SetSpeed[0]=(sint16)((Speed*(sint16)(Cos_ang*100)-Speed*(sint16)(Sin_ang*100))/100)+(sint16)(w*(CAR_L+CAR_W));//vy-vx+w(a+b)    R1
    pid.SetSpeed[1]=(sint16)((Speed*(sint16)(Cos_ang*100)+Speed*(sint16)(Sin_ang*100))/100)-(sint16)(w*(CAR_L+CAR_W));//vy+vx-w(a+b)    L1
    pid.SetSpeed[2]=(sint16)((Speed*(sint16)(Cos_ang*100)-Speed*(sint16)(Sin_ang*100))/100)-(sint16)(w*(CAR_L+CAR_W));//vy-vx-w(a+b)    L2
    pid.SetSpeed[3]=(sint16)((Speed*(sint16)(Cos_ang*100)+Speed*(sint16)(Sin_ang*100))/100)+(sint16)(w*(CAR_L+CAR_W));//vy+vx+w(a+b)    R2


pid.ActualSpeed[0]=encSpeed[0];//r1
pid.ActualSpeed[1]=encSpeed[1];//l1
pid.ActualSpeed[2]=encSpeed[2];//l2
pid.ActualSpeed[3]=encSpeed[3];//r2

    pid.err[0]=(pid.SetSpeed[0]-pid.ActualSpeed[0]);
    pid.err[1]=(pid.SetSpeed[1]-pid.ActualSpeed[1]);
    pid.err[2]=(pid.SetSpeed[2]-pid.ActualSpeed[2]);
    pid.err[3]=(pid.SetSpeed[3]-pid.ActualSpeed[3]);
//-----------棒棒算法-----
if(pid.err[0]>BANGBANG)
{
  pid.OutSpeed[0]=800;
  pid.OutSpeed[4]=0;
}
if(pid.err[0]<-BANGBANG)
{
    pid.OutSpeed[0]=0;
    pid.OutSpeed[4]=800;
}
if(pid.err[1]>BANGBANG)
{
  pid.OutSpeed[1]=800;
  pid.OutSpeed[5]=0;
}
if(pid.err[1]<-BANGBANG)
{
  pid.OutSpeed[1]=0;
  pid.OutSpeed[5]=800;
}

if(pid.err[2]>BANGBANG)
{
  pid.OutSpeed[2]=800;
  pid.OutSpeed[6]=0;
}
if(pid.err[2]<-BANGBANG)
{
  pid.OutSpeed[2]=0;
  pid.OutSpeed[6]=800;
}
  if(pid.err[3]>BANGBANG)
{
  pid.OutSpeed[3]=800;
  pid.OutSpeed[7]=0;
}
if(pid.err[3]<-BANGBANG)
{
  pid.OutSpeed[3]=0;
  pid.OutSpeed[7]=800;
}


  //  incrementSpeed[0]=pid.Kp*(pid.err[0]-pid.err_next[0])+pid.Ki*pid.err[0]+pid.Kd*(pid.err-2*pid.err_next+pid.err_last);
if(pid.err[0]<=BANGBANG||pid.err[0]>=-BANGBANG)
    incrementSpeed[0]=(int)(pid.Kp*(pid.err[0]-pid.err_next[0]))+(int)(pid.Ki[0]*pid.err[0]);
if(pid.err[1]<=BANGBANG||pid.err[1]>=-BANGBANG)
    incrementSpeed[1]=(int)(pid.Kp*(pid.err[1]-pid.err_next[1]))+(int)(pid.Ki[1]*pid.err[1]);
if(pid.err[2]<=BANGBANG||pid.err[2]>=-BANGBANG)
    incrementSpeed[2]=(int)(pid.Kp*(pid.err[2]-pid.err_next[2]))+(int)(pid.Ki[2]*pid.err[2]);
if(pid.err[3]<=BANGBANG||pid.err[3]>=-BANGBANG)
    incrementSpeed[3]=(int)(pid.Kp*(pid.err[3]-pid.err_next[3]))+(int)(pid.Ki[3]*pid.err[3]);
//-------------积分抗饱和-------------------
  if((pid.OutSpeed[0]<-MAXSPEED)&&(incrementSpeed[0]<0))incrementSpeed[0]=0;
  if((pid.OutSpeed[1]<-MAXSPEED)&&(incrementSpeed[1]<0))incrementSpeed[1]=0;
  if((pid.OutSpeed[2]<-MAXSPEED)&&(incrementSpeed[2])<0)incrementSpeed[2]=0;
  if((pid.OutSpeed[3]<-MAXSPEED)&&(incrementSpeed[3]<0))incrementSpeed[3]=0;
  if((pid.OutSpeed[0]>MAXSPEED)&&(incrementSpeed[0]>0))incrementSpeed[0]=0;
  if((pid.OutSpeed[1]>MAXSPEED)&&(incrementSpeed[1]>0))incrementSpeed[1]=0;
  if((pid.OutSpeed[2]>MAXSPEED)&&(incrementSpeed[2]>0))incrementSpeed[2]=0;
  if((pid.OutSpeed[3]>MAXSPEED)&&(incrementSpeed[3]>0))incrementSpeed[3]=0;
    pid.Out_Speed[0]+=incrementSpeed[0];
    pid.Out_Speed[1]+=incrementSpeed[1];
    pid.Out_Speed[2]+=incrementSpeed[2];
    pid.Out_Speed[3]+=incrementSpeed[3];


    pid.err_last[0]=pid.err_next[0];
    pid.err_last[1]=pid.err_next[1];
    pid.err_last[2]=pid.err_next[2];
    pid.err_last[3]=pid.err_next[3];

    pid.err_next[0]=pid.err[0];
    pid.err_next[1]=pid.err[1];
    pid.err_next[2]=pid.err[2];
    pid.err_next[3]=pid.err[3];
  if(pid.Out_Speed[0]<-MAXSPEED)pid.Out_Speed[0]=-MAXSPEED;
  if(pid.Out_Speed[1]<-MAXSPEED)pid.Out_Speed[1]=-MAXSPEED;
  if(pid.Out_Speed[2]<-MAXSPEED)pid.Out_Speed[2]=-MAXSPEED;
  if(pid.Out_Speed[3]<-MAXSPEED)pid.Out_Speed[3]=-MAXSPEED;
  if(pid.Out_Speed[0]>MAXSPEED)pid.Out_Speed[0]=MAXSPEED;
  if(pid.Out_Speed[1]>MAXSPEED)pid.Out_Speed[1]=MAXSPEED;
  if(pid.Out_Speed[2]>MAXSPEED)pid.Out_Speed[2]=MAXSPEED;
  if(pid.Out_Speed[3]>MAXSPEED)pid.Out_Speed[3]=MAXSPEED;
  if(pid.err[0]<=BANGBANG||pid.err[0]>=-BANGBANG)
  {
    if(pid.Out_Speed[0]>=0)
    {
      pid.OutSpeed[0]=pid.Out_Speed[0];//r1
      pid.OutSpeed[4]=0;
    }
    else
    {
      pid.OutSpeed[0]=0;
      pid.OutSpeed[4]=-pid.Out_Speed[0];
    }
  }
 if(pid.err[1]<=BANGBANG||pid.err[1]>=-BANGBANG)
 {
  if(pid.Out_Speed[1]>=0)
  {
    pid.OutSpeed[1]=pid.Out_Speed[1];//r1
    pid.OutSpeed[5]=0;
  }
  else
  {
    pid.OutSpeed[1]=0;
    pid.OutSpeed[5]=-pid.Out_Speed[1];
  }
 }
 if(pid.err[2]<=BANGBANG||pid.err[2]>=-BANGBANG)
 {
  if(pid.Out_Speed[2]>=0)
  {
    pid.OutSpeed[2]=pid.Out_Speed[2];//r1
    pid.OutSpeed[6]=0;
  }
  else
  {
    pid.OutSpeed[2]=0;
    pid.OutSpeed[6]=-pid.Out_Speed[2];
  }
 }
if(pid.err[3]<=BANGBANG||pid.err[3]>=-BANGBANG)
 {
  if(pid.Out_Speed[3]>=0)
  {
    pid.OutSpeed[3]=pid.Out_Speed[3];//r1
    pid.OutSpeed[7]=0;
  }
  else
  {
    pid.OutSpeed[3]=0;
    pid.OutSpeed[7]=-pid.Out_Speed[3];
  }
 }
}
/*************************************************************************
*  函数名称：float Angle_W(float image_angle)
*  功能说明：PD方向闭环
*  参数说明：image_angle:原信标图像偏差
*  函数返回：无
*  修改时间：2020年5月13日
*  备    注：
*************************************************************************/
float angleP=12,angleD=5.2;//5.2   5   200 //18    5  250//24  4   300
float Angle_error=0,Angle_next=0;
float Angle_W(float image_angle)
{
float Angle_error;

float AngleP_out;
float AngleD_out,W_out;

Angle_error=image_angle*57.3;
AngleP_out=angleP*Angle_error;
 AngleD_out=angleD*(Angle_error-Angle_next);
//AngleD_out=angleD*(gyro);
W_out = AngleP_out+AngleD_out;
W_out=limit(W_out,500);
// Angle_next=Angle_error;
 return -W_out;
}
/*************************************************************************
*  函数名称：void Motor_Duty(uint16 Motno, uint32 duty
*  功能说明：输出电机占空比
*  参数说明：Motno:马达序号 P:正转 N: 翻转
*  函数返回：无
*  修改时间：2020年5月13日
*  备    注：驱动单个电机单个方向
*************************************************************************/


void Motor_Duty(uint16 Motno, uint32 duty)
{

  switch(Motno)
  {
  case Mot0_P:
    ATOM_PWM_SetDuty(ATOMPWM0, duty, 12500);
    break;
  case Mot0_N:
    ATOM_PWM_SetDuty(ATOMPWM1, duty, 12500);
    break;
  case Mot1_P:
	ATOM_PWM_SetDuty(ATOMPWM2, duty, 12500);
    break;
  case Mot1_N:
	ATOM_PWM_SetDuty(ATOMPWM3, duty, 12500);
    break;
  case Mot2_P:
	ATOM_PWM_SetDuty(ATOMPWM4, duty, 12500);
    break;
  case Mot2_N:
	ATOM_PWM_SetDuty(ATOMPWM5, duty, 12500);
    break;
  case Mot3_P:
	ATOM_PWM_SetDuty(ATOMPWM6, duty, 12500);
    break;
  case Mot3_N:
	ATOM_PWM_SetDuty(ATOMPWM7, duty, 12500);
    break;
  default:
    break;
  }

}

/*************************************************************************
*  函数名称：void Motor_out()
*  功能说明：输出PID占空比,驱动四个电机
*  参数说明：无
*  函数返回：无
*  修改时间：2020年5月13日
*  备    注：驱动四个电机，硬件要规定的电机顺序布线，逆时针方向，车头为左右一，车尾为左右二。
*  0代表右一电机，1对应左一电机，2代表左二电机，3代表右二电机
*  电机正反转修改pid.OutSpeed[数字]即可。
*  按照算法，(4,0),(5,1),(6,2),(7,3)分别对应一个电机
*************************************************************************/
void Motor_out()
 {
   Motor_Duty(Mot0_P, (int)pid.OutSpeed[4]);
   Motor_Duty(Mot0_N, (int)pid.OutSpeed[0]);
   Motor_Duty(Mot1_P, (int)pid.OutSpeed[5]);
   Motor_Duty(Mot1_N, (int)pid.OutSpeed[1]);
   Motor_Duty(Mot2_P, (int)pid.OutSpeed[6]);
   Motor_Duty(Mot2_N, (int)pid.OutSpeed[2]);
   Motor_Duty(Mot3_P, (int)pid.OutSpeed[7]);
   Motor_Duty(Mot3_N, (int)pid.OutSpeed[3]);
 }

