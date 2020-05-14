/*
 * AICT_UserInit.c
 *
 *  Created on: 2020年5月13日
 *      Author: 小波666
 */
#include<AICT_UserInit.h>
#include <LQ_Atom_Motor.h>
#include <AICT_Mecanum.h>
#include <LQ_GTM.h>
void Motor_init()
{
	ATOM_PWM_InitConfig(ATOMPWM0, 500, 12500);
	ATOM_PWM_InitConfig(ATOMPWM1, 500, 12500);
	ATOM_PWM_InitConfig(ATOMPWM2, 500, 12500);
	ATOM_PWM_InitConfig(ATOMPWM3, 500, 12500);
	ATOM_PWM_InitConfig(ATOMPWM4, 500, 12500);
	ATOM_PWM_InitConfig(ATOMPWM5, 500, 12500);
	ATOM_PWM_InitConfig(ATOMPWM6, 500, 12500);
	ATOM_PWM_InitConfig(ATOMPWM7, 500, 12500);
}
/*************************************************************************
*  函数名称：void PID_init()
*  功能说明：电机PI参数初始化
*  参数说明：
*  函数返回：无
*  修改时间：2020年5月13日
*  备    注：驱动单个电机单个方向
*************************************************************************/
void PID_init()
{
  pid.SetSpeed[0]=0;
  pid.SetSpeed[1]=0;
  pid.SetSpeed[2]=0;
  pid.SetSpeed[3]=0;


  pid.ActualSpeed[0]=0;
  pid.ActualSpeed[1]=0;
  pid.ActualSpeed[2]=0;
  pid.ActualSpeed[3]=0;

  pid.OutSpeed[0]=0;
  pid.OutSpeed[1]=0;
  pid.OutSpeed[2]=0;
  pid.OutSpeed[3]=0;
  pid.OutSpeed[4]=0;
  pid.OutSpeed[5]=0;
  pid.OutSpeed[6]=0;
  pid.OutSpeed[7]=0;

  pid.err[0]=0;
  pid.err[1]=0;
  pid.err[2]=0;
  pid.err[3]=0;

  pid.err_last[0]=0;
  pid.err_last[1]=0;
  pid.err_last[2]=0;
  pid.err_last[3]=0;

  pid.err_next[0]=0;
  pid.err_next[1]=0;
  pid.err_next[2]=0;
  pid.err_next[3]=0;

  pid.Kp=5;
  pid.Ki[0]=0.6;
  pid.Ki[1]=0.6;
  pid.Ki[2]=0.6;
  pid.Ki[3]=0.6;
  pid.Kd=0;
}
