/*
 * AICT_UserInit.c
 *
 *  Created on: 2020��5��13��
 *      Author: С��666
 */
#include "..\Driver\include.h"//����ģ���ͷ�ļ�
//#include<AICT_UserInit.h>
//#include <LQ_Atom_Motor.h>
//#include <AICT_Mecanum.h>
//#include <LQ_GTM.h>
/*************************************************************************
*  �������ƣ�void Motor_init()
*  ����˵����������ų�ʼ��
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2020��5��13��
*  ��    ע��ʹ���Ƽ�������
*************************************************************************/
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
*  �������ƣ�void PID_init()
*  ����˵�������PI������ʼ��
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2020��5��13��
*  ��    ע���������������������
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
/*************************************************************************
*  �������ƣ�void PID_init()
*  ����˵�������PI������ʼ��
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2020��5��13��
*  ��    ע���������������������
*************************************************************************/
void Mic_init()
{
	ADC_InitConfig(ADC0, 80000); //��ʼ��
	ADC_InitConfig(ADC1, 80000);
	ADC_InitConfig(ADC2, 80000);
	ADC_InitConfig(ADC3, 80000);
	ADC_InitConfig(ADC4, 80000);
	ADC_InitConfig(ADC5, 80000);
	ADC_InitConfig(ADC6, 80000);
	ADC_InitConfig(ADC7, 80000);
}



void Encoder_init()
{
	  ENC_InitConfig(ENC2_InPut_P33_7, ENC2_Dir_P33_6);
	  //ENC_InitConfig(ENC3_InPut_P02_6, ENC3_Dir_P02_7);//����ͷ��ͻ����������
	  ENC_InitConfig(ENC4_InPut_P02_8, ENC4_Dir_P33_5);
	  ENC_InitConfig(ENC5_InPut_P10_3, ENC5_Dir_P10_1);
	  ENC_InitConfig(ENC6_InPut_P20_3, ENC6_Dir_P20_0);
}
