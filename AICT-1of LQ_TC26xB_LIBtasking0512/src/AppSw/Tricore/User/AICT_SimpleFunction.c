/*
 * AICT_SimpleFunction.c
 *
 *  Created on: 2020��5��13��
 *      Author: С��666
 */
/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		AICT_SimpleFunction
 *
 * @author     		��ɿƼ�(QQ3184284598)
 *
 * @Software
 * @Target core		TC264DA
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-05-13
 ********************************************************************************************************************/


#include "AICT_SimpleFunction.h"


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ֵ����
//  @param      dat				��Ҫ�����ֵ����
//  @return     int				���ؾ���ֵ
//  Sample usage:				dat = myabs(dat);//��dat�������
//-------------------------------------------------------------------------------------------------------------------
int  myabs(int dat)
{
    if(dat>=0)  return dat;
    else        return -dat;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ֵ����
//  @param      dat				��Ҫ�����ֵ����
//  @return     float				���ؾ���ֵ
//  Sample usage:				dat = myabs(dat);//��dat�������
//-------------------------------------------------------------------------------------------------------------------
float  myfabs(float dat)
{
    if(dat>=0)  return dat;
    else        return -dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������ʱ
//  @param      t				��ʱʱ��
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void my_delay(long t)
{
    while(t--);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      �޷�
//  @param      x				���޷�������
//  @param      y				�޷���Χ(���ݻᱻ������-y��+y֮��)
//  @return     float			�޷�֮�������
//  Sample usage:				float dat = limit(500,300);//���ݱ�������-300��+300֮��  ��˷��صĽ����300
//-------------------------------------------------------------------------------------------------------------------
float limit(float x, uint16 y)
{
    if(x>y)             return y;
    else if(x<-y)       return -y;
    else                return x;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ˫���޷�
//  @param      x				���޷�������
//  @param      a				�޷���Χ��߽�
//  @param      b				�޷���Χ�ұ߽�
//  @return     float			�޷�֮�������
//  Sample usage:				sint16 dat = limit_ab(500,-300��400);//���ݱ�������-300��+400֮��  ��˷��صĽ����400
//-------------------------------------------------------------------------------------------------------------------
sint16 limit_ab(sint16 x, sint16 a, sint16 b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}


