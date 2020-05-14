/*
 * AICT_Mecanum.h
 *
 *  Created on: 2020年5月13日
 *      Author: 小波666
 */

#ifndef SRC_APPSW_TRICORE_APP_AICT_MECANUM_H_
#define SRC_APPSW_TRICORE_APP_AICT_MECANUM_H_
#include "Cpu\Std\Platform_Types.h"
#define ppr 512.0
#define r  3.0
#define DriveRatio 2.0/5
#define MAXSPEED   800
#define CAR_L 0.18
#define CAR_W 0.18
#define BANGBANG 250

typedef struct
{
  sint16  SetSpeed[4];
  sint16 OutSpeed[8];

  sint16 Out_Speed[4];

  sint16 ActualSpeed[4];
  sint16 err[4];
  sint16 err_next[4];
  sint16 err_last[4];
  float Kp,Ki[4],Kd;
}PID;

extern PID pid;
extern float Angle_next,Out_W;
extern sint16 Wheel_speed[4];//  cm/ms
void PID_realize(sint16 Speed,float ang ,sint16 w,sint16 encSpeed[4]);
float Angle_W(float image_angle) ;
void Motor_out(void);

//---------------------------------------------------------------------



typedef enum
{
    Mot0_P,
    Mot0_N,
    Mot1_P,
    Mot1_N,
    Mot2_P,
    Mot2_N,
    Mot3_P,
    Mot3_N,
}Mot_CHn_e;
void Motor_Duty(uint16 Motno, uint32 duty);



#endif /* SRC_APPSW_TRICORE_APP_AICT_MECANUM_H_ */
