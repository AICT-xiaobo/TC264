/*
 * AICT_SimpleFunction.h
 *
 *  Created on: 2020年5月13日
 *      Author: 小波666
 */

#ifndef SRC_APPSW_TRICORE_APP_AICT_SIMPLEFUNCTION_H_
#define SRC_APPSW_TRICORE_APP_AICT_SIMPLEFUNCTION_H_
#include "Cpu\Std\Platform_Types.h"

int   myabs(int dat);
void  my_delay(long t);
float limit(float x, uint16 y);
sint16 limit_ab(sint16 x, sint16 a, sint16 b);
float  myfabs(float dat);




#endif /* SRC_APPSW_TRICORE_APP_AICT_SIMPLEFUNCTION_H_ */
