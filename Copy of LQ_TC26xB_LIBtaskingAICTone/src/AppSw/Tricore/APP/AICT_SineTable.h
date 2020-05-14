/*
 * AICT_SineTable.H
 *
 *  Created on: 2020年5月13日
 *      Author: 小波666
 */

#ifndef SRC_APPSW_TRICORE_APP_AICT_SINETABLE_H_
#define SRC_APPSW_TRICORE_APP_AICT_SINETABLE_H_
#include "AICT_SimpleFunction.h"

/* constants */
#define TableSize	8193
#define TWO_PI 		6.283185306
#define HALF_PI 	1.5707963265
/* the LUT */
extern const float sinetable[8193];

/* public f(x) */
float lookup_sin(float x);
float lookup_cos(float x);
float lookup_tan(float x);
float lookup_cot(float x);


#endif /* SRC_APPSW_TRICORE_APP_AICT_SINETABLE_H_ */
