/*
 * filters.h
 *
 *  Created on: Feb 10, 2023
 *      Author: Eng.Hazem
 */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_
#include "stm32g4xx_hal.h"

/* Filter parameters */
#define ALPHA 0.95  //dc filter alpha value
#define MEAN_FILTER_SIZE        15


typedef struct{
	float w;
	float result;
}DC_FILTER_T;

typedef struct
{
  float v[2];
  float result;
}BUTTERWORTH_FILTER_T;

typedef struct
{
  float values[MEAN_FILTER_SIZE];
  uint8_t index;
  float sum;
  uint8_t count;
}MEAN_DIFF_FILTER_T;

DC_FILTER_T dcRemoval(float x, float prev_w, float alpha);
void lowPassButterworthFilter(float x, BUTTERWORTH_FILTER_T * filterResult);
float meanDiff(float M, MEAN_DIFF_FILTER_T* filterValues);

#endif /* INC_FILTERS_H_ */
