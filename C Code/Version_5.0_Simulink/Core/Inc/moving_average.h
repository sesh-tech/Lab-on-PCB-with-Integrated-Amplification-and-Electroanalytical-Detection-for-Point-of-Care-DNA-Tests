/*
 * movingaverage.h
 *
 *  Created on: Aug 14, 2024
 *      Author: setm
 */

#include <stdlib.h>
#include "main.h"

#ifndef INC_MOVING_AVERAGE_H_
#define INC_MOVING_AVERAGE_H_

struct MovingAverage{

    float data[1000];
    uint32_t currentIndex;
    uint32_t size;

    float sum;
    float movingAverage;
};

void initializeMovingAverage(struct MovingAverage * MV);
float calculateMovingAverage(struct MovingAverage * MV, float currentSample);
float getMovingAverage(struct MovingAverage * MV);


#endif /* INC_MOVING_AVERAGE_H_ */
