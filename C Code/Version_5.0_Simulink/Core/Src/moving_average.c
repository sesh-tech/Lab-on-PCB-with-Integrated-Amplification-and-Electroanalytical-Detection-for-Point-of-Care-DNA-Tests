/*
 * moving_average.c
 *
 *  Created on: Aug 14, 2024
 *      Author: setm
 */

#include "moving_average.h"

void initializeMovingAverage(struct MovingAverage * MV)
{
    for(int i = 0; i < sizeof(MV->data)/sizeof(MV->data[0]); i++)
    {
        MV->data[i] = 0;
    }

    MV->currentIndex = 0;
    MV->size = 1000;

    MV->sum = 0;
    MV->movingAverage = 0;
}

float calculateMovingAverage(struct MovingAverage * MV, float currentSample)
{

    MV->sum = MV->sum - MV->data[MV->currentIndex];

    MV->data[MV->currentIndex] = currentSample;

    MV->sum = MV->sum + MV->data[MV->currentIndex];

    MV->movingAverage = MV->sum/(float) MV->size;

    MV->currentIndex++;
    MV->currentIndex %= MV->size;

    return MV->movingAverage;
}

float getMovingAverage(struct MovingAverage * MV)
{
    return MV->movingAverage;
}
