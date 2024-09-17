/*
 * timer_interrupt.c
 *
 *  Created on: Mar 2, 2020
 *      Author: Sesh
 */

#include "pwm.h"


void PWMInit(struct PWM * pwm, TIM_TypeDef *timeNr, TIM_HandleTypeDef *TmrHandle, uint32_t TmrChnl, uint32_t tmrclkspd, uint32_t pwm_freq)
	/*
	 * Function:
	 * Initializes the interrupt and based on the timer, interrupt structure, timer speed and the interrupt interval
	 *
	 * Inputs:
	 * Argument 1: INTERRUPT struct, named based on the interrupt name, example: ControllerInterrupt
	 * Argument 2: Timer Number, example: TIM7
	 * Argument 3: Timer Handle Type Def, example: &htim7
	 * Argument 4: Timer Clock Speed: 80000000 (Hz)
	 * Argument 5: Sampling time/time between successive interrupts in microseconds, example: 500
	 */

	{

		pwm->timerNr = timeNr;						//Writes the timer number to the structure
		pwm->htim	= TmrHandle;						//Writes the timer handle to the structure
		pwm->chnl	= TmrChnl;
		pwm->Timer_Clock_Speed = tmrclkspd;			//Writes the timer clock speed to the structure
		pwm->PWM_Frequency = pwm_freq;		//Writes the Sampling time/time between interrupts to the structure


		UpdatePWMFreq (pwm, pwm_freq);		//Call the update interrupt time function to enter the values of the prescaler and counter resolution

	}

void UpdatePWMFreq (struct PWM * pwm, uint32_t pwm_freq)
	/*
	 * Function:
	 * Updates the timer interval for interrupt
	 *
	 * Inputs:
	 * Argument 1: INTERRUPT struct, named based on the interrupt name, example: ControllerInterrupt
	 * Argument 2: Sampling time/time between successive interrupts in microseconds, example: 500
	 */

{
	pwm->PWM_Frequency = pwm_freq;		//Writes the sampling time to the structure

	uint32_t frequency_fraction = pwm->Timer_Clock_Speed/pwm->PWM_Frequency;

	if(frequency_fraction < 65535)
	{
		pwm->Prescaler = 0;
		pwm->Counter_Resolution = frequency_fraction;
	}
	else
	{
		pwm->Prescaler = ((float) frequency_fraction)/65535.0f + 0.5f;
		pwm->Counter_Resolution = (frequency_fraction) / (pwm->Prescaler + 1);
	}

	pwm->timerNr->PSC = pwm->Prescaler;					//Write the prescaler to the timer control register
	pwm->timerNr->ARR = pwm->Counter_Resolution-1;		//Write the counter resolution resolution to the timer auto reload register
}

void SetPWMDuty (struct PWM * pwm, float duty)
{

	if(pwm->chnl == 1)																				//Check if servo channel is 1 as pointer pointing to pointers to pointer is not available
		pwm->timerNr->CCR1 = ( uint32_t) ( (float) ( pwm->Counter_Resolution) * ( (float) duty)/100 );	//Applies the duty cycle
	if(pwm->chnl == 2)																				//Check if servo channel is 3 as pointer pointing to pointers to pointer is not available
		pwm->timerNr->CCR2 = ( uint32_t) ( (float) ( pwm->Counter_Resolution) * ( (float) duty)/100 );	//Applies the duty cycle
	if(pwm->chnl == 3)																				//Check if servo channel is 3 as pointer pointing to pointers to pointer is not available
		pwm->timerNr->CCR3 = ( uint32_t) ( (float) ( pwm->Counter_Resolution) * ( (float) duty)/100 );	//Applies the duty cycle
	if(pwm->chnl == 4)																				//Check if servo channel is 4 as pointer pointing to pointers to pointer is not available
		pwm->timerNr->CCR4 = ( uint32_t) ( (float) ( pwm->Counter_Resolution) * ( (float) duty)/100 );	//Applies the duty cycle

}

void StartPWM (struct PWM * pwm)
	/*
	 * Function:
	 * Starts the timer and begins the interrupt cycle for the controller
	 *
	 * Input:
	 * Argument 1: INTERRUPT struct, named based on the interrupt name, example: ControllerInterrupt
	 */

	{

		if(pwm->chnl == 1)																			//Check if servo channel is 1 as pointer pointing to pointers to pointer is not available
			HAL_TIM_PWM_Start(pwm->htim, TIM_CHANNEL_1);
		if(pwm->chnl == 2)																			//Check if servo channel is 3 as pointer pointing to pointers to pointer is not available
			HAL_TIM_PWM_Start(pwm->htim, TIM_CHANNEL_2);
		if(pwm->chnl == 3)																			//Check if servo channel is 3 as pointer pointing to pointers to pointer is not available
			HAL_TIM_PWM_Start(pwm->htim, TIM_CHANNEL_3);
		if(pwm->chnl == 4)																			//Check if servo channel is 4 as pointer pointing to pointers to pointer is not available
			HAL_TIM_PWM_Start(pwm->htim, TIM_CHANNEL_4);

	}

void StopPWM (struct PWM * pwm)
	/*
	 * Function:
	 * Stops the timer and the interrupts
	 *
	 * Input:
	 * Argument 1: INTERRUPT struct, named based on the interrupt name, example: ControllerInterrupt
	 */

	{

		if(pwm->chnl == 1)																			//Check if servo channel is 1 as pointer pointing to pointers to pointer is not available
			HAL_TIM_PWM_Stop(pwm->htim, TIM_CHANNEL_1);
		if(pwm->chnl == 2)																			//Check if servo channel is 3 as pointer pointing to pointers to pointer is not available
			HAL_TIM_PWM_Stop(pwm->htim, TIM_CHANNEL_2);
		if(pwm->chnl == 3)																			//Check if servo channel is 3 as pointer pointing to pointers to pointer is not available
			HAL_TIM_PWM_Stop(pwm->htim, TIM_CHANNEL_3);
		if(pwm->chnl == 4)																			//Check if servo channel is 4 as pointer pointing to pointers to pointer is not available
			HAL_TIM_PWM_Stop(pwm->htim, TIM_CHANNEL_4);

	}
