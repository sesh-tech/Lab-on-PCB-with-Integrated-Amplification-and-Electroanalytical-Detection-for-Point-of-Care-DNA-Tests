/*
 * timer_interrupt.h
 *
 *  Created on: Mar 2, 2020
 *      Author: Sesh
 */

#include "main.h"			//This is generally enough as main.h has all the header declarations


/*
 * CubeMX deployments and settings:
 * Steps:
 * 1. Setup Timers (either 6, 7, 15 or 16) to "Activated" mode
 * 2. Under "NVIC Mode" enable global interrupts
 * 3. Repeat for other timers and setup files in the PWM code
 */


	struct PWM								//Structure to define the timer PWM channel
	{

		  uint32_t Timer_Clock_Speed;       //Clock Speed of the clock supplying the timer
		  uint32_t PWM_Frequency;		//Time between successive interrupts
		  uint32_t Counter_Resolution;		//The resolution of the counter for interrupts
		  uint32_t Prescaler;				//Prescaler for the clock from the timer

		  TIM_HandleTypeDef 	*htim;		//Hardware timer handle typedef (example: &htim4)
		  TIM_TypeDef 	*timerNr;			//Timer number typedef (example: TIM4)
		  uint32_t		chnl;

	};

	void PWMInit (struct PWM * pwm, TIM_TypeDef *timeNr,  TIM_HandleTypeDef *TmrHandle, uint32_t TmrChnl, uint32_t tmrclkspd, uint32_t pwm_freq); //Check timer_interrupt.c
	void UpdatePWMFreq (struct PWM * pwm, uint32_t pwm_freq);							//Check timer_interrupt.c
	void SetPWMDuty (struct PWM * pwm, float duty);
	void StartPWM (struct PWM * pwm);													//Check timer_interrupt.c
	void StopPWM (struct PWM * pwm);													//Check timer_interrupt.c

