/*
 * controller.c
 *
 *  Created on: Oct 16, 2021
 *      Author: Sesh
 */

#include "main.h"
#include "controller.h"

#include <math.h>
#include <stdlib.h>

void setControllerParameters(struct CONTROLLER *controller, float P_Gain, float I_Gain, float D_Gain, float Sampling_Freq, float Saturation_High, float Saturation_Low, float Default_Output)
{
  controller->p_Gain = P_Gain;
  controller->i_Gain = I_Gain;
  controller->d_Gain = D_Gain;

  controller->control_frequency = Sampling_Freq;	// Hz

  controller->saturation_High = Saturation_High;
  controller->saturation_Low  = Saturation_Low;

  controller->default_Output = Default_Output;
  controller->onoff = 0;

  controller->error = 0;
  controller->error_Prior = 0;
  controller->integral  = 0;
  controller->integral_Prior  = 0;
  controller->derivative  = 0;
  controller->windup  = 0;
  controller->proportional = 0;

  controller->controller_output = 0;
}

void setControllerGains(struct CONTROLLER *controller, float P_Gain, float I_Gain, float D_Gain, float Sampling_Freq)
{
  controller->p_Gain = P_Gain;
  controller->i_Gain = I_Gain;
  controller->d_Gain = D_Gain;

  controller->control_frequency = Sampling_Freq;	// Hz

  controller->saturation_High = INFINITY;
  controller->saturation_Low  = -INFINITY;

  controller->error = 0;
  controller->error_Prior = 0;
  controller->integral  = 0;
  controller->integral_Prior  = 0;
  controller->derivative  = 0;
  controller->windup  = 0;
  controller->proportional = 0;

  controller->controller_output = 0;
}

void changeControllerGains(struct CONTROLLER *controller, float P_Gain, float I_Gain, float D_Gain)
{
  controller->p_Gain = P_Gain;
  controller->i_Gain = I_Gain;
  controller->d_Gain = D_Gain;
}

void startController(struct CONTROLLER *controller, float State)
{
	controller->onoff = State;
}

float getControllerOutput(struct CONTROLLER *controller, float Error)
{

	if(controller->onoff == 1)
	{

		controller->error = Error;

		if(controller->i_Gain != 0)
		{
			controller->integral = controller->integral_Prior + (1/controller->control_frequency)*(controller->error + controller->error_Prior)*(controller->i_Gain) + controller->windup;
			controller->integral_Prior = controller->integral;
		}
		else
		{
			controller->integral = 0;
		}

		if(controller->d_Gain != 0)
		{
			controller->derivative = (controller->error_Prior-controller->error)/(1/controller->control_frequency);
			controller->error_Prior = controller->error;
		}
		else
		{
			controller->derivative = 0;
		}

		controller->proportional = controller->p_Gain*controller->error;

		controller->controller_output = controller->proportional + controller->integral + controller->derivative;

		if (controller->controller_output > controller->saturation_High)
		{
			controller->windup = controller->saturation_High - controller->controller_output;
			controller->controller_output = controller->saturation_High;
		}

		else if (controller->controller_output < controller->saturation_Low)
		{
			controller->windup = controller->saturation_Low - controller->controller_output;
			controller->controller_output = controller->saturation_Low;
		}

		else
			controller->windup = 0;

		return controller->controller_output;

	}

	else
	{
		controller->integral_Prior = 0;
		controller->error_Prior = 0;
		controller->windup = 0;

		return controller->default_Output;
	}
}

float calcualteMovingAverage(struct MOVINGAVERAGE *moving_average, float sample)
{


}
