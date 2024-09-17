/*
 * controller.h
 *
 *  Created on: Oct 16, 2021
 *      Author: Sesh
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_


struct CONTROLLER
{

	float p_Gain;
	float i_Gain;
	float d_Gain;

	float control_frequency;

	float saturation_High;
	float saturation_Low;

	float default_Output;
	float onoff;

	float error;
	float error_Prior;
	float integral;
	float integral_Prior;
	float derivative;
	float windup;
	float proportional;

	float controller_output;
};

void setControllerParameters(struct CONTROLLER *controller, float P_Gain, float I_Gain, float D_Gain, float Sampling_Freq, float Saturation_High, float Saturation_Low, float Default_Output);
void setControllerGains(struct CONTROLLER *controller, float P_Gain, float I_Gain, float D_Gain, float Sampling_Freq);
void changeControllerGains(struct CONTROLLER *controller, float P_Gain, float I_Gain, float D_Gain);
float getControllerOutput(struct CONTROLLER *controller, float Error);

#endif /* INC_CONTROLLER_H_ */
