/*
 * global_variables.h
 *
 *  Created on: Oct 16, 2021
 *      Author: Sesh
 */

#ifndef INC_GLOBAL_VARIABLES_H_
#define INC_GLOBAL_VARIABLES_H_

#include "main.h"


extern struct INTERRUPT Task1;
extern struct INTERRUPT Task2;

extern struct PWM airheater;
extern struct PWM mainheater;
extern struct PWM fan;

extern struct ANALOGIN Air_Heater_Voltage;		// ADC 1, Channel: 1, 					Rank: 1
extern struct ANALOGIN Main_Heater_Voltage;		// ADC 1, Channel: 2, 					Rank: 2

extern struct ANALOGIN Electro_Chem_Result;		// ADC 1, Chennel: 10,					Rank: 3

extern struct ANALOGIN Internal_Temp;			// ADC 1, Channel: Temprature Sensor, 	Rank: 4
extern struct ANALOGIN Internal_Ref_Voltage;	// ADC 1, Channel: Vrefint, 			Rank: 5

extern struct ANALOGIN Air_Heater_Current;		// ADC 2, Channel: 3,					Rank: 1
extern struct ANALOGIN Main_Heater_Current;		// ADC 2, Channel: 4, 					Rank: 2

extern struct ANALOGOUT ElectroChem_Offset;
extern struct ANALOGOUT ElectroChem_Set;

extern struct CONTROLLER main_Heater_Controller;

extern struct LP_FILTER_1_ORDER Main_Heater_Current_Filter;
extern struct LP_FILTER_1_ORDER Main_Heater_Voltage_Filter;
extern struct LP_FILTER_1_ORDER Main_Heater_Resistance_Filter;

extern struct MovingAverage Main_Heater_Voltage_MV;
extern struct MovingAverage Main_Heater_Current_MV;

extern float main_Heater_Current;
extern float air_Heater_Current;

extern float main_Heater_Voltage;
extern float air_Heater_Voltage;

extern float electro_Chem_Result;

extern float main_Heater_Resistance;
extern float air_Heater_Resistance;

extern float main_Heater_Reference;
extern float air_Heater_Reference;

extern float start_flag;

extern float AVDD;

extern float temperature_estimate;
extern float linear_term;
extern float offset_term;

#endif /* INC_GLOBAL_VARIABLES_H_ */
