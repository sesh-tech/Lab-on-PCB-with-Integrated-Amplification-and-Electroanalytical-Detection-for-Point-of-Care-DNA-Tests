/*
 * global_variables.c
 *
 *  Created on: Oct 16, 2021
 *      Author: Sesh
 */

#include "main.h"
#include "pwm.h"
#include "filter.h"
#include "analog_in.h"
#include "analog_out.h"
#include "controller.h"
#include "moving_average.h"
#include "timer_interrupt.h"
#include "global_variables.h"

struct INTERRUPT Task1;
struct INTERRUPT Task2;

struct PWM airheater;
struct PWM mainheater;
struct PWM fan;

struct ANALOGIN Air_Heater_Voltage;			// ADC 1, Channel: 1, 					Rank: 1
struct ANALOGIN Main_Heater_Voltage;		// ADC 1, Channel: 2, 					Rank: 2

struct ANALOGIN Electro_Chem_Result;		// ADC 1, Chennel: 10,					Rank: 3

struct ANALOGIN Internal_Temp;				// ADC 1, Channel: Temprature Sensor, 	Rank: 4
struct ANALOGIN Internal_Ref_Voltage;		// ADC 1, Channel: Vrefint, 			Rank: 5

struct ANALOGIN Air_Heater_Current;			// ADC 2, Channel: 3,					Rank: 1
struct ANALOGIN Main_Heater_Current;		// ADC 2, Channel: 4, 					Rank: 2

struct ANALOGOUT ElectroChem_Offset;
struct ANALOGOUT ElectroChem_Set;

struct CONTROLLER main_Heater_Controller;

struct LP_FILTER_1_ORDER Main_Heater_Current_Filter;
struct LP_FILTER_1_ORDER Main_Heater_Voltage_Filter;
struct LP_FILTER_1_ORDER Main_Heater_Resistance_Filter;

struct MovingAverage Main_Heater_Voltage_MV;
struct MovingAverage Main_Heater_Current_MV;

float main_Heater_Current;
float air_Heater_Current;

float main_Heater_Voltage;
float air_Heater_Voltage;

float electro_Chem_Result;

float main_Heater_Resistance;
float air_Heater_Resistance;

float main_Heater_Reference;
float air_Heater_Reference;

float start_flag;

float AVDD;

float temperature_estimate;
float linear_term;
float offset_term;
