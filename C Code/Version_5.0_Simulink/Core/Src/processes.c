/*
 * processes.c
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

#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart2;

void Task1ISR()		// Watchdog Task
{

	if(start_flag == 1)
		start_flag = 0;
	else
	{
		SetPWMDuty(&mainheater, 2.0);
		SetPWMDuty(&airheater, 0);
		SetPWMDuty(&fan, 0);
		DAC_Set_Value(&ElectroChem_Set, 2000);
		DAC_Set_Value(&ElectroChem_Offset, 3000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
	}


//	uint32_t a[7] = {0};
//
//	a[0] = Analog_Value(&Main_Heater_Current);
//	a[1] = Analog_Value(&Main_Heater_Voltage);
//	a[2] = Analog_Value(&Air_Heater_Current);
//	a[3] = Analog_Value(&Air_Heater_Voltage);

//	main_Heater_Resistance = main_Heater_Current * main_Heater_Voltage * 250;

//	SetPWMDuty (&mainheater, getControllerOutput(&main_Heater_Controller, main_Heater_Reference - main_Heater_Resistance));

//	printf("PA0: %d\n", Analog_Value(&Air_Heater_Voltage));
//	printf("PA1: %d\n", Analog_Value(&Main_Heater_Voltage));
//	printf("PA6: %d\n", Analog_Value(&Air_Heater_Current));
//	printf("PA7: %d\n", Analog_Value(&Main_Heater_Current));
//	printf("PF0: %d\n", Analog_Value(&Electro_Chem_Result));

//	printf("%s", UART2_rxBuffer);

//	sprintf((char*) UART2_txBuffer, "%f\n", (float) Analog_Value(&Air_Heater_Voltage));
//	printf("printing\n");
//	HAL_UART_Transmit_IT(&huart2, UART2_txBuffer, sizeof((char*) UART2_txBuffer));

//	printf("%f\n", ((float) Analog_Value(&Main_Heater_Voltage) / 4096) * (AVDD));

//	float temp = 18;


//	UART2_txBuffer[0] = "S";
//	UART2_txBuffer[9] = "D";
//
//	memcpy(&UART2_txBuffer[1], &main_Heater_Voltage, 4);
//	memcpy(&UART2_txBuffer[5], &main_Heater_Current, 4);
//
//	HAL_UART_Transmit_IT(&huart2, UART2_txBuffer, 10);

//	printf("S,%f,%f,%f,D\n", main_Heater_Voltage, main_Heater_Current, main_Heater_Resistance);

//	printf("S,%f,%f,D\n", temperature_estimate, main_Heater_Resistance);

//	printf("%f\n", getLPF_1Output(&Main_Heater_Resistance_Filter, ((float) Analog_Value(&Main_Heater_Voltage) / 4096) * (AVDD) / (((float) Analog_Value(&Main_Heater_Current) / 4096) * (AVDD / (50 * 0.01)))));

//	getLPF_1Output(&Main_Heater_Resistance_Filter, ((float) Analog_Value(&Main_Heater_Voltage) / 4096) * (AVDD) / (((float) Analog_Value(&Main_Heater_Current) / 4096) * (AVDD / (50 * 0.01))));

}

void Task2ISR()		//Controlling Task
{
	uint16_t vrefint_cal = *((uint16_t*)VREFINT_CAL_ADDR);

	if ((float) Analog_Value(&Internal_Ref_Voltage) > 0)
		AVDD = 3.0 * ((float) vrefint_cal / (float) Analog_Value(&Internal_Ref_Voltage));
	else
		AVDD = 0.1;

	main_Heater_Voltage = ((float) Analog_Value(&Main_Heater_Voltage) / 4095) * (AVDD);
	main_Heater_Current = ((float) Analog_Value(&Main_Heater_Current) / 4095) * (AVDD / (50 * 0.01));

	main_Heater_Voltage = calculateMovingAverage(&Main_Heater_Voltage_MV, main_Heater_Voltage);
	main_Heater_Current = calculateMovingAverage(&Main_Heater_Current_MV, main_Heater_Current);

	electro_Chem_Result = ((float) Analog_Value(&Electro_Chem_Result) / 4095) * (AVDD);


//	main_Heater_Voltage = ((float) Analog_Value(&Main_Heater_Voltage) / 4095) * (AVDD);
//	main_Heater_Current = ((float) Analog_Value(&Main_Heater_Current) / 4095) * (AVDD / (50 * 0.01));

//	main_Heater_Voltage = getLPF_1Output (&Main_Heater_Voltage_Filter, (float) Analog_Value(&Main_Heater_Voltage) );
//	main_Heater_Current = getLPF_1Output (&Main_Heater_Current_Filter, (float) Analog_Value(&Main_Heater_Current) );

//	main_Heater_Resistance = getLPF_1Output (&Main_Heater_Resistance_Filter, main_Heater_Voltage/main_Heater_Current);

//	SetPWMDuty(&mainheater, getControllerOutput(&main_Heater_Controller, main_Heater_Reference - main_Heater_Resistance));

//	uint16_t vrefint_cal = *((uint16_t*)VREFINT_CAL_ADDR);
//
//	if ((float) Analog_Value(&Internal_Ref_Voltage) > 0)
//		AVDD = 3.0 * ((float) vrefint_cal / (float) Analog_Value(&Internal_Ref_Voltage));
//	else
//		AVDD = 0;
//
//	main_Heater_Voltage = getLPF_1Output (&Main_Heater_Voltage_Filter, ((float) Analog_Value(&Main_Heater_Voltage) / 4095) * (AVDD));
//	main_Heater_Current = getLPF_1Output (&Main_Heater_Current_Filter, ((float) Analog_Value(&Main_Heater_Current) / 4095) * (AVDD / (50 * 0.01)));
//
//	main_Heater_Resistance =  getLPF_1Output (&Main_Heater_Resistance_Filter, 1000*main_Heater_Voltage*main_Heater_Current);
//
//	temperature_estimate = linear_term*main_Heater_Resistance + offset_term;
//
//	SetPWMDuty(&mainheater, getControllerOutput(&main_Heater_Controller, main_Heater_Reference - temperature_estimate));
//
//	if((temperature_estimate - main_Heater_Reference > 2) && main_Heater_Reference > 0)
//		SetPWMDuty(&fan, 100);
//
//	else
//		SetPWMDuty(&fan, 0);

}
