/*
 * processes.h
 *
 *  Created on: Oct 16, 2021
 *      Author: Sesh
 */

#ifndef INC_PROCESSES_H_
#define INC_PROCESSES_H_

void Task1ISR();
void Task2ISR();

extern UART_HandleTypeDef huart2;
extern DAC_HandleTypeDef hdac1;

#endif /* INC_PROCESSES_H_ */
