/*!
 * @file Temp.h
 * @brief Define the basic structure of Temperature control class
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0
 * @date 2023-03-06
 */

#ifndef TEMP_H
#define TEMP_H

#include "Magic.h"

#define Temperature 44

#define Temp_Control_ON 	1
#define Temp_Control_Off 	0

typedef struct
{
	
	uint16_t Duty_Cycle;
	
	short Ki;
	short Kp;
	short Kd;
	
	short Temp_setpoint;
	short Temp_icm42688;
	short Temp_thermistor;
	
	short delta_T;
	short PID_MAX;
	short PID_MIN;
	
	int flag;
	
} TEMP_t;

void Temp_Init(void);
void Temp_PID_Control(void);

#endif
