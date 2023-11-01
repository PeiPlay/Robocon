/*!
 * @file Temp.c
 * @brief Achieve constant operating temperature for ICM42688
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0
 * @date 2023-03-06
 */
#include "Temp.h"
#include "arm_math.h"

TEMP_t HEAT;
arm_pid_instance_q15 Temp_pid;

void Temp_Init(void){
	
	HEAT.Kp = -20000;
	HEAT.Ki = 7000;
	HEAT.Kd = -2000;
	
	Temp_pid.Kp = HEAT.Kp;
	Temp_pid.Ki = HEAT.Ki;
	Temp_pid.Kd = HEAT.Kd;
	
	HEAT.Temp_setpoint = Temperature;
	HEAT.Temp_icm42688 = round(icm42688.tempData);
	
	HEAT.PID_MAX = 9500;
	HEAT.PID_MIN = 1500;
	arm_pid_init_q15(&Temp_pid,HEAT.flag);
	LL_mDelay(50);
}

void Temp_PID_Control(void){
	
	HEAT.Temp_icm42688 = round(icm42688.tempData);
	HEAT.delta_T = HEAT.Temp_setpoint - HEAT.Temp_icm42688;
	if(HEAT.delta_T > 1 | HEAT.delta_T < -1) LL_GPIO_SetOutputPin(GPIOC,GPIO_PIN_13);
	else LL_GPIO_ResetOutputPin(GPIOC,GPIO_PIN_13);
	
	
	arm_pid_q15(&Temp_pid,HEAT.delta_T);
	
	if(Temp_pid.state[2] > HEAT.PID_MAX) {
		//overflow protection
		Temp_pid.state[2] = HEAT.PID_MAX;
	}
	else if(Temp_pid.state[2] < HEAT.PID_MIN) {
		//underflow protection
		Temp_pid.state[2] = HEAT.PID_MIN;
	}

	HEAT.Duty_Cycle = Temp_pid.state[2];
	
//	Temp_pid.Kp = HEAT.Kp;
//	Temp_pid.Ki = HEAT.Ki;
//	Temp_pid.Kd = HEAT.Kd;
//	arm_pid_init_q15(&Temp_pid,HEAT.flag);
	//set output power
	TIM17->CCR1 = HEAT.Duty_Cycle;
	
}
