/*!
 * @file bsp_fdcan.h
 * @brief Define the configuration of FDCAN
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0
 * @date 2023-03-06
 * @note Calculated parameters on https://phryniszak.github.io/stm32g-fdcan/
 */

#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H


#include "fdcan.h"
#include "Magic.h"


#define  IMU_STDID          0x300
#define  RATIO              1.74533f  //0.0174533f
#define  Ang_to_Rad(input)  (float)(input* RATIO)
#define  UINT               0       //1:rad, 0:angle

void FDCAN_Config(void);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void Fixed_Time_FDCAN(void);
void Accel_Filter(void);

typedef struct{
	
	uint16_t flag;
	uint16_t index;
	
	uint16_t send;
	
	float AccelX;
	float AccelY;
	float AccelZ;
	
	float AccelXoffset;
	float AccelYoffset;
	float AccelZoffset;
	
	float AX_Sample[SMAF_Size];
	float AY_Sample[SMAF_Size];
	float AZ_Sample[SMAF_Size];
	
	float AX_Offset[SMAF_Size];
	float AY_Offset[SMAF_Size];
	float AZ_Offset[SMAF_Size];
	
	float roll;
	float pitch;
	float yaw;
	
	float roll_spd;
	float pitch_spd;
	float yaw_spd;
	
	float Trans_Ax;
	float Trans_Ay;
	float Trans_Az;
	
} Comm_t;

extern FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];
extern FDCAN_TxHeaderTypeDef TxHeader;
extern uint8_t TxData[8];
extern Comm_t Comm_CAN;

#endif
