/*!
 * @file Magic.h
 * @brief Define the basic structure of Magic class
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0
 * @date 2023-03-03
 */

#ifndef MAGIC_H
#define MAGIC_H

#include "Fusion.h"
#include "spi.h"
#include "ICM42688.h"

/** The size of MAF is recommended to be 64.
* 	Larger size of MAF would not achieve better performance as the drift is not a single frequency dependent variable.
*		It is dependent on temperature, vibration, position of PCB and etc.
*		It is worth mentioning that the ADC is ¦¤¦²M, which reduce the quantization noise by a large factor.
*/
#define SMAF_Size 64
#define Omega_Limit 16384
#define Omega_Dead_Zone 16
#define Accel_Dead_Zone (uint8_t)(1/_accelRange*0.02f)

// Moving average filter (FIR)
typedef struct
{
	uint16_t MAF_index;

	int16_t MAF_arrayX[SMAF_Size];
	int16_t MAF_arrayY[SMAF_Size];
	int16_t MAF_arrayZ[SMAF_Size];

} MAF_t;

typedef struct
{
	float accelDataX;
	float accelDataY;
	float accelDataZ;
	float gyroDataX;
	float gyroDataY;
	float gyroDataZ;
	float tempData;
	
	int16_t OmegaXOffset;
	int16_t OmegaYOffset;
	int16_t OmegaZOffset;
	
	short OmegaXDrift;
	short OmegaYDrift;
	short OmegaZDrift;
	
	short IMU_Connection;
	
} ICM42688_t;

void Madgwick_Init(void);
void Simple_MAF(int16_t _gyroX, int16_t _gyroY, int16_t _gyroZ);
void IMU_Connection_Check(void);
void Madgwick_Run(void);
void Madgwick_RunwithMag(void);

extern ICM42688_t icm42688;
extern FusionEuler euler;
extern FusionVector earth;
extern float roll, pitch, yaw;
#endif

