/*!
 * @file Magic.c
 * @brief Perform Magic on IMU data
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0 2023-03-03 Lack Magnetometer, Yaw angles drift observed.
 * @version V1.1 2023-03-11 Magnetometer added, but the filter delay is significant (about 1s)
 * @version V1.2 2023-03-16 The latency of Magnetometer is removed. It is caused by typo :(
 * @version V1.3 2023-03-30 Changed from MMC5603NJ to AK09918, added soft-iron and hard-iron calibration. Failed.
 * @version V2.0 2023-04-11 Changed from AK09918 to MMC5983MA. @successful

 */
#include "Magic.h"
#include "ICM42688.h"
#include "MMC5983MA.h"
#include "bsp_fdcan.h"
#include "arm_math.h"
#include "math.h"
#include "stdlib.h"
#include <stdbool.h>
#include <stdio.h>

#define SAMPLE_RATE (6400) // The actual sample rate (Determined by TIM16)
#define SAMPLE_PERIOD (0.00015625f) // The actual sample period (1/SAMPLE_RATE)

/** Note:
		Numeric integration of gyro data is not a perfect science.
		The faster you can integrate (1,000 Hz or faster would be nice), the better your answer.
		However, due to the complexity of Madgwick filter algorithm,
		it is advised not to beyond 6400 Hz (For STM32F722RET6 and STM32G431CBU6).
		Maybe, there is a more efficient way?
*/
MAF_t SMAF;
ICM42688_t icm42688;

// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;

// Define calibration (replace with actual calibration data if available)
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};

//const FusionMatrix softIronMatrix =  {0.847408, 0.0661085, 0.0317029, 0.0661085, 0.965542, 0.0212551, 0.0317029, 0.0212551, 0.782953};
const FusionMatrix softIronMatrix =  {1, 0, 0, 0, 1, 0, 0, 0, 1};
// No.1 IMU
const FusionVector hardIronOffset = {-3.98806, -4.01935, -3.97341};
// No.2 IMU
//const FusionVector hardIronOffset =  {-4.01804, -4.05722, -3.97799};

// Define Output variables
FusionEuler euler;
FusionVector earth;
FusionVector linear;

// Define Dead zones parameters
int16_t X_diff[1] = {0};
int16_t Y_diff[1] = {0};
int16_t Z_diff[1] = {0};
int16_t X_absdiff[1] = {0};
int16_t Y_absdiff[1] = {0};
int16_t Z_absdiff[1] = {0};

int16_t OmegaX_diff[1] = {0};
int16_t OmegaY_diff[1] = {0};
int16_t OmegaZ_diff[1] = {0};
int16_t OmegaX_absdiff[1] = {0};
int16_t OmegaY_absdiff[1] = {0};
int16_t OmegaZ_absdiff[1] = {0};


void Madgwick_Init(void)
{

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings =
    {
        .gain = 0.5f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 20.0f,
        .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);
    FusionAhrsReset(&ahrs);

    SMAF.MAF_index = 0;
    memset(SMAF.MAF_arrayX, 0, sizeof(SMAF.MAF_arrayX));
    memset(SMAF.MAF_arrayY, 0, sizeof(SMAF.MAF_arrayY));
    memset(SMAF.MAF_arrayZ, 0, sizeof(SMAF.MAF_arrayZ));

    // Wait for IMU to be stablized
    for(uint16_t i = 0; i < SMAF_Size; i++) //Sample data of SMAF_Size
    {
        getFIFOData(); //Obtain data from fifo
        LL_mDelay(20);
    }

    for(uint16_t i = 0; i < SMAF_Size; i++) //Sample data of SMAF_Size
    {
        getFIFOData(); //Obtain data from fifo

        if(abs(_gyroX) > Omega_Limit) SMAF.MAF_arrayX[i] = 0;
        else SMAF.MAF_arrayX[i] = _gyroX;

        if(abs(_gyroY) > Omega_Limit)	SMAF.MAF_arrayY[i] = 0;
        else SMAF.MAF_arrayY[i] = _gyroY;

        if(abs(_gyroZ) > Omega_Limit)	SMAF.MAF_arrayZ[i] = 0;
        else SMAF.MAF_arrayZ[i] = _gyroZ;

        LL_mDelay(15);
    }

    arm_mean_q15(SMAF.MAF_arrayX, SMAF_Size, &icm42688.OmegaXOffset);	// 	Omega_X MAF
    arm_mean_q15(SMAF.MAF_arrayY, SMAF_Size, &icm42688.OmegaYOffset);	//	Omega_Y MAF
    arm_mean_q15(SMAF.MAF_arrayZ, SMAF_Size, &icm42688.OmegaZOffset);	//	Omega_Z MAF

    LL_mDelay(100);
}

void Simple_MAF(int16_t _gyroX, int16_t _gyroY, int16_t _gyroZ)
{

    X_diff[0] = _gyroX - icm42688.OmegaXOffset;
    Y_diff[0] = _gyroY - icm42688.OmegaYOffset;
    Z_diff[0] = _gyroZ - icm42688.OmegaZOffset;

    arm_abs_q15(X_diff, X_absdiff, 1);
    arm_abs_q15(Y_diff, Y_absdiff, 1);
    arm_abs_q15(Z_diff, Z_absdiff, 1);

    if(X_absdiff[0] < Omega_Dead_Zone )	SMAF.MAF_arrayX[SMAF.MAF_index] = _gyroX;

    if(Y_absdiff[0] < Omega_Dead_Zone )	SMAF.MAF_arrayY[SMAF.MAF_index] = _gyroY;

    if(Z_absdiff[0] < Omega_Dead_Zone )	SMAF.MAF_arrayZ[SMAF.MAF_index] = _gyroZ;

    SMAF.MAF_index ++;

    if (SMAF.MAF_index >= SMAF_Size) SMAF.MAF_index = 0;

    arm_mean_q15(SMAF.MAF_arrayX, SMAF_Size, &icm42688.OmegaXDrift);	// 	Omega_X MAF
    arm_mean_q15(SMAF.MAF_arrayY, SMAF_Size, &icm42688.OmegaYDrift);	// 	Omega_Y MAF
    arm_mean_q15(SMAF.MAF_arrayZ, SMAF_Size, &icm42688.OmegaZDrift);	// 	Omega_Z MAF

//	OmegaX_diff[0] = icm42688.OmegaXDrift - icm42688.OmegaXOffset;
//	OmegaY_diff[0] = icm42688.OmegaYDrift - icm42688.OmegaYOffset;
//	OmegaZ_diff[0] = icm42688.OmegaZDrift - icm42688.OmegaZOffset;

//	arm_abs_q15(OmegaX_diff,OmegaX_absdiff,1);
//	arm_abs_q15(OmegaY_diff,OmegaY_absdiff,1);
//	arm_abs_q15(OmegaZ_diff,OmegaZ_absdiff,1);

//	if(OmegaX_absdiff[0] < Omega_Dead_Zone) icm42688.OmegaXOffset = icm42688.OmegaXDrift;
//	if(OmegaY_absdiff[0] < Omega_Dead_Zone) icm42688.OmegaYOffset = icm42688.OmegaYDrift;
//	if(OmegaZ_absdiff[0] < Omega_Dead_Zone) icm42688.OmegaZOffset = icm42688.OmegaZDrift;

}

void IMU_Connection_Check(void)
{

    while(true)
    {
        LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_9);
        LL_mDelay(500);
        icm42688.IMU_Connection = begin();
        LL_mDelay(50);

        if(icm42688.IMU_Connection == ERR_OK)
        {
            LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_9);
            LL_mDelay(500);
            startTempMeasure();
            startGyroMeasure(/* mode= */LN_MODE);
            startAccelMeasure(/* mode= */LN_MODE);
            startFIFOMode();  //Enable FIFO

            /**
             * Note£ºIn FIFO mode, set gyroscope and accelerometer ODR to be the same
             * and the selected ODR should be no more than 8KHz.
             * Otherwise, the temperature data integration rate will not match reading rate.
             */
            setODRAndFSR(/* who= */GYRO,/* ODR= */ODR_8KHZ, /* FSR = */FSR_0);
            setODRAndFSR(/* who= */ACCEL,/* ODR= */ODR_8KHZ, /* FSR = */FSR_0);
            LL_mDelay(10);
            Madgwick_Init();
            break;
        }
    }

}

void Madgwick_Run(void)
{
    if(LL_TIM_IsActiveFlag_UPDATE(TIM16) == SET)
    {
        LL_TIM_ClearFlag_UPDATE(TIM16);//Clear Interrupt Flag

        // Acquire latest sensor data
        getFIFOData();
        Simple_MAF(_gyroX, _gyroY, _gyroZ);

        // Drift Reduction
        // When IMU is viewed as motionless, the angular velocity should be zero.
        if( Z_absdiff[0] < Omega_Dead_Zone) icm42688.gyroDataZ = 0;
        else icm42688.gyroDataZ = getGyroDataZ() - icm42688.OmegaZOffset * _gyroRange;

        if( X_absdiff[0] < Omega_Dead_Zone) icm42688.gyroDataX = 0;
        else icm42688.gyroDataX = getGyroDataX() - icm42688.OmegaXOffset * _gyroRange;

        if( Y_absdiff[0] < Omega_Dead_Zone) icm42688.gyroDataY = 0;
        else icm42688.gyroDataY = getGyroDataY() - icm42688.OmegaYOffset * _gyroRange;

        icm42688.tempData   = getTemperature();
        
        icm42688.accelDataX = getAccelDataX();
        icm42688.accelDataY = getAccelDataY();
        icm42688.accelDataZ = getAccelDataZ();
        //gyroscope data in degrees/s
        FusionVector gyroscope = {icm42688.gyroDataX, icm42688.gyroDataY, icm42688.gyroDataZ};
        //accelerometer data in g (gravitational acceleration)
        FusionVector accelerometer = {icm42688.accelDataX, icm42688.accelDataY, icm42688.accelDataZ};

        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

        // Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        // Print algorithm outputs
        euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        earth = FusionAhrsGetEarthAcceleration(&ahrs);
        
        linear = FusionAhrsGetLinearAcceleration(&ahrs);
//        Accel_Filter();

        Comm_CAN.AccelX =   linear.axis.x;
        Comm_CAN.AccelY =   linear.axis.y;
        Comm_CAN.AccelZ =   linear.axis.z;
//get the angular speed for every axis
        
//        Comm_CAN.AccelY= icm42688.accelDataX ;
//        Comm_CAN.AccelX= icm42688.accelDataY;
//        Comm_CAN.AccelZ= icm42688.accelDataZ;
        
        Comm_CAN.roll_spd = icm42688.gyroDataX;
        Comm_CAN.pitch_spd = icm42688.gyroDataY;
        Comm_CAN.yaw_spd = icm42688.gyroDataZ;
    }
}

void Madgwick_RunwithMag(void)
{
    if(LL_TIM_IsActiveFlag_UPDATE(TIM16) == SET)
    {
        LL_TIM_ClearFlag_UPDATE(TIM16);//Clear Interrupt Flag

        getFIFOData();
        Simple_MAF(_gyroX, _gyroY, _gyroZ);

        icm42688.tempData = getTemperature();
        icm42688.accelDataX = getAccelDataX();
        icm42688.accelDataY = getAccelDataY();
        icm42688.accelDataZ = getAccelDataZ();

        if( Z_absdiff[0] < Omega_Dead_Zone) icm42688.gyroDataZ = 0;
        else icm42688.gyroDataZ = getGyroDataZ() - icm42688.OmegaZOffset * _gyroRange;

        if( X_absdiff[0] < Omega_Dead_Zone) icm42688.gyroDataX = 0;
        else icm42688.gyroDataX = getGyroDataX() - icm42688.OmegaXOffset * _gyroRange;

        if( Y_absdiff[0] < Omega_Dead_Zone) icm42688.gyroDataY = 0;
        else icm42688.gyroDataY = getGyroDataY() - icm42688.OmegaYOffset * _gyroRange;


        //gyroscope data in degrees/s
        FusionVector gyroscope_mag = {icm42688.gyroDataX, icm42688.gyroDataY, icm42688.gyroDataZ};
        //accelerometer data in g (gravitational acceleration)
        FusionVector accelerometer_mag = {icm42688.accelDataX, icm42688.accelDataY, icm42688.accelDataZ};
        //magnetometer data in arbitrary units
        FusionVector magnetometer = {-mmc5983.magDataX, -mmc5983.magDataY, -mmc5983.magDataZ};
        // Apply calibration
        gyroscope_mag = FusionCalibrationInertial(gyroscope_mag, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer_mag = FusionCalibrationInertial(accelerometer_mag, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
        magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

        // Update gyroscope offset correction algorithm
        gyroscope_mag = FusionOffsetUpdate(&offset, gyroscope_mag);

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdate(&ahrs, gyroscope_mag, accelerometer_mag, magnetometer, SAMPLE_PERIOD);

        // Print algorithm outputs
        euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        earth = FusionAhrsGetEarthAcceleration(&ahrs);
        linear = FusionAhrsGetLinearAcceleration(&ahrs);

        Comm_CAN.AccelX =   linear.axis.x;
        Comm_CAN.AccelY =   linear.axis.y;
        Comm_CAN.AccelZ =   linear.axis.z;

        //get the angular speed for every axis
        Comm_CAN.roll_spd = icm42688.gyroDataX;
        Comm_CAN.pitch_spd = icm42688.gyroDataY;
        Comm_CAN.yaw_spd = icm42688.gyroDataZ;
    }
}
