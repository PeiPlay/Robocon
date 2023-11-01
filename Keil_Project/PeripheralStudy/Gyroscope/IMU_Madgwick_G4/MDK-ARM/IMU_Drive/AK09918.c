/*!
 * @file AK09918.c
 * @brief Interface with magnetometer AK09918
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0
 * @date 2023-03-30
 */
 
#include "AK09918.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "arm_math.h"

uint8_t _buffer[16];
enum AK09918_mode_type_t AK09918_mode;
enum AK09918_err_type_t AK09918_err;
AK09918_t ak09918;

uint16_t AK09918_getDeviceID(void);
uint8_t AK09918_getRawMode(void);

/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */

static q31_t firStateQ31_X[BLOCK_SIZE + NUM_TAPS - 1];
static q31_t firStateQ31_Y[BLOCK_SIZE + NUM_TAPS - 1];
static q31_t firStateQ31_Z[BLOCK_SIZE + NUM_TAPS - 1];

/* ----------------------------------------------------------------------
** FIR Coefficients buffer generated using fir1() MATLAB function.
** ------------------------------------------------------------------- */

const q31_t firCoeffsQ31_X[NUM_TAPS] = {
12399111,3561728,4053867,4576615,5129262,5710878,6320312,6956192,7616928,8300716,9005544,9729201,
10469287,11223221,11988258,12761500,13539917,14320359,15099578,15874247,16640981,17396356,18136936,
18859291,19560021,20235781,20883303,21499414,22081066,22625351,23129524,23591021,24007482,24376759,
24696941,24966362,25183615,25347562,25457342,25512376,25512376,25457342,25347562,25183615,24966362,
24696941,24376759,24007482,23591021,23129524,22625351,22081066,21499414,20883303,20235781,19560021,
18859291,18136936,17396356,16640981,15874247,15099578,14320359,13539917,12761500,11988258,11223221,
10469287,9729201,9005544,8300716,7616928,6956192,6320312,5710878,5129262,4576615,4053867,3561728,12399111};

const q31_t firCoeffsQ31_Y[NUM_TAPS] = {
12399111,3561728,4053867,4576615,5129262,5710878,6320312,6956192,7616928,8300716,9005544,9729201,
10469287,11223221,11988258,12761500,13539917,14320359,15099578,15874247,16640981,17396356,18136936,
18859291,19560021,20235781,20883303,21499414,22081066,22625351,23129524,23591021,24007482,24376759,
24696941,24966362,25183615,25347562,25457342,25512376,25512376,25457342,25347562,25183615,24966362,
24696941,24376759,24007482,23591021,23129524,22625351,22081066,21499414,20883303,20235781,19560021,
18859291,18136936,17396356,16640981,15874247,15099578,14320359,13539917,12761500,11988258,11223221,
10469287,9729201,9005544,8300716,7616928,6956192,6320312,5710878,5129262,4576615,4053867,3561728,12399111};

const q31_t firCoeffsQ31_Z[NUM_TAPS] = {
12399111,3561728,4053867,4576615,5129262,5710878,6320312,6956192,7616928,8300716,9005544,9729201,
10469287,11223221,11988258,12761500,13539917,14320359,15099578,15874247,16640981,17396356,18136936,
18859291,19560021,20235781,20883303,21499414,22081066,22625351,23129524,23591021,24007482,24376759,
24696941,24966362,25183615,25347562,25457342,25512376,25512376,25457342,25347562,25183615,24966362,
24696941,24376759,24007482,23591021,23129524,22625351,22081066,21499414,20883303,20235781,19560021,
18859291,18136936,17396356,16640981,15874247,15099578,14320359,13539917,12761500,11988258,11223221,
10469287,9729201,9005544,8300716,7616928,6956192,6320312,5710878,5129262,4576615,4053867,3561728,12399111};

/* ------------------------------------------------------------------
 * Global variables for FIR LPF Example
 * ------------------------------------------------------------------- */

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = FIR_Size/BLOCK_SIZE;

arm_fir_instance_q31 AK_FIR[3];
q31_t  *inputQ31_X, *outputQ31_X, *inputQ31_Y, *outputQ31_Y, *inputQ31_Z, *outputQ31_Z;

uint8_t IIC_ReadBytes(uint8_t dev_addr,uint8_t reg_addr, uint8_t length, uint8_t *pdata)
{
	uint8_t r_value = 0;
	uint8_t read_addr = dev_addr << 1 | 1;
    r_value = HAL_I2C_Mem_Read(&hi2c1, read_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, length, 0xFF);
	return r_value;
}
uint8_t IIC_WriteBytes(uint8_t dev_addr,uint8_t reg_addr, uint8_t length, uint8_t *pdata)
{
	uint8_t r_value = 0;
	uint8_t read_addr = dev_addr << 1 | 0;
    r_value = HAL_I2C_Mem_Write(&hi2c1, read_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, length, 0xFF);
	return r_value;
}

uint8_t IIC_ReadByte(uint8_t dev_addr, uint8_t reg_addr)
{
	uint8_t rx_data = 0;
	uint8_t read_addr = dev_addr << 1 | 1;
	if(HAL_I2C_Mem_Read(&hi2c1, read_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &rx_data, 1, 0xFF)!= HAL_OK)
		return 0xFF;
	else
		return rx_data;	
}
uint8_t IIC_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
	uint8_t r_value = 0;
	uint8_t read_addr = dev_addr << 1 | 0;
    r_value = HAL_I2C_Mem_Write(&hi2c1, read_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xFF);
	return r_value;
}

enum AK09918_mode_type_t AK09918_getMode(void) {
    return AK09918_mode;
}

enum AK09918_err_type_t AK09918_switchMode(enum AK09918_mode_type_t mode) {
    if (mode == AK09918_SELF_TEST) {
        return AK09918_ERR_WRITE_FAILED;
    }
    AK09918_mode = mode;
		IIC_WriteByte(AK09918_I2C_ADDR, AK09918_CNTL2, mode);
    return AK09918_ERR_OK;
}

enum AK09918_err_type_t AK09918_initialize(enum AK09918_mode_type_t mode) {
    if (mode == AK09918_SELF_TEST) {
        mode = AK09918_POWER_DOWN;
    }
    AK09918_mode = mode;

    if (mode == AK09918_NORMAL) {
        return AK09918_ERR_OK;
    } else {
        return AK09918_switchMode(AK09918_mode);
    }
}

enum AK09918_err_type_t AK09918_isDataReady(void) {
		_buffer[0] = IIC_ReadByte(AK09918_I2C_ADDR, AK09918_ST1);
    if (_buffer[0] == 0xFF) {
        return AK09918_ERR_READ_FAILED;
    } else {
        if (_buffer[0] & AK09918_DRDY_BIT) {
            return AK09918_ERR_OK;
        } else {
            return AK09918_ERR_NOT_RDY;
        }
    }
}

enum AK09918_err_type_t AK09918_isDataSkip(void) {
		_buffer[0] = IIC_ReadByte(AK09918_I2C_ADDR, AK09918_ST1);
    if (_buffer[0] == 0xFF) {
        return AK09918_ERR_READ_FAILED;
    } else {
        if (_buffer[0] & AK09918_DOR_BIT) {
            return AK09918_ERR_DOR;
        } else {
            return AK09918_ERR_OK;
        }
    }
}

enum AK09918_err_type_t AK09918_getRawData(int32_t* axis_x, int32_t* axis_y, int32_t* axis_z) {
//    if (AK09918_mode == AK09918_NORMAL) {
//        AK09918_switchMode(AK09918_NORMAL);
//        bool is_end = false;
//        int count = 0;
//        while (!is_end) {
//            if (AK09918_getRawMode() == 0x00) {
//                is_end = true;
//            }
//            if (count >= 15) {
//                return AK09918_ERR_TIMEOUT;
//            }
//            count ++;
//            LL_mDelay(1);
//        }
//    }
		_buffer[0] = IIC_ReadByte(AK09918_I2C_ADDR, AK09918_ST1);
		if (_buffer[0] & AK09918_DRDY_BIT) {
        IIC_ReadBytes(AK09918_I2C_ADDR,AK09918_HXL, 8, _buffer);
				int msbyte1 = (signed char)_buffer[1];
				int msbyte2 = (signed char)_buffer[3];
				int msbyte3 = (signed char)_buffer[5];
		
				*axis_x = (msbyte1 << 8) | (_buffer[0] & 0xFF);
				*axis_y = (msbyte2 << 8) | (_buffer[2] & 0xFF);
				*axis_z = (msbyte3 << 8) | (_buffer[4] & 0xFF);
				if (_buffer[7] & AK09918_HOFL_BIT) {
						return AK09918_ERR_OVERFLOW;
				}
				memset(_buffer,0x00,sizeof(_buffer));
				return AK09918_ERR_OK;    
     } 
		else{
        return AK09918_ERR_NOT_RDY;
     }
}

enum AK09918_err_type_t AK09918_getData(int32_t* axis_x, int32_t* axis_y, int32_t* axis_z) {
    enum AK09918_err_type_t err = AK09918_getRawData(axis_x, axis_y, axis_z);
    (*axis_x) = (double) (*axis_x) * 15.0 / 100.0f;
    (*axis_y) = (double) (*axis_y) * 15.0 / 100.0f;
    (*axis_z) = (double) (*axis_z) * 15.0 / 100.0f;
    return err;
}

// 1.Set Power-down mode. (MODE[4:0] bits = “00000”)
// 2.Set Self-test mode. (MODE[4:0] bits = “10000”)
// 3.Check Data Ready or not by polling DRDY bit of ST1 register.
// 4.When Data Ready, proceed to the next step. Read measurement data. (HXL to HZH)
enum AK09918_err_type_t AK09918_selfTest(void) {
    bool is_end = false;

		IIC_WriteByte(AK09918_I2C_ADDR, AK09918_CNTL2, AK09918_POWER_DOWN);
		LL_mDelay(10);
		IIC_WriteByte(AK09918_I2C_ADDR, AK09918_CNTL2, AK09918_SELF_TEST);
		LL_mDelay(10);
    while (!is_end) {
		AK09918_err = AK09918_isDataReady();
        if (AK09918_err == AK09918_ERR_OK) {
            is_end = true;
        } else if (AK09918_err == AK09918_ERR_READ_FAILED) {
            return AK09918_ERR_READ_FAILED;
        }
        LL_mDelay(10);
    }

    // read data and check
		IIC_ReadBytes(AK09918_I2C_ADDR,AK09918_HXL, 8, _buffer);
		int msbyte1 = (signed char)_buffer[1];
		int msbyte2 = (signed char)_buffer[3];
		int msbyte3 = (signed char)_buffer[5];
		
		ak09918.magX = (msbyte1 << 8) | (_buffer[0] & 0xFF);
		ak09918.magY = (msbyte2 << 8) | (_buffer[2] & 0xFF);
		ak09918.magZ = (msbyte3 << 8) | (_buffer[4] & 0xFF);
		
		if ((ak09918.magX >= -200) && (ak09918.magX <= 200) && (ak09918.magY >= -200) && (ak09918.magY <= 200) && \
						(ak09918.magZ >= -1000) && (ak09918.magZ <= -150)) {
				return AK09918_ERR_OK;
		} else {
				return AK09918_ERR_SELFTEST_FAILED;
		}
}

void AK09918_reset(void) {
		IIC_WriteByte(AK09918_I2C_ADDR, AK09918_CNTL3, AK09918_SRST_BIT);
	  LL_mDelay(10);
}

char* AK09918_strError(void) {
		char* result=(char*)calloc(100,sizeof(char*));
    switch (AK09918_err) {
        case AK09918_ERR_OK:
					result = "AK09918_ERR_OK: OK";
        break;

        case AK09918_ERR_DOR:
            result = "AK09918_ERR_DOR: Data skipped";
            break;

        case AK09918_ERR_NOT_RDY:
            result = "AK09918_ERR_NOT_RDY: Not ready";
            break;

        case AK09918_ERR_TIMEOUT:
            result = "AK09918_ERR_TIMEOUT: Timeout";
            break;

        case AK09918_ERR_SELFTEST_FAILED:
            result = "AK09918_ERR_SELFTEST_FAILED: Self test failed";
            break;

        case AK09918_ERR_OVERFLOW:
            result = "AK09918_ERR_OVERFLOW: Sensor overflow";
            break;

        case AK09918_ERR_WRITE_FAILED:
            result = "AK09918_ERR_WRITE_FAILED: Fail to write";
            break;

        case AK09918_ERR_READ_FAILED:
            result = "AK09918_ERR_READ_FAILED: Fail to read";
            break;

        default:
            result = "Unknown Error";
            break;
    }
    return result;
}

uint16_t AK09918_getDeviceID(void) {
		IIC_ReadBytes(AK09918_I2C_ADDR,AK09918_WIA1, 2, _buffer);
    return (((uint16_t)_buffer[0]) << 8) | _buffer[1];
}

uint8_t AK09918_getRawMode(void) {
		_buffer[0]= IIC_ReadByte(AK09918_I2C_ADDR, AK09918_CNTL2);
		return _buffer[0];
}

void AK09918_Check(void){
	LL_GPIO_ResetOutputPin(GPIOA,GPIO_PIN_8);
	LL_mDelay(200);
	AK09918_reset();
	AK09918_err = AK09918_initialize(AK09918_mode);
	
	AK09918_err = AK09918_selfTest();
	while (AK09918_err != AK09918_ERR_OK) {
			AK09918_err = AK09918_selfTest();
    }
  AK09918_err = AK09918_switchMode(AK09918_POWER_DOWN);
	LL_mDelay(20);
	AK09918_err = AK09918_switchMode(AK09918_CONTINUOUS_100HZ);
	
	ak09918.ID=AK09918_getDeviceID();
	AK09918_err = AK09918_isDataReady();
	while (AK09918_err != AK09918_ERR_OK) {
			LL_mDelay(10);
			AK09918_err = AK09918_isDataReady();
    }
	LL_GPIO_SetOutputPin(GPIOA,GPIO_PIN_8);
	LL_mDelay(200);
	for (uint32_t i = 0; i < FIR_Size; i++)
	{
		AK09918_getRawData(&ak09918.FIR_arrayX[i], &ak09918.FIR_arrayY[i], &ak09918.FIR_arrayZ[i]);
		LL_mDelay(1);
	}
	arm_mean_q31(ak09918.FIR_arrayX, FIR_Size, &ak09918.magX);	// 	Mag_X MAF	
	arm_mean_q31(ak09918.FIR_arrayY, FIR_Size, &ak09918.magY);	// 	Mag_Y MAF	
	arm_mean_q31(ak09918.FIR_arrayZ, FIR_Size, &ak09918.magZ);	// 	Mag_Z MAF	
	
}
void AK09918_FIR_Init(void){
	/* Initialize input and output buffer pointers */
	inputQ31_X = &ak09918.FIR_arrayX[0];
	outputQ31_X = &ak09918.FIR_OutX[0];
	inputQ31_Y = &ak09918.FIR_arrayY[0];
	outputQ31_Y = &ak09918.FIR_OutY[0];
	inputQ31_Z = &ak09918.FIR_arrayZ[0];
	outputQ31_Z = &ak09918.FIR_OutZ[0];
	ak09918.FIR_index = 0;
	/* Call FIR init function to initialize the instance structure. */
	arm_fir_init_q31(&AK_FIR[0], NUM_TAPS, (q31_t *)&firCoeffsQ31_X[0], &firStateQ31_X[0], blockSize);
	arm_fir_init_q31(&AK_FIR[1], NUM_TAPS, (q31_t *)&firCoeffsQ31_Y[0], &firStateQ31_Y[0], blockSize);
	arm_fir_init_q31(&AK_FIR[2], NUM_TAPS, (q31_t *)&firCoeffsQ31_Z[0], &firStateQ31_Z[0], blockSize);
}

void AK09918_RunMAF(void){
	AK09918_err = AK09918_getRawData(&ak09918.FIR_arrayX[ak09918.FIR_index], &ak09918.FIR_arrayY[ak09918.FIR_index], &ak09918.FIR_arrayZ[ak09918.FIR_index]);
	ak09918.FIR_index++;
	if(ak09918.FIR_index >= FIR_Size) ak09918.FIR_index = 0;
	arm_mean_q31(ak09918.FIR_arrayX, FIR_Size, &ak09918.magX);	// 	Mag_X MAF	
	arm_mean_q31(ak09918.FIR_arrayY, FIR_Size, &ak09918.magY);	// 	Mag_Y MAF	
	arm_mean_q31(ak09918.FIR_arrayZ, FIR_Size, &ak09918.magZ);	// 	Mag_Z MAF	
	
}

void AK09918_RunWindow(void) {
	AK09918_err = AK09918_getRawData(&ak09918.FIR_arrayX[ak09918.FIR_index], &ak09918.FIR_arrayY[ak09918.FIR_index], &ak09918.FIR_arrayZ[ak09918.FIR_index]);
	  /* ----------------------------------------------------------------------
  ** Call the FIR process function for every blockSize samples
  ** ------------------------------------------------------------------- */
	for(uint8_t i=0; i < numBlocks; i++)
  {
  arm_fir_fast_q31(&AK_FIR[0], inputQ31_X + (i * blockSize), outputQ31_X + (i * blockSize), blockSize);
	arm_fir_fast_q31(&AK_FIR[1], inputQ31_Y + (i * blockSize), outputQ31_Y + (i * blockSize), blockSize);
	arm_fir_fast_q31(&AK_FIR[2], inputQ31_Z + (i * blockSize), outputQ31_Z + (i * blockSize), blockSize);
	}
	ak09918.magX = ak09918.FIR_OutX[ak09918.FIR_index];
	ak09918.magY = ak09918.FIR_OutY[ak09918.FIR_index];
	ak09918.magZ = ak09918.FIR_OutZ[ak09918.FIR_index];
	ak09918.magDataX = ak09918.magX * 15.0 / 100.0;
	ak09918.magDataY = ak09918.magY * 15.0 / 100.0;
	ak09918.magDataZ = ak09918.magZ * 15.0 / 100.0;
	ak09918.FIR_index++;
	if(ak09918.FIR_index >= FIR_Size) ak09918.FIR_index = 0;
}

