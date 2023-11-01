/*!
 * @file AK09918.h
 * @brief Define the basic structure of magnetometer AK09918
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0
 * @date 2023-03-30
 */

#ifndef AK09918_H
#define AK09918_H

#include "i2c.h"
#include "main.h"

/***************************************************************
    AK09918 I2C Register
 ***************************************************************/
#define AK09918_I2C_ADDR    0x0c    // I2C address (Can't be changed)
#define AK09918_WIA1        0x00    // Company ID
#define AK09918_WIA2        0x01    // Device ID
#define AK09918_RSV1        0x02    // Reserved 1
#define AK09918_RSV2        0x03    // Reserved 2
#define AK09918_ST1         0x10    // DataStatus 1
#define AK09918_HXL         0x11    // X-axis data 
#define AK09918_HXH         0x12
#define AK09918_HYL         0x13    // Y-axis data
#define AK09918_HYH         0x14
#define AK09918_HZL         0x15    // Z-axis data
#define AK09918_HZH         0x16
#define AK09918_TMPS        0x17    // Dummy
#define AK09918_ST2         0x18    // Datastatus 2
#define AK09918_CNTL1       0x30    // Dummy
#define AK09918_CNTL2       0x31    // Control settings
#define AK09918_CNTL3       0x32    // Control settings

#define AK09918_SRST_BIT    0x01    // Soft Reset
#define AK09918_HOFL_BIT    0x08    // Sensor Over Flow
#define AK09918_DOR_BIT     0x02    // Data Over Run
#define AK09918_DRDY_BIT    0x01    // Data Ready

#define FIR_Size 64
#define BLOCK_SIZE            40
#define NUM_TAPS              80

// #define AK09918_MEASURE_PERIOD 9    // Must not be changed
// AK09918 has following seven operation modes:
// (1) Power-down mode: AK09918 doesn't measure
// (2) Single measurement mode: measure when you call any getData() function
// (3) Continuous measurement mode 1: 10Hz, measure 10 times per second,
// (4) Continuous measurement mode 2: 20Hz, measure 20 times per second,
// (5) Continuous measurement mode 3: 50Hz, measure 50 times per second,
// (6) Continuous measurement mode 4: 100Hz, measure 100 times per second,
// (7) Self-test mode
enum AK09918_mode_type_t {
    AK09918_POWER_DOWN = 0x00,
    AK09918_NORMAL = 0x01,
    AK09918_CONTINUOUS_10HZ = 0x02,
    AK09918_CONTINUOUS_20HZ = 0x04,
    AK09918_CONTINUOUS_50HZ = 0x06,
    AK09918_CONTINUOUS_100HZ = 0x08,
    AK09918_SELF_TEST = 0x10, // ignored by switchMode() and initialize(), call selfTest() to use this mode
};

enum AK09918_err_type_t {
    AK09918_ERR_OK = 0,                 // ok
    AK09918_ERR_DOR = 1,                // data skipped
    AK09918_ERR_NOT_RDY = 2,            // not ready
    AK09918_ERR_TIMEOUT = 3,            // read/write timeout
    AK09918_ERR_SELFTEST_FAILED = 4,    // self test failed
    AK09918_ERR_OVERFLOW = 5,           // sensor overflow, means |x|+|y|+|z| >= 4912uT
    AK09918_ERR_WRITE_FAILED = 6,       // fail to write
    AK09918_ERR_READ_FAILED = 7,        // fail to read

};

uint8_t IIC_ReadBytes(uint8_t dev_addr,uint8_t reg_addr, uint8_t length, uint8_t *pdata);
uint8_t IIC_WriteBytes(uint8_t dev_addr,uint8_t reg_addr, uint8_t length, uint8_t *pdata);
uint8_t IIC_ReadByte(uint8_t dev_addr, uint8_t reg_addr);
uint8_t IIC_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

// Return the working mode of AK09918
enum AK09918_mode_type_t AK09918_getMode(void);
// Switch the working mode of AK09918
enum AK09918_err_type_t AK09918_switchMode(enum AK09918_mode_type_t mode);

// default to AK09918_CONTINUOUS_10HZ mode
enum AK09918_err_type_t AK09918_initialize(enum AK09918_mode_type_t mode);
// At AK09918_CONTINUOUS_** mode, check if data is ready to read
enum AK09918_err_type_t AK09918_isDataReady(void);
// At AK09918_CONTINUOUS_** mode, check if data is skipped
enum AK09918_err_type_t AK09918_isDataSkip(void);
// Get raw I2C magnet data
enum AK09918_err_type_t AK09918_getRawData(int32_t* axis_x, int32_t* axis_y, int32_t* axis_z);
// Get magnet data in uT
enum AK09918_err_type_t AK09918_getData(int32_t* axis_x, int32_t* axis_y, int32_t* axis_z);


// Start a self-test, if pass, return AK09918_ERR_OK
enum AK09918_err_type_t AK09918_selfTest(void);
// Reset AK09918
void AK09918_reset(void);
// Get details of AK09918_err_type_t
char* AK09918_strError(void);
// Get device ID
uint16_t AK09918_getDeviceID(void);
uint8_t AK09918_getRawMode(void);

void AK09918_Check(void);
void AK09918_FIR_Init(void);
void AK09918_RunMAF(void);
void AK09918_RunWindow(void);

typedef struct {
	float magDataX;
	float magDataY;
	float magDataZ;
	
	int32_t magX;
	int32_t magY;
	int32_t magZ;
	
	uint32_t FIR_index;

	int32_t FIR_arrayX[FIR_Size];
	int32_t FIR_arrayY[FIR_Size];
	int32_t FIR_arrayZ[FIR_Size];
	
	int32_t FIR_OutX[FIR_Size];
	int32_t FIR_OutY[FIR_Size];
	int32_t FIR_OutZ[FIR_Size];
	
	uint16_t ID;
	int8_t Readreg;
} AK09918_t;

extern enum AK09918_mode_type_t AK09918_mode;
extern enum AK09918_err_type_t AK09918_err;
extern uint8_t _buffer[16];
extern AK09918_t ak09918;

#endif // AK09918_H
