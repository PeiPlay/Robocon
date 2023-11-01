/*!
 * @file MMC5983MA.h
 * @brief Define the basic structure of magnetometer MMC5983MA
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0
 * @date 2023-04-11
 */
#ifndef MMC5983MA_h
#define MMC5983MA_h

#include "main.h"
#include "i2c.h"

//Register map for MMC5983MA'
//http://www.memsic.com/userfiles/files/DataSheets/Magnetic-Sensors-Datasheets/MMC5983MA_Datasheet.pdf
#define MMC5983MA_XOUT_0        0x00
#define MMC5983MA_XOUT_1        0x01
#define MMC5983MA_YOUT_0        0x02
#define MMC5983MA_YOUT_1        0x03
#define MMC5983MA_ZOUT_0        0x04
#define MMC5983MA_ZOUT_1        0x05
#define MMC5983MA_XYZOUT_2      0x06
#define MMC5983MA_TOUT          0x07
#define MMC5983MA_STATUS        0x08
#define MMC5983MA_CONTROL_0     0x09
#define MMC5983MA_CONTROL_1     0x0A
#define MMC5983MA_CONTROL_2     0x0B
#define MMC5983MA_CONTROL_3     0x0C
#define MMC5983MA_PRODUCT_ID    0x2F // Should be 0x30

#define MMC5983MA_ADDRESS       0x30

// Sample rates
#define MODR_ONESHOT   0x00
#define MODR_1Hz       0x01
#define MODR_10Hz      0x02
#define MODR_20Hz      0x03
#define MODR_50Hz      0x04
#define MODR_100Hz     0x05
#define MODR_200Hz     0x06 // BW = 0x01 only
#define MODR_1000Hz    0x07 // BW = 0x11 only

//Bandwidths
#define MBW_100Hz 0x00  // 8 ms measurement time
#define MBW_200Hz 0x01  // 4 ms
#define MBW_400Hz 0x02  // 2 ms
#define MBW_800Hz 0x03  // 0.5 ms


// Set/Reset as a function of measurements
#define MSET_1     0x00 // Set/Reset each data measurement
#define MSET_25    0x01 // each 25 data measurements
#define MSET_75    0x02
#define MSET_100   0x03
#define MSET_250   0x04
#define MSET_500   0x05
#define MSET_1000  0x06
#define MSET_2000  0x07

#define MMC5983MA_ID 0X30
#define Mag_res 1.0f/16384.0f

typedef struct {
	float magDataX;
	float magDataY;
	float magDataZ;
	
	uint32_t magX;
	uint32_t magY;
	uint32_t magZ;
	
	uint32_t magOffset[3];
	
	float Temp;
	uint8_t Tout;
	
	uint32_t delta_data[3];
	
	uint8_t ID;
} MMC5983_t;

uint8_t IIC_ReadBytes(uint8_t dev_addr,uint8_t reg_addr, uint8_t length, uint8_t *pdata);
uint8_t IIC_WriteBytes(uint8_t dev_addr,uint8_t reg_addr, uint8_t length, uint8_t *pdata);
uint8_t IIC_ReadByte(uint8_t dev_addr, uint8_t reg_addr);
uint8_t IIC_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

void MMC5983MA_SET(void);
void MMC5983MA_RESET(void);
void MMC5983MA_powerDown(void);
void MMC5983MA_powerUp(uint8_t MODR);
uint8_t MMC5983MA_getChipID(void);
void MMC5983MA_SWreset(void);
void MMC5983MA_init(uint8_t MODR, uint8_t MBW, uint8_t MSET);
void MMC5983MA_selfTest(void);
void MMC5983MA_getOffset(void);
uint8_t MMC5983MA_status(void);
void MMC5983MA_clearInt(void);
void MMC5983MA_readData(void);
uint8_t MMC5983MA_readTemperature(void);

void MMC5983MA_Check(void);

extern MMC5983_t mmc5983;
#endif
