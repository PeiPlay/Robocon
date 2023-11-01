/*!
 * @file MMC5603NJ.h
 * @brief Define the basic structure of MMC5603NJ class
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0
 * @date 2023-03-10
 */

#ifndef _MMC5603NJ
#define _MMC5603NJ

#include <stdbool.h>
#include <stdint.h>

#define MMC5603NJ_ID 0x10
#define MMC5603NJ_I2C_ADDR 0x30

#define MMC5603NJ_ADDR_XOUT0 0x00
#define MMC5603NJ_ADDR_XOUT1 0x01
#define MMC5603NJ_ADDR_YOUT0 0x02
#define MMC5603NJ_ADDR_YOUT1 0x03
#define MMC5603NJ_ADDR_ZOUT0 0x04
#define MMC5603NJ_ADDR_ZOUT1 0x05
#define MMC5603NJ_ADDR_XOUT2 0x06
#define MMC5603NJ_ADDR_YOUT2 0x07
#define MMC5603NJ_ADDR_ZOUT2 0x08
#define MMC5603NJ_ADDR_TOUT 0x09
#define MMC5603NJ_ADDR_STATUS1 0x18
#define MMC5603NJ_ADDR_ODR 0x1A
#define MMC5603NJ_ADDR_INTCTRL0 0x1B
#define MMC5603NJ_ADDR_INTCTRL1 0x1C
#define MMC5603NJ_ADDR_INTCTRL2 0x1D
#define MMC5603NJ_ADDR_ST_X_TH 0x1E
#define MMC5603NJ_ADDR_ST_Y_TH 0x1F
#define MMC5603NJ_ADDR_ST_Z_TH 0x20
#define MMC5603NJ_ADDR_ST_X 0x27
#define MMC5603NJ_ADDR_ST_Y 0x28
#define MMC5603NJ_ADDR_ST_Z 0x29
#define MMC5603NJ_ADDR_PRODUCTID 0x39

//#define Sensitivity 1000.0/16384.0
#define Sensitivity 0.0625
#define FIR_Size 64
#define BLOCK_SIZE            32
#define NUM_TAPS              64

// Status1 REG
typedef struct {
		uint8_t meas_m_done_int : 1;
		uint8_t maas_t_done_int : 1;
		uint8_t mdt_flag_int : 1;
		uint8_t st_fail : 1;
		uint8_t otp_read_done : 1;
		uint8_t sat_sensor : 1;
		uint8_t meas_m_done : 1;
		uint8_t meas_t_done : 1;
} MMC5603NJ_STATUS1_REG;

// INTERNAL CONTROL 0 REG
typedef struct {
		uint8_t take_meas_m : 1;
		uint8_t take_meas_t : 1;
		uint8_t start_mdt : 1;
		uint8_t do_set : 1;
		uint8_t do_reset : 1;
		uint8_t auto_sr_en : 1;
		uint8_t auto_st_en : 1;
		uint8_t cmm_freq_en : 1;
} MMC5603NJ_INTCTRL0_REG;

// INTERNAL CONTROL 1 REG
typedef struct {
		uint8_t bw : 2;
		uint8_t x_inhibit : 1;
		uint8_t y_inhibit : 1;
		uint8_t z_inhibit : 1;
		uint8_t st_enp : 1;
		uint8_t st_enm : 1;
		uint8_t sw_reset : 1;
} MMC5603NJ_INTCTRL1_REG;

// INTERNAL CONTROL 2 REG
typedef struct {
		uint8_t prd_set : 3;
		uint8_t en_prd_set : 1;
		uint8_t cmm_en : 1;
		uint8_t int_mdt_en : 1;
		uint8_t int_meas_done_en : 1;
		uint8_t hpower : 1;
} MMC5603NJ_INTCTRL2_REG;


typedef struct {
	float magDataX;
	float magDataY;
	float magDataZ;
	float magTemp;
	
	int32_t magX;
	int32_t magY;
	int32_t magZ;
	
	int32_t Xset;
	int32_t Yset;
	int32_t Zset;
	
	int32_t XReset;
	int32_t YReset;
	int32_t ZReset;
	
	int32_t magXH;
	int32_t magYH;
	int32_t magZH;
	
	int32_t magXoffset;
	int32_t magYoffset;
	int32_t magZoffset;
	
	uint8_t ID;
	int8_t Readreg;
} MMC5603_t;

typedef struct
{
	uint32_t FIR_index;

	int32_t FIR_arrayX[FIR_Size];
	int32_t FIR_arrayY[FIR_Size];
	int32_t FIR_arrayZ[FIR_Size];
	
	int32_t FIR_OutX[FIR_Size];
	int32_t FIR_OutY[FIR_Size];
	int32_t FIR_OutZ[FIR_Size];
	
} FIR_t;
/**************************************************************************/
/*!
    @brief  MMC5603NJ Magnetic Sensor driver
*/
/**************************************************************************/
uint8_t IIC_ReadBytes(uint8_t dev_addr,uint8_t reg_addr, uint8_t length, uint8_t *pdata);
uint8_t IIC_WriteBytes(uint8_t dev_addr,uint8_t reg_addr, uint8_t length, uint8_t *pdata);
uint8_t IIC_ReadByte(uint8_t dev_addr, uint8_t reg_addr);
uint8_t IIC_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);


int getMilliGaussX(void);
int getMilliGaussY(void);
int getMilliGaussZ(void);
void getMilliGauss(int32_t *magX, int32_t *magY, int32_t *magZ);

void setContinuousMode(uint8_t odr);
void clearContinuousMode(void); 

uint8_t readProductId(void);
void softwareReset(void);

void Mag_Connection_Check(void);
void Mag_offset(void);
void Mag_FIR_Init(void);
void Mag_runMAF(void);
void Mag_runWindow(void);

extern int32_t _magX;
extern int32_t _magY;
extern int32_t _magZ;

extern MMC5603_t mmc5603;
extern FIR_t MAF;
#endif
