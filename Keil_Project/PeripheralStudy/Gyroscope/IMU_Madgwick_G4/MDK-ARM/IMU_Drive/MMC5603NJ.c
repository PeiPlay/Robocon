/*!
 * @file MMC5603NJ.c
 * @brief Define basic structure of MMC5603NJ class, the implementation of basic method
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0 2023-03-10 Basic implementation with FIR MAF filter
 * @version V1.1 2023-03-16 Added FIR Window filter (Hamming window Fs=500Hz Fc=200Hz 64-order)
 * @version V1.2 2023-03-23 Higher order FIR Window filter would result in latency and discrepancy of yaw angles.
 * Modified to FIR Window filter (Hamming window Fs=500Hz Fc=10Hz 32-order) with faster algorithm.
 * @note Tests had shown that MAF filter perform better than Window filter. 
 * It is advised to use @function void Mag_runMAF(void) in the @file stm32g4xx_it.c
 */

#include "MMC5603NJ.h"
#include "i2c.h"
#include "main.h"
#include "math.h"
#include "arm_math.h"

extern I2C_HandleTypeDef hi2c1;
MMC5603_t mmc5603;
FIR_t MAF;

MMC5603NJ_STATUS1_REG status1_reg;
MMC5603NJ_INTCTRL0_REG intctrl0_reg;
MMC5603NJ_INTCTRL1_REG intctrl1_reg;
MMC5603NJ_INTCTRL2_REG intctrl2_reg;

int32_t _magX;
int32_t _magY;
int32_t _magZ;
int32_t mag_last[3] = {0};
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
	17005,41538,91984,180985,327939,558135,903289,1401746,2098221,3042991,4290452,5897023,
7918390,10406186,13404241,16944586,21043471,25697683,30881457,36544286,42609888,48976544,
55518926,62091459,68533121,74673504,80339844,85364633,89593371,92891998,95153520,96303407,
96303407,95153520,92891998,89593371,85364633,80339844,74673504,68533121,62091459,55518926,
48976544,42609888,36544286,30881457,25697683,21043471,16944586,13404241,10406186,7918390,
5897023,4290452,3042991,2098221,1401746,903289,558135,327939,180985,91984,41538,17005};

const q31_t firCoeffsQ31_Y[NUM_TAPS] = {
	17005,41538,91984,180985,327939,558135,903289,1401746,2098221,3042991,4290452,5897023,
7918390,10406186,13404241,16944586,21043471,25697683,30881457,36544286,42609888,48976544,
55518926,62091459,68533121,74673504,80339844,85364633,89593371,92891998,95153520,96303407,
96303407,95153520,92891998,89593371,85364633,80339844,74673504,68533121,62091459,55518926,
48976544,42609888,36544286,30881457,25697683,21043471,16944586,13404241,10406186,7918390,
5897023,4290452,3042991,2098221,1401746,903289,558135,327939,180985,91984,41538,17005};

const q31_t firCoeffsQ31_Z[NUM_TAPS] = {
	17005,41538,91984,180985,327939,558135,903289,1401746,2098221,3042991,4290452,5897023,
7918390,10406186,13404241,16944586,21043471,25697683,30881457,36544286,42609888,48976544,
55518926,62091459,68533121,74673504,80339844,85364633,89593371,92891998,95153520,96303407,
96303407,95153520,92891998,89593371,85364633,80339844,74673504,68533121,62091459,55518926,
48976544,42609888,36544286,30881457,25697683,21043471,16944586,13404241,10406186,7918390,
5897023,4290452,3042991,2098221,1401746,903289,558135,327939,180985,91984,41538,17005};

/* ------------------------------------------------------------------
 * Global variables for FIR LPF Example
 * ------------------------------------------------------------------- */

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = FIR_Size/BLOCK_SIZE;

arm_fir_instance_q31 MMC_FIR[3];
q31_t  *inputQ31_X, *outputQ31_X, *inputQ31_Y, *outputQ31_Y, *inputQ31_Z, *outputQ31_Z;


uint8_t IIC_ReadBytes(uint8_t dev_addr,uint8_t reg_addr, uint8_t length, uint8_t *pdata)
{
	uint8_t r_value = 0;
	uint8_t read_addr = dev_addr << 1 | 1;
    r_value = HAL_I2C_Mem_Read(&hi2c1, read_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, length, 0x0F);
	return r_value;
}
uint8_t IIC_WriteBytes(uint8_t dev_addr,uint8_t reg_addr, uint8_t length, uint8_t *pdata)
{
	uint8_t r_value = 0;
	uint8_t read_addr = dev_addr << 1 | 0;
    r_value = HAL_I2C_Mem_Write(&hi2c1, read_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, length, 0x0F);
	return r_value;
}

uint8_t IIC_ReadByte(uint8_t dev_addr, uint8_t reg_addr)
{
	uint8_t rx_data = 0;
	uint8_t read_addr = dev_addr << 1 | 1;
	if(HAL_I2C_Mem_Read(&hi2c1, read_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &rx_data, 1, 0x0F)!= HAL_OK)
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


int getMilliGaussX(void) {
    int32_t mag = 0;
    mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_XOUT0) << 12;
    mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_XOUT1) << 4;
    mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_XOUT2) << 0;
//		mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_XOUT0) << 8;
//    mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_XOUT1) << 0;
	  _magX = mag - mmc5603.magXoffset - mmc5603.magXH;
    return _magX;
}
int getMilliGaussY(void) {
    int32_t mag = 0;
    mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_YOUT0) << 12;
    mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_YOUT1) << 4;
    mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_YOUT2) << 0;
//		mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_YOUT0) << 8;
//    mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_YOUT1) << 0;
	  _magY = mag - mmc5603.magYoffset - mmc5603.magYH;
    return _magY;
}
int getMilliGaussZ(void) {
    int32_t mag = 0;
    mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_ZOUT0) << 12;
    mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_ZOUT1) << 4;
    mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_ZOUT2) << 0;
//		mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_ZOUT0) << 8;
//    mag |= IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_ZOUT1) << 0;
	  _magZ = mag - mmc5603.magZoffset - mmc5603.magZH;
    return _magZ;
}
float getTemp(void){
	float temp;
	uint8_t _command = 0;
	intctrl0_reg.cmm_freq_en = 0; intctrl0_reg.auto_sr_en = 1; intctrl0_reg.take_meas_t = 1; intctrl0_reg.take_meas_m = 0; 
	_command = (intctrl0_reg.cmm_freq_en << 7) |(intctrl0_reg.auto_st_en << 6) | (intctrl0_reg.auto_sr_en << 5)|\
	(intctrl0_reg.do_reset << 4)|(intctrl0_reg.do_set << 3)|(intctrl0_reg.start_mdt << 2)|(intctrl0_reg.take_meas_t << 1)|\
	intctrl0_reg.take_meas_m;
  IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_INTCTRL0, _command);
	
	if( (IIC_ReadByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_STATUS1)) >> 7 == 1)
	{
		temp = IIC_ReadByte(MMC5603NJ_I2C_ADDR,MMC5603NJ_ADDR_TOUT) * 0.8f - 75.0f;
		return temp;
	}else
		return mmc5603.magTemp;
}
void getMilliGauss(int32_t *magX, int32_t *magY, int32_t *magZ) {
    uint8_t mag[9] = {0};
		mmc5603.Readreg = IIC_ReadByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_STATUS1) >> 6;
		if (mmc5603.Readreg == 1 || mmc5603.Readreg == 3){
    IIC_ReadBytes(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_XOUT0, 9, mag);
		*magX = ((mag[0] << 12 | mag[1] << 4 | mag[6]) - mmc5603.magXoffset - mmc5603.magXH);
		mag_last[0] = *magX;
		*magY = ((mag[2] << 12 | mag[3] << 4 | mag[7]) - mmc5603.magYoffset - mmc5603.magYH);
		mag_last[1] = *magY;
		*magZ = ((mag[4] << 12 | mag[5] << 4 | mag[8]) - mmc5603.magZoffset - mmc5603.magZH);
		mag_last[2] = *magZ;
		mmc5603.Readreg = 0;
		}
		else {
			*magX = mag_last[0]; *magY = mag_last[1]; *magZ = mag_last[2];
		}
		
}
void Mag_offset(void){
	uint8_t _command = 0;
	uint8_t mag[9] = {0};
	  intctrl0_reg.take_meas_m = 1; intctrl0_reg.do_set = 1; 
	  _command = (intctrl0_reg.cmm_freq_en << 7) |(intctrl0_reg.auto_st_en << 6) | (intctrl0_reg.auto_sr_en << 5)|\
	(intctrl0_reg.do_reset << 4)|(intctrl0_reg.do_set << 3)|(intctrl0_reg.start_mdt << 2)|(intctrl0_reg.take_meas_t << 1)|\
	intctrl0_reg.take_meas_m;
    IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_INTCTRL0, _command);
	  while(true){
		mmc5603.Readreg = IIC_ReadByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_STATUS1) >> 6;
		LL_mDelay(50);
	  if (mmc5603.Readreg == 1) break;
		}
		IIC_ReadBytes(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_XOUT0, 9, mag);
		mmc5603.Xset = (mag[0] << 12 | mag[1] << 4 | mag[6]);
		mmc5603.Yset = (mag[2] << 12 | mag[3] << 4 | mag[7]);
		mmc5603.Zset = (mag[4] << 12 | mag[5] << 4 | mag[8]);
//		mmc5603.Xset = (mag[0] << 8 | mag[1] << 0);
//		mmc5603.Yset = (mag[2] << 8 | mag[3] << 0);
//		mmc5603.Zset = (mag[4] << 8 | mag[5] << 0);
		
		softwareReset();
		mmc5603.Readreg = 0;
		LL_mDelay(10);
		
		 intctrl0_reg.take_meas_m = 1; intctrl0_reg.do_reset = 1; 
	  _command = (intctrl0_reg.cmm_freq_en << 7) |(intctrl0_reg.auto_st_en << 6) | (intctrl0_reg.auto_sr_en << 5)|\
	(intctrl0_reg.do_reset << 4)|(intctrl0_reg.do_set << 3)|(intctrl0_reg.start_mdt << 2)|(intctrl0_reg.take_meas_t << 1)|\
	intctrl0_reg.take_meas_m;
    IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_INTCTRL0, _command);
	  
	  while(true){
		mmc5603.Readreg = IIC_ReadByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_STATUS1) >> 6;
		LL_mDelay(50);
	  if (mmc5603.Readreg == 1) break;
		}
		IIC_ReadBytes(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_XOUT0, 9, mag);
		mmc5603.XReset = (mag[0] << 12 | mag[1] << 4 | mag[6]);
		mmc5603.YReset = (mag[2] << 12 | mag[3] << 4 | mag[7]);
		mmc5603.ZReset = (mag[4] << 12 | mag[5] << 4 | mag[8]);
//		mmc5603.XReset = (mag[0] << 8 | mag[1] << 0);
//		mmc5603.YReset = (mag[2] << 8 | mag[3] << 0);
//		mmc5603.ZReset = (mag[4] << 8 | mag[5] << 0);
		
		softwareReset();
		LL_mDelay(10);
		
//		mmc5603.magXH = (mmc5603.Xset - mmc5603.XReset) >> 1;
//		mmc5603.magYH = (mmc5603.YReset - mmc5603.Yset) >> 1;
//		mmc5603.magZH = (mmc5603.ZReset - mmc5603.Zset) >> 1;
		mmc5603.magXoffset = (mmc5603.Xset + mmc5603.XReset) >> 1;
		mmc5603.magYoffset = (mmc5603.Yset + mmc5603.YReset) >> 1;
		mmc5603.magZoffset = (mmc5603.Zset + mmc5603.ZReset) >> 1;
}
void setContinuousMode(uint8_t odr) {
	
		uint8_t _command = 0;
    IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_ODR, odr);
	  
	  intctrl0_reg.cmm_freq_en = 1; intctrl0_reg.auto_sr_en = 0; intctrl0_reg.take_meas_t = 1;
	  _command = (intctrl0_reg.cmm_freq_en << 7) |(intctrl0_reg.auto_st_en << 6) | (intctrl0_reg.auto_sr_en << 5)|\
	(intctrl0_reg.do_reset << 4)|(intctrl0_reg.do_set << 3)|(intctrl0_reg.start_mdt << 2)|(intctrl0_reg.take_meas_t << 1)|\
	intctrl0_reg.take_meas_m;
    IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_INTCTRL0, _command);
		
	  intctrl1_reg.bw = 2; intctrl1_reg.sw_reset = 0;
		_command = (intctrl1_reg.sw_reset << 7) |(intctrl1_reg.st_enm << 6) | (intctrl1_reg.st_enp << 5)|\
	(intctrl1_reg.z_inhibit << 4)|(intctrl1_reg.y_inhibit << 3)|(intctrl1_reg.x_inhibit << 2)|intctrl1_reg.bw;
    IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_INTCTRL1, _command);
	
	  intctrl2_reg.hpower = 1; intctrl2_reg.cmm_en = 1; intctrl2_reg.en_prd_set = 0; intctrl2_reg.prd_set = 0;
		_command = (intctrl2_reg.hpower << 7) |(intctrl2_reg.int_meas_done_en << 6) | (intctrl2_reg.int_mdt_en << 5)|\
	(intctrl2_reg.cmm_en << 4)|(intctrl2_reg.en_prd_set << 3)|intctrl2_reg.prd_set;
    IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_INTCTRL2, _command);
}

void clearContinuousMode(void) {
    IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_INTCTRL2, 0x00);
		uint8_t _command = 0;
    IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_ODR, 255);
	  
	  intctrl0_reg.cmm_freq_en = 0; intctrl0_reg.auto_sr_en = 1; intctrl0_reg.take_meas_t = 1; intctrl0_reg.take_meas_m = 1; 
	  _command = (intctrl0_reg.cmm_freq_en << 7) |(intctrl0_reg.auto_st_en << 6) | (intctrl0_reg.auto_sr_en << 5)|\
	(intctrl0_reg.do_reset << 4)|(intctrl0_reg.do_set << 3)|(intctrl0_reg.start_mdt << 2)|(intctrl0_reg.take_meas_t << 1)|\
	intctrl0_reg.take_meas_m;
    IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_INTCTRL0, _command);
		
	  intctrl1_reg.bw = 2; intctrl1_reg.sw_reset = 0;
		_command = (intctrl1_reg.sw_reset << 7) |(intctrl1_reg.st_enm << 6) | (intctrl1_reg.st_enp << 5)|\
	(intctrl1_reg.z_inhibit << 4)|(intctrl1_reg.y_inhibit << 3)|(intctrl1_reg.x_inhibit << 2)|intctrl1_reg.bw;
    IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_INTCTRL1, _command);
	
	  intctrl2_reg.hpower = 1; intctrl2_reg.cmm_en = 0; intctrl2_reg.en_prd_set = 1; intctrl2_reg.prd_set = 1;
		_command = (intctrl2_reg.hpower << 7) |(intctrl2_reg.int_meas_done_en << 6) | (intctrl2_reg.int_mdt_en << 5)|\
	(intctrl2_reg.cmm_en << 4)|(intctrl2_reg.en_prd_set << 3)|intctrl2_reg.prd_set;
    IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_INTCTRL2, _command);
}

uint8_t readProductId(void) {
    return IIC_ReadByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_PRODUCTID);
}

void softwareReset(void) {
    IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_INTCTRL1, 0x80);
}

void Mag_Connection_Check(void) {
	LL_GPIO_ResetOutputPin(GPIOA,GPIO_PIN_8);
	LL_mDelay(500);
	mmc5603.ID=readProductId();
	if(mmc5603.ID != MMC5603NJ_ID) Error_Handler();
	else{
		LL_GPIO_SetOutputPin(GPIOA,GPIO_PIN_8);
		LL_mDelay(100);
		softwareReset();
		Mag_offset();
//		setContinuousMode(255);
		clearContinuousMode();
		for (uint32_t i = 0; i < FIR_Size; i++)
		{
			getMilliGauss(&MAF.FIR_arrayX[i], &MAF.FIR_arrayY[i], &MAF.FIR_arrayZ[i]);
			LL_mDelay(1);
		}
		arm_mean_q31(MAF.FIR_arrayX, FIR_Size, &mmc5603.magX);	// 	Mag_X MAF	
		arm_mean_q31(MAF.FIR_arrayY, FIR_Size, &mmc5603.magY);	// 	Mag_Y MAF	
		arm_mean_q31(MAF.FIR_arrayZ, FIR_Size, &mmc5603.magZ);	// 	Mag_Z MAF	
	}
}
void Mag_FIR_Init(void){
	/* Initialize input and output buffer pointers */
	inputQ31_X = &MAF.FIR_arrayX[0];
	outputQ31_X = &MAF.FIR_OutX[0];
	inputQ31_Y = &MAF.FIR_arrayY[0];
	outputQ31_Y = &MAF.FIR_OutY[0];
	inputQ31_Z = &MAF.FIR_arrayZ[0];
	outputQ31_Z = &MAF.FIR_OutZ[0];
	MAF.FIR_index = 0;
	/* Call FIR init function to initialize the instance structure. */
	arm_fir_init_q31(&MMC_FIR[0], NUM_TAPS, (q31_t *)&firCoeffsQ31_X[0], &firStateQ31_X[0], blockSize);
	arm_fir_init_q31(&MMC_FIR[1], NUM_TAPS, (q31_t *)&firCoeffsQ31_Y[0], &firStateQ31_Y[0], blockSize);
	arm_fir_init_q31(&MMC_FIR[2], NUM_TAPS, (q31_t *)&firCoeffsQ31_Z[0], &firStateQ31_Z[0], blockSize);
}
void On_demand_mode(void){
	uint8_t _command = 0;
	intctrl0_reg.cmm_freq_en = 0; intctrl0_reg.auto_sr_en = 1; intctrl0_reg.take_meas_t = 0; intctrl0_reg.take_meas_m = 1; 
	_command = (intctrl0_reg.cmm_freq_en << 7) |(intctrl0_reg.auto_st_en << 6) | (intctrl0_reg.auto_sr_en << 5)|\
	(intctrl0_reg.do_reset << 4)|(intctrl0_reg.do_set << 3)|(intctrl0_reg.start_mdt << 2)|(intctrl0_reg.take_meas_t << 1)|\
	intctrl0_reg.take_meas_m;
  IIC_WriteByte(MMC5603NJ_I2C_ADDR, MMC5603NJ_ADDR_INTCTRL0, _command);
	MAF.FIR_arrayX[MAF.FIR_index] = getMilliGaussX();
	MAF.FIR_arrayY[MAF.FIR_index] = getMilliGaussY();
	MAF.FIR_arrayZ[MAF.FIR_index] = getMilliGaussZ();
}
void Mag_runMAF(void) {
	On_demand_mode();
//	getMilliGauss(&MAF.FIR_arrayX[MAF.FIR_index], &MAF.FIR_arrayY[MAF.FIR_index], &MAF.FIR_arrayZ[MAF.FIR_index]);
//	mmc5603.magDataZ = MAF.FIR_arrayZ[MAF.FIR_index] * Sensitivity;
	MAF.FIR_index++;
	if(MAF.FIR_index >= FIR_Size) MAF.FIR_index = 0;
	arm_mean_q31(MAF.FIR_arrayX, FIR_Size, &mmc5603.magX);	// 	Mag_X MAF	
	arm_mean_q31(MAF.FIR_arrayY, FIR_Size, &mmc5603.magY);	// 	Mag_Y MAF	
	arm_mean_q31(MAF.FIR_arrayZ, FIR_Size, &mmc5603.magZ);	// 	Mag_Z MAF	
	
	mmc5603.magDataX = mmc5603.magX * Sensitivity;
	mmc5603.magDataY = mmc5603.magY * Sensitivity;
	mmc5603.magDataZ = mmc5603.magZ * Sensitivity;
	mmc5603.magTemp =  getTemp();
}

void Mag_runWindow(void) {
	On_demand_mode();
//	getMilliGauss(&MAF.FIR_arrayX[MAF.FIR_index], &MAF.FIR_arrayY[MAF.FIR_index], &MAF.FIR_arrayZ[MAF.FIR_index]);
	  /* ----------------------------------------------------------------------
  ** Call the FIR process function for every blockSize samples
  ** ------------------------------------------------------------------- */
	for(uint8_t i=0; i < numBlocks; i++)
  {
  arm_fir_fast_q31(&MMC_FIR[0], inputQ31_X + (i * blockSize), outputQ31_X + (i * blockSize), blockSize);
	arm_fir_fast_q31(&MMC_FIR[1], inputQ31_Y + (i * blockSize), outputQ31_Y + (i * blockSize), blockSize);
	arm_fir_fast_q31(&MMC_FIR[2], inputQ31_Z + (i * blockSize), outputQ31_Z + (i * blockSize), blockSize);
	}
	mmc5603.magDataX = MAF.FIR_OutX[MAF.FIR_index] * Sensitivity;
	mmc5603.magDataY = MAF.FIR_OutY[MAF.FIR_index] * Sensitivity;
	mmc5603.magDataZ = MAF.FIR_OutZ[MAF.FIR_index] * Sensitivity;
	mmc5603.magTemp =  getTemp();
	MAF.FIR_index++;
	if(MAF.FIR_index >= FIR_Size) MAF.FIR_index = 0;
}
