/*!
 * @file MMC5983MA.c
 * @brief Interface with magnetometer MMC5983MA
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0
 * @date 2023-04-11
 */
 
#include "MMC5983MA.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "arm_math.h"

MMC5983_t mmc5983;

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

void MMC5983MA_SET(void)
{
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x08);
	LL_mDelay(1); // self clearing after 500 us
}


void MMC5983MA_RESET(void)
{
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x10);
  LL_mDelay(1); // self clearing after 500 us
}

void MMC5983MA_powerDown(void)
{
  uint8_t temp = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2); // read register contents
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, temp & ~(0x07));// clear lowest four bits
  LL_mDelay(20); // make sure to finish the last measurement

}


void MMC5983MA_powerUp(uint8_t MODR)
{

  uint8_t temp = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2); // read register contents
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, temp | MODR);
}

uint8_t MMC5983MA_getChipID(void)
{
	
	uint8_t c = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_PRODUCT_ID);
  return c;
}


void MMC5983MA_SWreset(void)
{
  // reset device
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_1, 0x80); // Set bit 7 to 1 to reset MMC5983MA
  LL_mDelay(10); // Wait 10 ms for all registers to reset 
}


void MMC5983MA_init(uint8_t MODR, uint8_t MBW, uint8_t MSET)
{
 // enable data ready interrupt (bit2 == 1), enable auto set/reset (bit 5 == 1)
 // this set/reset is a low current sensor offset measurement for normal use
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x20 | 0x04);
 
 // set magnetometer bandwidth
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_1, MBW);

// enable continuous measurement mode (bit 3 == 1), set sample rate
 // enable automatic Set/Reset (bit 7 == 1), set set/reset rate
 // this set/reset is a high-current "deGaussing" that should be used only to recover from 
 // high magnetic field detuning of the magnetoresistive film
// _i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, 0x80 | (MSET << 4) | 0x08 | MODR);  
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, 0x08 | MODR);
}


void MMC5983MA_selfTest(void)
{
	uint8_t rawData[7] = {0};  // x/y/z mag register data stored here
	uint32_t data_set[3] ={0}, data_reset[3] = {0};
    
   // clear control registers
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x00);
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_1, 0x00);
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, 0x00);

  MMC5983MA_SET(); // enable set current
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x01); //enable one-time mag measurement
  LL_mDelay(10);
   
	IIC_ReadBytes(MMC5983MA_ADDRESS,MMC5983MA_XOUT_0, 7, &rawData[0]); // Read the 6 raw data registers into data array
	data_set[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // x-axis
  data_set[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // y-axis
  data_set[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // z-axis

  MMC5983MA_RESET(); // enable reset current
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x01); //enable one-time mag measurement
  LL_mDelay(10);
   
	IIC_ReadBytes(MMC5983MA_ADDRESS,MMC5983MA_XOUT_0, 7, &rawData[0]); // Read the 6 raw data registers into data array
  data_reset[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // x-axis
  data_reset[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // y-axis
  data_reset[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // z-axis
 
  for (uint8_t ii = 0; ii < 3; ii++)
  {
		if(data_set[ii] > data_reset[ii]) 
   { 
		mmc5983.delta_data[ii] = data_set[ii] - data_reset[ii];
   }
		else
   {
		mmc5983.delta_data[ii] = data_reset[ii] - data_set[ii];
   }
  }
 
  }


void MMC5983MA_getOffset(void)
{
   uint8_t rawData[7] = {0};  // x/y/z mag register data stored here
   uint32_t data_set[3] ={0}, data_reset[3] = {0};
    
   MMC5983MA_powerDown();
 
   MMC5983MA_SET(); // enable set current
   IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x01);  //enable one-time mag measurement
   LL_mDelay(10);
   
   IIC_ReadBytes(MMC5983MA_ADDRESS,MMC5983MA_XOUT_0, 7, &rawData[0]);  // Read the 6 raw data registers into data array
   data_set[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // x-axis
   data_set[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // y-axis
   data_set[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // z-axis

   MMC5983MA_RESET(); // enable reset current
   IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x01);  //enable one-time mag measurement
   LL_mDelay(10);
   
   IIC_ReadBytes(MMC5983MA_ADDRESS,MMC5983MA_XOUT_0, 7, &rawData[0]);  // Read the 6 raw data registers into data array
   data_reset[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // x-axis
   data_reset[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // y-axis
   data_reset[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // z-axis
 
   for (uint8_t ii = 0; ii < 3; ii++)
   {
      mmc5983.magOffset[ii] = (data_set[ii] + data_reset[ii]) >> 2;
   }
}

uint8_t MMC5983MA_status(void)
{
  // Read status register
  uint8_t temp = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_STATUS);   
  return temp;
}


void MMC5983MA_clearInt(void)
{
  // Clear data ready interrupts
  uint8_t temp = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_STATUS);   
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_STATUS, temp & 0x01);
}


void MMC5983MA_readData(void)  
{
	MMC5983MA_clearInt(); // Clear Interrupt
  uint8_t rawData[7];  // x/y/z mag register data stored here
	IIC_ReadBytes(MMC5983MA_ADDRESS,MMC5983MA_XOUT_0, 7, &rawData[0]); // Read the 7 raw data registers into data array
  mmc5983.magX = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // Turn the 18 bits into a unsigned 32-bit value
  mmc5983.magY = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // Turn the 18 bits into a unsigned 32-bit value
  mmc5983.magZ = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // Turn the 18 bits into a unsigned 32-bit value
	mmc5983.magDataX = (mmc5983.magX-mmc5983.magOffset[0]) * Mag_res;
	mmc5983.magDataY = (mmc5983.magY-mmc5983.magOffset[1]) * Mag_res;
	mmc5983.magDataZ = (mmc5983.magZ-mmc5983.magOffset[2]) * Mag_res;
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x20 | 0x04); //Enable Interrupt
	
	
}


uint8_t MMC5983MA_readTemperature(void)
{
  uint8_t temp = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_TOUT);  // Read the raw temperature register 
  return temp;
}

void MMC5983MA_Check(void)
{
	LL_GPIO_ResetOutputPin(GPIOA,GPIO_PIN_8);
	LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_10);//关闭下降沿触发
	LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_10);//关闭中断
	LL_mDelay(200);
	mmc5983.ID = MMC5983MA_getChipID();
	if(mmc5983.ID != MMC5983MA_ID) Error_Handler();
	else{
	LL_GPIO_SetOutputPin(GPIOA,GPIO_PIN_8);
	LL_mDelay(100);
	MMC5983MA_selfTest();
  MMC5983MA_getOffset();

  MMC5983MA_SWreset(); // software reset MMC5983MA to default registers   
	MMC5983MA_clearInt();
  MMC5983MA_SET(); // "deGauss" magnetometer
  MMC5983MA_init(MODR_100Hz, MBW_100Hz, MSET_100);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_10);//关闭下降沿触发
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_10);//关闭中断
	}
}

