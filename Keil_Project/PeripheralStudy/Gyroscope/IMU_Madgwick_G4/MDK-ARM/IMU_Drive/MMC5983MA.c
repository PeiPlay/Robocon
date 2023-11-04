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
/*****************************************底层通信函数*****************************************/
//使用IIC进行对MMC5983MA进行寄存器读取（一段数据）
uint8_t IIC_ReadBytes(uint8_t dev_addr,uint8_t reg_addr, uint8_t length, uint8_t *pdata)
{
	uint8_t r_value = 0;                      // 返回值
	uint8_t read_addr = dev_addr << 1 | 1;    //实际读取地址 = 设备地址 + 读取命令位
    r_value = HAL_I2C_Mem_Read(&hi2c1, read_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, length, 0xFF);//读取数据
	return r_value; //返回读取结果
}
//使用IIC进行对MMC5983MA进行寄存器写入（一段数据）
uint8_t IIC_WriteBytes(uint8_t dev_addr,uint8_t reg_addr, uint8_t length, uint8_t *pdata)
{
	uint8_t r_value = 0;
	uint8_t read_addr = dev_addr << 1 | 0;
    r_value = HAL_I2C_Mem_Write(&hi2c1, read_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, length, 0xFF);
	return r_value;
}
//使用IIC进行对MMC5983MA进行寄存器读取（一字节数据）
uint8_t IIC_ReadByte(uint8_t dev_addr, uint8_t reg_addr)
{
	uint8_t rx_data = 0;
	uint8_t read_addr = dev_addr << 1 | 1;
	if(HAL_I2C_Mem_Read(&hi2c1, read_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &rx_data, 1, 0xFF)!= HAL_OK)
		return 0xFF;
	else
		return rx_data;	
}
//使用IIC进行对MMC5983MA进行寄存器写入（一字节数据）
uint8_t IIC_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
	uint8_t r_value = 0;
	uint8_t read_addr = dev_addr << 1 | 0;
    r_value = HAL_I2C_Mem_Write(&hi2c1, read_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xFF);
	return r_value;
}
//MMC5983MA使能，通过向MMC5983MA的MMC5983MA_CONTROL_0寄存器写入0x08来使能
void MMC5983MA_SET(void)
{
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x08);
	LL_mDelay(1); // self clearing after 500 us
}

//MMC5983MA复位，通过向MMC5983MA的MMC5983MA_CONTROL_0寄存器写入0x10来复位
void MMC5983MA_RESET(void)
{
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x10);
  LL_mDelay(1); // self clearing after 500 us
}
//通过将MMC5983MA的MMC5983MA_CONTROL_2寄存器的最低四位写0来关闭MMC5983MA，其中最低四位为MODR
void MMC5983MA_powerDown(void)
{
  uint8_t temp = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2); // read register contents
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, temp & ~(0x07));// clear lowest four bits
  LL_mDelay(20); // make sure to finish the last measurement

}

//通过将MMC5983MA的MMC5983MA_CONTROL_2寄存器的最低四位写MODR来开启MMC5983MA（非0）
void MMC5983MA_powerUp(uint8_t MODR)
{
  uint8_t temp = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2); // read register contents
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, temp | MODR);
}
//获取MMC5983MA的ID号，通过读取MMC5983MA的MMC5983MA_PRODUCT_ID寄存器
uint8_t MMC5983MA_getChipID(void)
{
	
	uint8_t c = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_PRODUCT_ID);
  return c;
}

//软件复位MMC5983MA，通过向MMC5983MA的MMC5983MA_CONTROL_1寄存器的第7位写1来软件复位
void MMC5983MA_SWreset(void)
{
  // reset device
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_1, 0x80); // Set bit 7 to 1 to reset MMC5983MA
  LL_mDelay(10); // Wait 10 ms for all registers to reset 
}

//初始化MMC5983MA
void MMC5983MA_init(uint8_t MODR, uint8_t MBW, uint8_t MSET)
{
 // enable data ready interrupt (bit2 == 1), enable auto set/reset (bit 5 == 1)
 // this set/reset is a low current sensor offset measurement for normal use
 // 使能数据准备中断（bit2 == 1），使能自动设置/复位（bit 5 == 1）。这个设置/复位是一个低电流传感器偏移测量，用于正常使用
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x20 | 0x04);
 
 // set magnetometer bandwidth
 //设置磁力计带宽
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_1, MBW);

// enable continuous measurement mode (bit 3 == 1), set sample rate
 // enable automatic Set/Reset (bit 7 == 1), set set/reset rate
 // this set/reset is a high-current "deGaussing" that should be used only to recover from 
 // high magnetic field detuning of the magnetoresistive film
// _i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, 0x80 | (MSET << 4) | 0x08 | MODR);  
// 使能连续测量模式（bit 3 == 1），设置采样率。使能自动设置/复位（bit 7 == 1），设置设置/复位率。这个设置/复位是一个高电流的“去高斯”，应该只用于从磁场失调的磁电阻薄膜中恢复
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, 0x08 | MODR);
}

//自检
void MMC5983MA_selfTest(void)
{
	uint8_t rawData[7] = {0};  // x/y/z mag register data stored here
	uint32_t data_set[3] ={0}, data_reset[3] = {0};
    
   // clear control registers
   //清除控制寄存器
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x00);
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_1, 0x00);
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, 0x00);

  MMC5983MA_SET(); // enable set current//使能设置电流
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x01); //enable one-time mag measurement//使能一次磁测量
  LL_mDelay(10);//等待测量完成
   
	IIC_ReadBytes(MMC5983MA_ADDRESS,MMC5983MA_XOUT_0, 7, &rawData[0]); // Read the 6 raw data registers into data array
  //将6个原始数据寄存器读入数据数组
	data_set[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // x-axis
  data_set[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // y-axis
  data_set[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // z-axis

  MMC5983MA_RESET(); // enable reset current
  //复位
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x01); //enable one-time mag measurement//使能一次磁测量
  LL_mDelay(10);
   
	IIC_ReadBytes(MMC5983MA_ADDRESS,MMC5983MA_XOUT_0, 7, &rawData[0]); // Read the 6 raw data registers into data array
  data_reset[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // x-axis
  data_reset[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // y-axis
  data_reset[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // z-axis
 
  for (uint8_t ii = 0; ii < 3; ii++)//得到使能和复位的磁力计测量差值
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

//得到磁力计的偏移量？？？？
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

//读取陀螺仪数据，该函数在EXTI外部引脚中断函数中被调用，并且将读取到的数据存入MMC5983MA_t结构体中
void MMC5983MA_readData(void)  
{
	MMC5983MA_clearInt(); // 清除中断
  uint8_t rawData[7];  // 存储磁力计数据
	IIC_ReadBytes(MMC5983MA_ADDRESS,MMC5983MA_XOUT_0, 7, &rawData[0]); // 将磁力计的7个寄存器数据读入rawData数组中
  mmc5983.magX = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // Turn the 18 bits into a unsigned 32-bit value
  mmc5983.magY = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // Turn the 18 bits into a unsigned 32-bit value
  mmc5983.magZ = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // Turn the 18 bits into a unsigned 32-bit value
	mmc5983.magDataX = (mmc5983.magX-mmc5983.magOffset[0]) * Mag_res;
	mmc5983.magDataY = (mmc5983.magY-mmc5983.magOffset[1]) * Mag_res;
	mmc5983.magDataZ = (mmc5983.magZ-mmc5983.magOffset[2]) * Mag_res;
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x20 | 0x04); //设能中断
}

//读取温度函数
uint8_t MMC5983MA_readTemperature(void)
{
  uint8_t temp = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_TOUT);  // Read the raw temperature register 
  return temp;
}

void MMC5983MA_Check(void)
{
	LL_GPIO_ResetOutputPin(GPIOA,GPIO_PIN_8); // 使能磁力计
	LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_10);//禁用外部中断
	LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_10);
	LL_mDelay(200);//延时200ms
	mmc5983.ID = MMC5983MA_getChipID();//获取芯片ID
	if(mmc5983.ID != MMC5983MA_ID) Error_Handler();//如果芯片ID不是0x30，报错
	else{
	LL_GPIO_SetOutputPin(GPIOA,GPIO_PIN_8); // 关闭磁力计
	LL_mDelay(100);                         // 延时100ms
	MMC5983MA_selfTest();                  // 自检
  MMC5983MA_getOffset();               // 获取偏移量

  MMC5983MA_SWreset(); // software reset MMC5983MA to default registers   
	MMC5983MA_clearInt();//清除中断
  MMC5983MA_SET(); // "deGauss" magnetometer//去高斯磁力计
  MMC5983MA_init(MODR_100Hz, MBW_100Hz, MSET_100);//初始化磁力计，MODR_100Hz为100Hz采样率，MBW_100Hz为100Hz带宽，MSET_100为100次测量后设置/复位一次
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_10);//使能外部中断
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_10);//使能外部中断
	}
}

