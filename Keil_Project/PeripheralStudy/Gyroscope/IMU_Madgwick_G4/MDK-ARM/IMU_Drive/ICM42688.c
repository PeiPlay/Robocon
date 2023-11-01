/*!
 * @file ICM42688.c
 * @brief Define basic structure of ICM42688 class, the implementation of basic method
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0
 * @date 2023-03-03
 */
#include "ICM42688.h"
#include <stdbool.h>
#include <stdio.h>
#include "string.h"
#include "math.h"

uint8_t _r, _g, _b;
uint8_t _mode;
uint8_t _tapNum ;
uint8_t _tapAxis;
uint8_t _tapDir ;
bool FIFOMode;
float _gyroRange;
float _accelRange;
int16_t _accelX;
int16_t _accelY;
int16_t _accelZ;
int16_t _gyroX;
int16_t _gyroZ;
int16_t _gyroY;
int8_t _temp;
int8_t _INTPin;
uint8_t id=0;

sSignalPathReset_t SignalPathReset;
sAccelConfig0_t accelConfig0;
sPWRMgmt0_t PWRMgmt0;
sINTFConfig1_t INTFConfig1;
sAccelConfig1_t accelConfig1;
sGyroAccelConfig0_t gyroAccelConfig0;
sAPEXConfig7_t APEXConfig7;
sAPEXConfig8_t APEXConfig8;
sSMDConfig_t SMDConfig;
sGyroConfig1_t  gyroConfig1;
sFIFOConfig1_t FIFOConfig1;
sINTConfig_t INTConfig;
sGyroConfig0_t gyroConfig0;
sAPEXConfig0_t APEXConfig0;
sGyroConfigStatic9_t gyroConfigStatic9;
sGyroConfigStatic2_t gyroConfigStatic2;
sGyroConfigStatic5_t gyroConfigStatic5;
sAccelConfigStatic2_t accelConfigStatic2;
sAccelConfigStatic4_t accelConfigStatic4;
sINTSource_t  INTSource;

/*
 * @brief   write a byte through SPI and read feedback
 * @param   byte: byte to write
 * @return  received byte
 * */
static uint8_t SPI_WR_Byte(SPI_HandleTypeDef *hspi, uint8_t byte) {
	uint8_t feedback = 0;

	// wait SPI serial free
	while (HAL_SPI_GetState(hspi) == HAL_SPI_STATE_BUSY_TX_RX)
		;

	if (HAL_SPI_TransmitReceive(hspi, &byte, &feedback, 1, 0xff) != HAL_OK) {
		return 0xff;
	}

	return feedback;
}

void writeReg(uint8_t reg, uint8_t* pBuf, size_t size)
{
	
	SPI2_CS_L;
	LL_mDelay(1);
	SPI_WR_Byte(&hspi2, reg);
	
	for (uint8_t i = 0; i < size; i++)
		SPI_WR_Byte(&hspi2, pBuf[i]);
	
	SPI2_CS_H;

}

void readReg(uint8_t reg, uint8_t* pBuf, size_t size)
{
	uint8_t address = reg | 0x80;
	SPI2_CS_L;
//	LL_mDelay(1);
	HAL_SPI_Transmit(&hspi2, &address, 1, 0xff);
	HAL_SPI_Receive(&hspi2, pBuf, size, 0xff);
	SPI2_CS_H;
}

int begin(void)
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
	readReg(ICM42688_WHO_AM_I,&id,1);
  if( id == 0){

    return ERR_DATA_BUS;
  }

  if(id != ICM42688_ID){
    return ERR_IC_VERSION;
  }
  uint8_t reset = 0;
  writeReg(ICM42688_DEVICE_CONFIG,&reset,1);
  LL_mDelay(2);
	_accelX = 0; _accelY = 0; _accelZ = 0;
	_gyroX = 0; _gyroY = 0; _gyroZ = 0;
	LL_mDelay(2);
  return ERR_OK;
}

float getTemperature(void)
{
  float value;
  if(FIFOMode){
    value = (_temp/2.07) + 25;
  } else{
    uint8_t data[2];
    int16_t value2;
    readReg(ICM42688_TEMP_DATA1, data, 2);
    value2 = ((uint16_t )data[0]<<8) | (uint16_t )data[1];
    value = value2/132.48 + 25;
  }
  return value;
}

float getAccelDataX(void)
{
  float value;
  if(FIFOMode){
    value = _accelX;
  } else{
    uint8_t data[2];
    readReg(ICM42688_ACCEL_DATA_X1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*_accelRange;
}

float getAccelDataY(void)
{
  float value;
  if(FIFOMode){
    value = _accelY;
  } else{
    uint8_t data[2];
    readReg(ICM42688_ACCEL_DATA_Y1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*_accelRange;
}

float getAccelDataZ(void)
{
  float value;
  if(FIFOMode){
    value = _accelZ;
  } else{
    uint8_t data[2];
    readReg(ICM42688_ACCEL_DATA_Z1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*_accelRange;
}

float getGyroDataX(void)
{
  float value;
  if(FIFOMode){
    value = _gyroX;
  } else{
    uint8_t data[2];
    readReg(ICM42688_GYRO_DATA_X1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*_gyroRange;
}

float getGyroDataY(void)
{
  float value;
  if(FIFOMode){
    value = _gyroY;
  } else{
    uint8_t data[2];
    readReg(ICM42688_GYRO_DATA_Y1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*_gyroRange;
}

float getGyroDataZ(void)
{
  float value;
  if(FIFOMode){
    value = _gyroZ;
  } else{
    uint8_t data[2];
    readReg(ICM42688_GYRO_DATA_Z1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*_gyroRange;
}

void tapDetectionInit(uint8_t accelMode)
{
  uint8_t bank = 0;
	uint8_t _command = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(accelMode == 0){
		
    accelConfig0.accelODR = 15;
		_command = (accelConfig0.accelFsSel << 5) | (accelConfig0.reserved << 4) | accelConfig0.accelODR;
    writeReg(ICM42688_ACCEL_CONFIG0,&_command,1);
		
    PWRMgmt0.accelMode = 2;
		_command = (PWRMgmt0.reserved << 6) | (PWRMgmt0.tempDis << 5) | (PWRMgmt0.idle << 4) | (PWRMgmt0.gyroMode << 2) |PWRMgmt0.accelMode;
    writeReg(ICM42688_PWR_MGMT0,&_command,1);
    LL_mDelay(1);
			
    INTFConfig1.accelLpClkSel = 0;
		_command = (INTFConfig1.reserved << 4) | (INTFConfig1.accelLpClkSel << 3) | ( INTFConfig1.rtcMode << 2) | INTFConfig1.clksel;
    writeReg(ICM42688_INTF_CONFIG1,&_command,1);
		
    accelConfig1.accelUIFiltORD = 2;
		_command = (accelConfig1.reserved2 << 5) | (accelConfig1.accelUIFiltORD << 3) | ( accelConfig1.accelDec2M2ORD << 1) | accelConfig1.reserved;
    writeReg(ICM42688_ACCEL_CONFIG1,&_command,1);
		
    gyroAccelConfig0.accelUIFiltBW = 0;
		_command = (gyroAccelConfig0.accelUIFiltBW << 4) | gyroAccelConfig0.gyroUIFiltBW;
    writeReg(ICM42688_GYRO_ACCEL_CONFIG0,&_command,1);
		
  } else if(accelMode == 1){
		
    accelConfig0.accelODR = 6;
		_command = (accelConfig0.accelFsSel << 5) | (accelConfig0.reserved << 4) | accelConfig0.accelODR;
    writeReg(ICM42688_ACCEL_CONFIG0,&_command,1);
		
    PWRMgmt0.accelMode = 3;
		_command = (PWRMgmt0.reserved << 6) | (PWRMgmt0.tempDis << 5) | (PWRMgmt0.idle << 4) | (PWRMgmt0.gyroMode << 2) |PWRMgmt0.accelMode;
    writeReg(ICM42688_PWR_MGMT0,&_command,1);
		
    LL_mDelay(1);
		
    accelConfig1.accelUIFiltORD = 2;
		_command = (accelConfig1.reserved2 << 5) | (accelConfig1.accelUIFiltORD << 3) | ( accelConfig1.accelDec2M2ORD << 1) | accelConfig1.reserved;
    writeReg(ICM42688_ACCEL_CONFIG1,&_command,1);
		
    gyroAccelConfig0.accelUIFiltBW = 0;
		_command = (gyroAccelConfig0.accelUIFiltBW << 4) | gyroAccelConfig0.gyroUIFiltBW;
    writeReg(ICM42688_GYRO_ACCEL_CONFIG0,&_command,1);
  } else{
    printf("accelMode invalid !");
    return;
  }
	
  LL_mDelay(1);
  bank = 4;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
	
  APEXConfig8.tapTmin = 3;
  APEXConfig8.tapTavg = 3;
  APEXConfig8.tapTmax = 2;
	_command = (APEXConfig8.reserved << 7) | (APEXConfig8.tapTmax << 5) |(APEXConfig8.tapTavg << 3) |APEXConfig8.tapTmin;
  writeReg(ICM42688_APEX_CONFIG8,&_command,1);
	
  APEXConfig7.tapMinJerkThr = 17;
  APEXConfig7.tapMaxPeakTol = 1;

	_command = (APEXConfig7.tapMinJerkThr << 2) | APEXConfig7.tapMaxPeakTol;
  writeReg(ICM42688_APEX_CONFIG7,&_command,1);
  LL_mDelay(1);
	
  INTSource.tapDetIntEn = 1;
	_command = (INTSource.reserved << 6) | (INTSource.stepDetIntEn << 5)| (INTSource.stepCntOflIntEn << 4)| (INTSource.tiltDetIntEn << 3)|\
	(INTSource.wakeDetIntEn << 2)|(INTSource.sleepDetIntEn << 1)|INTSource.tapDetIntEn;	
	
  if(_INTPin==1){
    writeReg(ICM42688_INT_SOURCE6,&_command,1);
  } else {
    writeReg(ICM42688_INT_SOURCE7,&_command,1);
  }
	
  LL_mDelay(50);
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
	
  APEXConfig0.tapEnable = 1;
	_command = (APEXConfig0.DMPPowerSave << 7) | (APEXConfig0.tapEnable << 6)|(APEXConfig0.PEDEnable << 5)|\
	(APEXConfig0.tiltEnable << 4)|(APEXConfig0.R2WEn << 3)|(APEXConfig0.reserved << 2)|APEXConfig0.dmpODR;
  writeReg(ICM42688_APEX_CONFIG0,&_command,1);
}

void getTapInformation(void)
{
  uint8_t data;
  readReg(ICM42688_APEX_DATA4, &data, 1);
  _tapNum = data & 0x18;
  _tapAxis = data & 0x06;
  _tapDir = data & 0x01;
}
uint8_t numberOfTap(void)
{
  return _tapNum;
}
uint8_t axisOfTap(void)
{
  return _tapAxis;
}
void wakeOnMotionInit(void)
{
  uint8_t bank = 0;
	uint8_t _command = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
	
  accelConfig0.accelODR = 9;
	_command = (accelConfig0.accelFsSel << 5) | (accelConfig0.reserved << 4) | accelConfig0.accelODR;
  writeReg(ICM42688_ACCEL_CONFIG0,&_command,1);
	
  PWRMgmt0.accelMode = 2;
	_command = (PWRMgmt0.reserved << 6) | (PWRMgmt0.tempDis << 5) | (PWRMgmt0.idle << 4) | (PWRMgmt0.gyroMode << 2) |PWRMgmt0.accelMode;
  writeReg(ICM42688_PWR_MGMT0,&_command,1);
	
  LL_mDelay(1);
	
  INTFConfig1.accelLpClkSel = 0;
	_command = (INTFConfig1.reserved << 4) | (INTFConfig1.accelLpClkSel << 3) | ( INTFConfig1.rtcMode << 2) | INTFConfig1.clksel;
  writeReg(ICM42688_INTF_CONFIG1,&_command,1);
  LL_mDelay(1);
}
void setWOMTh(uint8_t axis,uint8_t threshold)
{
  uint8_t bank = 4;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  uint8_t womValue = threshold;
  if(axis == X_AXIS){
    writeReg(ICM42688_ACCEL_WOM_X_THR,&womValue,1);
  } else if(axis == Y_AXIS){
    writeReg(ICM42688_ACCEL_WOM_Y_THR,&womValue,1);
  } else if(axis == Z_AXIS){
    writeReg(ICM42688_ACCEL_WOM_Z_THR,&womValue,1);
  } else if(axis == ALL){
    writeReg(ICM42688_ACCEL_WOM_X_THR,&womValue,1);
    writeReg(ICM42688_ACCEL_WOM_Y_THR,&womValue,1);
    writeReg(ICM42688_ACCEL_WOM_Z_THR,&womValue,1);
  }
  LL_mDelay(1);
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
}
void setWOMInterrupt(uint8_t axis)
{
  uint8_t bank = 0;
	uint8_t _command = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(_INTPin == 1){
    writeReg(ICM42688_INT_SOURCE1,&axis,1);
  } else {
    writeReg(ICM42688_INT_SOURCE4,&axis,1);
  }
  LL_mDelay(50);
	
  SMDConfig.SMDMode = 1;
  SMDConfig.WOMMode = 1;
  SMDConfig.WOMIntMode = 0;
	_command = (SMDConfig.reserved << 4) | (SMDConfig.WOMIntMode << 3) |(SMDConfig.WOMMode << 2)|SMDConfig.SMDMode;
  writeReg(ICM42688_SMD_CONFIG,&_command,1);
}
void enableSMDInterrupt(uint8_t mode)
{
  uint8_t bank = 0;
	uint8_t _command = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  uint8_t INT = 1<<3 ;
  if(mode != 0){
    if(_INTPin == 1){
      writeReg(ICM42688_INT_SOURCE1,&INT,1);
    } else {
      writeReg(ICM42688_INT_SOURCE4,&INT,1);
    }
  }
  LL_mDelay(50);
  SMDConfig.SMDMode = mode;
  SMDConfig.WOMMode = 1;
  SMDConfig.WOMIntMode = 0;
	_command = (SMDConfig.reserved << 4) | (SMDConfig.WOMIntMode << 3) |(SMDConfig.WOMMode << 2)|SMDConfig.SMDMode;
  writeReg(ICM42688_SMD_CONFIG,&_command,1);
}

uint8_t readInterruptStatus(uint8_t reg)
{
  uint8_t bank = 0;
  uint8_t status = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  readReg(reg,&status,1);
  return status;
}

bool setODRAndFSR(uint8_t who,uint8_t ODR,uint8_t FSR)
{
  bool ret = true;
  uint8_t bank = 0;
	uint8_t _command = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(who == GYRO){
    if(ODR > ODR_12_5KHZ || FSR > FSR_7){
      ret = false;
    }else{
      gyroConfig0.gyroODR = ODR;
      gyroConfig0.gyroFsSel = FSR;
			_command = (gyroConfig0.gyroFsSel << 5) | (gyroConfig0.reserved << 4) |gyroConfig0.gyroODR;
      writeReg(ICM42688_GYRO_CONFIG0,&_command,1);
      switch(FSR){
        case FSR_0:
          _gyroRange = 4000/65535.0;
          break;
        case FSR_1:
          _gyroRange = 2000/65535.0;
          break;
        case FSR_2:
          _gyroRange = 1000/65535.0;
          break;
        case FSR_3:
          _gyroRange = 500/65535.0;
          break;
        case FSR_4:
          _gyroRange = 250/65535.0;
          break;
        case FSR_5:
          _gyroRange = 125/65535.0;
          break;
        case FSR_6:
          _gyroRange = 62.5/65535.0;
          break;
        case FSR_7:
          _gyroRange = 31.25/65535.0;
          break;
      }
    }
  } else if(who == ACCEL){
    if(ODR > ODR_500HZ || FSR > FSR_3){
      ret = false;
    } else{
      accelConfig0.accelODR = ODR;
      accelConfig0.accelFsSel = FSR;
			_command = (accelConfig0.accelFsSel << 5) | (accelConfig0.reserved << 4) | accelConfig0.accelODR;
      writeReg(ICM42688_ACCEL_CONFIG0,&_command,1);
      switch(FSR){
        case FSR_0:
          _accelRange = 1/2048.0f;
          break;
        case FSR_1:
          _accelRange = 1/4096.f;
          break;
        case FSR_2:
          _accelRange = 1/8192.0f;
          break;
        case FSR_3:
          _accelRange = 1/16384.0f;
          break;
      }
    }
  } 
  return ret;
}

void setFIFODataMode(void)
{
  uint8_t bank = 0;
	uint8_t _command = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  FIFOConfig1.FIFOHiresEn = 0;
  FIFOConfig1.FIFOAccelEn = 1;
  FIFOConfig1.FIFOGyroEn = 1;
  FIFOConfig1.FIFOTempEn = 1;
  FIFOConfig1.FIFOTmstFsyncEn = 0;
	_command = (FIFOConfig1.reseved << 7) | (FIFOConfig1.FIFOResumeParialRd << 6)|(FIFOConfig1.FIFOWmGtTh << 5)|(FIFOConfig1.FIFOHiresEn << 4)|\
	(FIFOConfig1.FIFOTmstFsyncEn << 3)|(FIFOConfig1.FIFOTempEn << 2) |(FIFOConfig1.FIFOGyroEn << 1)|FIFOConfig1.FIFOAccelEn;
  writeReg(ICM42688_FIFO_CONFIG1,&_command,1);

}

void startFIFOMode(void)
{
  uint8_t bank = 0;
  FIFOMode = true;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  setFIFODataMode();
  uint8_t start = 1<<6;
  writeReg(ICM42688_FIFO_CONFIG,&start,1);
  getFIFOData();
}
void getFIFOData(void)
{
  uint8_t data[16];
	memset(data,0,sizeof(data));
  readReg(ICM42688_FIFO_DATA,data,16);
  _accelX = (uint16_t)data[1]<<8 | (uint16_t)data[2];

  _accelY = (uint16_t)data[3]<<8 | (uint16_t)data[4];

  _accelZ = (uint16_t)data[5]<<8 | (uint16_t)data[6];

  _gyroX = (uint16_t)data[7]<<8 | (uint16_t)data[8];

  _gyroY = (uint16_t)data[9]<<8 | (uint16_t)data[10];

  _gyroZ = (uint16_t)data[11]<<8 | (uint16_t)data[12];

  _temp = (uint8_t)data[13];

}
void stopFIFOMode(void)
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  uint8_t start = 1<<7;
  writeReg(ICM42688_FIFO_CONFIG,&start,1);
}

void setINTMode(uint8_t INTPin,uint8_t INTmode,uint8_t INTPolarity,uint8_t INTDriveCircuit)
{
  uint8_t bank = 0;
	uint8_t _command = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(INTPin == 1){
    _INTPin = 1;
    INTConfig.INT1Mode = INTmode;
    INTConfig.INT1DriveCirCuit = INTDriveCircuit;
    INTConfig.INT1Polarity = INTPolarity;
  } else if(INTPin == 2){
    _INTPin = 2;
    INTConfig.INT2Mode = INTmode;
    INTConfig.INT2DriveCirCuit = INTDriveCircuit;
    INTConfig.INT2Polarity = INTPolarity;
  }

	_command = (INTConfig.reversed << 6) |(INTConfig.INT2Mode << 5)|(INTConfig.INT2DriveCirCuit << 4)|(INTConfig.INT2Polarity << 3)|\
	(INTConfig.INT1Mode << 2)|(INTConfig.INT1DriveCirCuit << 1)|INTConfig.INT1Polarity;
  writeReg(ICM42688_INT_CONFIG,&_command,1);
}

void startTempMeasure(void)
{
  PWRMgmt0.tempDis = 0;
  uint8_t bank = 0;
	uint8_t _command = 0;
	_command = (PWRMgmt0.reserved << 6) | (PWRMgmt0.tempDis << 5) | (PWRMgmt0.idle << 4) | (PWRMgmt0.gyroMode << 2) |PWRMgmt0.accelMode;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  writeReg(ICM42688_PWR_MGMT0,&_command,1);
  LL_mDelay(1);
}
void startGyroMeasure(uint8_t mode)
{
  PWRMgmt0.gyroMode = mode;
  uint8_t bank = 0;
	uint8_t _command = 0;
	_command = (PWRMgmt0.reserved << 6) | (PWRMgmt0.tempDis << 5) | (PWRMgmt0.idle << 4) | (PWRMgmt0.gyroMode << 2) |PWRMgmt0.accelMode;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  writeReg(ICM42688_PWR_MGMT0,&_command,1);
  LL_mDelay(1);
}

void startAccelMeasure(uint8_t mode)
{
  PWRMgmt0.accelMode = mode;
  uint8_t bank = 0;
	uint8_t _command = 0;
	_command = (PWRMgmt0.reserved << 6) | (PWRMgmt0.tempDis << 5) | (PWRMgmt0.idle << 4) | (PWRMgmt0.gyroMode << 2) |PWRMgmt0.accelMode;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  writeReg(ICM42688_PWR_MGMT0,&_command,1);
  LL_mDelay(10);
}
void setGyroNotchFilterFHz(double freq,uint8_t axis)
{
  uint8_t bank = 1;
	uint8_t _command = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  double fdesired = freq * 1000;
  double coswz = cos(2*3.14*fdesired/32);
  uint8_t nfCoswz;
  uint8_t nfCoswzSel;
  if(fabs(coswz)<=0.875){
    nfCoswz = round(coswz*256);
    nfCoswzSel = 0;
  } else {
    nfCoswzSel = 1;
    if(coswz> 0.875){
      nfCoswz = round(8*(1-coswz)*256);
    } else if(coswz < -0.875){
      nfCoswz = round(-8*(1+coswz)*256);
    }
  }
  if(axis == X_AXIS){
    gyroConfigStatic9.gyroNFCoswzSelX = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzX8 = nfCoswz>>8;
		_command = (gyroConfigStatic9.reserved << 6) | (gyroConfigStatic9.gyroNFCoswzSelZ << 5)|(gyroConfigStatic9.gyroNFCoswzSelY << 4)|\
		(gyroConfigStatic9.gyroNFCoswzSelX << 3)|(gyroConfigStatic9.gyroNFCoswzZ8 << 2)|(gyroConfigStatic9.gyroNFCoswzY8 << 1)|gyroConfigStatic9.gyroNFCoswzX8;
    writeReg(ICM42688_GYRO_CONFIG_STATIC6,&nfCoswz,1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC9,&_command,1);
  } else if(axis == Y_AXIS){
    gyroConfigStatic9.gyroNFCoswzSelY = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzY8 = nfCoswz>>8;
    writeReg(ICM42688_GYRO_CONFIG_STATIC7,&nfCoswz,1);
		_command = (gyroConfigStatic9.reserved << 6) | (gyroConfigStatic9.gyroNFCoswzSelZ << 5)|(gyroConfigStatic9.gyroNFCoswzSelY << 4)|\
		(gyroConfigStatic9.gyroNFCoswzSelX << 3)|(gyroConfigStatic9.gyroNFCoswzZ8 << 2)|(gyroConfigStatic9.gyroNFCoswzY8 << 1)|gyroConfigStatic9.gyroNFCoswzX8;
    writeReg(ICM42688_GYRO_CONFIG_STATIC9,&_command,1);
  } else if(axis == Z_AXIS){
    gyroConfigStatic9.gyroNFCoswzSelZ = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzZ8 = nfCoswz>>8;
    writeReg(ICM42688_GYRO_CONFIG_STATIC8,&nfCoswz,1);
		_command = (gyroConfigStatic9.reserved << 6) | (gyroConfigStatic9.gyroNFCoswzSelZ << 5)|(gyroConfigStatic9.gyroNFCoswzSelY << 4)|\
		(gyroConfigStatic9.gyroNFCoswzSelX << 3)|(gyroConfigStatic9.gyroNFCoswzZ8 << 2)|(gyroConfigStatic9.gyroNFCoswzY8 << 1)|gyroConfigStatic9.gyroNFCoswzX8;
    writeReg(ICM42688_GYRO_CONFIG_STATIC9,&_command,1);
  } else if(axis == ALL)
  {
    gyroConfigStatic9.gyroNFCoswzSelX = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzX8 = nfCoswz>>8;
    gyroConfigStatic9.gyroNFCoswzSelY = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzY8 = nfCoswz>>8;
    gyroConfigStatic9.gyroNFCoswzSelZ = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzZ8 = nfCoswz>>8;
    writeReg(ICM42688_GYRO_CONFIG_STATIC6,&nfCoswz,1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC7,&nfCoswz,1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC8,&nfCoswz,1);
		_command = (gyroConfigStatic9.reserved << 6) | (gyroConfigStatic9.gyroNFCoswzSelZ << 5)|(gyroConfigStatic9.gyroNFCoswzSelY << 4)|\
		(gyroConfigStatic9.gyroNFCoswzSelX << 3)|(gyroConfigStatic9.gyroNFCoswzZ8 << 2)|(gyroConfigStatic9.gyroNFCoswzY8 << 1)|gyroConfigStatic9.gyroNFCoswzX8;
    writeReg(ICM42688_GYRO_CONFIG_STATIC9,&_command,1);
  }
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
}

void setGyroNFbandwidth(uint8_t bw)
{
  uint8_t bank = 1;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  uint8_t bandWidth = (bw<<4) | 0x01;
  writeReg(ICM42688_GYRO_CONFIG_STATIC10,&bandWidth,1);
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
}

void setGyroNotchFilter(bool mode)
{
  if(mode){
    gyroConfigStatic2.gyroNFDis = 0;
  } else {
    gyroConfigStatic2.gyroNFDis = 1;
  }
  uint8_t bank = 1;
	uint8_t _command = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);

	_command = (gyroConfigStatic2.reserved << 2) |(gyroConfigStatic2.gyroAAFDis << 1)|gyroConfigStatic2.gyroNFDis;
  writeReg(ICM42688_GYRO_CONFIG_STATIC2,&_command,1);
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
}
void setAAFBandwidth(uint8_t who,uint8_t BWIndex)
{
  uint8_t bank = 0;
	uint8_t _command = 0;
  uint8_t AAFDeltsqr = BWIndex*BWIndex;
  if(who == GYRO){
    bank = 1;
    writeReg(ICM42688_REG_BANK_SEL,&bank,1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC3,&BWIndex,1);
		
    writeReg(ICM42688_GYRO_CONFIG_STATIC4,&AAFDeltsqr,1);
    gyroConfigStatic5.gyroAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      gyroConfigStatic5.gyroAAFBitshift = 15;
    } else if(BWIndex == 2){
      gyroConfigStatic5.gyroAAFBitshift = 13;
    } else if(BWIndex == 3){
      gyroConfigStatic5.gyroAAFBitshift = 12;
    } else if(BWIndex == 4){
      gyroConfigStatic5.gyroAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      gyroConfigStatic5.gyroAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      gyroConfigStatic5.gyroAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      gyroConfigStatic5.gyroAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      gyroConfigStatic5.gyroAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      gyroConfigStatic5.gyroAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      gyroConfigStatic5.gyroAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      gyroConfigStatic5.gyroAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      gyroConfigStatic5.gyroAAFBitshift = 3;
    }
 
		_command = (gyroConfigStatic5.gyroAAFBitshift << 4) |gyroConfigStatic5.gyroAAFDeltsqr;
    writeReg(ICM42688_GYRO_CONFIG_STATIC5,&_command,1);
    bank = 0;
    writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  } else if(who == ACCEL){
    bank = 2;
    writeReg(ICM42688_REG_BANK_SEL,&bank,1);
    accelConfigStatic2.accelAAFDelt = BWIndex;

		_command = (accelConfigStatic2.reserved << 7) |(accelConfigStatic2.accelAAFDelt << 1)|accelConfigStatic2.accelAAFDis;
    writeReg(ICM42688_ACCEL_CONFIG_STATIC2,&_command,1);
    writeReg(ICM42688_ACCEL_CONFIG_STATIC3,&AAFDeltsqr,1);
    accelConfigStatic4.accelAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      accelConfigStatic4.accelAAFBitshift = 15;
    } else if(BWIndex == 2){
      accelConfigStatic4.accelAAFBitshift = 13;
    } else if(BWIndex == 3){
      accelConfigStatic4.accelAAFBitshift = 12;
    } else if(BWIndex == 4){
      accelConfigStatic4.accelAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      accelConfigStatic4.accelAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      accelConfigStatic4.accelAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      accelConfigStatic4.accelAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      accelConfigStatic4.accelAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      accelConfigStatic4.accelAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      accelConfigStatic4.accelAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      accelConfigStatic4.accelAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      accelConfigStatic4.accelAAFBitshift = 3;
    }

		_command = (accelConfigStatic4.accelAAFBitshift << 4) |accelConfigStatic4.accelAAFDeltsqr;
    writeReg(ICM42688_ACCEL_CONFIG_STATIC4,&_command,1);

    bank = 0;
    writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  } else if(who == ALL){
    bank = 1;
    writeReg(ICM42688_REG_BANK_SEL,&bank,1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC3,&BWIndex,1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC4,&AAFDeltsqr,1);
    gyroConfigStatic5.gyroAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      gyroConfigStatic5.gyroAAFBitshift = 15;
    } else if(BWIndex == 2){
      gyroConfigStatic5.gyroAAFBitshift = 13;
    } else if(BWIndex == 3){
      gyroConfigStatic5.gyroAAFBitshift = 12;
    } else if(BWIndex == 4){
      gyroConfigStatic5.gyroAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      gyroConfigStatic5.gyroAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      gyroConfigStatic5.gyroAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      gyroConfigStatic5.gyroAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      gyroConfigStatic5.gyroAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      gyroConfigStatic5.gyroAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      gyroConfigStatic5.gyroAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      gyroConfigStatic5.gyroAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      gyroConfigStatic5.gyroAAFBitshift = 3;
    }
		_command = (gyroConfigStatic5.gyroAAFBitshift << 4) |gyroConfigStatic5.gyroAAFDeltsqr;
    writeReg(ICM42688_GYRO_CONFIG_STATIC5,&_command,1);
		
    bank = 2;
    writeReg(ICM42688_REG_BANK_SEL,&bank,1);
    accelConfigStatic2.accelAAFDelt = BWIndex;
		_command = (accelConfigStatic2.reserved << 7) |(accelConfigStatic2.accelAAFDelt << 1)|accelConfigStatic2.accelAAFDis;
    writeReg(ICM42688_ACCEL_CONFIG_STATIC2,&_command,1);
    writeReg(ICM42688_ACCEL_CONFIG_STATIC3,&AAFDeltsqr,1);
    accelConfigStatic4.accelAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      accelConfigStatic4.accelAAFBitshift = 15;
    } else if(BWIndex == 2){
      accelConfigStatic4.accelAAFBitshift = 13;
    } else if(BWIndex == 3){
      accelConfigStatic4.accelAAFBitshift = 12;
    } else if(BWIndex == 4){
      accelConfigStatic4.accelAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      accelConfigStatic4.accelAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      accelConfigStatic4.accelAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      accelConfigStatic4.accelAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      accelConfigStatic4.accelAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      accelConfigStatic4.accelAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      accelConfigStatic4.accelAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      accelConfigStatic4.accelAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      accelConfigStatic4.accelAAFBitshift = 3;
    }
		_command = (accelConfigStatic4.accelAAFBitshift << 4) |accelConfigStatic4.accelAAFDeltsqr;
    writeReg(ICM42688_ACCEL_CONFIG_STATIC4,&_command,1);
    bank = 0;
    writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  }
}
void setAAF(uint8_t who,bool mode)
{
  uint8_t bank = 0;
	uint8_t _command = 0;
  if(who == GYRO){
    if(mode){
      gyroConfigStatic2.gyroAAFDis = 0;
    } else {
      gyroConfigStatic2.gyroAAFDis = 1;
    }
    bank = 1;
    writeReg(ICM42688_REG_BANK_SEL,&bank,1);
		_command = (gyroConfigStatic2.reserved << 2) |(gyroConfigStatic2.gyroAAFDis << 1)|gyroConfigStatic2.gyroNFDis;
    writeReg(ICM42688_GYRO_CONFIG_STATIC2,&_command,1);
  }else if(who == ACCEL){
    if(mode){
      accelConfigStatic2.accelAAFDis = 0;
    } else {
      accelConfigStatic2.accelAAFDis = 1;
    }
    bank = 2;
    writeReg(ICM42688_REG_BANK_SEL,&bank,1);
		_command = (accelConfigStatic2.reserved << 7) |(accelConfigStatic2.accelAAFDelt << 1)|accelConfigStatic2.accelAAFDis;
    writeReg(ICM42688_ACCEL_CONFIG_STATIC2,&_command,1);
  } else if(who == ALL){
    if(mode){
      gyroConfigStatic2.gyroAAFDis = 0;
      accelConfigStatic2.accelAAFDis = 0;
    } else {
      gyroConfigStatic2.gyroAAFDis = 1;
      accelConfigStatic2.accelAAFDis = 1;
    }
    bank = 1;
    writeReg(ICM42688_REG_BANK_SEL,&bank,1);
		_command = (gyroConfigStatic2.reserved << 2) |(gyroConfigStatic2.gyroAAFDis << 1)|gyroConfigStatic2.gyroNFDis;
    writeReg(ICM42688_GYRO_CONFIG_STATIC2,&_command,1);
    bank = 2;
    writeReg(ICM42688_REG_BANK_SEL,&bank,1);
		_command = (accelConfigStatic2.reserved << 7) |(accelConfigStatic2.accelAAFDelt << 1)|accelConfigStatic2.accelAAFDis;
    writeReg(ICM42688_ACCEL_CONFIG_STATIC2,&_command,1);
  }
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
}

bool setUIFilter(uint8_t who,uint8_t filterOrder ,uint8_t UIFilterIndex)
{
  bool ret = true;
  uint8_t bank = 0;
	uint8_t _command = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(filterOrder > 3 || UIFilterIndex > 15){
    ret = false;
  } else{
    if(who == GYRO){
      gyroConfig1.gyroUIFiltODR = filterOrder;
			_command = (gyroConfig1.agyroFiltBW << 5) |(gyroConfig1.reserved << 4)|(gyroConfig1.gyroUIFiltODR << 2)|\
			gyroConfig1.gyroDec2M2ODR;
      writeReg(ICM42688_GYRO_CONFIG1,&_command,1);

      gyroAccelConfig0.gyroUIFiltBW = UIFilterIndex;
			_command = (gyroAccelConfig0.accelUIFiltBW << 4) |gyroAccelConfig0.gyroUIFiltBW;
      writeReg(ICM42688_GYRO_ACCEL_CONFIG0,&_command,1);
    } else if(who == ACCEL){
      accelConfig1.accelUIFiltORD = filterOrder;
			_command = (accelConfig1.reserved2 << 5) |(accelConfig1.accelUIFiltORD << 3)|(accelConfig1.accelDec2M2ORD << 1)|\
			accelConfig1.reserved;
      writeReg(ICM42688_ACCEL_CONFIG1,&_command,1);
			
      gyroAccelConfig0.accelUIFiltBW = UIFilterIndex;
			_command =  (gyroAccelConfig0.accelUIFiltBW << 4) | gyroAccelConfig0.gyroUIFiltBW;
      writeReg(ICM42688_GYRO_ACCEL_CONFIG0,&_command,1);
    } else if(who == ALL){
      gyroConfig1.gyroUIFiltODR = filterOrder;
			_command = (gyroConfig1.agyroFiltBW << 5) |(gyroConfig1.reserved << 4)|(gyroConfig1.gyroUIFiltODR << 2)|\
			gyroConfig1.gyroDec2M2ODR;
      writeReg(ICM42688_GYRO_CONFIG1,&_command,1);
			
      accelConfig1.accelUIFiltORD = filterOrder;
			_command = (accelConfig1.reserved2 << 5) |(accelConfig1.accelUIFiltORD << 3)|(accelConfig1.accelDec2M2ORD << 1)|\
			accelConfig1.reserved;
      writeReg(ICM42688_ACCEL_CONFIG1,&_command,1);
      gyroAccelConfig0.gyroUIFiltBW = UIFilterIndex;
      gyroAccelConfig0.accelUIFiltBW = UIFilterIndex;
			_command =  (gyroAccelConfig0.accelUIFiltBW << 4) | gyroAccelConfig0.gyroUIFiltBW;
      writeReg(ICM42688_GYRO_ACCEL_CONFIG0,&_command,1);
    }
  }
  return ret;
}

