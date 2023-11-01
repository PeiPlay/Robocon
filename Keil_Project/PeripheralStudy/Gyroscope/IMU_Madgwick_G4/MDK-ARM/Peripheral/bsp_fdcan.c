/*!
 * @file bsp_fdcan.c
 * @brief FDCAN in classic mode
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0
 * @date 2023-03-06
 */

#include "bsp_fdcan.h"
#include "Magic.h"
#include "stm32g4xx_it.h"
#include "arm_math.h"
#include "math.h"
#include "string.h"

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
Comm_t Comm_CAN;

/**
  * @brief  Configures the FDCAN.
  * @param  None
  * @retval None
  */
void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0000;
  sFilterConfig.FilterID2 = 0x0000;
  /*滤波器初始化*/
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* 设置FDCAN1滤波器0全局配置  */
  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Prepare Tx Header */
//  TxHeader.Identifier = 0x321;
  TxHeader.IdType =             FDCAN_STANDARD_ID;
  TxHeader.TxFrameType =        FDCAN_DATA_FRAME;
  TxHeader.DataLength =         FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch =      FDCAN_BRS_OFF;
  TxHeader.FDFormat =           FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
    Error_Handler();
    }

    /* Display LEDx */
    if ((RxHeader.Identifier == 0x321) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_2))
    {
//      LED_Display(RxData[0]);
//      ubKeyNumber = RxData[0];
    }
  }
}

void Fixed_Time_FDCAN(void)
{
		float tempFloat[2];                   //定义的临时变量
#if UINT
    //rad
	    Comm_CAN.roll =     roundf(euler.angle.roll*    RATIO *1000.0f)/100000.f;
		Comm_CAN.pitch =    roundf(euler.angle.pitch*   RATIO *1000.0f)/100000.f;
		Comm_CAN.yaw =      roundf(euler.angle.yaw*     RATIO *1000.0f)/100000.f;
#else
    //angle
        Comm_CAN.roll = euler.angle.roll;
		Comm_CAN.pitch = euler.angle.pitch;
		Comm_CAN.yaw = euler.angle.yaw;
#endif

        TxHeader.Identifier = IMU_STDID + Comm_CAN.send;             /*32位 ID*/
		switch (Comm_CAN.send)
		{

			case 0:
			tempFloat[0] = Comm_CAN.pitch; 
			tempFloat[1] = Comm_CAN.pitch_spd ; 
			memmove(TxData,tempFloat,8);
			Comm_CAN.send = 1;	
			break;
			
			case 1:
			tempFloat[0] = Comm_CAN.yaw; 
			tempFloat[1] = Comm_CAN.yaw_spd; 
			memmove(TxData,tempFloat,8);
			Comm_CAN.send = 2;	
			break;
            
            case 2:
			tempFloat[0] = Comm_CAN.roll; 
			tempFloat[1] = Comm_CAN.roll_spd; 
			memmove(TxData,tempFloat,8);
			Comm_CAN.send = 3;
			break;
            
            case 3:
			tempFloat[0] = Comm_CAN.AccelX; 
			tempFloat[1] = Comm_CAN.AccelY; 
			memmove(TxData,tempFloat,8);
			Comm_CAN.send = 4;
			break;
            
            case 4:
			tempFloat[0] = Comm_CAN.AccelZ; 
//			tempFloat[1] = Comm_CAN.roll_spd; 
			memmove(TxData,tempFloat,8);
			Comm_CAN.send = 0;
			break;
			
			default:
				break;
		}
		 //send three Euler angles and angular accelerations
		/* Start the Transmission process */
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
		{
			/* Transmission request Error */
			Error_Handler();
		}
		
}

void Accel_Filter(void)
{
	if(Comm_CAN.flag == 0)
	{
		Comm_CAN.AX_Sample[Comm_CAN.index] = earth.axis.x;
		Comm_CAN.AY_Sample[Comm_CAN.index] = earth.axis.y;
		Comm_CAN.AZ_Sample[Comm_CAN.index] = earth.axis.z;
		
		Comm_CAN.AX_Offset[Comm_CAN.index] = Comm_CAN.AX_Sample[Comm_CAN.index];
		Comm_CAN.AY_Offset[Comm_CAN.index] = Comm_CAN.AY_Sample[Comm_CAN.index];
		Comm_CAN.AZ_Offset[Comm_CAN.index] = Comm_CAN.AZ_Sample[Comm_CAN.index];
		
		Comm_CAN.index++;
		if(Comm_CAN.index >= SMAF_Size) // Sample Complete
		{
			arm_mean_f32(Comm_CAN.AX_Offset, SMAF_Size, &Comm_CAN.AccelXoffset);	// 	Accel_X_Offset MAF	
			arm_mean_f32(Comm_CAN.AY_Offset, SMAF_Size, &Comm_CAN.AccelYoffset);	// 	Accel_Y_Offset MAF	
			arm_mean_f32(Comm_CAN.AZ_Offset, SMAF_Size, &Comm_CAN.AccelZoffset);	// 	Accel_Z_Offset MAF	
			Comm_CAN.index = 0;
			Comm_CAN.flag = 1;
		}
	}else{
		if(fabs(earth.axis.x-Comm_CAN.AccelXoffset) < 0.01)
		Comm_CAN.AX_Offset[Comm_CAN.index] = earth.axis.x;
		if(fabs(earth.axis.y-Comm_CAN.AccelYoffset) < 0.01)
		Comm_CAN.AY_Offset[Comm_CAN.index] = earth.axis.y;
		if(fabs(earth.axis.z-Comm_CAN.AccelZoffset) < 0.01)
		Comm_CAN.AZ_Offset[Comm_CAN.index] = earth.axis.z;
		
		Comm_CAN.AX_Sample[Comm_CAN.index] = earth.axis.x;
		Comm_CAN.AY_Sample[Comm_CAN.index] = earth.axis.y;
		Comm_CAN.AZ_Sample[Comm_CAN.index] = earth.axis.z;
		
		Comm_CAN.index++;
		if(Comm_CAN.index >= SMAF_Size) Comm_CAN.index = 0;
		
		arm_mean_f32(Comm_CAN.AX_Offset, SMAF_Size, &Comm_CAN.AccelXoffset);	// 	Accel_X_Offset MAF	
		arm_mean_f32(Comm_CAN.AY_Offset, SMAF_Size, &Comm_CAN.AccelYoffset);	// 	Accel_Y_Offset MAF	
		arm_mean_f32(Comm_CAN.AZ_Offset, SMAF_Size, &Comm_CAN.AccelZoffset);	// 	Accel_Z_Offset MAF	
		
		arm_mean_f32(Comm_CAN.AX_Sample, SMAF_Size, &Comm_CAN.AccelX);
		arm_mean_f32(Comm_CAN.AY_Sample, SMAF_Size, &Comm_CAN.AccelY);		
		arm_mean_f32(Comm_CAN.AZ_Sample, SMAF_Size, &Comm_CAN.AccelZ);
		
		Comm_CAN.Trans_Ax = 9.8f*1000.0f*roundf((Comm_CAN.AccelX - Comm_CAN.AccelXoffset)*1000.0f)/1000.0f;
		Comm_CAN.Trans_Ay = 9.8f*1000.0f*roundf((Comm_CAN.AccelY - Comm_CAN.AccelYoffset)*1000.0f)/1000.0f;
		Comm_CAN.Trans_Az = 9.8f*1000.0f*roundf((Comm_CAN.AccelZ - Comm_CAN.AccelZoffset)*1000.0f)/1000.0f;	
	}
}
