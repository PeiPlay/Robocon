#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__
#include "can.h"
#include "main.h"
//can通信的自定义扩展
// 滤波器编号
#define CAN_FILTERCONFIG_FILTER(x) ((x) << 3)

// 接收邮箱编号
#define CAN_FILTERCONFIG_FIFO_0             (0 << 2)
#define CAN_FILTERCONFIG_FIFO_1             (1 << 2)
// 滤波器接收的标准帧或扩展帧
#define CAN_FILTERCONFIG_ID_STD             (0 << 1)
#define CAN_FILTERCONFIG_ID_EXT             (1 << 1)
// 滤波器接收数据帧或遥控帧
#define CAN_FILTERCONFIG_FRAME_DATA         (0 << 0)
#define CAN_FILTERCONFIG_FRAME_REMOTE       (1 << 0)


// 发送数据帧ID格式
#define CAN_TRANSMIT_ID_STD                 (0 << 0)
#define CAN_TRANSMIT_ID_EXT                 (1 << 0)
// 发送数据帧类型
#define CAN_TRANSMIT_FRAME_DATA             (0 << 1)
#define CAN_TRANSMIT_FRAME_REMOTE           (1 << 1)


#define CAN_TIMEOUT             1000
#define CAN_DEFAULT_DLC         8

typedef struct _CAN_ConfigTypeDef
{
    CAN_HandleTypeDef *hcan;
    uint8_t Filter_Para;
    uint32_t Mask_ID;
    uint32_t ID;
} CAN_ConfigTypeDef;





HAL_StatusTypeDef CAN_Init(CAN_ConfigTypeDef *Config);
HAL_StatusTypeDef CAN_Init_Default(CAN_HandleTypeDef *hcan);

HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *hcan, uint8_t transmit_Para, uint32_t ID, void *pData, uint32_t Len);
HAL_StatusTypeDef CAN_Transmit_STD_DATA(CAN_HandleTypeDef *hcan, uint32_t ID, void *pData, uint32_t Len);
HAL_StatusTypeDef CAN_Transmit_STD_REMOTE(CAN_HandleTypeDef *hcan, uint32_t ID);
HAL_StatusTypeDef CAN_Transmit_EXT_DATA(CAN_HandleTypeDef *hcan, uint32_t ID, void *pData, uint32_t Len);
HAL_StatusTypeDef CAN_Transmit_EXT_REMOTE(CAN_HandleTypeDef *hcan, uint32_t ID);
HAL_StatusTypeDef CAN_Transmit_Default(CAN_HandleTypeDef *hcan, uint32_t ID, void *pData);
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint8_t *pData, uint16_t ID);















#endif

