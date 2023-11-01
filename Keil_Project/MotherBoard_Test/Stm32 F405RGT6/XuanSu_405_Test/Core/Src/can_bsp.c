#include "can_bsp.h"



HAL_StatusTypeDef CAN_Init(CAN_ConfigTypeDef *Config)
{
    CAN_FilterTypeDef can_filter_init_structure;

    // 检测关键传参
    assert_param(Config != NULL && hcan != NULL);

    if ((Config->Filter_Para & 0x02))
    {
        // 数据帧
        // 掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = Config->ID << 3 << 16;
        // 掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = Config->ID << 3 | ((Config->Filter_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Config->Mask_ID << 3 << 16;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = Config->Mask_ID << 3 | ((Config->Filter_Para & 0x03) << 1);
    }
    else
    {
        // 遥控帧
        // 掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = Config->ID << 5;
        // 掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ((Config->Filter_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Config->Mask_ID << 5;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = ((Config->Filter_Para & 0x03) << 1);
    }

    // 滤波器序号, 0-27, 共28个滤波器, can1是0~13, can2是14~27
    can_filter_init_structure.FilterBank = Config->Filter_Para >> 3;
    // 滤波器绑定FIFOx, 只能绑定一个
    can_filter_init_structure.FilterFIFOAssignment = (Config->Filter_Para >> 2) & 0x01;
    // 使能滤波器
    can_filter_init_structure.FilterActivation = ENABLE;
    // 滤波器模式, 设置ID掩码模式
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    // 32位滤波
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //从机模式选择开始单元
    can_filter_init_structure.SlaveStartFilterBank = (Config->hcan->Instance == hcan1.Instance) ? 1 : 15;
    HAL_CAN_ConfigFilter(Config->hcan, &can_filter_init_structure);
    HAL_CAN_Start(Config->hcan); //开启CAN

    return HAL_CAN_ActivateNotification(Config->hcan, (Config->Filter_Para & 0x04) ? CAN_IT_RX_FIFO1_MSG_PENDING : CAN_IT_RX_FIFO0_MSG_PENDING);
}
HAL_StatusTypeDef CAN_Init_Default(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //CAN_FILTERMODE_IDLIST  CAN_FILTERMODE_IDMASK
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000; //filter id
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    uint32_t CAN_IT_RX_FIFOx_MSG_PENDING;
    /*这里默认CAN1和CAN2使用不同的FIFO*/
    if (hcan->Instance == hcan1.Instance)
    {
        sFilterConfig.FilterBank = 0;                     //过滤器组
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; //用FIFO接收
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.SlaveStartFilterBank = 1;
        CAN_IT_RX_FIFOx_MSG_PENDING = CAN_IT_RX_FIFO0_MSG_PENDING;
    }
    else if (hcan->Instance == hcan2.Instance)
    {
        sFilterConfig.FilterBank = 15;                    //过滤器组
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1; //用FIFO接收
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.SlaveStartFilterBank = 15;
        CAN_IT_RX_FIFOx_MSG_PENDING = CAN_IT_RX_FIFO1_MSG_PENDING;
    }
    else
    {
        return HAL_ERROR;
    }

    HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
    HAL_CAN_Start(hcan); //开启CAN
    return HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFOx_MSG_PENDING);
}

HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *hcan, uint8_t transmit_Para, uint32_t ID, void *pData, uint32_t Len)
{
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.DLC = Len; 
    TxMessage.RTR = (transmit_Para & 0x2) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    if(transmit_Para & 0x1)
    {
        TxMessage.IDE = CAN_ID_EXT;
        TxMessage.ExtId = ID;
    }
    else
    {
        TxMessage.IDE = CAN_ID_STD;
        TxMessage.StdId = ID;
    }

    //超时跳出
    uint32_t start_time = HAL_GetTick();
    while (!HAL_CAN_GetTxMailboxesFreeLevel(hcan)) 
    {
        if (HAL_GetTick() - start_time > (uint32_t)CAN_TIMEOUT) 
            return HAL_TIMEOUT;
    }
    return HAL_CAN_AddTxMessage(hcan, &TxMessage, (uint8_t*)pData, (uint32_t *)CAN_TX_MAILBOX1);
}
HAL_StatusTypeDef CAN_Transmit_STD_DATA(CAN_HandleTypeDef *hcan, uint32_t ID, void *pData, uint32_t Len)
{
    return CAN_Transmit(hcan, CAN_TRANSMIT_ID_STD | CAN_TRANSMIT_FRAME_DATA, ID, pData, Len);
}
HAL_StatusTypeDef CAN_Transmit_STD_REMOTE(CAN_HandleTypeDef *hcan, uint32_t ID)
{
    return CAN_Transmit(hcan, CAN_TRANSMIT_ID_STD | CAN_TRANSMIT_FRAME_REMOTE, ID, NULL, 0);
}
HAL_StatusTypeDef CAN_Transmit_EXT_DATA(CAN_HandleTypeDef *hcan, uint32_t ID, void *pData, uint32_t Len)
{
    return CAN_Transmit(hcan, CAN_TRANSMIT_ID_EXT | CAN_TRANSMIT_FRAME_DATA, ID, pData, Len);
}
HAL_StatusTypeDef CAN_Transmit_EXT_REMOTE(CAN_HandleTypeDef *hcan, uint32_t ID)
{
    return CAN_Transmit(hcan, CAN_TRANSMIT_ID_EXT | CAN_TRANSMIT_FRAME_REMOTE, ID, NULL, 0);
}
HAL_StatusTypeDef CAN_Transmit_Default(CAN_HandleTypeDef *hcan, uint32_t ID, void *pData)
{
    return CAN_Transmit(hcan, CAN_TRANSMIT_ID_STD | CAN_TRANSMIT_FRAME_DATA, ID, pData, CAN_DEFAULT_DLC);
}


uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint8_t *pData, uint16_t ID)
{
    return CAN_Transmit_Default(hcan, ID, pData);
}



