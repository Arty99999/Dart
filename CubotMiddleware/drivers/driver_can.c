#include <driver_can.h>

CAN_Object can1 = {
			.DevicesList = {&(can1.DevicesList),&(can1.DevicesList)}
};

CAN_Object can2= {
			.DevicesList = {&(can2.DevicesList),&(can2.DevicesList)}
};		



void CANx_Init(FDCAN_HandleTypeDef* handle, CAN_RxCpltCallback rxCallback)
{
	//< 初始化can1
	if (handle->Instance == FDCAN1)
	{
		can1.Handle = handle;
		can1.RxCpltCallback = rxCallback;
	}
	
	//< 初始化can2
	if (handle->Instance == FDCAN2)
	{
		can2.Handle = handle;
		can2.RxCpltCallback = rxCallback;
	}
}


/**
  * @brief CAN设备初始化，配置过滤器为空，使能fifo0接收到新信息中断 ，注册用户回调
  */
void CAN_Open(CAN_Object* can) 
{ 
  FDCAN_FilterTypeDef filter;                   	//< 声明局部变量 can过滤器结构体
	filter.IdType       = FDCAN_STANDARD_ID;       	//< id设置为标准id
	filter.FilterIndex  = 0;                      	//< 设值筛选器的编号，标准id选择0-127
	filter.FilterType   = FDCAN_FILTER_MASK;       	//< 设置工作模式为掩码模式
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 	//< 将经过过滤的数据存储到 fifo0
	filter.FilterID1    = 0x000;                   	//< 筛选器的id
	filter.FilterID2    = 0x000;
	
	HAL_FDCAN_ConfigFilter(can->Handle, &filter);   //< 配置过滤器	
	HAL_FDCAN_ActivateNotification(can->Handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使能fifo0接收到新信息中断
	
  HAL_FDCAN_Start(can->Handle);                   //< 使能can
}


/**
  * @brief CAN发送函数, 将CAN_Object下的txBuffer中的data发送出去。数据为8个字节
  */
uint8_t CAN_Send(CAN_Object* can, CAN_TxBuffer* txBuffer)
{
	FDCAN_TxHeaderTypeDef txHeader;
	txHeader.Identifier = txBuffer->Identifier;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0x00;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(can->Handle, &txHeader, txBuffer->Data) != HAL_OK)
	{
			return 0;
	}
	else
	{
		return 1;
	}		
}
/**
  * @brief CAN发送函数, 将CAN_Object下的txBuffer中的data发送出去.数据为4个字节
  */
uint8_t CAN_Send1(CAN_Object* can, CAN_TxBuffer1* txBuffer)
{
	FDCAN_TxHeaderTypeDef txHeader;
	txHeader.Identifier = txBuffer->Identifier;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_4;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0x00;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(can->Handle, &txHeader, txBuffer->Data) != HAL_OK)
	{
			return 0;
	}
	else
	{
		return 1;
	}		
}


/**
  * @brief CAN设备弱函数回调
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* handle, uint32_t RxFifo0ITs)
{
	CAN_RxBuffer rxBuffer;
	
	if (handle->Instance == FDCAN1)
	{
		if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &rxBuffer.Header, rxBuffer.Data) != HAL_ERROR)
		{			
			can1.RxCpltCallback(&rxBuffer);
		}
	}
	
 if(handle->Instance == FDCAN2)
	{
		if(HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &rxBuffer.Header, rxBuffer.Data) != HAL_ERROR)
		{
			can2.RxCpltCallback(&rxBuffer);
		}
	}
}
