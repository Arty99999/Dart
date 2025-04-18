#include <driver_can.h>

CAN_Object can1 = {
			.DevicesList = {&(can1.DevicesList),&(can1.DevicesList)}
};

CAN_Object can2= {
			.DevicesList = {&(can2.DevicesList),&(can2.DevicesList)}
};		



void CANx_Init(FDCAN_HandleTypeDef* handle, CAN_RxCpltCallback rxCallback)
{
	//< ��ʼ��can1
	if (handle->Instance == FDCAN1)
	{
		can1.Handle = handle;
		can1.RxCpltCallback = rxCallback;
	}
	
	//< ��ʼ��can2
	if (handle->Instance == FDCAN2)
	{
		can2.Handle = handle;
		can2.RxCpltCallback = rxCallback;
	}
}


/**
  * @brief CAN�豸��ʼ�������ù�����Ϊ�գ�ʹ��fifo0���յ�����Ϣ�ж� ��ע���û��ص�
  */
void CAN_Open(CAN_Object* can) 
{ 
  FDCAN_FilterTypeDef filter;                   	//< �����ֲ����� can�������ṹ��
	filter.IdType       = FDCAN_STANDARD_ID;       	//< id����Ϊ��׼id
	filter.FilterIndex  = 0;                      	//< ��ֵɸѡ���ı�ţ���׼idѡ��0-127
	filter.FilterType   = FDCAN_FILTER_MASK;       	//< ���ù���ģʽΪ����ģʽ
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 	//< ���������˵����ݴ洢�� fifo0
	filter.FilterID1    = 0x000;                   	//< ɸѡ����id
	filter.FilterID2    = 0x000;
	
	HAL_FDCAN_ConfigFilter(can->Handle, &filter);   //< ���ù�����	
	HAL_FDCAN_ActivateNotification(can->Handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // ʹ��fifo0���յ�����Ϣ�ж�
	
  HAL_FDCAN_Start(can->Handle);                   //< ʹ��can
}


/**
  * @brief CAN���ͺ���, ��CAN_Object�µ�txBuffer�е�data���ͳ�ȥ������Ϊ8���ֽ�
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
  * @brief CAN���ͺ���, ��CAN_Object�µ�txBuffer�е�data���ͳ�ȥ.����Ϊ4���ֽ�
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
  * @brief CAN�豸�������ص�
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
