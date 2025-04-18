#ifndef DRV_CAN_H_
#define DRV_CAN_H_
#include <stm32h7xx_hal.h>
#include "fdcan.h"
#include "linux_list.h" 

typedef enum 
{
	CAN1 = 0x01U,
	CAN2 = 0x02U
}CanNumber;

typedef struct
{
	FDCAN_RxHeaderTypeDef	Header;
	uint8_t								Data[8];
}CAN_RxBuffer;

typedef uint8_t(*CAN_RxCpltCallback)(CAN_RxBuffer* rxBuffer);

typedef struct
{
	uint32_t							Identifier;
	uint8_t								Data[8];
}CAN_TxBuffer;
typedef struct
{
	uint32_t							Identifier;
	uint8_t								Data[4];
}CAN_TxBuffer1;

typedef struct{
   FDCAN_HandleTypeDef* Handle;
   list_t								DevicesList;
	 CAN_RxCpltCallback		RxCpltCallback;

}CAN_Object;

void CANx_Init(FDCAN_HandleTypeDef* handle, CAN_RxCpltCallback rxCallback);
void CAN_Open(CAN_Object* can);
uint8_t CAN_Send(CAN_Object* can, CAN_TxBuffer* txBuffer);
uint8_t CAN_Send1(CAN_Object* can, CAN_TxBuffer1* txBuffer);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* handle, uint32_t RxFifo0ITs);

extern CAN_Object can1;
extern CAN_Object can2;

#endif








