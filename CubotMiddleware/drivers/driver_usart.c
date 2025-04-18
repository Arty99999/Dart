#include "driver_usart.h"

UART_Object uart1;
UART_Object uart2;
UART_Object uart3;
UART_Object uart4;
UART_Object uart5;
UART_Object uart6;
UART_Object uart7;
UART_Object uart8;
int a1;
void UARTx_Init(UART_HandleTypeDef* handle, UART_RxIdleCallback rxIdleCallback)
{	__HAL_UART_CLEAR_IDLEFLAG(handle);
    if(handle->Instance==USART1)
		{
		   uart1.Handle=handle;
			 uart1.RxIdleCallback=rxIdleCallback;
		}
		if(handle->Instance==USART2)
		{
		   uart2.Handle=handle;
			 uart2.RxIdleCallback=rxIdleCallback;
		}
		if(handle->Instance==USART3)
		{
		   uart3.Handle=handle;
			 uart3.RxIdleCallback=rxIdleCallback;
		}
		
			// ��ʼ��uart4
	  if (handle->Instance == UART4)
	  {
	  	uart4.Handle = handle;
	  	uart4.RxIdleCallback = rxIdleCallback;
  	}
	
	// ��ʼ��uart5
  	if (handle->Instance == UART5)
	  {
	  	uart5.Handle = handle;
	  	uart5.RxIdleCallback = rxIdleCallback;
	  }
	
	// ��ʼ��uart6
	  if (handle->Instance == USART6)
	  {
	  	uart6.Handle = handle;
	  	uart6.RxIdleCallback = rxIdleCallback;
	  }
		if (handle->Instance == UART7)
	  {
	  	uart7.Handle = handle;
	  	uart7.RxIdleCallback = rxIdleCallback;
	  }
		if (handle->Instance == UART8)
	  {
	  	uart8.Handle = handle;
	  	uart8.RxIdleCallback = rxIdleCallback;
	  }
		

}

void UART_ENABLE_IT(UART_Object* uart, UART_RxBuffer* rxBuffer)
{//HAL_UART_Receive_DMA(uart->Handle, rxBuffer->Data, rxBuffer->Size);
	__HAL_UART_ENABLE_IT(uart->Handle, UART_IT_IDLE);   		//<�����ж�ǰ����ص��ȽϺ�
} 


void UART_Receive_DMA(UART_Object* uart, UART_RxBuffer* rxBuffer)
{
   HAL_UART_Receive_DMA(uart->Handle, rxBuffer->Data, rxBuffer->Size);


}

/**
  * @brief  �����豸�жϺ�����ִ���ж�DMA���������ô����û��ص����� 
  */
void UART_Idle_Handler(UART_Object* uart, UART_RxBuffer* rxBuffer)
{
	assert_param(uart != NULL);
	
	uint16_t usart_rx_num;
	
	if((__HAL_UART_GET_FLAG(uart->Handle, UART_FLAG_IDLE) != RESET))
	{	
		HAL_UART_DMAStop(uart->Handle);																																					//< �ر�DMA����ֹ������������ݸ��£���ɶ�ʧ����															
		__HAL_UART_CLEAR_IDLEFLAG(uart->Handle);																																//< ���idle��־λ����ֹ�ٴν����ж�			
		__HAL_UART_CLEAR_OREFLAG(uart->Handle);	
//		if ((((DMA_Stream_TypeDef*)uartManage->huart.hdmarx->Instance)->NDTR) == uartManage->rxBufferSize)		//< �жϻ��������� �Ƿ���� DMA��δ�������ݳ��ȡ���Ϊ���ڣ����ݸ�����һ�Ρ� 
		usart_rx_num = rxBuffer->Size - ((DMA_Stream_TypeDef*)uart->Handle->hdmarx->Instance)->NDTR;  			//< 
		{																																																				//< ������ͷ�������������շ���ʼ���պ��Ƿ���Ҫƥ��ͷ�ֽں�β�ֽ�																																														
			if((*uart).RxIdleCallback!=NULL)
			  uart->RxIdleCallback(rxBuffer->Data, usart_rx_num);            				 		   												//<�û��ص�
		}
		
		
		a1=usart_rx_num;
		HAL_UART_DMAResume(uart->Handle);		
		HAL_UART_Receive_DMA(uart->Handle, rxBuffer->Data, rxBuffer->Size);
	}
}


