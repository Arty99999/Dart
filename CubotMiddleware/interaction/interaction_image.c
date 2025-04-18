//#include "interaction_image.h"
//#include "interaction.h"
//#include "string.h"
//#include "usart.h"
//#include "brain.h"
//#include "referee.h"
//#include "driver_timer.h"
//#include "driver_usart.h"
//#include "hardware_config.h"
//#include "control_logic.h"
//#include "ore.h"
//#include "lifting.h"
//#include "dr16.h"
//#include "attitude.h"
//#define image_data_lenth  30   
//custom_robot_data_t Custom;

//unsigned char Usart4_RxBuffer[USART4_RXBUF_SIZE]__attribute__((at(0x24005000)));
//unsigned char Usart4_TxBuffer[USART4_TXBUF_SIZE]__attribute__((at(0x24005800)));
//UART_RxBuffer uart4_buffer={
//	.Data = Usart4_RxBuffer,
//	.Size = USART4_RXBUF_SIZE
//};
///**
//  * @brief  自定义控制器给图传发送数据
//	*/
//void imagetrans_modifydata(Controller_s* controllers,unsigned char* pBuffer)
//{
//	int16_t seq=0;
//	uint16_t crc16_temp;
//	Custom.image_send.image_frame_header.SOF = 0xA5;//数据帧起始字节，固定值为 0xA5
//	Custom.image_send.image_frame_header.data_length=image_data_lenth;//数据帧中自定义data的长度
//	Custom.image_send.image_frame_header.seq = 8;//包序号
//	Custom.image_send.image_frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&Custom.image_send.image_frame_header,4,0xFF);//帧头 CRC8 校验
//	Custom.image_send.cmd_id=0x0302;//自定义控制器图传链路命令码 ID：0x0302
//	//自定义data
//	Custom.image_send.check=0x7A;//这个是随便给的，目的是多加一个校验
//	Custom.image_send.angle_send .yaw=controllers->DP.yaw*1000;
//	Custom.image_send.angle_send.pitch1=controllers->DP.pitch1*1000;
//	Custom.image_send.angle_send.pitch2=controllers->DP.pitch2*1000;
//	Custom.image_send.angle_send.roll1=controllers->DP.roll1*1000;
//	Custom.image_send.angle_send.pitch3=controllers->DP.pitch3*1000;
//	Custom.image_send.angle_send.roll2=controllers->DP.roll2*1000;
//	//帧尾 CRC16 校验
//	crc16_temp = Get_CRC16_Check((unsigned char*)&Custom.image_send, 7+image_data_lenth, 0xFFFF);//7+image_data_lenth为前面所有数据长度
//	Custom.image_send.CRC16[0] = crc16_temp & 0xFF;
//	Custom.image_send.CRC16[1] = crc16_temp >> 8;
//	memcpy(pBuffer,&Custom.image_send,7+image_data_lenth+2);
////	HAL_UART_Transmit_DMA(&huart4,(unsigned char*)&Custom.image_send,7+image_data_lenth+2);//发送全部字节
//}
//void imagetrans_send_data(UART_HandleTypeDef* huart_x,Controller_s* controllers)
//{
//if(huart_x == &huart4)
//	{
//		imagetrans_modifydata(controllers,Usart4_TxBuffer);
//		HAL_UART_Transmit_DMA(huart_x,Usart4_TxBuffer,USART4_TXBUF_SIZE);
//	}

//}
///**
//  * @brief  自定义控制器给图传接收数据
//	*/
//void imagetrans_datas_recv(uint8_t *pdata,custom_robot_data_t* Custom)
//{
//	uint16_t cmd_id,check;
//	int16_t xs,ys,zs;
//	cmd_id=*(pdata+6)<<8|*(pdata+5); 
////	check=*(pdata+7); 
////	if(cmd_id==0x0302&&check==0x7A)
//	if(cmd_id==0x0302)
//	{
//		Custom->image_recv.Coordinate.sx=*(pdata+8)|*(pdata+9)<<8|*(pdata+10)<<16|*(pdata+11)<<24;
//		Custom->image_recv.Coordinate.sy=*(pdata+12)|*(pdata+13)<<8|*(pdata+14)<<16|*(pdata+15)<<24;
//		Custom->image_recv.Coordinate.sz=*(pdata+16)|*(pdata+17)<<8|*(pdata+18)<<16|*(pdata+19)<<24;
//		Custom->image_recv.Coordinate.x=(float)Custom->image_recv.Coordinate.sx/1000;
//		Custom->image_recv.Coordinate.y=(float)Custom->image_recv.Coordinate.sy/1000;
//		Custom->image_recv.Coordinate.z=(float)Custom->image_recv.Coordinate.sz/1000;
//		xs=(pdata[20]|pdata[21]<<8);
//		ys=(pdata[22]|pdata[23]<<8);
//		zs=(pdata[24]|pdata[25]<<8);
//		Custom->image_recv.Coordinate.x_speed=(float)xs/100;
//		Custom->image_recv.Coordinate.y_speed=(float)ys/100;
//		Custom->image_recv.Coordinate.z_speed=(float)zs/100;
//	}
//}
////uint8_t uart4_receive(UART_HandleTypeDef* huart_x,custom_robot_data_t* Custom)
////{
////	
////	if((__HAL_UART_GET_FLAG(huart_x, UART_FLAG_IDLE) != RESET))
////	{ 
////		__HAL_UART_CLEAR_IDLEFLAG(huart_x);	
////		__HAL_UART_CLEAR_OREFLAG(huart_x);
////		HAL_UART_DMAStop(huart_x);			
////		//将数据包数据解到结构体中
////		imagetrans_datas_recv(Usart4_RxBuffer,Custom);		
////		HAL_UART_Receive_DMA(huart_x, Usart4_RxBuffer, USART4_RXBUF_SIZE);
////		HAL_UART_DMAResume(huart_x);//DMA 接收	
////		return 1;
////	}
////}//多余了
//uint8_t imagetrans_fake_callback(uint8_t * recBuffer, uint16_t len)
//{
//	imagetrans_datas_recv(Usart4_RxBuffer,&Custom);	
//	return 0;
//}