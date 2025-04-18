//#ifndef INTERACTION_IMAGE_H__
//#define INTERACTION_IMAGE_H__
//#include "stm32h7xx.h"
//#include "cmsis_armcc.h"
//#include "interaction.h"
////#include "hardware_config.h"
//#include "Custom_controller.h"
//#include "driver_usart.h"
//typedef float float32_t;

//#define USART4_RXBUF_SIZE 7+image_data_lenth+3
//#define USART4_TXBUF_SIZE 7+image_data_lenth+2
//typedef __packed struct //自定义结构体
//{
// int sx;
// int sy;
// int sz;
//int16_t x_speed;
//int16_t y_speed;
//int16_t z_speed;
//int16_t yaw;
// int16_t pitch;
// int16_t roll;
//	
//}coordinates_send;
//typedef __packed struct //自定义结构体
//{
//	int yaw;
//	int pitch1;
//	
//	int pitch2;
//	int roll1;
//	int pitch3;
//	int roll2;
//	
//}coordinates_send_1;
//typedef __packed struct //自定义结构体
//{
// int sx;
// int sy;
// int sz;
//float x;
//float y;
//float z;
//float x_speed;
//float y_speed;
//float z_speed;
//float yaw;
//float pitch;
//float roll;
//}coordinates_recv;
//typedef struct
//{
//	__packed struct
//{
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t image_frame_header;  
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	//< 自定义数据(-byte) 
//	uint8_t check;//多加一个校验位，可能没什么用
////	 coordinates_send Coordinate;//坐标
//	coordinates_send_1 angle_send;
//	uint8_t other_data[5];
//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//	
//}image_send;
//	__packed struct
//{
//	coordinates_recv Coordinate;//坐标
//}image_recv;

//}custom_robot_data_t;
//extern  UART_RxBuffer uart4_buffer;
//void imagetrans_modifydata(Controller_s* controllers,unsigned char* pBuffer);
//void imagetrans_send_data(UART_HandleTypeDef* huart_x,Controller_s* controllers);
//uint8_t imagetrans_fake_callback(uint8_t * recBuffer, uint16_t len);
//uint8_t uart4_receive(UART_HandleTypeDef* huart_x,custom_robot_data_t* Custom);
//extern custom_robot_data_t Custom;
//#endif