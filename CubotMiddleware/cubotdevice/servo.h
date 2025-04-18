//#ifndef _SERVO_H
//#define _SERVO_H
//#include <stm32h7xx_hal.h>
//#include "attitude.h"
//#include<dr16.h>
////#include <brain.h>
//#include "lifting.h"
//typedef struct
//{
//   int16_t ServoPOS_1;
//	 int16_t ServoPOS_2;
//	 int16_t ServoPOS_3;
//   int16_t ServoPOS_1_last;
//	 int16_t ServoPOS_2_last;
//	 int16_t ServoPOS_3_last;
//	 float ServoSpeed;
//	 float ServoTorque;
//	uint32_t servo_lastpos;
//   uint8_t SendData[8];
//   uint8_t mode;
//   
//	struct
//	{
//	  float sideway_dis;
//		float lifting_dis;
//		float protract_dis;
//		float yall1_dis;
//		float pitch2_dis;
//		float roll3_dis;
//	  float Vx_sideway;
//		float Vy_protract;
//	}oremsg;
//	struct
//	{
//	  int16_t fig_yall_target;
//		int16_t fig_pitch_target;

//	}figure;
//}Servo_t;




//void Servo_init(uint8_t id,int16_t pos);
//void Servo_Angle_Control(uint8_t servo_id,int pos);
//void Servo_Param_Setting();
//void Servo_pos_Control(RC_Ctrl *rc_ctrl,Servo_t *servo);
//void Servo_Reset();
//void Servo_Flash_Init();
//void Servo_Flash_Write(Lifting_t *lifting);
//void Servo_Flash_Read(Servo_t *servo);
//uint8_t LobotCheckSum(uint8_t buf[]);
//void LobotSerialServoMove(uint8_t id, uint16_t time,int16_t position);
//void figure_servo_control(RC_Ctrl *rc_ctrl,Servo_t *servo);
//uint8_t feiteservo_callback(uint8_t * recBuffer, uint16_t len);
////void auto_ore_assignment(Servo_t *servo,CubotBrain_t* brain);
////uint8_t servo_callback(uint8_t * recBuffer, uint16_t len);
////void send_message(Servo *ser,uint8_t id);
//extern uint8_t Ser_recBuffer[300];
//extern UART_RxBuffer uart5_buffer;
//extern float move_angle;
//#endif