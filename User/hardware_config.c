#include "hardware_config.h"
#include "driver_usart.h"
#include <dr16.h>
#include <control_logic.h>
#include <driver_can.h>
#include <driver_timer.h>
#include <attitude.h>
#include "protract.h"
#include "sideway.h"
#include <brain.h>
#include <et08.h>
#include "mecanum_chassis.h"
#include "Gyro.h"
#include "mpu6050.h"
#include "referee.h"
#include "driver_flash.h"
//#include "referee.h"
BasePID_Object run_pid;
BasePID_Object Motors2006_SpeedPID;
BasePID_Object Motors2006_AngelPID;
BasePID_Object Motors3508_SpeedPID;
BasePID_Object Motors3508_AngelPID;
BasePID_Object Motors2006_yaw_SpeedPID;
BasePID_Object Motors2006_yaw_AngelPID;
BasePID_Object Motors2006_pitch_SpeedPID;
BasePID_Object Motors2006_pitch_AngelPID;
extern  uint8_t Reload_mode;
BasePID_Object Motors6020_pitch_SpeedPID;
BasePID_Object Motors6020_pitch_AngelPID;
 BasePID_Object Motors2006_load_SpeedPID;
 BasePID_Object Motors2006_load_AngelPID;

Motors motor2006;
Motors motor6020;
Motors motor3508;
uint8_t hwt_rec[50]={0};
uint8_t Camera_rec[50]={0};
UART_RxBuffer uart7_buffer={
		.Data = hwt_rec,
		.Size = 50
	};
UART_RxBuffer uart2_buffer={
		.Data = Camera_rec,
		.Size = 50
	};
extern  ReloadState_t Reload_state;
uint8_t LCD_callback(uint8_t * recBuffer, uint16_t len);
void HardwareConfig()
{
	
		check_robot_state.usart_state.Check_receiver=30;

	
	BasePID_Init(&run_pid,50,0,0,0);
	
  BasePID_Init(&Motors3508_SpeedPID,15,0,0,0);
	 BasePID_Init(&Motors3508_AngelPID,1,0,0,0);
	
	  BasePID_Init(&Motors2006_SpeedPID,15,0,0,0);
	 BasePID_Init(&Motors2006_AngelPID,1,0,0,0);
	
	Motors2006_AngelPID.Speed_detach=7000;
	
		  BasePID_Init(&Motors2006_yaw_SpeedPID,15,0,0,0);
	 BasePID_Init(&Motors2006_yaw_AngelPID,1,0,0,0);

	Motors2006_yaw_AngelPID.Speed_detach=7000;//2006位置环限制速度，防止太快丢圈
	
		  BasePID_Init(&Motors2006_pitch_SpeedPID,15,0,0,0);
	 BasePID_Init(&Motors2006_pitch_AngelPID,1,0,0,0);
	
	Motors2006_pitch_AngelPID.Speed_detach=7000;//2006位置环限制速度，防止太快丢圈
	
			  BasePID_Init(&Motors2006_load_SpeedPID,15,0,0,0);
	 BasePID_Init(&Motors2006_load_AngelPID,1,0,0,0);


	 Motors2006_load_AngelPID.Speed_detach=9000;
	 Motors2006_load_AngelPID.Speed_detach_Negative=-6500;//2006位置环限制速度，正反不同
	 
	 	  BasePID_Init(&Motors6020_pitch_SpeedPID,250,0,0,0);
	 BasePID_Init(&Motors6020_pitch_AngelPID,6,0.01,0,0);
	 Motors6020_pitch_AngelPID.Speed_detach=96;
	 
	UARTx_Init(&huart1,ET08_callback);
	UART_ENABLE_IT(&uart1,&uart1_buffer);
	
	MotorInit(&motor3508.motor[0], 0, Motor3508, CAN2, 0X203);
	MotorInit(&motor2006.motor[0], 0, Motor2006, CAN2, 0X201);//皮筋
	MotorInit(&motor2006.motor[1], 0, Motor2006, CAN2, 0X202);//	Yaw
	MotorInit(&motor2006.motor[2], 0, Motor2006, CAN2, 0X204);//Pitch
	
	MotorInit(&motor2006.motor[3], 0, Motor2006, CAN2, 0X205);//换弹
	MotorInit(&motor6020.motor[0],4268, Motor6020, CAN1, 0X208);
	

	UARTx_Init(&huart3,Referee_callback);
	UART_ENABLE_IT(&uart3,&uart3_buffer);
	UART_Receive_DMA(&uart3, &uart3_buffer); 

	UARTx_Init(&huart2,Carema_callback);//上位机
	UART_ENABLE_IT(&uart2,&uart2_buffer);
	UART_Receive_DMA(&uart2, &uart2_buffer);

	UARTx_Init(&huart7,LCD_callback);  //
	UART_ENABLE_IT(&uart7,&uart7_buffer);
	UART_Receive_DMA(&uart7, &uart7_buffer);

UART_Receive_DMA(&uart1, &uart1_buffer);



	CANx_Init(&hfdcan1, CAN1_rxCallBack);
	CAN_Open(&can1);

	CANx_Init(&hfdcan2, CAN2_rxCallBack);
	CAN_Open(&can2);
  HAL_Delay(3000);
	TIMx_Init(&htim14, TIM14_Task);
	TIM_Open(&tim14);

	TIMx_Init(&htim13, TIM13_Task);
	TIM_Open(&tim13);
	

}


	
