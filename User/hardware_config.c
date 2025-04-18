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
Attitude_t attiTude;
MecanumChassis mecanumchassis;
Motors motor2006;
Motors motor6020;
Motors motor3508;

extern  ReloadState_t Reload_state;
uint8_t LCD_callback(uint8_t * recBuffer, uint16_t len);
void HardwareConfig()
{
	DR16Init(&rc_Ctrl);
	
		check_robot_state.usart_state.Check_receiver=30;

	
	
	BasePID_Init(&run_pid,50,0,0,0);
	
	
  BasePID_Init(&Motors3508_SpeedPID,15,0,0,0);
	 BasePID_Init(&Motors3508_AngelPID,1,0,0,0);
	
	  BasePID_Init(&Motors2006_SpeedPID,15,0,0,0);
	 BasePID_Init(&Motors2006_AngelPID,1,0,0,0);
	
	Motors2006_AngelPID.Speed_detach=7000;
	
		  BasePID_Init(&Motors2006_yaw_SpeedPID,15,0,0,0);
	 BasePID_Init(&Motors2006_yaw_AngelPID,1,0,0,0);

	Motors2006_yaw_AngelPID.Speed_detach=7000;
	
		  BasePID_Init(&Motors2006_pitch_SpeedPID,15,0,0,0);
	 BasePID_Init(&Motors2006_pitch_AngelPID,1,0,0,0);
	
	Motors2006_pitch_AngelPID.Speed_detach=7000;
	
			  BasePID_Init(&Motors2006_load_SpeedPID,15,0,0,0);
	 BasePID_Init(&Motors2006_load_AngelPID,1,0,0,0);


	 Motors2006_load_AngelPID.Speed_detach=9000;
	 Motors2006_load_AngelPID.Speed_detach_Negative=-5000;
	 
	 	  BasePID_Init(&Motors6020_pitch_SpeedPID,250,0,0,0);
	 BasePID_Init(&Motors6020_pitch_AngelPID,6,0.01,0,0);
	 Motors6020_pitch_AngelPID.Speed_detach=100;
	 
	UARTx_Init(&huart1,DR16_callback);
	UART_ENABLE_IT(&uart1,&uart1_buffer);
	
	MotorInit(&motor3508.motor[0], 0, Motor3508, CAN2, 0X203);
	MotorInit(&motor2006.motor[0], 0, Motor2006, CAN2, 0X201);//Æ¤½î
	MotorInit(&motor2006.motor[1], 0, Motor2006, CAN2, 0X202);//	Yaw
	MotorInit(&motor2006.motor[2], 0, Motor2006, CAN2, 0X204);//Pitch
	
	MotorInit(&motor2006.motor[3], 0, Motor2006, CAN2, 0X205);//»»µ¯
	MotorInit(&motor6020.motor[0],4268, Motor6020, CAN1, 0X208);//
	
//	MotorInit(&motor2006.motor[3], 0, Motor2006, CAN2, 0X205);//Pitch
  
	//OnlineCheck_Init(can2,&check);
//		MotorInit(&motor2006.motor[3], 0, Motor2006, CAN2, 0X201);
//	MotorInit(&motor3508.motor[0], 0, Motor3508, CAN2, 0X203);
//	MotorInit(&motor6020.motor[0], 0, Motor6020, CAN2, 0X206);
	//DualPID_Init(&motor6020.anglepid[0], pid_angle.ShellPID->Kp,pid_angle.ShellPID->Ki,pid_angle.ShellPID->Kd,pid_angle.CorePID->Kp,pid_angle.CorePID->Ki,pid_angle.CorePID->Kd,CAN2);
//MecanumChassisInit(&mecanumchassis,run_pid,CAN2);

	//UARTx_Init(&huart2,Gyro_callback);  //Referee_callback,Brain_callback
	//UART_ENABLE_IT(&uart2,&uart2_buffer);
	//UART_Receive_DMA(&uart2, &uart2_buffer);

	UARTx_Init(&huart3,Referee_callback);
	UART_ENABLE_IT(&uart3,&uart3_buffer);
	UART_Receive_DMA(&uart3, &uart3_buffer); 

//	UARTx_Init(&huart4,Gyro_callback);
//	UART_ENABLE_IT(&uart4,&uart4_buffer);
//	UART_Receive_DMA(&uart4, &uart4_buffer); 
	//UARTx_Init(&huart5,Gyro_callback);  //Referee_callback,Brain_callback
	//UART_ENABLE_IT(&uart5,&uart5_buffer);
	//UART_Receive_DMA(&uart5, &uart5_buffer);

	//UARTx_Init(&huart6,Gyro_callback);  //Referee_callback,Brain_callback
	//UART_ENABLE_IT(&uart6,&uart6_buffer);
	//UART_Receive_DMA(&uart6, &uart6_buffer);
	UARTx_Init(&huart7,LCD_callback);  //
	UART_ENABLE_IT(&uart7,&uart7_buffer);
	UART_Receive_DMA(&uart7, &uart7_buffer);

UART_Receive_DMA(&uart1, &uart1_buffer);
	//UARTx_Init(&huart8,Gyro_callback);  //
	//UART_ENABLE_IT(&uart8,&uart8_buffer);
	//UART_Receive_DMA(&uart7, &uart7_buffer);





	CANx_Init(&hfdcan1, CAN1_rxCallBack);
	CAN_Open(&can1);

	CANx_Init(&hfdcan2, CAN2_rxCallBack);
	CAN_Open(&can2);

	TIMx_Init(&htim14, TIM14_Task);
	TIM_Open(&tim14);

	TIMx_Init(&htim13, TIM13_Task);
	TIM_Open(&tim13);

//	HAL_TIM_Base_Start(&htim4);
					//UART_Receive_DMA(&uart1, &uart1_buffer);
					//UART_Receive_DMA(&uart2, &uart2_buffer);
//				UART_Receive_DMA(&uart3, &uart3_buffer); 
					//UART_Receive_DMA(&uart5, &uart5_buffer);
					//UART_Receive_DMA(&uart3, &uart3_buffer);
					//UART_Receive_DMA(&uart4, &uart2_buffer);
					//UART_Receive_DMA(&uart8, &uart7_buffer);
//					UART_Receive_DMA(&uart7, &uart7_buffer);

}


	
