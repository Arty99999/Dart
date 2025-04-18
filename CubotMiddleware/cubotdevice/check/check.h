#ifndef __CHECK_H
#define __CHECK_H

#include "stdint.h"
#include "referee.h"
#include "dr16.h"
#include "motor.h"
#include "hardware_config.h"
typedef	 struct 
	{	
	 uint16_t* Online;
	 uint8_t size_Online;
	 uint16_t* Offline;
	 uint8_t size_Offline;
	}Check_Motor;
typedef struct
{
	struct
	{
	  uint16_t Check_receiver;//英文意为接收
	  uint16_t Check_vision;//视觉
	  uint16_t Check_referee;//裁判系统
	}usart_state;
	

	struct
	{
		uint16_t power;
		uint16_t power_limit;
		uint16_t heat;
		uint16_t heat_limit;
		uint16_t power_buffer;
		uint16_t power_buffer_limit;
	}referee_state;
	struct
	{
		uint8_t Receiver;
		uint8_t Vision;
		uint8_t Referee;
	}Online_Flag;
	  Check_Motor Check_Can1;
	Check_Motor Check_Can2;
	
}Check_Robot_State;

typedef struct
{
	uint16_t Vision_FPS;
	uint16_t Receiver_FPS;
	uint16_t Referee_FPS;

	uint16_t Receiver_cnt;
	uint16_t Referee_cnt;
	uint16_t Vision_cnt;

}FPS;

void RobotOnlineState(Check_Robot_State *CheckRobotState, Referee2022 *referee2022,RC_Ctrl *rc_ctrl);
void FPS_Check(FPS * fps);
extern Check_Robot_State check_robot_state;
extern FPS tim14_FPS;

void  Motor_Check(Check_Motor* check,CAN_Object canx) ;

#endif  
