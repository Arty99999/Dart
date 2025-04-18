#include "check.h"
#include <driver_timer.h>"
#include "referee.h"
#include "dr16.h"
#include "motor.h"
#define STB_DS_IMPLEMENTATION
#include "stb_ds.h"
Check_Robot_State check_robot_state={
.usart_state.Check_receiver=300
};
FPS tim14_FPS={
.Receiver_cnt= 0,
.Referee_cnt = 0,
.Vision_cnt = 0,
};

/**
  *读取机器人状态，各部分是否在线，
	*/
	
void RobotOnlineState(Check_Robot_State *CheckRobotState, Referee2022 *referee2022,RC_Ctrl *rc_ctrl)
{
	Motor* motor = NULL;
	list_t *node = NULL;	
	CheckRobotState->usart_state.Check_receiver++;
	CheckRobotState->usart_state.Check_vision++;
	CheckRobotState->usart_state.Check_referee++;

	

//	for (node = can2.DevicesList.next;    		//< 对循环链表遍历一圈
//			 node != (can2.DevicesList.prev->next);
//			 node = node->next)
//	{

//		motor = list_entry(node, Motor, list);  //< 输入链表头部所在结点、被嵌入链表的结构体类型、被嵌入链表的结构体类型中链表结点的名称：即可返回嵌入头部所在结点的结构体
//			motor->Data.Online_check.StatusCnt++;
//		if (motor->Data.Online_check.StatusCnt>=30) {motor->Data.Online_check.StatusCnt=30;motor->Data.Online_check.Status=0;}
//			else motor->Data.Online_check.Status=1;
//	}
		for (node = can2.DevicesList.next;    		//< 对循环链表遍历一圈
			 node != (can2.DevicesList.prev->next);
			 node = node->next)
	{
		motor = list_entry(node, Motor, list);  //< 输入链表头部所在结点、被嵌入链表的结构体类型、被嵌入链表的结构体类型中链表结点的名称：即可返回嵌入头部所在结点的结构体
			motor->Data.Online_check.StatusCnt++;
		if (motor->Data.Online_check.StatusCnt>=30) {motor->Data.Online_check.StatusCnt=30;motor->Data.Online_check.Status=0;}
			else motor->Data.Online_check.Status=1;
	}
	for (node = can1.DevicesList.next;    		//< 对循环链表遍历一圈
			 node != (can1.DevicesList.prev->next);
			 node = node->next)
	{
		motor = list_entry(node, Motor, list);  //< 输入链表头部所在结点、被嵌入链表的结构体类型、被嵌入链表的结构体类型中链表结点的名称：即可返回嵌入头部所在结点的结构体
			motor->Data.Online_check.StatusCnt++;
		if (motor->Data.Online_check.StatusCnt>=30) {motor->Data.Online_check.StatusCnt=30;motor->Data.Online_check.Status=0;}
			else motor->Data.Online_check.Status=1;
	}

  Motor_Check(&CheckRobotState->Check_Can1,can1) ;
  Motor_Check(&CheckRobotState->Check_Can2,can2) ;
	
	CheckRobotState->referee_state.heat          = referee2022->power_heat_data.shooter_id1_17mm_cooling_heat;           //枪口当前热量
	CheckRobotState->referee_state.heat_limit    = referee2022->game_robot_status.shooter_id1_17mm_cooling_limit;        //枪口热量限制
	CheckRobotState->referee_state.power         = referee2022->power_heat_data.chassis_power;                           //底盘当前功率
	CheckRobotState->referee_state.power_limit   = referee2022->game_robot_status.chassis_power_limit;                   //底盘功率限制
	CheckRobotState->referee_state.power_buffer  = referee2022->power_heat_data.chassis_power_buffer;                    //底盘当前缓冲能量
//	CheckRobotState->referee_state.power_buffer_limit = referee2022->game_robot_status.                                //底盘缓冲能量限制（暂未读到）

	if(CheckRobotState->usart_state.Check_vision > 200) CheckRobotState->usart_state.Check_vision = 200;  
	if(check_robot_state.usart_state.Check_vision>100)  { CheckRobotState->Online_Flag.Vision= 0 ;} else CheckRobotState->Online_Flag.Vision= 1;
	

			if(CheckRobotState->usart_state.Check_referee >200) CheckRobotState->usart_state.Check_referee = 200;  
	if(check_robot_state.usart_state.Check_referee>100)  {CheckRobotState->Online_Flag.Referee=0;} else CheckRobotState->Online_Flag.Referee=1;
		if(CheckRobotState->usart_state.Check_receiver > 100) CheckRobotState->usart_state.Check_receiver = 100;  
	
	if(check_robot_state.usart_state.Check_receiver>30)  {rc_ctrl->isOnline = 0 ;CheckRobotState->Online_Flag.Receiver=0;} else {rc_ctrl->isOnline = 1;CheckRobotState->Online_Flag.Receiver=1;}
	
}

/*各模块帧率检测*/
void FPS_Check(FPS * fps)
{
		fps->Receiver_FPS=fps->Receiver_cnt;
		fps->Referee_FPS=fps->Referee_cnt;
		fps->Vision_FPS=fps->Vision_cnt;

		fps->Receiver_cnt=0;
		fps->Referee_cnt=0;
		fps->Vision_cnt=0;
}

 int k,m;
void  Motor_Check(Check_Motor* check,CAN_Object canx) 
{
	Motor* motor = NULL;
	list_t *node = NULL;		
 
	for (node = canx.DevicesList.next;    		//< 对循环链表遍历一圈
			 node != (canx.DevicesList.prev->next);
			 node = node->next)
	{
		motor = list_entry(node, Motor, list);
		
		
		if (motor->Data.Online_check.Status== 1)
			
		{
			k=-1;m=-1;
			for (int i=0;i<	check->size_Online;i++) 
			if (check->Online[i]==motor->Param.CanId) k=i;
	 

	 for (int i=0;i<check->size_Offline;i++)		 
			if (check->Offline[i]==motor->Param.CanId) m=i;
	 
		if (k==-1) arrput(check->Online,motor->Param.CanId);
		if (m!=-1) arrdel(check->Offline,m);
		}
		
		
		else {
			
				k=-1;m=-1;
			for (int i=0;i<	check->size_Online;i++) 
			if (check->Online[i]==motor->Param.CanId) k=i;
	 

	 for (int i=0;i<check->size_Offline;i++)
			if (check->Offline[i]==motor->Param.CanId) m=i;

	  

		if (k!=-1) arrdel(check->Online,k);
		if (m==-1) arrput(check->Offline,motor->Param.CanId);
			
	
		}
		
		
			check->size_Offline=arrlen(check->Offline) ;
		check->size_Online=arrlen(check->Online) ;
		
		
       	
	
	}

		

}
