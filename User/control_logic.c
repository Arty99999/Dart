#include <tim.h>
#include <driver_timer.h>
#include "control_logic.h"
#include "hardware_config.h"
#include "mecanum_chassis.h"
#include "attitude.h"
#include "usart.h"
#include "stdio.h"
#include "servo.h"
#include "modbus.h"
#include "lifting.h"
#include "protract.h"
#include "ore.h"
#include "interaction.h"
#include "mit.h"
#include "Custom_controller.h"
#include "interaction_image.h"
#include "et08.h"
#include "gyro.h"
#include "referee.h"
#include "check.h"
uint8_t flag_completely;
uint8_t cnt_2006_down,cnt_2006_up,cnt_6020_move,cnt_6020_back;
uint8_t  servo_check_number(uint8_t buf[]);//校验位
void servo_move(uint16_t id,uint16_t time,int16_t angle);//位置控制模式
//需要修改的代码：plunging reset
int b1=0;

#define abs(x) ((x)>0? (x):(-(x)))
//以下为测试临时变量
float test01 = -1.0f;	//初始位置-1.35，数值增大向右偏，右侧限幅0，左侧限幅-2.7
float test02 = 0;			//初始位置-1.00，数值增大向上摆，上侧限幅0.65
uint16_t test03;
uint16_t test04;
uint32_t UI_speed=0;
uint32_t reset_count = 0;
uint8_t Auto_state=0;		//自动模式核心标志位
uint8_t Reset_flag = 0;
int m11;  
uint8_t flag_yaw_stop=0;
uint8_t flag_pitch_stop=0;
uint8_t flag_shoot=10;
//uint8_t flag_shoot=10;
uint8_t  flag_stop=0;
uint8_t  flag_reset=0;
int cnt_2006_yaw,cnt_2006_pitch;
int cnt,cnt_2,flag_2006,cnt_2006;
float Target_Angle_3508=0;
float Target_Angle_2006=0;
float Target_Angle_2006_yaw=0;
float Target_Angle_2006_pitch=0;
int cnt_servo_;
/**
  * @brief  主任务函数
  */
uint8_t	flag11;
uint16_t cnt1111;
float a22,a23;
int cntmm;
float Yaw,Pitch;
int b22=80;
int Target_Angle_2006_load;
uint8_t Reload_mode;


RubberState_t Rubber_state=Rubber_IDLE;
ReloadState_t Reload_state = RELOAD_IDLE;
uint16_t cnt6020_down,cnt6020_up,cnt2006_down,cnt2006_up,cnt_servo,cnt_error;
float Target_6020=-270;
int Timer=0;
void TIM14_Task(void)
{
	int i;
static int cnt_up_stop=0,cnt_shoot=0,cnt_yaw_stop=0,cnt_reset=0,cnt_pitch_stop=0,cnt_down_stop;	
	tim14.ClockTime++;
//		if(tim14.ClockTime%1000==0)
//		{
//			
//        Motor_Check();
//		}
  RobotOnlineState(&check_robot_state, &referee2022, &rc_Ctrl);
	//上堵转
		if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==0) cnt_up_stop++;else cnt_up_stop=0;
		if (cnt_up_stop>5) {cnt_up_stop=0;flag_stop=1;Target_Angle_3508=motor3508.motor[0].Data.TotalAngle-200;motor3508.motor[0].Data.Target=0;}

				
	
	
	motor2006.motor[0].Data.Output =
	BasePID_YawSpeedControl((BasePID_Object*)(&Motors2006_SpeedPID) , 
	BasePID_YawAngleControl((BasePID_Object*)(&Motors2006_AngelPID) , Target_Angle_2006 , motor2006.motor[0].Data.TotalAngle)  ,motor2006.motor[0].Data.SpeedRPM);	
				
				

			if (rc_Ctrl.isOnline==1 && flag_2006==0 &&rc_Ctrl.rc.s2!=2 ) Target_Angle_2006+=(rc_Ctrl.rc.ch3 -1024)* 0.05;
			if (rc_Ctrl.isOnline==1  &&rc_Ctrl.rc.s2==2 ) 
			{
				Target_Angle_2006_yaw-=(rc_Ctrl.rc.ch2 -1024)* 0.05;	
	Target_Angle_2006_pitch-=(rc_Ctrl.rc.ch3 -1024)* 0.05;
			
			
			}
//				if (abs(motor2006.motor[0].Data.Output)>3000&&rc_Ctrl.isOnline==1 )   cnt_2++;else cnt_2=0;
//			if  (cnt_2>=300)
//{flag_2006=1;cnt_2=0;Target_Angle_2006=motor2006.motor[0].Data.TotalAngle;}	
				
	if  (rc_Ctrl.rc.s1==3) m11=0;
	else if (rc_Ctrl.rc.s1==2) m11=-6000;
	else if (rc_Ctrl.rc.s1==1) m11=7000;
//消堵转	
	if  (rc_Ctrl.rc.s2==1 && rc_Ctrl.rc.s2_last==3&& flag_stop==1) flag_stop=0;

	


	
	if    (abs(rc_Ctrl.rc.ch1-1024)>500&& rc_Ctrl.rc.s2==2)  cnt_2006_pitch++; else cnt_2006_pitch=0;
	if    (abs(rc_Ctrl.rc.ch0-1024)>500&& rc_Ctrl.rc.s2==2)  cnt_2006_yaw++; else cnt_2006_yaw=0;
	if  (cnt_2006_pitch>=800&&rc_Ctrl.rc.ch1-1024>=0)  {Target_Angle_2006_pitch-=12900;cnt_2006_pitch=0;}
		if  (cnt_2006_pitch>=800&&rc_Ctrl.rc.ch1-1024<=0)  {Target_Angle_2006_pitch+=12900;cnt_2006_pitch=0;}
			
	if  (cnt_2006_yaw>=800&&rc_Ctrl.rc.ch0-1024>=0){Target_Angle_2006_yaw-=3800;cnt_2006_yaw=0;}
	if  (cnt_2006_yaw>=800&&rc_Ctrl.rc.ch0-1024<=0){Target_Angle_2006_yaw+=3800;cnt_2006_yaw=0;}
	
	if    (abs(rc_Ctrl.rc.ch2-1024)>500&& rc_Ctrl.rc.s2!=2)  cnt_2006++; else cnt_2006=0;
if  (cnt_2006>=800&&rc_Ctrl.rc.ch2-1024>=0)
{Target_Angle_2006-=3800;cnt_2006=0;}
else if  (cnt_2006>=800&&rc_Ctrl.rc.ch2-1024<0)
	{Target_Angle_2006+=3800;cnt_2006=0;}
	
//开关舵机		
	if    (abs(rc_Ctrl.rc.ch0-1024)>500 && rc_Ctrl.rc.s2!=2)  cnt_shoot++; else cnt_shoot=0;
	
if  (cnt_shoot>=1000)
 {
 if (flag_shoot==10) flag_shoot=17;else flag_shoot=10;
 cnt_shoot=0;
 }
// 
// if (flag_shoot==7) cntmm++;
// 
// if (cntmm>200) {flag_shoot=20;cntmm=0;}
 
//		flag_yaw_stop=1;
//		flag_pitch_stop=1;

				if(abs(motor3508.motor[0].Data.Target-motor3508.motor[0].Data.SpeedRPM)>250 && flag_stop==0)//堵转判断
					cnt_down_stop++;else cnt_down_stop=0;
				if(cnt_down_stop>=50)
				{
					cnt_down_stop=0;
					Target_Angle_3508=motor3508.motor[0].Data.TotalAngle-400;
					flag_stop=1;
					motor3508.motor[0].Data.Target=0;
          
				}


	if (flag_reset==1)  
	{
	motor2006.motor[1].Data.Target=-4000;	
	motor2006.motor[2].Data.Target=-6000;	
		
		if (flag_yaw_stop==0)
	motor2006.motor[1].Data.Output = BasePID_SpeedControl(&run_pid,motor2006.motor[1].Data.Target,motor2006.motor[1].Data.SpeedRPM);
	  else    
		{
			motor2006.motor[1].Data.Output=
	 BasePID_PitchSpeedControl((BasePID_Object*)(&Motors2006_yaw_SpeedPID) , 
   BasePID_YawAngleControl((BasePID_Object*)(&Motors2006_yaw_AngelPID) , Target_Angle_2006_yaw ,motor2006.motor[1].Data.TotalAngle ),motor2006.motor[1].Data.SpeedRPM);
			if (motor2006.motor[1].Data.Output>5000) motor2006.motor[1].Data.Output=5000;
	   else if (motor2006.motor[1].Data.Output<-5000) motor2006.motor[1].Data.Output=-5000;
		}

				if (flag_pitch_stop==0)
	motor2006.motor[2].Data.Output = BasePID_SpeedControl(&run_pid,motor2006.motor[2].Data.Target,motor2006.motor[2].Data.SpeedRPM);
	  else    
		{
			motor2006.motor[2].Data.Output=
	 BasePID_PitchSpeedControl((BasePID_Object*)(&Motors2006_pitch_SpeedPID) , 
   BasePID_YawAngleControl((BasePID_Object*)(&Motors2006_pitch_AngelPID) , Target_Angle_2006_pitch ,motor2006.motor[2].Data.TotalAngle ),motor2006.motor[2].Data.SpeedRPM);
			if (motor2006.motor[2].Data.Output>5000) motor2006.motor[2].Data.Output=5000;
	   else if (motor2006.motor[2].Data.Output<-5000) motor2006.motor[2].Data.Output=-5000;
		}
		
				if(abs(motor2006.motor[1].Data.Target-motor2006.motor[1].Data.SpeedRPM)>250 &&flag_yaw_stop==0)//堵转判断
			cnt_yaw_stop++;else cnt_yaw_stop=0;
		if(cnt_yaw_stop>=50)
		{
			cnt_yaw_stop=0;
			
			flag_yaw_stop=1;
			a22=motor2006.motor[1].Data.TotalAngle;
Target_Angle_2006_yaw=motor2006.motor[1].Data.TotalAngle+391263;
			
		}
		
		
		
	if(abs(motor2006.motor[2].Data.Target-motor2006.motor[2].Data.SpeedRPM)>250 &&flag_pitch_stop==0)//堵转判断
			cnt_pitch_stop++;else cnt_pitch_stop=0;
		if(cnt_pitch_stop>=50)
		{
			cnt_pitch_stop=0;
			a23=motor2006.motor[1].Data.TotalAngle;
			flag_pitch_stop=1;
Target_Angle_2006_pitch=motor2006.motor[2].Data.TotalAngle+200000;
			
		}
		
		
		
		
	}
	
	
	
	
	
	
	    switch (Reload_state) {
        case RELOAD_IDLE:
        if  (Reload_mode)
				{
					
                
					      b22=495;
					cnt_servo++;
					if (cnt_servo>1000) {cnt_servo=0;Reload_state = RELOAD_STEP1;}
		
   
					
            }
            break;

        case RELOAD_STEP1:
        				if (Reload_mode==1)
                Target_6020 += 91;  // 初始化第一步目标
					else if (Reload_mode==2) Target_6020 += 181;
					else if (Reload_mode==3) Target_6020 += 270;
            
                
            Reload_state = RELOAD_STEP1_5;
          break;

						     case RELOAD_STEP1_5:
                if (abs(Target_6020 - motor6020.motor[0].Data.TotalAngle) < 1.5) 
							cnt_6020_move++;else cnt_6020_move = 0;
							
                if (cnt_6020_move > 50) {
                    cnt_6020_move = 0;
                    Reload_state = RELOAD_STEP2;
                    Target_Angle_2006_load -= 60436;  // 初始化第二步目标
                }
            
                
            
          break;		
								

								
        case RELOAD_STEP2:
				if 	(abs(motor2006.motor[3].Data.SpeedRPM-Motors2006_load_AngelPID.Out)>500 ) cnt_error++;else cnt_error=0;
				if (cnt_error>100) Reload_state=RELOAD_ERROR;
				
            if (abs(Target_Angle_2006_load - motor2006.motor[3].Data.TotalAngle) < 200) 
							cnt_2006_down++;else cnt_2006_down = 0;
							
                if (cnt_2006_down > 100) {
                    cnt_2006_down = 0;
									
                    Reload_state = RELOAD_STEP3;
                    Target_Angle_2006_load += 60436;  // 初始化第三步目标
                }
      
            break;

								 case RELOAD_STEP3:
									 
								 
            if (abs(Target_Angle_2006_load - motor2006.motor[3].Data.TotalAngle) < 9000) 
							cnt_2006_up++;else cnt_2006_up = 0;
						
						 if (abs(Target_Angle_2006_load - motor2006.motor[3].Data.TotalAngle) < 45000) 
							cnt_servo++;else cnt_servo = 0;
						 if (cnt_servo>50) {cnt_servo=0;	b22=80;}
						
                if (cnt_2006_up > 100) {
                    cnt_2006_up = 0;
                    Reload_state = RELOAD_STEP4;
											if (Reload_mode==1)
                Target_6020 -= 91;  // 初始化第一步目标
					else if (Reload_mode==2) Target_6020 -= 181;
					else if (Reload_mode==3) Target_6020 -= 270;
                  
                }
      
            break;
					case RELOAD_STEP4:
               if (abs(Target_6020 - motor6020.motor[0].Data.TotalAngle) < 1) 
							cnt_6020_back++;else cnt_6020_back = 0;
							
                if (cnt_6020_back > 100) {
                    cnt_6020_back = 0;
                    Reload_state = RELOAD_COMPLETE;
                 
                }
      
            break;	
								
        // 其他状态类似...
        case RELOAD_COMPLETE:
            
//				if (rc_Ctrl.rc.s2 == 1 && rc_Ctrl.rc.s2_last == 3) 
//				{Reload_mode++;
//				Reload_state = RELOAD_IDLE;}
				
//            Reload_state = RELOAD_IDLE;
            break;
				case RELOAD_ERROR:
				motor2006.motor[3].Data.Output=0;
				motor6020.motor[0].Data.Output=0;
				b22=495;
				
				
				break;
    }
			
			    switch (Rubber_state) {
        case Rubber_IDLE:
				{
					
					
				}
				
            break;

        case Rubber_STEP1:
				{
					m11=-4000;
					if  (flag_stop==1) Rubber_state=Rubber_STEP2;
				}
                
            
          break;		
								
								
								
        case Rubber_STEP2:
		 flag_shoot=17;
				cnt_servo_++;
					if (flag_completely==1) 		{Reload_mode++;
		if (Reload_state ==RELOAD_COMPLETE) 
				Reload_state = RELOAD_STEP1;flag_completely=0;}
				if (cnt_servo_>1200) {cnt_servo_=0;Rubber_state=Rubber_STEP3;flag_stop=0;}
				
            break;

						    case Rubber_STEP3:
									

            	m11=5000;
     if  (flag_stop==1) Rubber_state=Rubber_COMPLETE;
				
            break;	
				
				
        case Rubber_COMPLETE:

			
				
				
            break;
				case Rubber_ERROR:
				
				
				break;
    }
		
		
			if (flag_stop==0)	motor3508.motor[0].Data.Target=m11;	else {motor3508.motor[0].Data.Target=0;}
		
		
		
		  if (flag_stop==1) 		
   motor3508.motor[0].Data.Output=
	 BasePID_PitchSpeedControl((BasePID_Object*)(&Motors3508_SpeedPID) , 
   BasePID_PitchAngleControl((BasePID_Object*)(&Motors3508_AngelPID) , Target_Angle_3508 ,motor3508.motor[0].Data.TotalAngle ),motor3508.motor[0].Data.SpeedRPM);
  else motor3508.motor[0].Data.Output = BasePID_SpeedControl(&run_pid, motor3508.motor[0].Data.Target,motor3508.motor[0].Data.SpeedRPM);
		
		
		
			if(rc_Ctrl.isOnline==0) motor2006.motor[3].Data.Output=0;
			else 			motor2006.motor[3].Data.Output=
	 BasePID_PitchSpeedControl((BasePID_Object*)(&Motors2006_load_SpeedPID) , 
   BasePID_YawAngleControl((BasePID_Object*)(&Motors2006_load_AngelPID) , Target_Angle_2006_load ,motor2006.motor[3].Data.TotalAngle ),motor2006.motor[3].Data.SpeedRPM);
			
						
		if(rc_Ctrl.isOnline==0 ) motor6020.motor[0].Data.Output=0;
			else 			motor6020.motor[0].Data.Output=
	 BasePID_PitchSpeedControl((BasePID_Object*)(&Motors6020_pitch_SpeedPID) , 
   BasePID_YawAngleControl((BasePID_Object*)(&Motors6020_pitch_AngelPID) , Target_6020 ,motor6020.motor[0].Data.TotalAngle),motor6020.motor[0].Data.SpeedRPM);
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
//开关舵机				

	
		if(rc_Ctrl.isOnline != 1)
	{
		DR16Init(&rc_Ctrl);
		flag_stop=0;

		flag_reset=0;
	}
	
	
	if (rc_Ctrl.isOnline==0) {motor3508.motor[0].Data.Output=0;motor2006.motor[0].Data.Output=0;motor2006.motor[1].Data.Output=0;motor2006.motor[2].Data.Output=0;}
Motor* temp_motor ;
		int k=0;
for ( i=0;i<check_robot_state.Check_Can1.size_Online;i++)
		{
			temp_motor = MotorFind((check_robot_state.Check_Can1.Online)[i],can1);
			
			if (tim14.ClockTime%80==k*5)
		  UsartDmaPrintf_("离线检测.t%d.txt=\"%x:Online TEMP:%d°\"",k,temp_motor->Param.CanId,temp_motor->Data.Temperature);
			k++;
		}
for (int j=0;j<check_robot_state.Check_Can1.size_Offline;j++)
		{
			temp_motor = MotorFind(check_robot_state.Check_Can1.Offline[j],can1);
			if (tim14.ClockTime%80==k*5)
		  UsartDmaPrintf_("离线检测.t%d.txt=\"%x:Offline TEMP:Null\"",k,temp_motor->Param.CanId);;
			k++;
		}
		for ( i=0;i<check_robot_state.Check_Can2.size_Online;i++)
		{
			temp_motor = MotorFind((check_robot_state.Check_Can2.Online)[i],can2);
			
			if (tim14.ClockTime%80==k*5)
		  UsartDmaPrintf_("离线检测.t%d.txt=\"%x:Online TEMP:%d°\"",k,temp_motor->Param.CanId,temp_motor->Data.Temperature);
			k++;
		}
for (int j=0;j<check_robot_state.Check_Can2.size_Offline;j++)
		{
			temp_motor = MotorFind(check_robot_state.Check_Can2.Offline[j],can2);
			if (tim14.ClockTime%80==k*5)
		  UsartDmaPrintf_("离线检测.t%d.txt=\"%x:Offline TEMP:Null\"",k,temp_motor->Param.CanId);
			k++;
		}
		if (tim14.ClockTime%80==k*5)
			UsartDmaPrintf_("改变角度.x1.val=%d",(int) (Yaw*1000));
		k++;
				if (tim14.ClockTime%80==k*5)
			UsartDmaPrintf_("改变角度.x0.val=%d",(int) (Pitch*1000));
		k++;
//OnlineCheck_Fresh(can2,&check);
		
	MotorFillData(&motor3508.motor[0],motor3508.motor[0].Data.Output);
  MotorFillData(&motor2006.motor[0],motor2006.motor[0].Data.Output);
	MotorFillData(&motor2006.motor[1],motor2006.motor[1].Data.Output);
	MotorFillData(&motor2006.motor[2],motor2006.motor[2].Data.Output);
	MotorFillData(&motor6020.motor[0],motor6020.motor[0].Data.Output);
	MotorFillData(&motor2006.motor[3],motor2006.motor[3].Data.Output);
	
	MotorCanOutput(can2, 0x1FF);
	MotorCanOutput(can1, 0x1FF);
		if (tim14.ClockTime%30==0)
		servo_move(0xFE,100,b22);
	//Usart1DmaPrintf("%d,%d,%d\r\n",motor6020.motor[0].Data.RawEcd,motor6020.motor[0].Data.LastEcd,motor6020.motor[0].Data.RoundCnt);
		float Yaw_l;
		Yaw_l=(209-(fabs(motor2006.motor[1].Data.TotalAngle-a22)/533234*164));
		
	if (Yaw_l<88) Yaw=-atan((88-Yaw_l)/663.1467)*57.3;
		else Yaw=atan((Yaw_l-88)/663.1467)*57.3;
	//	Yaw=atan((12.2-(motor2006.motor[1].Data.TotalAngle-a22)/360/89.085366)/56.7)*57.29578;
		float Pitch_l;
		Pitch_l=(593.5-(motor2006.motor[2].Data.TotalAngle-a23)/413228 *128.53);
		
		Pitch=acos((Pitch_l*Pitch_l+543.13405*543.13405-280.71*280.71)/(2*Pitch_l*543.13405))*57.29578;
	MotorCanOutput(can2, 0x200);
//		if (tim14.ClockTime%30==0)
//		servo_move(0xFE,100,b22);
}

/**
  * @brief  CAN1接收中断回调
  */
uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(can1, (*rxBuffer)); 
	return 0;
}

/**
  * @brief  CAN2接收中断回调
  */
uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(can2, (*rxBuffer)); 	
	return 0;
}

uint8_t  servo_check_number(uint8_t buf[])//校验位
{   uint8_t i;
    uint16_t temp=0;
    for(i=2;i<buf[3]+2;i++)
    {
      temp+=buf[i];
    }
      temp=~temp;
      i=(uint8_t)temp;
    return i;
}
uint8_t buf[10];
void servo_move(uint16_t id,uint16_t time,int16_t angle)//位置控制模式
{   
    buf[0]=buf[1]=0x55;
    buf[2]=id;
    buf[3]=7;
    buf[4]=1;
    buf[5]=((uint8_t )(angle));
    buf[6]=((uint8_t )((angle)>>8));
    buf[7]=((uint8_t )(time));
    buf[8]=((uint8_t )(time>>8));
    buf[9]=servo_check_number(buf);
    HAL_UART_Transmit_DMA(&huart5,(unsigned char*)buf,10);
}
void TIM13_Task(void)
{
			Timer++;
		if  (Timer<=flag_shoot) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_SET);else HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_RESET);
    if  (Timer==200) Timer=0;
	
	
	
}

uint8_t LCD_callback(uint8_t * recBuffer, uint16_t len)
{
	if (recBuffer[0]==0x55&&recBuffer[1]==0x01 &&recBuffer[2]==0x00 &&recBuffer[3]==0x01) //换弹
	{
		Reload_mode++;
		if (Reload_state ==RELOAD_COMPLETE) 
				Reload_state = RELOAD_IDLE;
	}
	else if (recBuffer[0]==0x55&&recBuffer[1]==0x01 &&recBuffer[2]==0x00 &&recBuffer[3]==0x00) //蓄能
	{
		
	if (Rubber_state== Rubber_IDLE ||Rubber_state== Rubber_COMPLETE)
	{Rubber_state= Rubber_STEP1;flag_stop=0;}
		
	}
	else if (recBuffer[0]==0x55&&recBuffer[1]==0x01 &&recBuffer[2]==0x00 &&recBuffer[3]==0x02) //一次全流程
	{
		if (Rubber_state== Rubber_IDLE ||Rubber_state== Rubber_COMPLETE)
	{Rubber_state= Rubber_STEP1;flag_stop=0;}
		flag_completely=1;
		b22=495;
		
	}
	else if (recBuffer[0]==0x55&&recBuffer[1]==0x01 &&recBuffer[2]==0x00 &&recBuffer[3]==0x03) //换弹复位
	{
		
		
		
		
	}	
	else if (recBuffer[0]==0x55&&recBuffer[1]==0x01 &&recBuffer[2]==0x00 &&recBuffer[3]==0x04) //Yaw Pitch 复位
	{
		flag_reset=1;
		
		
		
	}		
  //Referee_Data_Diapcak(recBuffer,len);
	return 0;
}