//#include "servo.h"
//#include "modbus.h"
//#include "usart.h"
//#include "hardware_config.h"
//#include "attitude.h"
//#include "lifting.h"
//#include <brain.h>
//#include<dr16.h>
//#include<ore.h>
//#include<driver_timer.h>
////#define SERVO_ID             1      //舵机ID号
////#define SERVO_POS_ADDRESS    128    //位置控制地址（查阅舵机内存表）
////#define SERVO_TORQUE_ADDRESS 129    //扭矩控制地址
////#define SERVO_ACC_ADDRESS    130    //加速度控制地址
////#define TORQUE_ON            1      //打开扭矩开关
////#define TORQUE_OFF           0      //关闭扭矩开关
////void ServoInit(uint8_t servo_id,int init_pos)
////{

////      MODH_Send06H(servo_id,SERVO_POS_ADDRESS,init_pos);         
////    	MODH_Send06H(servo_id,SERVO_TORQUE_ADDRESS,TORQUE_OFF);
////}

////void Servo_Angle_Control(uint8_t servo_id,int pos)
////{
////	
////	 MODH_Send06H(SERVO_ID,SERVO_ACC_ADDRESS,40);      //加速度控制，ID为1，地址130，寄存器值为40，表示4000步每秒s^2。
////   MODH_Send06H(servo_id,SERVO_POS_ADDRESS,pos);


////}

///*寄存器地址宏定义*/
//#define SERVO_POS_ADDRESS    	128    	//位置控制地址（查阅舵机内存表）
//#define SERVO_TORQUE_ADDRESS 	129    	//扭矩控制地址
//#define SERVO_ACC_ADDRESS    	130    	//加速度控制地址
//#define TORQUE_ON           	1      	//打开扭矩开关
//#define TORQUE_OFF           	0       //关闭扭矩开关
//#define ANGLE_LIMIT_MIN      	13      //最小角度限制
//#define ANGLE_LIMIT_MAX				14			//最大角度限制                  
//#define PARAM_P								17			//比例系数P
//#define PARAM_D								18			//微分系数D
//#define PARAM_I 							19			//积分系数I
//#define DELAY_TIME            50     //执行指令之间的延迟间隔

//#define Getadddata32(addr)     *((uint32_t*)(addr))

///*舵机ID宏定义*/
//#define YAW  		1	
//#define PITCH 	2
//#define ROLL 		3

///*舵机初始位置宏定义*/
//#define YAW_init_pos  		0	
//#define PITCH_init_pos 		0
//#define ROLL_init_pos 		0

///*舵机角度限制宏定义*/
//#define YAW_min  		0	
//#define YAW_max  		1000

//#define PITCH_min 	0
//#define PITCH_max 	4095

//#define ROLL_min 		0
//#define ROLL_max 		0

//#define GET_LOW_BYTE(A) ((uint8_t)(A))
////宏函数 获得A的低八位
//#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
////宏函数 获得A的高八位
//#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
////宏函数 将高地八位合成为十六位
////float sideway_parma=2.0;

//uint8_t Ser_recBuffer[300];
//uint8_t HTD_recBuffer[35];
//uint8_t buf[10];
//float servosince=0.01;
//uint16_t ser_time=0;
//uint16_t ser1_time=50;
//uint16_t ser2_time=200;
//uint16_t ser3_time=400;
//float move_angle;
////uint32_t auto_ore_time=0;
//float servo_vel_param = 2;
//uint8_t BUFFF[4]={0x01,0x02,0x03,0x04};
//float dataaa;
//UART_RxBuffer uart5_buffer={
//		.Data = HTD_recBuffer,
//		.Size = 35
//	};
///**
//*  @breif 舵机组参数设置函数
//*  @param[in] NULL   
//*  @see   MODH_Send06H()
//*  @note 舵机ID,参数设置参见宏定义
//*/
////void Servo_Param_Setting()
////{
////		MODH_Send06H(YAW,ANGLE_LIMIT_MIN,YAW_min);		//角度限幅
////		MODH_Send06H(YAW,ANGLE_LIMIT_MAX,YAW_max);
////	
////		MODH_Send06H(PITCH,ANGLE_LIMIT_MIN,PITCH_min);
////		MODH_Send06H(PITCH,ANGLE_LIMIT_MAX,PITCH_max);
////	
////		MODH_Send06H(ROLL,ANGLE_LIMIT_MIN,ROLL_min);
////		MODH_Send06H(ROLL,ANGLE_LIMIT_MAX,ROLL_max);
////}

/////**
////*  @breif 舵机组初始化函数 
////*  @param[in] NULL   
////*  @see   MODH_Send06H()
////*  @note 舵机ID,初始位置参见宏定义
////*/
////void ServoInit()
////{
////		MODH_Send06H(YAW,SERVO_POS_ADDRESS,YAW_init_pos);
////		MODH_Send06H(PITCH,SERVO_POS_ADDRESS,PITCH_init_pos);
////		MODH_Send06H(ROLL,SERVO_POS_ADDRESS,ROLL_init_pos);
////}

///**
//*  @breif 舵机组位置控制函数 
//*  @param[in] a init_pos[3] 描述姿态的数组      
//*  @see   MODH_Send06H()
//*  @note 舵机ID参见宏定义 
//*/
////void Servo_pos_Control(RC_Ctrl *rc_ctrl,Servo_t *servo)
////{
////  if(rc_ctrl->key_SH_CT_Z_flag%2==1)
////	{
////      ser_time++;
////			if(rc_ctrl->key_E_flag==1&&rc_ctrl->key_ctrl_flag==1)
////		 {
////				servo->ServoPOS_1=servo->ServoPOS_1+servo_vel_param; 
////		 }
////			if(rc_ctrl->key_F_flag==1&&rc_ctrl->key_ctrl_flag==1)
////		{
////			servo->ServoPOS_1=servo->ServoPOS_1-servo_vel_param; 
////		}
////		
////		
////		if(rc_ctrl->key_Z_flag==1&&rc_ctrl->key_ctrl_flag==1&&rc_ctrl->key_shift_flag!=1)
////		{
////		  servo->ServoPOS_2++; 
////		}
////		if(rc_ctrl->key_X_flag==1&&rc_ctrl->key_ctrl_flag==1&&rc_ctrl->key_shift_flag!=1)
////		{
////		  servo->ServoPOS_2--; 
////		}
//		
//	
////			if(rc_ctrl->key_C_flag==1&&rc_ctrl->key_ctrl_flag==1)
////			{
////				servo->ServoPOS_3=servo->ServoPOS_3+servo_vel_param; 
////			}
////			if(rc_ctrl->key_V_flag==1&&rc_ctrl->key_ctrl_flag==1)
////			{
////				servo->ServoPOS_3=servo->ServoPOS_3-servo_vel_param; 
////			}
////			if(rc_ctrl->key_C_flag==1&&rc_ctrl->key_ctrl_flag==1&&rc_ctrl->key_shift_flag==1)
////			{
////				 servo->ServoPOS_1=683;
////				 servo->ServoPOS_2=683;
////				 servo->ServoPOS_3=683;
////				servo->oremsg.yall1_dis=0;
////				servo->oremsg.pitch2_dis=0;
////				servo->oremsg.roll3_dis=0;
////			}
////			 if(servo->ServoPOS_1>8000)servo->ServoPOS_1=8000;
////			if(servo->ServoPOS_1<-8000)servo->ServoPOS_1=-8000;
////			
////			if(servo->ServoPOS_2>8000)servo->ServoPOS_1=8000;
////			if(servo->ServoPOS_2<-8000)servo->ServoPOS_1=-8000;
////			
////			if(servo->ServoPOS_3>9000)servo->ServoPOS_3=9000;
////			if(servo->ServoPOS_3<-9000)servo->ServoPOS_3=-9000;
////			
////			if(ser_time==ser1_time)
////			{
////				MODH_Send06H(1,SERVO_POS_ADDRESS,servo->ServoPOS_1+servo->oremsg.yall1_dis);
////			}
////		 if(ser_time==ser2_time)
////			{
////				MODH_Send06H(2,SERVO_POS_ADDRESS,servo->ServoPOS_2+servo->oremsg.pitch2_dis);
////			}
////			if(ser_time>=ser3_time)
////			{
////				MODH_Send06H(3,SERVO_POS_ADDRESS,servo->ServoPOS_3+servo->oremsg.roll3_dis);
////				ser_time=0;
////			}
////		}
////	  servo->ServoPOS_1_last=servo->ServoPOS_1;
////		servo->ServoPOS_2_last=servo->ServoPOS_2;
////		servo->ServoPOS_3_last=servo->ServoPOS_3;
////}
////void Servo_init(uint8_t id,int16_t pos)
////{
////		servo.ServoPOS_1=683;
////		servo.ServoPOS_2=683;
////		servo.ServoPOS_3=683;
////		MODH_Send06H(id,130,50);      //加速度设置
////		 HAL_Delay(DELAY_TIME);	
////		MODH_Send06H(id,403,1);      //角度分辨率设置，默认值1，最大值3，最小值1，当设置为3时，可旋转最多正负100圈（未实现）
////		 HAL_Delay(DELAY_TIME);	
////		MODH_Send06H(id,14,4095); //32768 //32760       //最大角度限制，0-4096为一圈
////		 HAL_Delay(DELAY_TIME);	
////		MODH_Send06H(id,13,0);                       //最小角度限制，-4096-0为一圈
////		 HAL_Delay(DELAY_TIME);	
////		MODH_Send06H(id,386,300);                      //速度最大值设置
////		 HAL_Delay(DELAY_TIME);
////		MODH_Send06H(id,131,200);                      //速度设置（未实现）
////		 HAL_Delay(DELAY_TIME);
////		MODH_Send06H(id,128,pos);                    //位置控制
////		 HAL_Delay(DELAY_TIME);


////}
////void Servo_Reset()
////{
////  MODH_Send06H(1,128,683);     //每个舵机设置中位2048，分辨率为3，初始化位置为2048/3=683。车下电前一定要复位到683
////	  HAL_Delay(DELAY_TIME);
////	MODH_Send06H(2,128,683);
////	  HAL_Delay(DELAY_TIME);
////	MODH_Send06H(3,128,683);
////	  HAL_Delay(DELAY_TIME);
////}
////void send_message(Servo *ser,uint8_t id)
////{
////   ser->SendData[0]=0xff;
////   ser->SendData[1]=0xff;
////	 ser->SendData[2]=id;
////   ser->SendData[3]=0x04;
////	 ser->SendData[4]=0x02;
////   ser->SendData[5]=0xfb;
//////   ser->SendData[6]=0x02;
//////	 ser->SendData[7]=0xbe;
//////	
////	HAL_UART_Transmit_DMA(&huart3,ser->SendData,6);
////}
////uint8_t servo_callback(uint8_t * recBuffer, uint16_t len)
////{
////	if(recBuffer[0]==0x01&&recBuffer[1]==0x03)
////	{
////	
////		servo.ServoPOS=(float)((int16_t)(recBuffer[4]<<8|recBuffer[3]));
////		servo.ServoPOS=servo.ServoPOS*0.2637;
////	}
////}
//void Servo_Flash_Init()
//{
//  HAL_FLASH_Unlock();
//	uint32_t PageError=0;
//	FLASH_EraseInitTypeDef EraseInit;
//	EraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;   //擦除方式有bank擦除和sector擦除，这里选择sector擦除
//	EraseInit.Sector=FLASH_SECTOR_7;             //选择擦除的扇区为sector7
//	EraseInit.Banks=FLASH_BANK_1;               //H750有两个bank，每个bank有7个扇区（sector）
//	EraseInit.NbSectors=1;                     //要删除的扇区个数，一个
//	EraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;//用于设置编程的并行位数，电压不同，位数不同
//	HAL_FLASHEx_Erase(&EraseInit,&PageError);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,0x080E0000,dataaa);
//  HAL_FLASH_Lock();

//}
//void Servo_Flash_Write(Lifting_t *lifting)
//{
//  HAL_FLASH_Unlock();
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,0x080E0000,BUFFF[0]);
//  HAL_FLASH_Lock();
//}
//void Servo_Flash_Read(Servo_t *servo)
//{
// servo->servo_lastpos=Getadddata32(0x080E0000);

//}
//uint8_t LobotCheckSum(uint8_t buf[])
//{
//  uint8_t i;
//  uint16_t temp = 0;
//  for (i = 2; i < buf[3] + 2; i++) {
//    temp += buf[i];
//  }
//  temp = ~temp;
//  i = (uint8_t)temp;
//  return i;
//}

//void LobotSerialServoMove(uint8_t id, uint16_t time,int16_t position)
//{

//	if(position>=1000)position=1000;
//	if(position<=600)position=600;
//  buf[0] = buf[1] = 0x55;
//  buf[2] = id;
//  buf[3] = 7;
//  buf[4] = 1;
//  buf[5] = GET_LOW_BYTE(position);
//  buf[6] = GET_HIGH_BYTE(position);
//  buf[7] = GET_LOW_BYTE(time);
//  buf[8] = GET_HIGH_BYTE(time);
//  buf[9] = LobotCheckSum(buf);
////  LobotSerialWrite(buf, 10);
//	HAL_UART_Transmit_DMA(&huart5,buf,10);
//}
//void figure_servo_control(RC_Ctrl *rc_ctrl,Servo_t *servo)
//{
//		if(rc_ctrl->key_E_flag%2==1&&rc_ctrl->key_SH_C_flag%2==0)
//		{
//			move_angle=0;
//			pitchangel=0;
//		}
//		if(rc_ctrl->key_E_flag%2==0&&rc_ctrl->key_SH_C_flag%2==0)
//		{
//			move_angle=90;
//			pitchangel=90;
//		}
//		
//		if(rc_ctrl->key_SH_C_flag%2==1)
//		{
//			 move_angle=attiTude.Motors6020.motor[1].Data.Angle;
//		
//		}
//		if((manual_mining_flag==0)&&(rc_ctrl->key_G_flag%2==1))
//		{
//		if(rc_Ctrl.mouse.y>0)
//		{   
//				dataaa=rc_Ctrl.mouse.y*0.0001;
//				servo->figure.fig_pitch_target-=dataaa;
//		}
//		if(rc_Ctrl.mouse.y<0)
//			servo->figure.fig_pitch_target-=rc_Ctrl.mouse.y*0.1;
//		if(servo->figure.fig_pitch_target>=1000)servo->figure.fig_pitch_target=1000;
//		if(servo->figure.fig_pitch_target<=600)servo->figure.fig_pitch_target=600;
//		}	
//	  LobotSerialServoMove(3,20,servo->figure.fig_pitch_target);

//}
//uint8_t feiteservo_callback(uint8_t * recBuffer, uint16_t len)
//{
// 
//	return 0;
//}