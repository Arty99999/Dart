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
////#define SERVO_ID             1      //���ID��
////#define SERVO_POS_ADDRESS    128    //λ�ÿ��Ƶ�ַ�����Ķ���ڴ��
////#define SERVO_TORQUE_ADDRESS 129    //Ť�ؿ��Ƶ�ַ
////#define SERVO_ACC_ADDRESS    130    //���ٶȿ��Ƶ�ַ
////#define TORQUE_ON            1      //��Ť�ؿ���
////#define TORQUE_OFF           0      //�ر�Ť�ؿ���
////void ServoInit(uint8_t servo_id,int init_pos)
////{

////      MODH_Send06H(servo_id,SERVO_POS_ADDRESS,init_pos);         
////    	MODH_Send06H(servo_id,SERVO_TORQUE_ADDRESS,TORQUE_OFF);
////}

////void Servo_Angle_Control(uint8_t servo_id,int pos)
////{
////	
////	 MODH_Send06H(SERVO_ID,SERVO_ACC_ADDRESS,40);      //���ٶȿ��ƣ�IDΪ1����ַ130���Ĵ���ֵΪ40����ʾ4000��ÿ��s^2��
////   MODH_Send06H(servo_id,SERVO_POS_ADDRESS,pos);


////}

///*�Ĵ�����ַ�궨��*/
//#define SERVO_POS_ADDRESS    	128    	//λ�ÿ��Ƶ�ַ�����Ķ���ڴ��
//#define SERVO_TORQUE_ADDRESS 	129    	//Ť�ؿ��Ƶ�ַ
//#define SERVO_ACC_ADDRESS    	130    	//���ٶȿ��Ƶ�ַ
//#define TORQUE_ON           	1      	//��Ť�ؿ���
//#define TORQUE_OFF           	0       //�ر�Ť�ؿ���
//#define ANGLE_LIMIT_MIN      	13      //��С�Ƕ�����
//#define ANGLE_LIMIT_MAX				14			//���Ƕ�����                  
//#define PARAM_P								17			//����ϵ��P
//#define PARAM_D								18			//΢��ϵ��D
//#define PARAM_I 							19			//����ϵ��I
//#define DELAY_TIME            50     //ִ��ָ��֮����ӳټ��

//#define Getadddata32(addr)     *((uint32_t*)(addr))

///*���ID�궨��*/
//#define YAW  		1	
//#define PITCH 	2
//#define ROLL 		3

///*�����ʼλ�ú궨��*/
//#define YAW_init_pos  		0	
//#define PITCH_init_pos 		0
//#define ROLL_init_pos 		0

///*����Ƕ����ƺ궨��*/
//#define YAW_min  		0	
//#define YAW_max  		1000

//#define PITCH_min 	0
//#define PITCH_max 	4095

//#define ROLL_min 		0
//#define ROLL_max 		0

//#define GET_LOW_BYTE(A) ((uint8_t)(A))
////�꺯�� ���A�ĵͰ�λ
//#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
////�꺯�� ���A�ĸ߰�λ
//#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
////�꺯�� ���ߵذ�λ�ϳ�Ϊʮ��λ
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
//*  @breif �����������ú���
//*  @param[in] NULL   
//*  @see   MODH_Send06H()
//*  @note ���ID,�������òμ��궨��
//*/
////void Servo_Param_Setting()
////{
////		MODH_Send06H(YAW,ANGLE_LIMIT_MIN,YAW_min);		//�Ƕ��޷�
////		MODH_Send06H(YAW,ANGLE_LIMIT_MAX,YAW_max);
////	
////		MODH_Send06H(PITCH,ANGLE_LIMIT_MIN,PITCH_min);
////		MODH_Send06H(PITCH,ANGLE_LIMIT_MAX,PITCH_max);
////	
////		MODH_Send06H(ROLL,ANGLE_LIMIT_MIN,ROLL_min);
////		MODH_Send06H(ROLL,ANGLE_LIMIT_MAX,ROLL_max);
////}

/////**
////*  @breif ������ʼ������ 
////*  @param[in] NULL   
////*  @see   MODH_Send06H()
////*  @note ���ID,��ʼλ�òμ��궨��
////*/
////void ServoInit()
////{
////		MODH_Send06H(YAW,SERVO_POS_ADDRESS,YAW_init_pos);
////		MODH_Send06H(PITCH,SERVO_POS_ADDRESS,PITCH_init_pos);
////		MODH_Send06H(ROLL,SERVO_POS_ADDRESS,ROLL_init_pos);
////}

///**
//*  @breif �����λ�ÿ��ƺ��� 
//*  @param[in] a init_pos[3] ������̬������      
//*  @see   MODH_Send06H()
//*  @note ���ID�μ��궨�� 
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
////		MODH_Send06H(id,130,50);      //���ٶ�����
////		 HAL_Delay(DELAY_TIME);	
////		MODH_Send06H(id,403,1);      //�Ƕȷֱ������ã�Ĭ��ֵ1�����ֵ3����Сֵ1��������Ϊ3ʱ������ת�������100Ȧ��δʵ�֣�
////		 HAL_Delay(DELAY_TIME);	
////		MODH_Send06H(id,14,4095); //32768 //32760       //���Ƕ����ƣ�0-4096ΪһȦ
////		 HAL_Delay(DELAY_TIME);	
////		MODH_Send06H(id,13,0);                       //��С�Ƕ����ƣ�-4096-0ΪһȦ
////		 HAL_Delay(DELAY_TIME);	
////		MODH_Send06H(id,386,300);                      //�ٶ����ֵ����
////		 HAL_Delay(DELAY_TIME);
////		MODH_Send06H(id,131,200);                      //�ٶ����ã�δʵ�֣�
////		 HAL_Delay(DELAY_TIME);
////		MODH_Send06H(id,128,pos);                    //λ�ÿ���
////		 HAL_Delay(DELAY_TIME);


////}
////void Servo_Reset()
////{
////  MODH_Send06H(1,128,683);     //ÿ�����������λ2048���ֱ���Ϊ3����ʼ��λ��Ϊ2048/3=683�����µ�ǰһ��Ҫ��λ��683
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
//	EraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;   //������ʽ��bank������sector����������ѡ��sector����
//	EraseInit.Sector=FLASH_SECTOR_7;             //ѡ�����������Ϊsector7
//	EraseInit.Banks=FLASH_BANK_1;               //H750������bank��ÿ��bank��7��������sector��
//	EraseInit.NbSectors=1;                     //Ҫɾ��������������һ��
//	EraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;//�������ñ�̵Ĳ���λ������ѹ��ͬ��λ����ͬ
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