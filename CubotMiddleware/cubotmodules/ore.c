//#include <tim.h>
//#include "ore.h"
//#include "lifting.h"
//#include "protract.h"
//#include "hardware_config.h"
//#include "servo.h"
//#include "modbus.h"
//#include "servo.h"
//#include "motor.h"
//#include "mit.h"
//#include "mecanum_chassis.h"
//#include <driver_timer.h>
//#include "interaction.h"
//#include "lifting.h"

////�����˶���־�궨��
//#define OUT 		1
//#define BACK 		0
//#define UP 			1
//#define DOWN 		0
//#define OPEN 		0
//#define CLOSE 	1

///*�Զ�ȡ��ҿ��־λ*/
//uint8_t auto_test_flag=0;
////�սӱ�־λ
//uint8_t auto_ore_aircat_mode1flag=0;
//uint8_t auto_ore_aircat_mode2flag=0;
//uint8_t auto_ore_aircat_mode3flag=0;

////ȡ�����־λ
//uint8_t auto_silver_grab_mode1flag=0;
//uint8_t auto_silver_grab_mode2flag=0;
//uint8_t auto_silver_grab_mode3flag=0;

////ȡ����־λ
//uint8_t auto_gold_grab_mode1flag=0;
//uint8_t auto_gold_grab_mode2flag=0;
//uint8_t auto_gold_grab_mode3flag=0;

////�ҿ�ǰ�ñ�־λ
//uint8_t auto_ore_conver_mode1_flag=0;
//uint8_t auto_ore_conver_mode2_flag=0;
//uint8_t auto_ore_conver_mode3_flag=0;

////ȡ�ؿ��־λ
//uint8_t auto_ore_land_mining_flag=0;

//uint8_t Mining_mode_flag=0;
//uint8_t manual_mining_flag=0;
//uint8_t Q_mode2flag=0;
//uint8_t test_flag=0;

//uint8_t auto_conver_reset_modeone1_flag=0;
//uint8_t conver_reset1=0;
//uint8_t conver_reset2=0;
//uint8_t auto_conver_reset_modetwo2_flag=0;
//uint8_t bypass_motor_open_flag=0;
//uint8_t auto_sliver_slow_flag=0;
//uint32_t s1open_time;

////��翪�ر�־λ
//uint8_t photo_one_flag=0;
//uint8_t photo_two_flag=0;
//uint8_t photo_three_flag=0;

////������־λ
//uint8_t spin_flag=0;
//uint8_t mining_mode_flag1=0;
//uint8_t mining_mode_flag2=0;

////����IO��װ��������
//void Gasdynamic_grab1(uint8_t state);
//void Gasdynamic_grab2(uint8_t state);
//void Gasdynamic_storage_lift(uint8_t state);
//void Gasdynamic_storage_clamp(uint8_t state);
//void Gasdynamic_claw(uint8_t state);
//void Gasdynamic_exchange(uint8_t state);

////�Զ�ģʽ��ʱ��
//uint32_t auto_pretimer = 0;
//uint32_t auto_timer = 0;

//float roll_sinse=-0.15;
//float pitch_sinse=0.0035;
//void mode_intervene()
//{
//  if(auto_ore_aircat_mode1flag==1)
//	{
//	  auto_ore_aircat_mode2flag=0;
//		auto_ore_aircat_mode3flag=0;
//	}
//	if(auto_ore_aircat_mode2flag==1)
//	{
//	  auto_ore_aircat_mode1flag=0;
//		auto_ore_aircat_mode3flag=0;
//	}
//	if(auto_ore_aircat_mode3flag==1)
//	{
//	  auto_ore_aircat_mode1flag=0;
//		auto_ore_aircat_mode2flag=0;
//	}
//	if((auto_ore_aircat_mode1flag==1)||(auto_ore_aircat_mode2flag==1)||(auto_ore_aircat_mode3flag==1)||
//		(auto_silver_grab_mode1flag==1)||(auto_silver_grab_mode2flag==1)||(auto_silver_grab_mode3flag==1)||
//	  (auto_gold_grab_mode1flag==1)||(auto_gold_grab_mode2flag==1)||(auto_gold_grab_mode3flag==1)||
//	  (auto_ore_conver_mode1_flag==1)||(auto_ore_conver_mode2_flag==1)||(auto_ore_conver_mode3_flag==1)||
//	  (conver_reset1==1)||(conver_reset2==1))
//	{
//		 mining_mode_flag1 = 0;
//		 mining_mode_flag2 = 0;
//	}
//}
///**
//  * @brief  ����IO��ʼ��
//  */
//void Aerodynamic_init()
//{
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);		  	  //����н�����PE10 
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);		  	//����̧������PE11
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);		  	//ץȡ����ǰ��PE12
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);		  	//ץȡһ��ǰ��PE13
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);		  	//�ҿ�ǰ��PE14
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);		  	//��צ�н�PE15
//}

///**
//  * @brief  �սӹ����
//  */
//void photoelectric_detection()
//{
//   if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14)==GPIO_PIN_RESET)  //PD12 ��⵽��ʯ�øߵ�ƽ����־λ��1
//		 photo_one_flag=1; 
//}

///**
//  * @brief  ��־λɨ��
//  */
//void key_scan(RC_Ctrl *rc_ctrl, Attitude_t *attitude)
//{
//	/*�����翪�ر�־λ��ֵ*/
//	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13)==GPIO_PIN_SET)  //PD13 ��⵽��ʯ�øߵ�ƽ����־λ��1
//		 photo_two_flag=1;
//	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13)==GPIO_PIN_RESET)  
//		 photo_two_flag=0;
//	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12)==GPIO_PIN_SET)  //PD14 ��⵽��ʯ�øߵ�ƽ����־λ��1
//		 photo_three_flag=1;
//	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12)==GPIO_PIN_RESET)  
//		 photo_three_flag=0;

//	/*�Զ�ģʽ��־λ��ֵ*/
//	
//	//�ս�ģʽ���
//	if(rc_ctrl->key_F_flag==1&&rc_ctrl->key_Z_flag==1)		//�ս� Aircatch		
//	{
//	   if((photo_two_flag==0)&&(photo_three_flag==0))		
//			 auto_ore_aircat_mode1flag=1;
//		 else if(((photo_two_flag==1)&&(photo_three_flag==0))||((photo_two_flag==0)&&(photo_three_flag==1)))	
//			 auto_ore_aircat_mode2flag=1;
//		 else if((photo_two_flag==1)&&(photo_three_flag==1))		
//			 auto_ore_aircat_mode3flag=1;
//	}
//	
//	//ȡ����ģʽ���
//	if(rc_ctrl->key_F_flag==1&&rc_ctrl->key_X_flag==1)
//	{
//	   if((photo_two_flag==0)&&(photo_three_flag==0))
//			 auto_silver_grab_mode1flag = 1;
//		 if((photo_two_flag==1)&&(photo_three_flag==0))	
//			 auto_silver_grab_mode2flag = 1;
//		 if((photo_two_flag==1)&&(photo_three_flag==1))
//			 auto_silver_grab_mode3flag = 1;
//	}
//	
//	//ȡ���ģʽ���
//	if(rc_ctrl->key_F_flag==1&&rc_ctrl->key_C_flag==1)
//	{
//	   if((photo_two_flag==0)&&(photo_three_flag==0))
//			 auto_gold_grab_mode1flag = 1;
//		 if((photo_two_flag==1)&&(photo_three_flag==0))	
//			 auto_gold_grab_mode2flag = 1;
//		 if((photo_two_flag==1)&&(photo_three_flag==1))
//			 auto_gold_grab_mode3flag = 1;
//	}
//	
//	//ȡ�ؿ�ģʽ
//  if(rc_ctrl->key_F_flag==1&&rc_ctrl->key_B_flag==1)
//	{
//	   auto_ore_land_mining_flag=1;
//	}
//	//�ҿ�ǰ��ģʽ���
//	if(rc_ctrl->key_F_flag==1&&rc_ctrl->key_V_flag==1)	
//	{
//		if(((photo_two_flag==1)&&(photo_three_flag==0))||((photo_two_flag==0)&&(photo_three_flag==1)))
//			auto_ore_conver_mode1_flag = 1;
//		else if((photo_two_flag==1)&&(photo_three_flag==1)&&(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15)==GPIO_PIN_RESET))
//		  auto_ore_conver_mode2_flag = 1;
//		else if((photo_two_flag==1)&&(photo_three_flag==1)&&(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15)==GPIO_PIN_SET))
//		  auto_ore_conver_mode3_flag = 1;
//	}
//	
//	//ctrl+F�˳�ģʽ���
//	if(rc_ctrl->key_F_flag==1&&rc_ctrl->key_ctrl_flag==1)   //RC_Ctl->key_ctrl_flag=1
//	{
//		if(auto_conver_reset_modeone1_flag==1)
//		 conver_reset1=1;
//		if((auto_ore_aircat_mode1flag==1)||(auto_ore_aircat_mode2flag==1)||(auto_ore_aircat_mode3flag==1))
//		 conver_reset2=1;  
//	}
//	
//	//Q������Ұģʽ���
//	if((rc_ctrl->isOnline==1)&&(rc_ctrl->key_Q_flag%3==1))
//	{
//     if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)==GPIO_PIN_SET)
//		 {
//			 mining_mode_flag1 = 1;
//		 }
//		 else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)==GPIO_PIN_RESET)
//		 {
//			 mining_mode_flag2 = 1;
//		 }
//		 rc_ctrl->key_Q_flag=0;
//	}

//	//�ֶ�һ�ζ�ת��λ
//	if(rc_ctrl->key_V_flag==1&&rc_ctrl->key_shift_flag==1)   //RC_Ctl->key_ctrl_flag=1
//	{
//      plugging_reset_flag1=1;
//	} 	
//	
//	//�ֶ����ζ�ת��λ
//	if(rc_ctrl->key_F_flag==1&&rc_ctrl->key_shift_flag==1)   //RC_Ctl->key_ctrl_flag=1
//	{
//      plugging_reset_flag=1;
//	} 
//	/*�����ֶ�����*/
//	if(rc_ctrl->rc.s2==1&&(rc_ctrl->isOnline==1))
//	{
//		s1open_time++;
//	  if(s1open_time==1)
//	  {
//			rc_ctrl->key_ZT_flag=0;
//			rc_ctrl->key_XT_flag=0;
//			rc_ctrl->key_CT_flag=0;
//			rc_ctrl->key_VT_flag=0;
//			rc_ctrl->key_BT_flag=0;
//			rc_ctrl->key_GT_flag=0;
//	  }
//	  if(s1open_time>1)
//	  {

//			 if((rc_ctrl->isOnline==1)&&(rc_ctrl->key_ZT_flag%3==1))
//				{
//					 if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13)==GPIO_PIN_SET)
//					 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);		  	
//					else 
//					 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);		  	
//					rc_ctrl->key_ZT_flag=0;
//				}
//				if((rc_ctrl->isOnline==1)&&(rc_ctrl->key_XT_flag%3==1))
//				{
//					 if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12)==GPIO_PIN_SET)
//					 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);		  	
//					else 
//					 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);		  	
//					rc_ctrl->key_XT_flag=0;
//				}
//				if((rc_ctrl->isOnline==1)&&(rc_ctrl->key_CT_flag%3==1))
//				{
//					 if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15)==GPIO_PIN_SET)
//					 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);		  	
//					else 
//					 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);		  	
//					rc_ctrl->key_CT_flag=0;
//				}
//				if((rc_ctrl->isOnline==1)&&(rc_ctrl->key_VT_flag%3==1))
//				{
//					 if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)==GPIO_PIN_SET)
//					 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);		  	
//					else 
//					 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);		  	
//					rc_ctrl->key_VT_flag=0;
//				}
//				if((rc_ctrl->isOnline==1)&&(rc_ctrl->key_BT_flag%3==1))
//				{
//					 if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)==GPIO_PIN_SET)
//					 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);		  	
//					else 
//					 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);		  	
//					rc_ctrl->key_BT_flag=0;
//				}
//				if((rc_ctrl->isOnline==1)&&(rc_ctrl->key_GT_flag%3==1))
//				{
//					 if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11)==GPIO_PIN_SET)
//					 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);		  	
//					else 
//					 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);		  	
//					rc_ctrl->key_GT_flag=0;
//				}	 
//				auto_ore_aircat_mode1flag=0;
//				auto_ore_aircat_mode2flag=0;
//				auto_silver_grab_mode1flag=0;
//				auto_silver_grab_mode2flag=0;
//				auto_gold_grab_mode1flag=0;
//				auto_gold_grab_mode2flag=0;
//				auto_ore_conver_mode1_flag=0;
//				auto_ore_conver_mode2_flag=0;
//			}
//  }
//	
//				//ת��ģʽ
//		 if((rc_ctrl->rc.s2==2)&&(rc_ctrl->isOnline==1))
//	   {
//				if(rc_ctrl->rc.ch2-1024>300)
//					attitude->Motors_DM.pos_yaw = attitude->Motors_DM.pos_yaw - 0.0007;
//				if(rc_ctrl->rc.ch2-1024<-300)
//					attitude->Motors_DM.pos_yaw = attitude->Motors_DM.pos_yaw + 0.0007;
//				if(attitude->Motors_DM.pos_yaw>2.33)attitude->Motors_DM.pos_yaw=2.33;
//				if(attitude->Motors_DM.pos_yaw<0.33)attitude->Motors_DM.pos_yaw=0.33;
//			 
//				if(rc_ctrl->rc.ch1-1024>300)
//				{
//					attitude->Motors6020.target_out +=0.2;
//				}
//				else if(rc_ctrl->rc.ch1-1024<-300)
//				{
//					attitude->Motors6020.target_out -=0.2;
//				}
//			 
//			if(rc_ctrl->rc.ch0-1024>300) 
//				attitude->Motors2006.target_out += 10;
//			if(rc_ctrl->rc.ch0-1024<-300)
//				attitude->Motors2006.target_out -= 10;
//				
//			 if(attitude->Motors2006.target_angle>=12960)attiTude.Roll.Target_Angle=12960;
//			 if(attitude->Motors2006.target_angle<=-12960)attiTude.Roll.Target_Angle=-12960;
//	 }
//	if((rc_ctrl->rc.s2!=1)&&(rc_ctrl->isOnline==1))
//	{
//		s1open_time=0;
//	}
//}

///**
//  * @brief  ������Ұ
//  */
//void mining_mode(RC_Ctrl *rc_ctrl)
//{
//   if(mining_mode_flag1 == 1)		//�ջ�
//	 {
//		 auto_timer++;
//		 if(auto_timer == 1)
//		 {
//			 Auto_state = 1;
//		 }
//		 if(auto_timer<=600)
//		 {
//			 liftIng.lifting_target_1 = 10500;   //move 2.3
//			 liftIng.lifting_target_2 = 5840;
//			 Gasdynamic_claw(OPEN); 
//		 }
//		 if(auto_timer==800)
//		 {
//			 Gasdynamic_grab2(BACK);
//		 }
//		 if(auto_timer==1400)
//		 {
//			 liftIng.lifting_target_1 = 4000; 
//		 }
//		 if(auto_timer==1800)
//		 {
//		   auto_timer=0;
//			 rc_ctrl->key_Q_flag=0;
//			 Auto_state = 0;
//			 mining_mode_flag1 = 0;
//		 }
//	 }
//	 
//	 if(mining_mode_flag2 == 1)		//���
//	 {
//		 auto_timer++;
//		 if(auto_timer == 1)
//		 {
//			 Auto_state = 1;
//		 }
//		 if(auto_timer<=600)
//		 {
//			 liftIng.lifting_target_1 = 10500;
//			 liftIng.lifting_target_2 = 5840;
//		 }
//		 if(auto_timer==900)
//		 {
//			 Gasdynamic_grab2(OUT);
//		 }
//		 if(auto_timer==1300)
//		 {
//		   auto_timer=0;
//			 rc_ctrl->key_Q_flag=0;
//			 Auto_state = 0;
//			 mining_mode_flag2 = 0;
//		 }
//	 }
//}

///**
//  * @brief  �ֶ��ҿ�ģʽ
//  */
//void manual_mining(Attitude_t *attitude,RC_Ctrl* rc_ctrl,MecanumChassis* chassis,Lifting_t *lifting)     //�һ�ǰ������
//{
//  if(manual_mining_flag==1)
//	{
//		attitude->Motors2006.target_out-=rc_ctrl->mouse.x*roll_sinse;
//		
//		attitude->Motors6020.target_out+=rc_ctrl->mouse.y*pitch_sinse;
//		
//		if(rc_ctrl->key_ctrl_flag==1&&rc_ctrl->key_Z_flag==1)
//			attitude->Motors_DM.pos_yaw = attitude->Motors_DM.pos_yaw - 0.0007;
//		if(rc_ctrl->key_ctrl_flag==1&&rc_ctrl->key_X_flag==1)
//			attitude->Motors_DM.pos_yaw = attitude->Motors_DM.pos_yaw + 0.0007;
////		if(attitude->Yaw.Target_Angle>2.33)attitude->Yaw.Target_Angle=2.33;
////		if(attitude->Yaw.Target_Angle<0.33)attitude->Yaw.Target_Angle=0.33;
//		if(attitude->Motors_DM.pos_yaw>2.33)attitude->Motors_DM.pos_yaw=2.33;
//		if(attitude->Motors_DM.pos_yaw<0.33)attitude->Motors_DM.pos_yaw=0.33;
//				
//		if(rc_ctrl->key_ctrl_flag==1&&rc_ctrl->key_C_flag==1)
//		{
//			lifting->Lifting_left_2.Target_Angle-=lifting_vel_param;
//			lifting->Lifting_right_2.Target_Angle+=lifting_vel_param;
//		}
//		if(rc_ctrl->key_ctrl_flag==1&&rc_ctrl->key_V_flag==1)
//		{
//			lifting->Lifting_left_2.Target_Angle+=lifting_vel_param;
//			lifting->Lifting_right_2.Target_Angle-=lifting_vel_param;
//		}
//		
//		if(rc_ctrl->key_shift_flag==1&&rc_ctrl->key_Z_flag==1)  //RC_Ctl->key_shift_flag
//			   rc_ctrl->Chassis_Z_Integ=-700;
//			else if(rc_ctrl->key_shift_flag==1&&rc_ctrl->key_X_flag==1)  //RC_Ctl->key_shift_flag
//			   rc_ctrl->Chassis_Z_Integ=700;
//		  else rc_ctrl->Chassis_Z_Integ=0;
//		
//		
//	}

//}

///***************����Ϊ�Զ�ģʽ����***************/

///**
//  * @brief  �ս�ģʽһ,����˵�������ĵ�
//  */

//void Aircatch_modeone()
//{  
//	if(auto_ore_aircat_mode1flag==1)
//	{
//		 auto_pretimer++;
//		 if(auto_pretimer == 1)
//		 {
//		  	Auto_state = 1;
//		 }
//		 if(auto_pretimer<1200)
//		 {
//			 liftIng.lifting_target_1 = 33300;	//move 1.1
//			 liftIng.lifting_target_2 = 11881;
//			 Gasdynamic_storage_clamp(OPEN);
//		 }
//		 if(auto_pretimer==100)
//		 {
//			 Gasdynamic_claw(OPEN); 				//move 1.2
//			 Gasdynamic_storage_clamp(OPEN);
//			 Gasdynamic_storage_lift(UP);
//		 }
//		 if(auto_pretimer==500)					//move 1.3
//		 {
//			 Gasdynamic_grab1(OUT);
//       Gasdynamic_grab2(OUT);			 
//		 }
//		 
//		 if(auto_pretimer>=2000)
//		 {
//			photoelectric_detection();         //��צ����翪�ؼ��
//		 }
//		 if(photo_one_flag==1)             
//		 {
//			 auto_timer++;
//			 if(auto_timer==60)		          //���ƹ�翪�ؼ��ͼ�צ֮�����ʱ
//			 {
//			   Gasdynamic_claw(CLOSE);       //move 2.1 ��⵽�п�ʯ����צ�����պ�
//			 }
//			 
//			 if(auto_timer==1000)
//			 {
//				 Gasdynamic_grab1(BACK);			 //move 2.2
//				 Gasdynamic_grab2(BACK);  
//			 }						 
//			 if(auto_timer>=1600)
//			 {
//				 liftIng.lifting_target_1 = 2324;   //move 2.3
//				 liftIng.lifting_target_2 = 5840;
//			 }
//			 if(auto_timer==3000)
//			 {
//				 Gasdynamic_claw(OPEN);    			 //move 2.4
//			 }
// 
//			 if(auto_timer==3600)
//			 {
//					Gasdynamic_storage_lift(DOWN);		 //move 2.6
//			 }
//			 if(auto_timer==4600)
//			 {
//				  Gasdynamic_storage_clamp(CLOSE);
//			 }  
//			 if(auto_timer==4700)
//			 {
//				 auto_timer=0;
//				 photo_one_flag=0;
//				 auto_ore_aircat_mode1flag=0;
//         auto_pretimer=0;
//				 Auto_state=0;
//			 }
//		 }
//	 }
//}



///**
//  * @brief  �ս�ģʽ��,����˵�������ĵ�
//  */
//void Aircatch_modetwo()
//{ 
//	if(auto_ore_aircat_mode2flag==1)
//	{
//		 auto_pretimer++;
//		 if(auto_pretimer == 1)
//		 {
//		  	Auto_state = 1;
//		 }
//		 if(auto_pretimer<1000)
//		 {
//			 liftIng.lifting_target_1 = 32000;	//move 1.1
//			 liftIng.lifting_target_2 = 11881;
//		 }
//		 if(auto_pretimer==100)
//		 {
//			 Gasdynamic_claw(OPEN); 				//move 1.2
//			 Gasdynamic_storage_clamp(OPEN);
//		 }
//		 if(auto_pretimer==1200)					//move 1.3
//		 {
//			 Gasdynamic_grab1(OUT);
//       Gasdynamic_grab2(OUT);			 
//		 }
//		 if(auto_pretimer>=2000)
//		 {
//			 photoelectric_detection();         //��צ����翪�ؼ��
//		 }
//		 if(photo_one_flag==1)             
//		 {
//			 auto_timer++;
//			 if(auto_timer==60)		          //���ƹ�翪�ؼ��ͼ�צ֮�����ʱ
//			 {
//			   Gasdynamic_claw(CLOSE);       //move 2.1 ��⵽�п�ʯ����צ�����պ�
//			 }
//			 
//			 if(auto_timer==1000)
//			 {
//				 Gasdynamic_grab1(BACK);			 //move 2.2
//				 Gasdynamic_grab2(BACK);  
//			 }						 
//			 if(auto_timer>=1600)
//			 {
//				 liftIng.lifting_target_1 = 2324;   //move 2.3
//				 liftIng.lifting_target_2 = 5840;
//			 }
//			 if(auto_timer==3000)
//			 {
//				 Gasdynamic_claw(OPEN);    			 //move 2.4
//				 Gasdynamic_storage_lift(DOWN);
//			 }
//		   if(auto_timer==3300)
//			 {
//				  Gasdynamic_storage_clamp(CLOSE);
//			 }  
//			 if(auto_timer==3600)
//			 {
//				 auto_timer=0;
//				 photo_one_flag=0;
//				 auto_ore_aircat_mode2flag=0;
//         auto_pretimer=0;
//				 Auto_state=0;
//			 }
//		 }
//	 }
//}

///**
//  * @brief  �ս�ģʽ��,����˵�������ĵ�
//  */
//void Aircatch_modethree()
//{  
//	if(auto_ore_aircat_mode3flag==1)
//	{
//		 auto_pretimer++;
//		 if(auto_pretimer == 1)
//		 {
//		  	Auto_state = 1;
//		 }
//		 if(auto_pretimer<1000)
//		 {
//			 liftIng.lifting_target_1 = 32000;	//move 1.1
//			 liftIng.lifting_target_2 = 11881;
//		 }
//		 if(auto_pretimer==100)
//		 {
//			 Gasdynamic_claw(OPEN); 				//move 1.2
//		 }
//		 if(auto_pretimer==1200)					//move 1.3
//		 {
//			 Gasdynamic_grab1(OUT);
//       Gasdynamic_grab2(OUT);			 
//		 }
// 
//		 if(auto_pretimer>=2000)
//		 {
//			 photoelectric_detection();         //��צ����翪�ؼ��
//		 }
//		 if(photo_one_flag==1)             
//		 {
//			 auto_timer++;
//			 if(auto_timer==60)		          //���ƹ�翪�ؼ��ͼ�צ֮�����ʱ
//			 {
//			   Gasdynamic_claw(CLOSE);       //move 2.1 ��⵽�п�ʯ����צ�����պ�
//			 }
//			 
//			 if(auto_timer==1000)
//			 {
//				 Gasdynamic_grab1(BACK);  
//			 }						 
//			 if(auto_timer>=1600)
//			 {
//				 liftIng.lifting_target_1 = 2324;   //move 2.3
//				 liftIng.lifting_target_2 = 5840;
//			 } 

//			 if(auto_timer==2500)
//			 {
//				 auto_timer=0;
//				 photo_one_flag=0;
//				 auto_ore_aircat_mode3flag=0;
//         auto_pretimer=0;
//				 Auto_state=0;
//			 }
//		 }
//	 }
//}


///******************ȡ����******************/

///**
//  * @brief  ȡ����ģʽһ,����˵�������ĵ�
//  */
//void auto_silver_grab_modeone()
//{ 
//	if(auto_silver_grab_mode1flag==1)
//	{
//		auto_timer++;

//		 if(auto_timer == 1)
//		 {
//			 Auto_state = 1;
//		 }
//		 if(auto_timer<=1950)
//		 {
//			 if(auto_timer%5==0)
//			 {
//					auto_mecanumChassis_angle_control(&mecanumChassis,-20000,0,650);
//			 }
//		 }
//		 if(auto_timer<=700)
//		 {
//			 Gasdynamic_claw(OPEN);
//			 liftIng.lifting_target_1 = 0;
//			 liftIng.lifting_target_2 = 5840;
//			 Gasdynamic_storage_clamp(OPEN);
//		 }
//		 if((auto_timer>700)&&(auto_timer<=1000)) 
//		 {
//			 Gasdynamic_grab2(OUT);						//move02
//			 Gasdynamic_storage_lift(UP);
//		 }
//		 if(auto_timer==1600) 		//move03
//		 {
//				Gasdynamic_claw(CLOSE);
//		 }
//		 if((auto_timer>1950)&&(auto_timer<=6000))  		//move04
//		 {
//			 if(auto_timer%5==0)
//	     {
//		     auto_mecanumChassis_angle_control(&mecanumChassis,-20000,0,1150);    //8100
//	     }
//		 }
//		 if((auto_timer>2300)&&(auto_timer<=2900))  		//move04
//		 {
//				liftIng.lifting_target_1 = 11000;
//			  liftIng.lifting_target_2 = 11881;
//		 }
//		 if(auto_timer==3000) 		//move05
//		 {
//				Gasdynamic_grab2(BACK);
//		 }
//		 if((auto_timer>3800)&&(auto_timer<=4500)) 		//move06
//		 {
//				liftIng.lifting_target_1 = 2324;
//			  liftIng.lifting_target_2 = 5840;
//		 }
//		 if(auto_timer==4500) 		//move07
//		 {
//				Gasdynamic_claw(OPEN);
//		 }
//		 if(auto_timer==4700) 		//move08
//		 {
//				Gasdynamic_storage_lift(DOWN);
//		 }
//		 if(auto_timer==5900) 		//move08
//		 {
//			  Gasdynamic_storage_clamp(CLOSE);
//		 }
//		 if(auto_timer==6000) 		//END
//		 {
//				auto_timer=0;
//		    auto_silver_grab_mode1flag=0;
//			  Auto_state = 0;
//				mecanumChassis.IDoneAngle_uf =0;
//				mecanumChassis.IDtwoAngle_uf =0;
//				mecanumChassis.IDthreeAngle_uf =0;
//				mecanumChassis.IDfourAngle_uf =0;
//				target_0=0;
//				target_1=0;
//				target_2=0;
//				target_3=0;
//		 }
//	 }
//}


///**
//  * @brief  ȡ����ģʽ��,����˵�������ĵ�
//  */
//void auto_silver_grab_modetwo()
//{
//	if(auto_silver_grab_mode2flag==1)
//	{
//		auto_timer++;
//		 if(auto_timer == 1)
//		 {
//			 Auto_state = 1;
//		 }
//		 if(auto_timer<=1950)
//		 {
//			 if(auto_timer%5==0)
//			 {
//					auto_mecanumChassis_angle_control(&mecanumChassis,-20000,0,650);
//			 }
//		 }
//		 if(auto_timer<=700)
//		 {
//			 Gasdynamic_claw(OPEN);
//			 Gasdynamic_storage_clamp(OPEN);
//			 liftIng.lifting_target_1 = 0;
//			 liftIng.lifting_target_2 = 5840;
//		 }
//		 if((auto_timer>700)&&(auto_timer<=1000)) 
//		 {
//			 Gasdynamic_grab2(OUT);						//move02
//		 }
//		 if(auto_timer==1600) 		//move03
//		 {
//				Gasdynamic_claw(CLOSE);
//		 }
//		 if((auto_timer>1950)&&(auto_timer<=5100))  		//move04
//		 {
//			 if(auto_timer%5==0)
//	     {
//		     auto_mecanumChassis_angle_control(&mecanumChassis,-20000,0,1150);    //8100
//	     }
//		 }
//		 if((auto_timer>2300)&&(auto_timer<=2950))  		//move04
//		 {
//				liftIng.lifting_target_1 = 11000;
//			  liftIng.lifting_target_2 = 11881;
//		 }
//		 if(auto_timer==3000) 		//move05
//		 {
//				Gasdynamic_grab2(BACK);
//		 }
//		 if((auto_timer>3800)&&(auto_timer<=4500)) 		//move06
//		 {
//				liftIng.lifting_target_1 = 2324;
//			  liftIng.lifting_target_2 = 5840;
//		 }
//		 if(auto_timer==4500) 		//move07
//		 {
//				Gasdynamic_claw(OPEN);
//		 }
//		 if(auto_timer==4700) 		//move08
//		 {
//				Gasdynamic_storage_lift(DOWN);
//		 }
//		 if(auto_timer==4900) 		//move08
//		 {
//			  Gasdynamic_storage_clamp(CLOSE);
//		 }
//		 if(auto_timer==5100) 		//END
//		 {
//				auto_timer=0;
//		    auto_silver_grab_mode2flag=0;
//			  Auto_state = 0;
//			 	mecanumChassis.IDoneAngle_uf =0;
//				mecanumChassis.IDtwoAngle_uf =0;
//				mecanumChassis.IDthreeAngle_uf =0;
//				mecanumChassis.IDfourAngle_uf =0;
//				target_0=0;
//				target_1=0;
//				target_2=0;
//				target_3=0;
//		 }
//	 }
//}

///**
//  * @brief  ȡ����ģʽ��,����˵�������ĵ�
//  */
//void auto_silver_grab_modethree()
//{ 
//	if(auto_silver_grab_mode3flag==1)
//	{
//		auto_timer++;
//		 if(auto_timer == 1)
//		 {
//			 Auto_state = 1;
//		 }
//		 if(auto_timer<=2900)
//		 {
//			 if(auto_timer%5==0)
//			 {
//					auto_mecanumChassis_angle_control(&mecanumChassis,-20000,0,650);
//			 }
//		 }
//		 if(auto_timer<=700)
//		 {
//	  	//move01
//			 Gasdynamic_claw(OPEN);
//			 liftIng.lifting_target_1 = 10000;
//			 liftIng.lifting_target_2 = 5840;
//		 }
//		 if((auto_timer>700)&&(auto_timer<=1400)) 
//		 {
//			 Gasdynamic_grab2(OUT);						//move02
//		 }
//		 if((auto_timer>1500)&&(auto_timer<=2300)) 
//		 {
//				liftIng.lifting_target_1 = 0;
//			  liftIng.lifting_target_2 = 5840;
//		 }

//		 if(auto_timer==2300) 		//move03
//		 {
//				Gasdynamic_claw(CLOSE);
//		 }
//		 if((auto_timer>2300)&&(auto_timer<=4400))  		//move04
//		 {
//			 if(auto_timer%5==0)
//	     {
//		     auto_mecanumChassis_angle_control(&mecanumChassis,-20000,0,1150);    //8100
//	     }
//		 }
//		 if((auto_timer>2900)&&(auto_timer<=3900))  		//move04
//		 {
//				liftIng.lifting_target_1 = 11000;
//			  liftIng.lifting_target_2 = 5840;
//		 }
//		 if(auto_timer==4400) 		//END
//		 {
//				auto_timer=0;
//		    auto_silver_grab_mode3flag=0;
//			  Auto_state = 0;
//			  mecanumChassis.IDoneAngle_uf =0;
//				mecanumChassis.IDtwoAngle_uf =0;
//				mecanumChassis.IDthreeAngle_uf =0;
//				mecanumChassis.IDfourAngle_uf =0;
//				target_0=0;
//				target_1=0;
//				target_2=0;
//				target_3=0;
//		 }
//	 }
//}


///******************ȡ���******************/
///**
//* @brief  ȡ����Դ���ۿ�ģʽһ,����˵�������ĵ�
//  */
//void auto_golden_grab_modeone()
//{ 
//	if(auto_gold_grab_mode1flag==1)
//	{
//		auto_timer++;
//	

//		 if(auto_timer == 1)
//		 {
//			 Auto_state = 1;
//		 }
// 
//		 if(auto_timer<=700)
//		 {
//			 Gasdynamic_storage_clamp(OPEN);		  	//move01
//			 Gasdynamic_claw(OPEN);
//			 liftIng.lifting_target_1 = 0;
//			 liftIng.lifting_target_2 = 5840;
//		 }
//		 if((auto_timer>700)&&(auto_timer<=1000)) 
//		 {
//			 Gasdynamic_grab1(OUT);
//			 Gasdynamic_grab2(OUT);						//move02
//			 Gasdynamic_storage_lift(UP);
//		 }
//		 if(auto_timer==1600) 		//move03
//		 {
//				Gasdynamic_claw(CLOSE);
//		 }
//		 if((auto_timer>1700)&&(auto_timer<=2300))  		//move04
//		 {
//				liftIng.lifting_target_1 = 11000;
//			  liftIng.lifting_target_2 = 11881;
//		 }
//		 if(auto_timer==2400) 		//move05
//		 {
//			  Gasdynamic_grab1(BACK);
//				Gasdynamic_grab2(BACK);
//		 }
//		 if((auto_timer>3200)&&(auto_timer<=3900)) 		//move06
//		 {
//				liftIng.lifting_target_1 = 2324;
//			  liftIng.lifting_target_2 = 5840;
//		 }
//		 if(auto_timer==3900) 		//move07
//		 {
//				Gasdynamic_claw(OPEN);
//		 }
//		 if(auto_timer==4100) 		//move08
//		 {
//				Gasdynamic_storage_lift(DOWN);
//		 }
//		 if(auto_timer==5000) 		//move08
//		 {
//			  Gasdynamic_storage_clamp(CLOSE);
//		 }
//		 if(auto_timer==5100) 		//END
//		 {
//				auto_timer=0;
//		    auto_gold_grab_mode1flag=0;
//			  Auto_state = 0;
//		 }
//	 }
//}

///**
//  * @brief  ȡ���ģʽ��,����˵�������ĵ�
//  */
//void auto_golden_grab_modetwo()
//{
//	if(auto_gold_grab_mode2flag==1)
//	{
//		auto_timer++;
//		 if(auto_timer == 1)
//		 {
//			 Auto_state = 1;
//		 }
//		 if(auto_timer<=700)
//		 {
//			 Gasdynamic_storage_clamp(OPEN);		  	//move01
//			 Gasdynamic_claw(OPEN);
//			 liftIng.lifting_target_1 = 0;
//			 liftIng.lifting_target_2 = 5840;
//		 }
//		 if((auto_timer>700)&&(auto_timer<=1000)) 
//		 {
//			 Gasdynamic_grab1(OUT);
//			 Gasdynamic_grab2(OUT);						//move02
//		 }
//		 if(auto_timer==1500) 		//move03
//		 {
//				Gasdynamic_claw(CLOSE);
//		 }
//		 if((auto_timer>1600)&&(auto_timer<=2200))  		//move04
//		 {
//				liftIng.lifting_target_1 = 11000;
//			  liftIng.lifting_target_2 = 11881;
//		 }
//		 if(auto_timer==2300) 		//move05
//		 {
//			  Gasdynamic_grab1(BACK);			 
//				Gasdynamic_grab2(BACK);
//		 }
//		 if((auto_timer>3200)&&(auto_timer<=3900)) 		//move06
//		 {
//				liftIng.lifting_target_1 = 2324;
//			  liftIng.lifting_target_2 = 5840;
//		 }
//		 if(auto_timer==3900) 		//move07
//		 {
//				Gasdynamic_claw(OPEN);
//		 }
//		 if(auto_timer==4100) 		//move08
//		 {
//				Gasdynamic_storage_lift(DOWN);
//		 }
//		 if(auto_timer==4300) 		//move08
//		 {
//			  Gasdynamic_storage_clamp(CLOSE);
//		 }
//		 if(auto_timer==4600) 		//END
//		 {
//				auto_timer=0;
//		    auto_gold_grab_mode2flag=0;
//			  Auto_state = 0;
//		 }
//	 }
//}


///**
//  * @brief  ȡ���ģʽ��,����˵�������ĵ�
//  */
//void auto_golden_grab_modethree()
//{ 
//	if(auto_gold_grab_mode3flag==1)
//	{
//		auto_timer++;
//		 if(auto_timer == 1)
//		 {
//			 Auto_state = 1;
//		 }
//		 if(auto_timer<=800)
//		 {
//	  	//move01
//			 Gasdynamic_claw(OPEN);
//			 liftIng.lifting_target_1 = 10000;
//			 liftIng.lifting_target_2 = 5840;
//		 }
//		 if((auto_timer>1000)&&(auto_timer<=2000)) 
//		 {
//			 Gasdynamic_grab1(OUT);
//			 Gasdynamic_grab2(OUT);						//move02
//		 }
//		 if((auto_timer>2000)&&(auto_timer<=2700)) 
//		 {
//				liftIng.lifting_target_1 = 0;
//			  liftIng.lifting_target_2 = 5840;
//		 }
//		 if(auto_timer==3000) 		//move03
//		 {
//				Gasdynamic_claw(CLOSE);
//		 }
//		 if((auto_timer>3500)&&(auto_timer<=4200))  		//move04
//		 {
//				liftIng.lifting_target_1 = 11000;
//			  liftIng.lifting_target_2 = 5840;
//		 }
//		 if(auto_timer==4300) 		//move03
//		 {
//				Gasdynamic_grab1(BACK);
//		 }
//		 if(auto_timer==4800) 		//END
//		 {
//				auto_timer=0;
//		    auto_gold_grab_mode3flag=0;
//			  Auto_state = 0;
//		 }
//	 }
//}


///******************�ҿ�ǰ��******************/

//void auto_conver_mode1()
//{ 
//	if(auto_ore_conver_mode1_flag==1)
//	{
//		auto_timer++;
//		if(auto_timer == 1)
//		{
//			Auto_state = 1;
//      Gasdynamic_storage_clamp(OPEN);
//		}
//		if(auto_timer<=700)
//		{
//			Gasdynamic_storage_lift(UP);
//		  liftIng.lifting_target_1 = 11000;
//		  liftIng.lifting_target_2 = 5840;
//			
//		}
//		if(auto_timer==100) 		//END
//		{
//			Gasdynamic_exchange(OUT);  
//		}
//		if(auto_timer==1000) 		//END
//		{
//		  attiTude.Motors6020.target_out = 310;
//		  bypass_motor_open_flag=1;
//		}
//		if(auto_timer==2000) 		//END
//		{
//			Gasdynamic_exchange(BACK);  
//		}
//		if((auto_timer>3300)&&(auto_timer<4500))  		//move04
//		{
//			attiTude.Motors6020.target_out = attiTude.Motors6020.angle;
//			liftIng.lifting_target_1 = 23500;
//		}
//		if(auto_timer==4500)  		//move04
//		{
//			attiTude.Motors6020.target_out = 310;
//		}
//		if(auto_timer==5500) 		//END
//		{
//			Gasdynamic_exchange(OUT);  
//		}
//		if(auto_timer==6300) 		//move04
//		{
//			attiTude.Motors6020.target_out = -310;
//			liftIng.lifting_target_1 = 32000;
//			liftIng.lifting_target_2 = 1000;
//			Gasdynamic_storage_clamp(CLOSE);
//		}
//		if(auto_timer==6700) 		//move04
//		{
//			attiTude.Motors2006.target_out = 12900;
//		}		
//		if(auto_timer==8800) 		//END
//		{
//			auto_timer=0;
//			auto_ore_conver_mode1_flag=0;
//			Auto_state = 0;
//			manual_mining_flag=1;
//			auto_conver_reset_modeone1_flag = 1;
//		 }
//	 }
//}


///**
//  * @brief  �ҿ�ǰ��ģʽ2,����˵�������ĵ�
//  */
//void auto_conver_mode2()
//{ 
//	if(auto_ore_conver_mode2_flag==1)
//	{
//		auto_timer++;
//		if(auto_timer == 1)
//		{
//			Auto_state = 1;
//			liftIng.lifting_target_1 = liftIng.Lifting_right_1.Target_Angle;
//			liftIng.lifting_target_2 = liftIng.Lifting_right_2.Target_Angle;
//		}
//		if(auto_timer<=700)
//		{
//			Gasdynamic_storage_lift(DOWN);
//			Gasdynamic_storage_clamp(OPEN);
//		  liftIng.lifting_target_1 = 11000;
//		  liftIng.lifting_target_2 = 5840;
//		}
//		if(auto_timer==100) 		//END
//		{
//			Gasdynamic_exchange(OUT);  
//		}
//		if(auto_timer==1000) 		//END
//		{
//		  attiTude.Motors6020.target_out = 310;
//		  bypass_motor_open_flag=1;
//		}
//		if(auto_timer==2000) 		//END
//		{
//			Gasdynamic_exchange(BACK);  
//		}
//		if((auto_timer>3300)&&(auto_timer<4500))  		//move04
//		{
//			attiTude.Motors6020.target_out = attiTude.Motors6020.angle;
//			liftIng.lifting_target_1 = 23500;
//		}
//		if(auto_timer==4500)  		//move04
//		{
//			attiTude.Motors6020.target_out = 310;
//		}
//		if(auto_timer==5500) 		//END
//		{
//			Gasdynamic_exchange(OUT);  
//		}
//		if(auto_timer==6300) 		//move04
//		{
//			attiTude.Motors6020.target_out = -310;
//			liftIng.lifting_target_1 = 32000;
//			liftIng.lifting_target_2 = 1000;
//			Gasdynamic_storage_clamp(CLOSE);
//		}
//		if(auto_timer==6700) 		//move04
//		{
//			attiTude.Motors2006.target_out = 12900;
//		}		
//		if(auto_timer==8800) 		//END
//		{
//			auto_timer=0;
//			auto_ore_conver_mode2_flag=0;
//			Auto_state = 0;
//			manual_mining_flag=1;
//			auto_conver_reset_modeone1_flag = 1;
//		 }
//	 }
//}

///**
//  * @brief  �ҿ�ǰ��ģʽ3,����˵�������ĵ�
//  */
//void auto_conver_mode3()
//{ 
//	if(auto_ore_conver_mode3_flag==1)
//	{
//		auto_timer++;
//		if(auto_timer == 1)
//		{
//			Auto_state = 1;
//		}
//		if(auto_timer<=700)
//		{
//			Gasdynamic_storage_lift(DOWN);
//			Gasdynamic_storage_clamp(OPEN);
//		  liftIng.lifting_target_1 = 20500;
//		  liftIng.lifting_target_2 = 12525;
//		}
//		if(auto_timer==100) 		//END
//		{
//			Gasdynamic_exchange(OUT);  
//		}
//		if(auto_timer==1000) 		//END
//		{
//		  attiTude.Motors6020.target_out = 310;
//		  bypass_motor_open_flag=1;
//		}
//		if(auto_timer==2000) 		//END
//		{
//			Gasdynamic_exchange(BACK);  
//		}
//		if((auto_timer>3300)&&(auto_timer<4500))  		//move04
//		{
//			attiTude.Motors6020.target_out = attiTude.Motors6020.angle;
//			liftIng.lifting_target_1 = 31000;
//		}
//		if(auto_timer==4500)  		//move04
//		{
//			attiTude.Motors6020.target_out = 310;
//		}
//		if(auto_timer==5300) 		//END
//		{
//			Gasdynamic_exchange(OUT);
//			Gasdynamic_grab2(BACK);
//		}
//		if(auto_timer==7000) 		//END
//		{
//			liftIng.lifting_target_2 = 5840;
//			attiTude.Motors6020.target_out = -310;
//			
//		}
//		
//		if(auto_timer==8000) 		//END
//		{
//			liftIng.lifting_target_1 = 2324;
//		}
//		if(auto_timer==9500) 		//move04
//		{
//			
//			Gasdynamic_claw(OPEN);
//			//liftIng.lifting_target_1 = 32000;
//		}
//		if(auto_timer==10000) 		//move04
//		{
//			attiTude.Motors2006.target_out = 12900;
//		}
//		if(auto_timer==11000) 		//move04
//		{
//			liftIng.lifting_target_1 = 32000;
//			liftIng.lifting_target_2 = 1000;
//			Gasdynamic_storage_clamp(CLOSE);
//		}		
//		if(auto_timer==13000) 		//END
//		{
//			auto_timer=0;
//			auto_ore_conver_mode3_flag=0;
//			Auto_state = 0;
//			manual_mining_flag=1;
//			auto_conver_reset_modeone1_flag = 1;
//		 }
//	 }
//}

///**
//  * @brief  ����IO���Ʒ�װ
//  */
//void Gasdynamic_grab1(uint8_t state)		//ȡ��һ��ǰ�죬��ʼΪRESET-BACK-0
//{
//	if(state==1)
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);		
//	if(state==0)
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
//}

//void Gasdynamic_grab2(uint8_t state)		//ȡ�����ǰ�죬��ʼΪRESET-BACK-0
//{
//	if(state==1)
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);		
//	if(state==0)
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
//}

//void Gasdynamic_storage_lift(uint8_t state)		//����̧��PE11 ��ʼΪRESET-UP-0
//{
//	if(state==0)
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);		
//	if(state==1)
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
//}

//void Gasdynamic_storage_clamp(uint8_t state)		//����н�-PE10 ��ʼΪSET-OPEN-0
//{
//	if(state==1)
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);		
//	if(state==0)
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
//}

//void Gasdynamic_claw(uint8_t state)							//��צ-PE15 ��ʼΪRESET-OPEN-0
//{
//	if(state==1)
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);		
//	if(state==0)
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
//}

//void Gasdynamic_exchange(uint8_t state)						//�һ�ǰ��-PE14 ��ʼΪRESET-BACK-0
//{
//	if(state==1)
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);		
//	if(state==0)
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
//}

//	uint8_t grab1_state;
//	uint8_t grab2_state;
//	uint8_t exchange_state;
//	uint8_t claw_state;
//	uint8_t storage1_state;
//	uint8_t storage2_state;


///**
//  * @brief  Ctrl+F �˳��ҿ�ģʽ/�˳��ս�
//  */
//void auto_ore_reset(Attitude_t *attitude)
//{
//   if(conver_reset1==1)			//�˳��ҿ�ģʽ
//	 {
//		 auto_timer++;
//		 if(auto_timer == 1)
//		 {
//			 Auto_state = 1;
//			 liftIng.lifting_target_1 = liftIng.Lifting_right_1.Target_Angle;
//			 liftIng.lifting_target_2 = liftIng.Lifting_right_2.Target_Angle;
//		 }
//		 bypass_motor_open_flag=0;
//		 manual_mining_flag=0;
//	  if(auto_timer==500)
//		{
//			attitude->Motors2006.target_out=0;
//			attitude->Motors6020.target_out=0;
//			attitude->Motors_DM.pos_yaw=1.33;
//		}
//		if(auto_timer==800)
//		{
//		   liftIng.lifting_target_2 = 5840;
//			 liftIng.lifting_target_1 = 3000;
//		}
//		if(auto_timer==1000)
//		{
//		   Gasdynamic_exchange(BACK);
//		}
//		if(auto_timer>1200)
//		{
//			auto_timer=0;
//			Auto_state=0;
//			conver_reset1=0;
//			auto_conver_reset_modeone1_flag=0;
//		}
//	 }

//	 if(conver_reset2==1)		//�˳��ս�
//	 {
//	   auto_timer++;
//		 if(auto_timer == 1)
//		 {
//			 Auto_state = 1;
//		 }
//		 auto_ore_aircat_mode1flag=0;
//		 auto_ore_aircat_mode2flag=0;
//		 auto_ore_aircat_mode3flag=0;
//		 auto_pretimer=0;
//		 Gasdynamic_claw(OPEN);

//		 if(auto_timer==1)
//		 {
//		   Gasdynamic_grab1(BACK);
//	     Gasdynamic_grab2(BACK);
//		 }
//		 if(auto_timer<1500)
//		 {
//		  liftIng.lifting_target_1 = 2000;
//		  liftIng.lifting_target_2 = 5840;
//		 }
//		 if(auto_timer>=1500)
//		 {
//			 conver_reset2=0;
//			 auto_timer=0;
//			 Auto_state=0;
//		 }
//	 }
//}
//void land_mining()
//{
//	if(auto_ore_land_mining_flag==1)
//	{
//	  auto_timer++;
//		if(auto_timer == 1)
//		{
//			Auto_state = 1;
//		}
//	  if(auto_timer<1700)
//		{
//		  liftIng.lifting_target_1 = 4000;//4000
//			liftIng.lifting_target_2 = 5900;  //5900
//		}
//		if(auto_timer<1200)
//		{
//		  if(auto_timer%5==0)
//			 {
//					auto_mecanumChassis_angle_control(&mecanumChassis,-20000,0,0);
//			 }
//		}
//		
//		if(auto_timer>=1200)
//		{
//			bypass_motor_open_flag=1;
//		  Gasdynamic_exchange(OUT);	
//			if(auto_timer%5==0)
//			 {
//					auto_mecanumChassis_angle_control(&mecanumChassis,-15000,0,3000);
//			 }
//		}
//		if(auto_timer>=1700&&auto_timer<=3000)
//		{
//		  liftIng.lifting_target_1 = 4000;
//			liftIng.lifting_target_2 = 10000;
//		}
//		if(auto_timer>3000)
//		{
//		  liftIng.lifting_target_1 = 4000;
//			liftIng.lifting_target_2 = 5900;
//		}
//		if(auto_timer>4000)
//		{
//		  auto_timer=0;
//			auto_ore_land_mining_flag=0;
//			Auto_state = 0;
//			mecanumChassis.IDoneAngle_uf =0;
//			mecanumChassis.IDtwoAngle_uf =0;
//			mecanumChassis.IDthreeAngle_uf =0;
//			mecanumChassis.IDfourAngle_uf =0;
//			target_0=0;
//			target_1=0;
//			target_2=0;
//			target_3=0;
//		}
//	}
//}
