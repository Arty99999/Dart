//#include "interaction.h"
//#include "string.h"
//#include "usart.h"
//#include "brain.h"
//#include "referee.h"
//#include "driver_timer.h"
//#include "hardware_config.h"
//#include "control_logic.h"
//#include "ore.h"
//#include "lifting.h"
//#include "dr16.h"


//uint8_t UI_test=0;
//uint16_t x_t1;
//uint16_t x_t2;
//uint16_t x_t3;
//uint16_t x_t4;


///*

//			 ***************�������*****************
//	Data_Pack[n]���ڴ�����Ƿ�������ϵͳ�����ݣ�������ͨ������3���͸�����ϵͳ������ϵͳ
//	����Э�齫�ڲ����ֽ�������ʾ�û��Զ���UI���档
//	Data_Pack[0]-Data_Pack[4]��֡ͷ�����е����ݺ����ڲ���ϵͳЭ���ֲ��￪ͷ������������
//	Ҫ�ĵľ�ֻ��Data_Pack[1]��Data_Pack[2]����λ������λ���ڴ�����ݳ��ȣ����Ʋ�ͬ��ͼ��
//	��������ݳ����ǲ�һ���ģ�

//	����ID				���ݳ���				����
//	0x0101					21				����һ��ͼ��
//	0x0102					36				��������ͼ��
//	0x0103					81				�������ͼ��
//	0x0104					111				�����߸�ͼ��
//	0x0110					51				�����ַ�ͼ��

//	���У�ÿ��ͼ����ռ����λΪ15λ���ַ�ռ45λ�����ݶ�ID���ݡ�������ID��������ID��ռ2λ��Ҳ��������
//	��ǰ��λ���ڴ��ID��Ϣ������21=6+15,36=6+15*2,�������ơ�

//	���Ƚ���һ��ͼ�νṹ�����ڴ��ͼ�����ݣ��ڶ�����ع��ܺ������ṹ�巢�ͳ�ȥ�����UI�������ʾ


//				**************ʹ�÷���****************
//	����Ҫ��ʾ��ͼ�κ������ں���Client_Display��������������������ﲻͣ����Client_Display��������
// 
// */
// 
// uint8_t Data_Pack[128]__attribute__((at(0x24065000)));
//  
//UI_t UI;


///*UI�����������ڶ�ʱ���ص������*/
//void UI_show(void)
//{
//	
//	
///*���׿��ձ�־λ��ֵ(0��Ϊ��ʼ״̬����ʾΪ��Ȧ)*/
//	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)==GPIO_PIN_RESET)	//PE10	����н�
//		control_UI_all.storage2_state=1;
//	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)==GPIO_PIN_SET) 
//		control_UI_all.storage2_state=0;
//	
//	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11)==GPIO_PIN_SET)	//PE11	����̧��
//		control_UI_all.storage1_state=1;
//	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11)==GPIO_PIN_RESET) 
//		control_UI_all.storage1_state=0;

//	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12)==GPIO_PIN_SET)	//PE12	��צ����ǰ��
//		control_UI_all.grab2_state=1;
//	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12)==GPIO_PIN_RESET) 
//		control_UI_all.grab2_state=0;
//	
//	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13)==GPIO_PIN_SET)	//PE13	��צһ��ǰ��
//		control_UI_all.grab1_state=1;
//	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13)==GPIO_PIN_RESET) 
//		control_UI_all.grab1_state=0;	

//	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)==GPIO_PIN_SET)	//PE14	�ҿ�ǰ��
//		control_UI_all.exchange_state=1;
//	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)==GPIO_PIN_RESET) 
//		control_UI_all.exchange_state=0;		
//	
//	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15)==GPIO_PIN_SET)	//PE15	��צ�н�
//		control_UI_all.claw_state=1;
//	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15)==GPIO_PIN_RESET) 
//		control_UI_all.claw_state=0;

//	control_UI_all.lift1 = liftIng.Lifting_right_1.Target_Angle/56.71;		// 
//	control_UI_all.lift2 = -liftIng.Lifting_right_2.Target_Angle/59.13;

//	if(bypass_motor_open_flag==1||rc_Ctrl.key_R_flag%2==1) 
//		control_UI_all.pump_state=1;
//	else control_UI_all.pump_state=0;

//	 control_UI_all.speed=speed_sence;
////		 if(rc_Ctrl.mouse.press_l_flag==1)
////			 control_UI_all.speed=2;
////	 if(rc_Ctrl.mouse.press_r_flag%2==1&&rc_Ctrl.mouse.press_l_flag==0)
////      control_UI_all.speed=0;

////	  if(rc_Ctrl.mouse.press_r_flag%2==0&&rc_Ctrl.mouse.press_l_flag==0)
////	 control_UI_all.speed=3;

//	referee_draw_basic_(referee2022.game_robot_status.robot_id,tim14.UI_Time);
//	referee_draw_patterning_all_1(referee2022.game_robot_status.robot_id,tim14.UI_Time);

//}

///**
//  * @brief  ��0��ͼ�㻭�ַ��ĳ����30���ַ�
//	*/
//void referee_draw_string(uint8_t robot_id,char *string,uint8_t string_dex,uint8_t control_way,uint8_t color,uint8_t on_layer, uint16_t x,uint16_t y,uint16_t large,uint16_t width)
//{
//	uint16_t crc16_temp;
//	UI.ext_client_custom_character.frame_header.SOF = 0xA5;
//	UI.ext_client_custom_character.frame_header.data_length = 51;   //< sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)
//	UI.ext_client_custom_character.frame_header.seq = 2;
//	UI.ext_client_custom_character.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_character.frame_header,4,0xFF); 
//	UI.ext_client_custom_character.cmd_id = 0x0301;
//	UI.ext_client_custom_character.ext_student_interactive_header_data.data_cmd_id = 0x0110;
//	UI.ext_client_custom_character.ext_student_interactive_header_data.sender_ID = robot_id;
//	UI.ext_client_custom_character.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
//	UI.ext_client_custom_character.grapic_data_struct.graphic_name[0]=string_dex;
//	UI.ext_client_custom_character.grapic_data_struct.graphic_name[1]=0x0;
//	UI.ext_client_custom_character.grapic_data_struct.graphic_name[2]=0x0;
//	UI.ext_client_custom_character.grapic_data_struct.operate_tpye = control_way;
//	UI.ext_client_custom_character.grapic_data_struct.graphic_tpye = 0x07;         //< �����ַ���
//	UI.ext_client_custom_character.grapic_data_struct.layer = on_layer;
//	UI.ext_client_custom_character.grapic_data_struct.color = color;
//	UI.ext_client_custom_character.grapic_data_struct.start_angle = large;
//	UI.ext_client_custom_character.grapic_data_struct.end_angle = 20;
//	UI.ext_client_custom_character.grapic_data_struct.width = width;
//	UI.ext_client_custom_character.grapic_data_struct.start_x = x;
//	UI.ext_client_custom_character.grapic_data_struct.start_y = y;
//	UI.ext_client_custom_character.grapic_data_struct.radius = 0;
//	UI.ext_client_custom_character.grapic_data_struct.end_x = 0;
//	UI.ext_client_custom_character.grapic_data_struct.end_y = 0;
//	memset(UI.ext_client_custom_character.data, 0, 30);					//< ÿ�λ���ǰ����ϴ��ַ�������
//	strcpy((char *)UI.ext_client_custom_character.data, string);
//	
//	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_character, 58, 0xFFFF);
//	UI.ext_client_custom_character.CRC16[0] = crc16_temp & 0xFF;
//	UI.ext_client_custom_character.CRC16[1] = crc16_temp >> 8;
//	HAL_UART_Transmit_DMA(&huart2,(unsigned char*)&UI.ext_client_custom_character,60);
//}

///*����ɾ��ͼ�㣨Ӧ���ܳɣ���û�⣩*/
//void referee_graphic_delete(uint8_t del_layer, uint8_t operation, uint8_t robot_id) 
//{
//	uint16_t crc16_temp;
//	UI.ext_client_graphic_delete.frame_header.SOF = 0xA5;
//	UI.ext_client_graphic_delete.frame_header.data_length = 8;   //< sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)
//	UI.ext_client_graphic_delete.frame_header.seq = 0;
//	UI.ext_client_graphic_delete.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_graphic_delete.frame_header,4,0xFF); 
//	UI.ext_client_graphic_delete.cmd_id = 0x0301;
//	UI.ext_client_graphic_delete.ext_student_interactive_header_data.data_cmd_id = 0x0100;
//	UI.ext_client_graphic_delete.ext_student_interactive_header_data.sender_ID = robot_id;
//	UI.ext_client_graphic_delete.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
//	UI.ext_client_graphic_delete.ext_client_custom_graphic_delete.operate_tpye=operation;
//	UI.ext_client_graphic_delete.ext_client_custom_graphic_delete.layer=del_layer;
//	
//	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_graphic_delete, 15, 0xFFFF);
//	UI.ext_client_graphic_delete.CRC16[0] = crc16_temp & 0xFF;
//	UI.ext_client_graphic_delete.CRC16[1] = crc16_temp >> 8;
//	HAL_UART_Transmit_DMA(&huart2,(unsigned char*)&UI.ext_client_graphic_delete,17);
//}


///**
//  * @brief  �������ͼ������ĺ���
//	*/
//void draw_graph(graphic_data_struct_t *patterning, uint16_t index, uint8_t control_way, uint8_t graph, uint8_t layer, uint8_t color, uint8_t Sa, uint8_t Ea, uint8_t With,  uint16_t x,uint16_t y, uint8_t R, uint16_t Ex, uint16_t Ey)
//{
//	(*patterning).graphic_name[0]=index;
//	(*patterning).graphic_name[1]=0;
//	(*patterning).graphic_name[2]=0;
//	(*patterning).operate_tpye=control_way;
//	(*patterning).graphic_tpye=graph;
//	(*patterning).layer=layer;
//	(*patterning).color=color;
//	(*patterning).start_angle=Sa;
//	(*patterning).end_angle=Ea;
//	(*patterning).width=With;
//	(*patterning).start_x=x;
//	(*patterning).start_y=y;
//	(*patterning).radius=R;
//	(*patterning).end_x=Ex;
//	(*patterning).end_y=Ey;
//}


//void referee_draw_basic_1(uint8_t robot_id)
//{
//	uint16_t crc16_temp;
//	UI.ext_client_custom_graphic_patterning_first.frame_header.SOF = 0xA5;
//	UI.ext_client_custom_graphic_patterning_first.frame_header.data_length = 111;
//	UI.ext_client_custom_graphic_patterning_first.frame_header.seq = 0;
//	UI.ext_client_custom_graphic_patterning_first.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_first.frame_header,4,0xFF); 
//	UI.ext_client_custom_graphic_patterning_first.cmd_id = 0x0301;
//	UI.ext_client_custom_graphic_patterning_first.ext_student_interactive_header_data.data_cmd_id = 0x0104;
//	UI.ext_client_custom_graphic_patterning_first.ext_student_interactive_header_data.sender_ID = robot_id;
//	UI.ext_client_custom_graphic_patterning_first.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
//	
//	draw_graph(&UI.ext_client_custom_graphic_patterning_first.grapic_data_struct[0],90,ADD,LINE,2,GREEN,0,0,2,1080,0,0,1080,780); 
//	draw_graph(&UI.ext_client_custom_graphic_patterning_first.grapic_data_struct[1],91,ADD,LINE,2,GREEN,0,0,2,778,0,0,778,780); 
//	//������ֱ׼��
// 	
//	draw_graph(&UI.ext_client_custom_graphic_patterning_first.grapic_data_struct[2],92,ADD,LINE,2,YELLOW,0,0,2,835,300,0,835,1080); 
//	draw_graph(&UI.ext_client_custom_graphic_patterning_first.grapic_data_struct[3],93,ADD,LINE,2,YELLOW,0,0,2,1015,300,0,1015,1080); 

//	
//	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_first, 118, 0xFFFF);
//	UI.ext_client_custom_graphic_patterning_first.CRC16[0] = crc16_temp & 0xFF;
//	UI.ext_client_custom_graphic_patterning_first.CRC16[1] = crc16_temp >> 8;
//	
//	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_patterning_first,sizeof(UI.ext_client_custom_graphic_patterning_first));

//	HAL_UART_Transmit_DMA(&huart2,Data_Pack,120);
//}


//void referee_draw_basic_1_test(uint8_t robot_id)
//{
//	uint16_t crc16_temp;
//	UI.ext_client_custom_graphic_patterning_first.frame_header.SOF = 0xA5;
//	UI.ext_client_custom_graphic_patterning_first.frame_header.data_length = 111;
//	UI.ext_client_custom_graphic_patterning_first.frame_header.seq = 0;
//	UI.ext_client_custom_graphic_patterning_first.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_first.frame_header,4,0xFF); 
//	UI.ext_client_custom_graphic_patterning_first.cmd_id = 0x0301;
//	UI.ext_client_custom_graphic_patterning_first.ext_student_interactive_header_data.data_cmd_id = 0x0104;
//	UI.ext_client_custom_graphic_patterning_first.ext_student_interactive_header_data.sender_ID = robot_id;
//	UI.ext_client_custom_graphic_patterning_first.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
//	
//	draw_graph(&UI.ext_client_custom_graphic_patterning_first.grapic_data_struct[0],90,MODIFY,LINE,2,GREEN,0,0,2,x_t1,0,0,x_t1,780); 
//	draw_graph(&UI.ext_client_custom_graphic_patterning_first.grapic_data_struct[1],91,MODIFY,LINE,2,GREEN,0,0,2,x_t2,0,0,x_t2,780); 
//	//������ֱ׼��
// 	
//	draw_graph(&UI.ext_client_custom_graphic_patterning_first.grapic_data_struct[2],92,MODIFY,LINE,2,YELLOW,0,0,2,x_t3,300,0,x_t3,1080); 
//	draw_graph(&UI.ext_client_custom_graphic_patterning_first.grapic_data_struct[3],93,MODIFY,LINE,2,YELLOW,0,0,2,x_t4,300,0,x_t4,1080); 

//	
//	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_first, 118, 0xFFFF);
//	UI.ext_client_custom_graphic_patterning_first.CRC16[0] = crc16_temp & 0xFF;
//	UI.ext_client_custom_graphic_patterning_first.CRC16[1] = crc16_temp >> 8;
//	
//	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_patterning_first,sizeof(UI.ext_client_custom_graphic_patterning_first));

//	HAL_UART_Transmit_DMA(&huart2,Data_Pack,120);
//}






//void referee_draw_basic_(uint8_t robot_id,uint32_t cnt)
//{
//	/*UI�����ַ���ʶ*/
//	char *bypass="bypass";
//	char *grab1="grab1";
//	char *grab2="grab2";
//	char *exchange="exchange";
//	char *claw="claw";
//	char *storage1="storage1";
//	char *storage2="storage2";
//	char *speed="speed";
//	char *pitch="pitch";
//	char *yaw="yaw";

//	
//	if(cnt%156==0&&cnt<=7800&&cnt!=0)
//		referee_draw_basic_1(robot_id);
//	if(cnt%156==12&&cnt<=7800&&cnt!=0)
//		referee_draw_string(robot_id, bypass, 10,ADD, YELLOW, 3, 120, 760,20,2);
//	if(cnt%156==24&&cnt<=7800&&cnt!=0)
//		referee_draw_string(robot_id, grab1, 11,ADD, YELLOW, 3, 745, 240,10,1);
//	if(cnt%156==36&&cnt<=7800&&cnt!=0)
//		referee_draw_string(robot_id, grab2, 12,ADD, YELLOW, 3, 975, 240,10,1);
//	if(cnt%156==48&&cnt<=7800&&cnt!=0)
//		referee_draw_string(robot_id, exchange, 13,ADD, YELLOW, 3, 975, 280,10,1);
//	if(cnt%156==60&&cnt<=7800&&cnt!=0)
//		referee_draw_string(robot_id, claw, 14,ADD, YELLOW, 3, 745, 280,10,1);
//	if(cnt%156==72&&cnt<=7800&&cnt!=0)
//		referee_draw_string(robot_id, storage1, 15,ADD, YELLOW, 3, 745, 200,10,1);
//	if(cnt%156==84&&cnt<=7800&&cnt!=0)
//		referee_draw_string(robot_id, storage2, 16,ADD, YELLOW, 3, 975, 200,10,1);
//	if(cnt%156==96&&cnt<=7800&&cnt!=0)
//		referee_draw_string(robot_id, speed, 17,ADD, YELLOW, 3, 1600, 760,30,3);
////	if(cnt%156==108&&cnt<=7800&&cnt!=0)
////		referee_draw_string(robot_id, pitch, 18,ADD, YELLOW, 3, 1600, 600,20,2);
////	if(cnt%156==120&&cnt<=7800&&cnt!=0)
////		referee_draw_string(robot_id, yaw, 19,ADD, YELLOW, 3, 1600, 520,20,2);


////	if(cnt%50==0&&cnt>7800)
////		referee_draw_basic_1_test(robot_id);
//	
//}

//void referee_draw_patterning_all_1(uint8_t robot_id,uint32_t cnt)
//{

//	if(cnt%156==132&&cnt<=7800&&cnt!=0)
//		referee_draw_patterning_1_basic(robot_id);
//	if(cnt%156==144&&cnt<=7800&&cnt!=0)
//		referee_draw_patterning_2_basic(robot_id);
//	if(cnt%30==0&&cnt>7800)
//		referee_draw_patterning_1(robot_id);
//	if(cnt%30==15&&cnt>7800)
//		referee_draw_patterning_2(robot_id);
//}



//void referee_draw_patterning_1(uint8_t robot_id)
//	{
//	
//	control_UI_all.t1=(uint8_t)(control_UI_all.lift1*1000)&0xff;
//	control_UI_all.t2=(float)(((uint8_t)(control_UI_all.lift1*1000)>>8)&0xff)/4.0;
//	control_UI_all.t3=(float)(((uint8_t)(control_UI_all.lift1*1000)>>16)&0xff)/32.0;

//	
//	uint16_t crc16_temp;
//	UI.ext_client_custom_graphic_patterning_third.frame_header.SOF = 0xA5;
//	UI.ext_client_custom_graphic_patterning_third.frame_header.data_length =  111;
//	UI.ext_client_custom_graphic_patterning_third.frame_header.seq = 3;
//	UI.ext_client_custom_graphic_patterning_third.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_third.frame_header,4,0xFF); 
//	UI.ext_client_custom_graphic_patterning_third.cmd_id = 0x0301;
//	UI.ext_client_custom_graphic_patterning_third.ext_student_interactive_header_data.data_cmd_id = 0x0104;
//	UI.ext_client_custom_graphic_patterning_third.ext_student_interactive_header_data.sender_ID = robot_id;
//	UI.ext_client_custom_graphic_patterning_third.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
//	

//	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[0],70,MODIFY,INT,4,GREEN,20,0,2,120,680,(uint16_t)(control_UI_all.lift1)&0XFF,((uint16_t)(control_UI_all.lift1)>>8)&0xff,0);   
//	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[1],71,MODIFY,INT,4,GREEN,20,0,2,350,680,(uint16_t)(control_UI_all.lift2)&0XFF,((uint16_t)(control_UI_all.lift2)>>8)&0xff,0);   
//	//̧��1��̧��2
//		
//	if(control_UI_all.pump_state==1)//bypass
//		draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[2],72,MODIFY,CIRCLE,4,GREEN,0,0,4,270,750,15,0,0);   
//	else
//		draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[2],72,MODIFY,CIRCLE,4,FUCHSIA,0,0,4,270,750,15,0,0);   

//	if(control_UI_all.grab1_state==1)//grab1
//		draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[3],73,MODIFY,CIRCLE,4,GREEN,0,0,4,925,230,8,0,0);   
//	else
//		draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[3],73,MODIFY,CIRCLE,4,FUCHSIA,0,0,4,925,230,8,0,0);   

//	if(control_UI_all.grab2_state==1)//grab2
//		draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[4],74,MODIFY,CIRCLE,4,GREEN,0,0,4,1155,230,8,0,0);   
//	else
//		draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[4],74,MODIFY,CIRCLE,4,FUCHSIA,0,0,4,1155,230,8,0,0);   

//	if(control_UI_all.exchange_state==1)//exchange
//		draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[5],75,MODIFY,CIRCLE,4,GREEN,0,0,4,1155,270,8,0,0);   
//	else
//		draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[5],75,MODIFY,CIRCLE,4,FUCHSIA,0,0,4,1155,270,8,0,0);   
//		
//	if(control_UI_all.claw_state==1)//claw
//		draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[6],76,MODIFY,CIRCLE,4,GREEN,0,0,4,925,270,8,0,0);   
//	else
//		draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[6],76,MODIFY,CIRCLE,4,FUCHSIA,0,0,4,925,270,8,0,0);   
//		
//		
//	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_third, 118, 0xFFFF);
//	UI.ext_client_custom_graphic_patterning_third.CRC16[0] = crc16_temp & 0xFF;
//	UI.ext_client_custom_graphic_patterning_third.CRC16[1] = crc16_temp >> 8;
//	
//	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_patterning_third,sizeof(UI.ext_client_custom_graphic_patterning_third));
//	
//	HAL_UART_Transmit_DMA(&huart2,Data_Pack,120);
//}

//void referee_draw_patterning_1_basic(uint8_t robot_id)
//{


//	
//	uint16_t crc16_temp;
//	UI.ext_client_custom_graphic_patterning_third.frame_header.SOF = 0xA5;
//	UI.ext_client_custom_graphic_patterning_third.frame_header.data_length =  111;
//	UI.ext_client_custom_graphic_patterning_third.frame_header.seq = 3;
//	UI.ext_client_custom_graphic_patterning_third.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_third.frame_header,4,0xFF); 
//	UI.ext_client_custom_graphic_patterning_third.cmd_id = 0x0301;
//	UI.ext_client_custom_graphic_patterning_third.ext_student_interactive_header_data.data_cmd_id = 0x0104;
//	UI.ext_client_custom_graphic_patterning_third.ext_student_interactive_header_data.sender_ID = robot_id;
//	UI.ext_client_custom_graphic_patterning_third.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
//	

//	/*�˴�������д�߸�ͼ��*/	
//	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[0],70,ADD,INT,4,YELLOW,20,0,2,120,680,control_UI_all.lift1,0,0);   
//	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[1],71,ADD,INT,4,YELLOW,20,0,2,350,680,control_UI_all.lift2,0,0);   
//	//̧��1��̧��2

//	
//	
//	
//	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[2],72,ADD,CIRCLE,4,YELLOW,0,0,4,270,750,15,0,0);   
//	//pump		
//	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[3],73,ADD,CIRCLE,4,YELLOW,0,0,4,925,230,8,0,0);   
//	//grab1
//	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[4],74,ADD,CIRCLE,4,YELLOW,0,0,4,1155,230,8,0,0);   
//	//grab2
//	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[5],75,ADD,CIRCLE,4,YELLOW,0,0,4,1155,270,8,0,0);   
//	//exchange
//	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[6],76,ADD,CIRCLE,4,YELLOW,0,0,4,925,270,8,0,0);   
//	//claw
//	
//		
//	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_third, 118, 0xFFFF);
//	UI.ext_client_custom_graphic_patterning_third.CRC16[0] = crc16_temp & 0xFF;
//	UI.ext_client_custom_graphic_patterning_third.CRC16[1] = crc16_temp >> 8;
//	
//	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_patterning_third,sizeof(UI.ext_client_custom_graphic_patterning_third));
//	
//	HAL_UART_Transmit_DMA(&huart2,Data_Pack,120);
//}

//void referee_draw_patterning_2(uint8_t robot_id)
//	{
//	
//	
//	uint16_t crc16_temp;
//	UI.ext_client_custom_graphic_patterning_forth.frame_header.SOF = 0xA5;
//	UI.ext_client_custom_graphic_patterning_forth.frame_header.data_length =  111;
//	UI.ext_client_custom_graphic_patterning_forth.frame_header.seq = 3;
//	UI.ext_client_custom_graphic_patterning_forth.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_forth.frame_header,4,0xFF); 
//	UI.ext_client_custom_graphic_patterning_forth.cmd_id = 0x0301;
//	UI.ext_client_custom_graphic_patterning_forth.ext_student_interactive_header_data.data_cmd_id = 0x0104;
//	UI.ext_client_custom_graphic_patterning_forth.ext_student_interactive_header_data.sender_ID = robot_id;
//	UI.ext_client_custom_graphic_patterning_forth.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
//	
//	/*�˴�������д�߸�ͼ��*/	
//	if(control_UI_all.storage1_state==1)//storage1
//		draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[1],80,MODIFY,CIRCLE,4,GREEN,0,0,4,925,190,8,0,0);   
//	else
//		draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[1],80,MODIFY,CIRCLE,4,FUCHSIA,0,0,4,925,190,8,0,0);   
//		
//	if(control_UI_all.storage2_state==1)//storage2
//		draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[2],81,MODIFY,CIRCLE,4,GREEN,0,0,4,1155,190,8,0,0);   
//	else
//		draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[2],81,MODIFY,CIRCLE,4,FUCHSIA,0,0,4,1155,190,8,0,0);   
//		
//	draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[3],82,MODIFY,INT,4,YELLOW,30,0,3,1780,700,control_UI_all.ttt,0,0);   

//	draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[4],83,MODIFY,INT,4,GREEN,30,0,3,1780,756,control_UI_all.speed,0,0);   
//	//speed
//	
////	if(control_UI_all.pitch_state==1)//pitch
////		draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[5],84,MODIFY,CIRCLE,4,GREEN,0,0,4,1750,590,15,0,0);   
////	else
////		draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[5],84,MODIFY,CIRCLE,4,FUCHSIA,0,0,4,1750,590,15,0,0);   

////	if(control_UI_all.yaw_state==1)//yaw
////		draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[6],85,MODIFY,CIRCLE,4,GREEN,0,0,4,1750,510,15,0,0);   
////	else
////		draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[6],85,MODIFY,CIRCLE,4,FUCHSIA,0,0,4,1750,510,15,0,0);   
//	
//		
//	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_forth, 118, 0xFFFF);
//	UI.ext_client_custom_graphic_patterning_forth.CRC16[0] = crc16_temp & 0xFF;
//	UI.ext_client_custom_graphic_patterning_forth.CRC16[1] = crc16_temp >> 8;
//	
//	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_patterning_forth,sizeof(UI.ext_client_custom_graphic_patterning_forth));
//	
//	HAL_UART_Transmit_DMA(&huart2,Data_Pack,120);
//}

//void referee_draw_patterning_2_basic(uint8_t robot_id)
//{

//	
//	
//	uint16_t crc16_temp;
//	UI.ext_client_custom_graphic_patterning_forth.frame_header.SOF = 0xA5;
//	UI.ext_client_custom_graphic_patterning_forth.frame_header.data_length =  111;
//	UI.ext_client_custom_graphic_patterning_forth.frame_header.seq = 3;
//	UI.ext_client_custom_graphic_patterning_forth.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_forth.frame_header,4,0xFF); 
//	UI.ext_client_custom_graphic_patterning_forth.cmd_id = 0x0301;
//	UI.ext_client_custom_graphic_patterning_forth.ext_student_interactive_header_data.data_cmd_id = 0x0104;
//	UI.ext_client_custom_graphic_patterning_forth.ext_student_interactive_header_data.sender_ID = robot_id;
//	UI.ext_client_custom_graphic_patterning_forth.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
//	
//	/*�˴�������д�߸�ͼ��*/	
//	draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[1],80,ADD,CIRCLE,4,YELLOW,0,0,4,925,190,8,0,0);   
//	//storage1		
//	draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[2],81,ADD,CIRCLE,4,YELLOW,0,0,4,1155,190,8,0,0);   
//	//storage2
//		
//	draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[3],82,ADD,INT,4,YELLOW,30,0,3,1780,700,control_UI_all.ttt,0,0);   

//	draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[4],83,ADD,INT,4,WHITE,30,0,3,1780,756,control_UI_all.speed,0,0);   
//	//speed
//	
////	draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[5],84,ADD,CIRCLE,4,YELLOW,0,0,4,1750,590,15,0,0);   
////	//pitch
//	
////	draw_graph(&UI.ext_client_custom_graphic_patterning_forth.grapic_data_struct[6],85,ADD,CIRCLE,4,YELLOW,0,0,4,1750,510,15,0,0);   
////	//yaw
//	
//	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_forth, 118, 0xFFFF);
//	UI.ext_client_custom_graphic_patterning_forth.CRC16[0] = crc16_temp & 0xFF;
//	UI.ext_client_custom_graphic_patterning_forth.CRC16[1] = crc16_temp >> 8;
//	
//	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_patterning_forth,sizeof(UI.ext_client_custom_graphic_patterning_forth));
//	
//	HAL_UART_Transmit_DMA(&huart2,Data_Pack,120);
//}


//	
///**
//  * @brief  ����ˢ�º�ͼ��
//	*/
//void update(uint32_t cnt, uint8_t robot_id, uint8_t change)
//{
//	if(change)
//	{
//		if(cnt%350==0){
//		referee_graphic_delete(0, ALL_delete, robot_id);
//		cnt=0;
//		}
//	}
//}


//uint16_t referee_get_receiver_ID(uint32_t sender_id)
//{
//	switch(sender_id)
//	{
//		case RobotRedHero: return ClientRedHero; 
//		case RobotRedEngineer: return ClientRedEngineer;
//		case RobotRedInfantryNO3: return ClientRedInfantryNO3;
//		case RobotRedInfantryNO4: return ClientRedInfantryNO4; 
//		case RobotRedInfantryNO5: return ClientRedInfantryNO5; 
//		case RobotRedAerial: return ClientRedAerial;
//		case RobotBlueHero: return ClientBlueHero;
//		case RobotBlueEngineer: return ClientBlueEngineer; 
//		case RobotBlueInfantryNO3: return ClientBlueInfantryNO3;
//		case RobotBlueInfantryNO4: return ClientBlueInfantryNO4; 
//		case RobotBlueInfantryNO5: return ClientBlueInfantryNO5; 
//		case RobotBlueAerial: return ClientBlueAerial; 
//		default: return 0;
//	}
//}




//const unsigned char CRC8__INIT = 0xff;
//const unsigned char CRC8__TAB[256] =//8λ���256��
//{
//	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
//	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
//	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
//	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
//	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
//	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
//	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
//	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
//	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
//	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
//	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
//	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
//	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
//	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
//	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
//	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
//};
//unsigned char Get_CRC8_Check(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
//{
//	unsigned char ucIndex;                     //��0��򱣳ֲ��䣬��1���ת
//	while (dwLength--)   //ucCRC8��ʲô??
//	{
//		ucIndex = ucCRC8^(*pchMessage++);//��һ����ȡ��?? 
//		ucCRC8 = CRC8__TAB[ucIndex];//��ʽ��             
//	}                                                   
//	return(ucCRC8);
//}

//uint16_t CRC__INIT = 0xffff;
//const uint16_t wCRC__Table[256] =
//{
//	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
//	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
//	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
//	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
//	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
//	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
//	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
//	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
//	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
//	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
//	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
//	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
//	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
//	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
//	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
//	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
//	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
//	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
//	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
//	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
//	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
//	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
//	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
//	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
//	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
//	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
//	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
//	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
//	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
//	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
//	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
//	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
//};

//uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
//{
//	uint8_t chData;
//	if (pchMessage == NULL)//��Ч��ַ
//	{
//		return 0xFFFF;
//	}
//	while(dwLength--)
//	{
//		chData = *pchMessage++;
//		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC__Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &0x00ff];
//	}
//	return wCRC;
//}


