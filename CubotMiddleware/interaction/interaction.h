//#ifndef INTERACTION_H__
//#define INTERACTION_H__
//#include "stm32h7xx.h"


//#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
//#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
//#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
//#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


//#define SCREEN_WIDTH 1080
//#define SCREEN_LENGTH 1920


////< cmd_id
//#define ROBOTS_INTERACTION_ID 0x0301  //< 车间通信命令码

////< data_cmd_id
//#define DRAW_CHAR_ID 0x0110					  //< 画字符图形的数据命令码
//#define DELETE_GRAPHIC 0x0100         //< 删除图形的数据命令码
//#define DRAW_SINGLE_GRAPHIC 0x0101    //< 画单一图形的数据命令码
//#define DRAW_DOUBLE_GRAPHIC 0x0102    //< 画两个图形的数据命令码
//#define DRAW_FIVE_GRAPHIC 0x0103      //< 画两个图形的数据命令码
//#define DRAW_SEVEN_GRAPHIC 0x0104      //< 画两个图形的数据命令码


////< robots' & clients' ID
//#define RobotRedHero 1
//#define RobotRedEngineer 2
//#define RobotRedInfantryNO3 3
//#define RobotRedInfantryNO4 4
//#define RobotRedInfantryNO5 5
//#define RobotRedAerial 6
//#define RobotRedSentry 7
//#define RobotRedRadar 9

//#define ClientRedHero 0x0101
//#define ClientRedEngineer 0x0102
//#define ClientRedInfantryNO3 0x0103
//#define ClientRedInfantryNO4 0x0104
//#define ClientRedInfantryNO5 0x0105
//#define ClientRedAerial 0x0106

//#define RobotBlueHero 101
//#define RobotBlueEngineer 102
//#define RobotBlueInfantryNO3 103
//#define RobotBlueInfantryNO4 104
//#define RobotBlueInfantryNO5 105
//#define RobotBlueAerial 106
//#define RobotBlueSentry 107
//#define RobotBlueRadar 109

//#define ClientBlueHero 0x0165
//#define ClientBlueEngineer 0x0166
//#define ClientBlueInfantryNO3 0x0167
//#define ClientBlueInfantryNO4 0x0168
//#define ClientBlueInfantryNO5 0x0169
//#define ClientBlueAerial 0x016A


//#define HighVolt 25
//#define LowVolt 16

//extern uint16_t cnt_test_;
////extern uint32_t UI_speed;
//extern uint8_t UI_test;

//static uint8_t UI_basic_flag=0;


//typedef enum
//{
//	NONE_delete    = 0,
//	GRAPHIC_delete = 1,
//	ALL_delete     = 2
//}delete_Graphic_Operate;  //ext_client_custom_graphic_delete_t：uint8_t operate_type
///*图层删除操作*/


////bit 0-2
//typedef enum
//{
//	NONE   = 0,/*空操作*/
//	ADD    = 1,/*增加图层*/
//	MODIFY = 2,/*修改图层*/
//	DELETE = 3,/*删除图层*/
//}Graphic_Operate;

//typedef enum
//{
//	LINE      = 0,//直线
//	RECTANGLE = 1,//矩形
//	CIRCLE    = 2,//正圆
//	OVAL      = 3,//椭圆
//	ARC       = 4,//圆弧
//	FLOAT     = 5,//浮点数
//	INT       = 6,//整型数
//	CHAR      = 7 //字符
//}Graphic_Type;

//typedef enum
//{
//	RED_BLUE  = 0,//红蓝主色	
//	YELLOW    = 1,
//	GREEN     = 2,
//	ORANGE    = 3,
//	FUCHSIA   = 4,	/*紫红色*/
//	PINK      = 5,
//	CYAN_BLUE = 6,	/*青色*/
//	BLACK     = 7,
//	WHITE     = 8
//}Graphic_Color;


///*
// * 串口协议帧头
// */
//typedef __packed struct
//{
//	uint8_t SOF;			    //0xA5
//	uint16_t data_length; //data 字节数
//	uint8_t seq;				  //包序号
//	uint8_t CRC8;
//}frame_header_t;


///*
// * 数据段头结构，标明了数据的内容、发送者id和接收者id
// */
//typedef __packed struct //包头结构体
//{
// uint16_t data_cmd_id;
// uint16_t sender_ID;
// uint16_t receiver_ID;
//}ext_student_interactive_header_data_t;


//typedef __packed struct //0x100 删除图形
//{
//	uint8_t operate_tpye; 
//	uint8_t layer; 
//}ext_client_custom_graphic_delete_t;


///*
// * 绘制基础图形数据的结构体
// */
//typedef __packed struct 
//{ 
//uint8_t graphic_name[3]; //图形名称
//uint32_t operate_tpye:3; //图形操作				0 空				1 增加				2	修改				3	删除
//uint32_t graphic_tpye:3; //图形类型				0 直线			1 矩形				2 整圆				4	圆弧				5 浮点数				6 整型数				7 字符
//uint32_t layer:4; //图层
//uint32_t color:4; //颜色
//uint32_t start_angle:9;//开始角度
//uint32_t end_angle:9;//终止角度
//uint32_t width:10; //线宽
//uint32_t start_x:11; //开始X坐标
//uint32_t start_y:11;//开始Y坐标
//uint32_t radius:10; //半径或字体大小
//uint32_t end_x:11; //终止X坐标
//uint32_t end_y:11; //终止Y坐标
//}graphic_data_struct_t;


//typedef struct {

//	struct
//{
//	frame_header_t frame_header;
//	uint16_t cmd_id;
//	
//	uint16_t target_robot_ID;
//	float target_position_x;
//	float target_position_y;

//	uint8_t CRC16[2];
//	
//} ext_client_map_command_t;
//	
//	
///*
// * UI绘制单一图形的完整结构体，共30个字节
// */
// __packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(21-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< 共6个字节
//	ext_client_custom_graphic_delete_t ext_client_custom_graphic_delete;																		//< 共15个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_graphic_delete;  



///*
// * UI绘制单一字符的完整结构体，共60个字节
// */
// __packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(51-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< 共6个字节
//	graphic_data_struct_t grapic_data_struct;																		//< 共15个字节
//	uint8_t data[30];																														//< 共30个字节，直接将字符串char* 使用 strcpy 复制进该区域
//	
//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//} ext_client_custom_character;           



///*
// * UI绘制单一图形的完整结构体，共30个字节
// */
// __packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(21-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< 共6个字节
//	graphic_data_struct_t grapic_data_struct;																		//< 共15个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_float;  


// __packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(21-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< 共6个字节
//	graphic_data_struct_t grapic_data_struct;																		//< 共15个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_int;  

// __packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(21-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< 共6个字节
//	graphic_data_struct_t grapic_data_struct;																		//< 共15个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_line;  


// __packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(21-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< 共6个字节
//	graphic_data_struct_t grapic_data_struct;																		//< 共15个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_circle;  


// __packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(21-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< 共6个字节
//	graphic_data_struct_t grapic_data_struct;																		//< 共15个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_rectangle;  


// __packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(21-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< 共6个字节
//	graphic_data_struct_t grapic_data_struct;																		//< 共15个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_ellipe;  

// __packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(21-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< 共6个字节
//	graphic_data_struct_t grapic_data_struct;																		//< 共15个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_arc;  

///*
// * UI绘制两个图形的完整结构体，共45个字节
// */
// __packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(36-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< 共6个字节
//	graphic_data_struct_t grapic_data_struct[2];																	//< 共30个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_double_t;  


///*
// * UI绘制两个图形的完整结构体，共90个字节
// */
// __packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(81-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< 共6个字节
//	graphic_data_struct_t grapic_data_struct[5];																	//< 共30个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_patterning;  


//__packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(81-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< 共6个字节
//	graphic_data_struct_t grapic_data_struct[7];																	//< 共30个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_patterning_first;  

//__packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(81-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< 共6个字节
//	graphic_data_struct_t grapic_data_struct[7];																	//< 共30个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_patterning_second;  

//__packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(81-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< 共6个字节
//	graphic_data_struct_t grapic_data_struct[7];																	//< 共30个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_patterning_third;  

//__packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(81-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< 共6个字节
//	graphic_data_struct_t grapic_data_struct[7];																	//< 共30个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_patterning_forth;  

//__packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(81-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< 共6个字节
//	graphic_data_struct_t grapic_data_struct[7];																	//< 共30个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_patterning_fifth;  

//__packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(81-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< 共6个字节
//	graphic_data_struct_t grapic_data_struct[7];																	//< 共30个字节

//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//}ext_client_custom_graphic_patterning_sixth; 

///*
// * UI绘制单一字符的完整结构体，共60个字节
// */
// __packed struct
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	//< data(51-byte)
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< 共6个字节
//	graphic_data_struct_t grapic_data_struct;																		//< 共15个字节
//	uint8_t data[30];																														//< 共30个字节，直接将字符串char* 使用 strcpy 复制进该区域
//	
//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//} ext_client_custom_graphic_test1;

///*
// * 客户端接受信息
// */
// __packed struct 
//{ 
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	uint16_t target_robot_ID;     //< 机器人ID信息共2个字节
//	float target_position_x;			//< 坐标信息共4个字节
//	float target_position_y; 			//< 坐标信息共4个字节
//	
//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];
//} ext_client_map_command; 


// __packed struct
//{
//	//< frame_header(5-byte) 帧头部分
//	frame_header_t frame_header;   
//	
//	//< cmd_id(2-byte) 命令码
//	uint16_t cmd_id;
//	
//	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< 共6个字节
//	uint8_t data[15]; 
//	//< frame_tail(2-byte，CRC16，整包校验)
//	uint8_t CRC16[2];	
//} robot_interactive_data; 


//}UI_t;

//static struct {
//	uint8_t pump_state;
//	uint8_t grab1_state;
//	uint8_t grab2_state;
//	uint8_t exchange_state;
//	uint8_t claw_state;
//	uint8_t storage1_state;
//	uint8_t storage2_state;
//	double lift1;
//	double lift2;
//	uint8_t speed;
//	uint8_t pitch_state;
//	uint8_t yaw_state;
//	uint8_t chassis_speed;
//	uint8_t t1;
//	uint8_t t2;
//	uint8_t t3;
//	uint16_t ttt;
//	
//}control_UI_all;


//extern UI_t UI;

///**
//  * @brief  在0号图层画字符的程序，最长30个字符
//  * @param[in] robot_id  	  该机器人的发送ID
//	* @param[in] string  		  字符串变量缓存区首地址
//	* @param[in] string_dex   该字符的标识符，用于选中该字符进行删除或修改的操作
//	* @param[in] control_way  操作方法：0：空操作；1：增加；2：修改；3：删除；
//	* @param[in] color        颜色：0：红蓝主色；1：黄色；2：绿色；4：紫红色等等
//	* @param[in] layer        该字符所绘制的图层，共0-9 十个图层，每个图层可以绘制七个图像。
//	* @param[in] x        		横轴位置坐标 0-1023 （11bit）
//	* @param[in] y        		纵轴位置坐标 0-1023 （11bit）
//	*/
//void referee_draw_string(uint8_t robot_id,char *string,uint8_t string_dex,uint8_t control_way,uint8_t color,uint8_t on_layer, uint16_t x,uint16_t y,uint16_t large,uint16_t width);


///*协助组合数据的发送*/
//void draw_graph(graphic_data_struct_t *patterning, uint16_t index, uint8_t control_way, uint8_t graph, uint8_t layer, uint8_t color, uint8_t Sa, uint8_t Ea, uint8_t With,  uint16_t x,uint16_t y, uint8_t R, uint16_t Ex, uint16_t Ey);


///*删除图层操作*/
//void referee_graphic_delete(uint8_t del_layer, uint8_t operation, uint8_t robot_id);


///*更新图形，没测过，小地图，也没测过*/
//void update(uint32_t cnt, uint8_t robot_id, uint8_t change);


///*新*/
//void referee_draw_basic_1(uint8_t robot_id);
//void referee_draw_basic_(uint8_t robot_id,uint32_t cnt);
//void referee_draw_patterning_1(uint8_t robot_id);
//void referee_draw_patterning_1_basic(uint8_t robot_id);
//void referee_draw_patterning_all_1(uint8_t robot_id,uint32_t cnt);
//void referee_draw_patterning_2(uint8_t robot_id);
//void referee_draw_patterning_2_basic(uint8_t robot_id);


///**
//  * @brief  根据该机器人的发送ID自动判断对应的客户端ID
//  * @param[in]  robot_id  	  该机器人的ID
//  * @param[out] client_id  	  目标客户端的ID
//	*/
//uint16_t referee_get_receiver_ID(uint32_t sender_id);


///*
//0		3		图形名
//3		4		b0-b2		图形操作
//								0 空
//								1 增加
//								2	修改
//								3	删除
//				b3-b5		图形类型
//								0 直线
//								1 矩形
//								2 整圆
//								3 椭圆
//								4	圆弧
//								5 浮点数
//								6 整型数
//								7 字符
//				b6-b9		图层数 0-9
//				b10-b13	颜色
//								0 红蓝主色 1黄 2绿 3橙 4紫红 5粉 6青 7黑 8白
//				b14-b22	起始角度 0-360
//				b32-b31 终止角度 0-360
//7		4		b0-b9		线宽
//				b10-b20	起点x
//				b21-b31	起点y
//11	4		b0-b9		字体大小或半径
//				b10-b20	终点x
//				b21-b31	终点y
//*/



//extern uint8_t Data_Pack[128]__attribute__((at(0x24065000)));
//extern uint16_t x_t1;
//extern uint16_t x_t2;
//extern uint16_t x_t3;
//extern uint16_t x_t4;



//unsigned char Get_CRC8_Check(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
//uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);

//void Interact_Pack_Shoot(void);
//void Client_Display(void);
//void UI_show(void);
//#endif
