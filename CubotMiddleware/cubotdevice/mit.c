//#include "mit.h"
//#include "stm32h7xx_hal.h"
//#include <driver_can.h>
//#include <motor.h>

//#define P_MIN  -12.5f
//#define P_MAX   12.5f
//#define V_MIN  -30.0f
//#define V_MAX   30.0f
//#define KP_MIN  0.0f
//#define KP_MAX  500.0f   //Kp 的范围为[0,500]
//#define KD_MIN  0.0f     //Kd 的范围为[0,5]
//#define KD_MAX  5.0f
//#define T_MIN   -10.0f
//#define T_MAX   10.0f
//CAN_TxBuffer txbuf_MODE1;
//CAN_TxBuffer txbuf_MODE2;
//CAN_TxBuffer1 txbuf_MODE3;
//CAN_TxBuffer txbuf_start;
//CAN_TxBuffer txbuf_lock;

//void ctrl_motor1(CAN_Object* hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
//{
//  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
//	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
//  vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
//	kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
//	kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
//  tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
//	
//	txbuf_MODE1.Identifier = id;
//	txbuf_MODE1.Data[0]= (pos_tmp >> 8);
//	txbuf_MODE1.Data[1] = pos_tmp;
//	txbuf_MODE1.Data[2] = (vel_tmp >> 4);
//	txbuf_MODE1.Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
//	txbuf_MODE1.Data[4] = kp_tmp;
//	txbuf_MODE1.Data[5] = (kd_tmp >> 4);
//	txbuf_MODE1.Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
//	txbuf_MODE1.Data[7] = tor_tmp;

//  CAN_Send(hcan, &txbuf_MODE1);

//}
//void ctrl_motor2(CAN_Object* hcan,uint16_t id, float _pos, float _vel)
//{
//	uint8_t *pbuf,*vbuf;
//	pbuf=(uint8_t*)&_pos;
//	vbuf=(uint8_t*)&_vel;
//	
//  txbuf_MODE2.Identifier=id;
//	txbuf_MODE2.Data[0]  = *pbuf;
//	txbuf_MODE2.Data[1]  = *(pbuf+1);
//	txbuf_MODE2.Data[2]  = *(pbuf+2);
//	txbuf_MODE2.Data[3]  = *(pbuf+3);
//	txbuf_MODE2.Data[4]  = *vbuf;
//	txbuf_MODE2.Data[5]  = *(vbuf+1);
//	txbuf_MODE2.Data[6]  = *(vbuf+2);
//	txbuf_MODE2.Data[7]  = *(vbuf+3);
//	

//	CAN_Send(hcan, &txbuf_MODE2);
//}	
//void ctrl_motor3(CAN_Object* hcan,uint16_t id, float _vel)
//{
//	uint8_t *vbuf;
//	vbuf=(uint8_t*)&_vel;
//	
//  txbuf_MODE3.Identifier=id;
//	txbuf_MODE3.Data[0]  = *vbuf;
//	txbuf_MODE3.Data[1]  = *(vbuf+1);
//	txbuf_MODE3.Data[2]  = *(vbuf+2);
//	txbuf_MODE3.Data[3]  = *(vbuf+3);
//	

//	CAN_Send1(hcan, &txbuf_MODE3);
//}	
//void start_motor(CAN_Object* hcan,uint16_t id)
//{
//	txbuf_start.Identifier = id;
//	txbuf_start.Data[0] = 0xFF;
//	txbuf_start.Data[1] = 0xFF;
//	txbuf_start.Data[2] = 0xFF;
//	txbuf_start.Data[3] = 0xFF;
//	txbuf_start.Data[4] = 0xFF;
//	txbuf_start.Data[5] = 0xFF;
//	txbuf_start.Data[6] = 0xFF;
//	txbuf_start.Data[7] = 0xFC;
//	
//	CAN_Send(hcan, &txbuf_start);
//}	

//void lock_motor(CAN_Object* hcan,uint16_t id)
//{
//	txbuf_lock.Identifier = id;
//	txbuf_lock.Data[0] = 0xFF;
//	txbuf_lock.Data[1] = 0xFF;
//	txbuf_lock.Data[2] = 0xFF;
//	txbuf_lock.Data[3] = 0xFF;
//	txbuf_lock.Data[4] = 0xFF;
//	txbuf_lock.Data[5] = 0xFF;
//	txbuf_lock.Data[6] = 0xFF;
//	txbuf_lock.Data[7] = 0xFD;
//	
//	CAN_Send(hcan, &txbuf_lock);
//}		