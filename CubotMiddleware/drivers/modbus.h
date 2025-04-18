#ifndef _MODBUS_H
#define _MODBUS_H
#include "stm32h7xx_hal.h"
#define SlaveAddr		0x01			/* 面板作为时，主板作从机 */
#define HBAUD485		115200

/* 01H 读强制单线圈 */
/* 05H 写强制单线圈 */
#define REG_D01		0x0101
#define REG_D02		0x0102
#define REG_D03		0x0103
#define REG_D04		0x0104
#define REG_DXX 	REG_D04

/* 02H 读取输入状态 */
#define REG_T01		0x0201
#define REG_T02		0x0202
#define REG_T03		0x0203
#define REG_TXX		REG_T03

/* 03H 读保持寄存器 */
/* 06H 写保持寄存器 */
/* 10H 写多个保存寄存器 */
#define REG_P01		0x0301		
#define REG_P02		0x0302	

/* 04H 读取输入寄存器(模拟信号) */
#define REG_A01		0x0401
#define REG_AXX		REG_A01

/* RTU 应答代码 */
#define RSP_OK				0		/* 成功 */
#define RSP_ERR_CMD			0x01	/* 不支持的功能码 */
#define RSP_ERR_REG_ADDR	0x02	/* 寄存器地址错误 */
#define RSP_ERR_VALUE		0x03	/* 数据值域错误 */
#define RSP_ERR_WRITE		0x04	/* 写入失败 */
#define H_RX_BUF_SIZE		64
#define H_TX_BUF_SIZE      	128
typedef struct
{
	uint8_t RxBuf[H_RX_BUF_SIZE];
	uint8_t RxBufadr;
	uint8_t RxBufgn;
	uint8_t RxBufnum;
	float RxBufjuli1;
	float RxBufjuli2;
	float RxBufjuli3;
	float RxBufjuli4;
	float RxBufjuli5;
	float RxBufjuli6;
	uint8_t RxBufcrcdi;
	uint8_t RxBufcrcgao;
	uint8_t RxCount;
	uint8_t RxStatus;
	uint8_t RxNewFlag;

	uint8_t RspCode;

	uint8_t TxBuf[H_TX_BUF_SIZE];
	uint8_t TxCount;
	
	uint16_t Reg01H;		/* 保存主机发送的寄存器首地址 */
	uint16_t Reg02H;
	uint16_t Reg03H;		
	uint16_t Reg04H;

	uint8_t RegNum;			/* 寄存器个数 */

	uint8_t fAck01H;		/* 应答命令标志 0 表示执行失败 1表示执行成功 */
	uint8_t fAck02H;
	uint8_t fAck03H;
	uint8_t fAck04H;
	uint8_t fAck05H;		
	uint8_t fAck06H;		
	uint8_t fAck10H;
	
}MODH_T;
void build_buff(uint8_t * buf,uint8_t bit1,uint8_t bit2,uint8_t bit3,uint8_t bit4,uint8_t bit5,uint8_t bit6,uint8_t bit7,uint8_t bit8);
typedef struct
{
	/* 03H 06H 读写保持寄存器 */
	uint16_t P01;
	uint16_t P02;
	
	/* 02H 读写离散输入寄存器 */
	uint16_t T01;
	uint16_t T02;
	uint16_t T03;
	
	/* 04H 读取模拟量寄存器 */
	uint16_t A01;
	
	/* 01H 05H 读写单个强制线圈 */
	uint16_t D01;
	uint16_t D02;
	uint16_t D03;
	uint16_t D04;
	
}VAR_T;

void MODH_Send03H(uint8_t _addr, uint16_t _reg, uint16_t _num,uint16_t crc);
void MODH_ReciveNew(MODH_T * G_tModH ,uint8_t *_data);
void MODH_Send05H(uint8_t _addr, uint16_t _reg, uint16_t _value);
extern MODH_T g_tModH;
extern MODH_T g_tModH02;
extern UART_HandleTypeDef* modbus_rtu_huart ;
extern uint8_t bit1_t,bit2_t,bit3_t,bit4_t,bit5_t,bit6_t,bit7_t,bit8_t;
void MODH_Send0FH(uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t buf);
//void MODH_Send06H(uint8_t _addr, uint16_t _reg, uint16_t _value);
void MODH_Send0x06(MODH_T*g_tmodh,uint8_t _addr, uint16_t _reg, uint16_t _value);
void MODH_Send061H(uint8_t _addr, uint16_t _reg, uint16_t _value);
uint16_t CRC16_Modbus1(MODH_T*txbuff);


#endif
