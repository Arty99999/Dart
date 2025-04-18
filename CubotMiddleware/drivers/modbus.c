#include "modbus.h"
#include "usart.h"
#define TIMEOUT		100		/* 接收命令超时时间, 单位ms */
#define NUM			1		/* 循环发送次数 */
MODH_T g_tModH;
MODH_T g_tModH02;
MODH_T g_tModH03;
uint8_t g_modh_timeout = 0;
UART_HandleTypeDef* modbus_rtu_huart;
static void MODH_RxTimeOut(void);
static void MODH_AnalyzeApp(void);

static void MODH_Read_01H(void);
static void MODH_Read_02H(void);
static void MODH_Read_03H(void);
static void MODH_Read_04H(void);
static void MODH_Read_05H(void);
static void MODH_Read_06H(void);
static void MODH_Read_10H(void);

VAR_T g_tVar;

static const uint8_t s_CRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;
// CRC 低位字节值表
const uint8_t s_CRCLo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
	0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
	0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
	0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
	0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
	0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
	0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
	0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
	0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
	0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
	0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
	0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
uint16_t CRC16_Modbus(uint8_t *_pBuf, uint16_t _usLen)
{
	uint8_t ucCRCHi = 0xFF; /* 高CRC字节初始化 */
	uint8_t ucCRCLo = 0xFF; /* 低CRC 字节初始化 */
	uint16_t usIndex;  /* CRC循环中的索引 */

    while (_usLen--)
    {
		usIndex = ucCRCHi ^ *_pBuf++; /* 计算CRC */
		ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
		ucCRCLo = s_CRCLo[usIndex];
    }
    return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
}

static void MODH_SendAckWithCRC(void)
{
	uint16_t crc;
	
	crc = CRC16_Modbus(g_tModH.TxBuf, g_tModH.TxCount);
	g_tModH.TxBuf[g_tModH.TxCount++] = crc >> 8;
	g_tModH.TxBuf[g_tModH.TxCount++] = crc;	
//	HAL_UART_Transmit(&huart7, g_tModH.TxBuf, g_tModH.TxCount, HAL_MAX_DELAY);
		HAL_UART_Transmit_DMA(&huart3,g_tModH.TxBuf,g_tModH.TxCount);


}
static void MODH_SendAckWithCRCUART4(void)
{
	uint16_t crc;
	
	crc = CRC16_Modbus(g_tModH02.TxBuf, g_tModH02.TxCount);
	g_tModH02.TxBuf[g_tModH02.TxCount++] = crc >> 8;
	g_tModH02.TxBuf[g_tModH02.TxCount++] = crc;	
//	HAL_UART_Transmit(&huart7, g_tModH.TxBuf, g_tModH.TxCount, HAL_MAX_DELAY);
		HAL_UART_Transmit_DMA(&huart3,g_tModH02.TxBuf,g_tModH02.TxCount);


}

static void MODH_SendAckWithCRCUART6(void)
{
	uint16_t crc;
	
	crc = CRC16_Modbus(g_tModH.TxBuf, g_tModH.TxCount);
	g_tModH.TxBuf[g_tModH.TxCount++] = crc >> 8;
	g_tModH.TxBuf[g_tModH.TxCount++] = crc;	
//	HAL_UART_Transmit(&huart7, g_tModH.TxBuf, g_tModH.TxCount, HAL_MAX_DELAY);
		HAL_UART_Transmit_DMA(&huart3,g_tModH.TxBuf,g_tModH.TxCount);


}

static void MODH_SendAckWithCRCUART7(void)
{
	uint16_t crc;
	
	crc = CRC16_Modbus(g_tModH03.TxBuf, g_tModH03.TxCount);
	g_tModH03.TxBuf[g_tModH03.TxCount++] = crc >> 8;
	g_tModH03.TxBuf[g_tModH03.TxCount++] = crc;	
//	HAL_UART_Transmit(&huart7, g_tModH.TxBuf, g_tModH.TxCount, HAL_MAX_DELAY);
		HAL_UART_Transmit_DMA(&huart3,g_tModH03.TxBuf,g_tModH03.TxCount);


}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send01H
*	功能说明: 发送01H指令，查询1个或多个保持寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send01H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x01;		/* 功能码 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck01H = 0;		/* 清接收标志 */
	g_tModH.RegNum = _num;		/* 寄存器个数 */
	g_tModH.Reg01H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send02H
*	功能说明: 发送02H指令，读离散输入寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send02H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x02;		/* 功能码 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck02H = 0;		/* 清接收标志 */
	g_tModH.RegNum = _num;		/* 寄存器个数 */
	g_tModH.Reg02H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send03H
*	功能说明: 发送03H指令，查询1个或多个保持寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send03H(uint8_t _addr, uint16_t _reg, uint16_t _num, uint16_t crc)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x03;		/* 功能码 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
  g_tModH.TxBuf[g_tModH.TxCount++] = crc >> 8;
	g_tModH.TxBuf[g_tModH.TxCount++] = crc;
	HAL_UART_Transmit_DMA(&huart3,g_tModH.TxBuf,g_tModH.TxCount);
	
//	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck03H = 0;		/* 清接收标志 */
	g_tModH.RegNum = _num;		/* 寄存器个数 */
	g_tModH.Reg03H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
}


/*
*********************************************************************************************************
*	函 数 名: MODH_Send04H
*	功能说明: 发送04H指令，读输入寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send04H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x04;		/* 功能码 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck04H = 0;		/* 清接收标志 */
	g_tModH.RegNum = _num;		/* 寄存器个数 */
	g_tModH.Reg04H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send05H
*	功能说明: 发送05H指令，写强置单线圈
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _value : 寄存器值,2字节
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send05H(uint8_t _addr, uint16_t _reg, uint16_t _value)
{
	int i=0;
	g_tModH02.TxCount = 0;
	g_tModH02.TxBuf[g_tModH02.TxCount++] = _addr;			/* 从站地址 */
	g_tModH02.TxBuf[g_tModH02.TxCount++] = 0x03;			/* 功能码 */	
	g_tModH02.TxBuf[g_tModH02.TxCount++] = _reg >> 8;		/* 寄存器编号 高字节 */
	g_tModH02.TxBuf[g_tModH02.TxCount++] = _reg;			/* 寄存器编号 低字节 */
	g_tModH02.TxBuf[g_tModH02.TxCount++] = _value >> 8;		/* 寄存器值 高字节 */
	g_tModH02.TxBuf[g_tModH02.TxCount++] = _value;			/* 寄存器值 低字节 */
	
	MODH_SendAckWithCRCUART4();		/* 发送数据，自动加CRC */
//	for(i=0;i<g_tModH02.TxCount;i++)
//	{
//	  g_tModH02.TxBuf[i++]=0;
//	}
	
	g_tModH02.fAck06H = 0;		/* 如果收到从机的应答，则这个标志会设为1 */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send06H
*	功能说明: 发送06H指令，写1个保持寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _value : 寄存器值,2字节
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send06H(uint8_t _addr, uint16_t _reg, uint16_t _value)
{
	int i=0;
	g_tModH02.TxCount = 0;
	g_tModH02.TxBuf[g_tModH02.TxCount++] = _addr;			/* 从站地址 */
	g_tModH02.TxBuf[g_tModH02.TxCount++] = 0x06;			/* 功能码 */	
	g_tModH02.TxBuf[g_tModH02.TxCount++] = _reg >> 8;		/* 寄存器编号 高字节 */
	g_tModH02.TxBuf[g_tModH02.TxCount++] = _reg;			/* 寄存器编号 低字节 */
	g_tModH02.TxBuf[g_tModH02.TxCount++] = _value >> 8;		/* 寄存器值 高字节 */
	g_tModH02.TxBuf[g_tModH02.TxCount++] = _value;			/* 寄存器值 低字节 */
	
	MODH_SendAckWithCRCUART4();		/* 发送数据，自动加CRC */
//	for(i=0;i<g_tModH02.TxCount;i++)
//	{
//	  g_tModH02.TxBuf[i++]=0;
//	}
	
	g_tModH02.fAck06H = 0;		/* 如果收到从机的应答，则这个标志会设为1 */
}
void MODH_Send061H(uint8_t _addr, uint16_t _reg, uint16_t _value)
{
	int i=0;
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x06;			/* 功能码 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;		/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;			/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _value >> 8;		/* 寄存器值 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _value;			/* 寄存器值 低字节 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	for(i=0;i<g_tModH.TxCount;i++)
	{
	  g_tModH02.TxBuf[i++]=0;
	}
	
	g_tModH02.fAck06H = 0;		/* 如果收到从机的应答，则这个标志会设为1 */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send10H
*	功能说明: 发送10H指令，连续写多个保持寄存器. 最多一次支持23个寄存器。
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数n (每个寄存器2个字节) 值域
*			  _buf : n个寄存器的数据。长度 = 2 * n
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send10H(uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t *_buf)
{
	uint16_t i;
	
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x10;		/* 从站地址 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 2 * _num;	/* 数据字节数 */
	
	for (i = 0; i < 2 * _num; i++)
	{
		if (g_tModH.TxCount > H_RX_BUF_SIZE - 3)
		{
			return;		/* 数据超过缓冲区超度，直接丢弃不发送 */
		}
		g_tModH.TxBuf[g_tModH.TxCount++] = _buf[i];		/* 后面的数据长度 */
	}
	
	MODH_SendAckWithCRC();	/* 发送数据，自动加CRC */
}
/**
*@bref 0x01/0x00, 0x02/0x00,0x04/0x00,0x08/0x00,0x10/0x00,0x20/0x00,0x40/0x00,0x80/0x00
*
*
*/
uint8_t bit1_t,bit2_t,bit3_t,bit4_t,bit5_t,bit6_t,bit7_t,bit8_t;
void build_buff(uint8_t * buf,uint8_t bit1,uint8_t bit2,uint8_t bit3,uint8_t bit4,uint8_t bit5,uint8_t bit6,uint8_t bit7,uint8_t bit8)
{
	(*buf)=0;
	(*buf)|=bit1;
	(*buf)|=bit2<<1;
	(*buf)|=bit3<<2;
	(*buf)|=bit4<<3;
	(*buf)|=bit5<<4;
	(*buf)|=bit6<<5;
	(*buf)|=bit7<<6;
	(*buf)|=bit8<<7;
	
}
void MODH_Send0FH(uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t buf)
{
	
	g_tModH03.TxCount = 0;
	g_tModH03.TxBuf[g_tModH03.TxCount++] = _addr;		/* 从站地址 */
	g_tModH03.TxBuf[g_tModH03.TxCount++] = 0x0F;		/* 从站地址 */	
	g_tModH03.TxBuf[g_tModH03.TxCount++] = _reg >> 8;	/* 线圈起始位置 高字节 */
	g_tModH03.TxBuf[g_tModH03.TxCount++] = _reg;		/* 线圈起始位置 低字节 */
	g_tModH03.TxBuf[g_tModH03.TxCount++] = _num >> 8;	/* 线圈个数 高字节 */
	g_tModH03.TxBuf[g_tModH03.TxCount++] = _num;		/* 线圈个数 低字节 */
	g_tModH03.TxBuf[g_tModH03.TxCount++] = 0x01;	/* 数据字节数 */
	g_tModH03.TxBuf[g_tModH03.TxCount++] = buf ;

	
	MODH_SendAckWithCRCUART7();	/* 发送数据，自动加CRC */
}

uint16_t BEBufToUint16(uint8_t *_pBuf)
{
    return (((uint16_t)_pBuf[0] << 8) | _pBuf[1]);
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_01H
*	功能说明: 分析01H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_01H(void)
{
	uint8_t bytes;
	uint8_t *p;
	
	if (g_tModH.RxCount > 0)
	{
		bytes = g_tModH.RxBuf[2];	/* 数据长度 字节数 */				
		switch (g_tModH.Reg01H)
		{
			case REG_D01:
				if (bytes == 8)
				{
					p = &g_tModH.RxBuf[3];	
					
					g_tVar.D01 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
					g_tVar.D02 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
					g_tVar.D03 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
					g_tVar.D04 = BEBufToUint16(p); p += 2;	/* 寄存器 */
					
					g_tModH.fAck01H = 1;
				}
				break;
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_02H
*	功能说明: 分析02H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_02H(void)
{
	uint8_t bytes;
	uint8_t *p;
	
	if (g_tModH.RxCount > 0)
	{
		bytes = g_tModH.RxBuf[2];	/* 数据长度 字节数 */				
		switch (g_tModH.Reg02H)
		{
			case REG_T01:
				if (bytes == 6)
				{
					p = &g_tModH.RxBuf[3];	
					
					g_tVar.T01 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
					g_tVar.T02 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
					g_tVar.T03 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
					
					g_tModH.fAck02H = 1;
				}
				break;
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_04H
*	功能说明: 分析04H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_04H(void)
{
	uint8_t bytes;
	uint8_t *p;
	
	if (g_tModH.RxCount > 0)
	{
		bytes = g_tModH.RxBuf[2];	/* 数据长度 字节数 */				
		switch (g_tModH.Reg04H)
		{
			case REG_T01:
				if (bytes == 2)
				{
					p = &g_tModH.RxBuf[3];	
					
					g_tVar.A01 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
					
					g_tModH.fAck04H = 1;
				}
				break;
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_05H
*	功能说明: 分析05H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_05H(void)
{
	if (g_tModH.RxCount > 0)
	{
		if (g_tModH.RxBuf[0] == SlaveAddr)		
		{
			g_tModH.fAck05H = 1;		/* 接收到应答 */
		}
	};
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_06H
*	功能说明: 分析06H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_06H(void)
{
	if (g_tModH.RxCount > 0)
	{
		if (g_tModH.RxBuf[0] == SlaveAddr)		
		{
			g_tModH.fAck06H = 1;		/* 接收到应答 */
		}
	}
}


/*
*********************************************************************************************************
*	函 数 名: MODH_Read_03H
*	功能说明: 分析03H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Read_03H(void)
{
	uint8_t bytes;
	uint8_t *p;
	
	if (g_tModH.RxCount > 0)
	{
		bytes = g_tModH.RxBuf[2];	/* 数据长度 字节数 */				
		switch (g_tModH.Reg03H)
		{
			case REG_P01:
				if (bytes == 32)
				{
					p = &g_tModH.RxBuf[3];	
					
					g_tVar.P01 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
					g_tVar.P02 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
		
					g_tModH.fAck03H = 1;
				}
				break;
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_10H
*	功能说明: 分析10H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Read_10H(void)
{
	/*
		10H指令的应答:
			从机地址                11
			功能码                  10
			寄存器起始地址高字节	00
			寄存器起始地址低字节    01
			寄存器数量高字节        00
			寄存器数量低字节        02
			CRC校验高字节           12
			CRC校验低字节           98
	*/
	if (g_tModH.RxCount > 0)
	{
		if (g_tModH.RxBuf[0] == SlaveAddr)		
		{
			g_tModH.fAck10H = 1;		/* 接收到应答 */
		}
	}
}
static void MODH_AnalyzeApp(void)
{	
	switch (g_tModH.RxBuf[1])			/* 第2个字节 功能码 */
	{
		case 0x01:	/* 读取线圈状态 */
			MODH_Read_01H();
			break;

		case 0x02:	/* 读取输入状态 */
			MODH_Read_02H();
			break;

		case 0x03:	/* 读取保持寄存器 在一个或多个保持寄存器中取得当前的二进制值 */
			MODH_Read_03H();
			break;

		case 0x04:	/* 读取输入寄存器 */
			MODH_Read_04H();
			break;

		case 0x05:	/* 强制单线圈 */
			MODH_Read_05H();
			break;

		case 0x06:	/* 写单个寄存器 */
			MODH_Read_06H();
			break;		

		case 0x10:	/* 写多个寄存器 */
			MODH_Read_10H();
			break;
		
		default:
			break;
	}
}


void MODH_Poll(void)
{	
	uint16_t crc1;
	
	if (g_modh_timeout == 0)	/* 超过3.5个字符时间后执行MODH_RxTimeOut()函数。全局变量 g_rtu_timeout = 1 */
	{
		/* 没有超时，继续接收。不要清零 g_tModH.RxCount */
		return ;
	}
}

void bsp_Idle(void)
{
	/* --- 喂狗 */

	/* --- 让CPU进入休眠，由Systick定时中断唤醒或者其他中断唤醒 */

	/* 例如 emWin 图形库，可以插入图形库需要的轮询函数 */
	//GUI_Exec();

	/* 例如 uIP 协议，可以插入uip轮询函数 */
	MODH_Poll();
}

/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_01H
*	功能说明: 单个参数. 通过发送01H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_01H(uint16_t _reg, uint16_t _num)
{
	int32_t time1;
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		MODH_Send01H (SlaveAddr, _reg, _num);		  /* 发送命令 */
//		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
		
		while (1)				/* 等待应答,超时或接收到应答则break  */
		{
			bsp_Idle();

//			if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;		/* 通信超时了 */
			}
			
			if (g_tModH.fAck01H > 0)
			{
				break;		/* 接收到应答 */
			}
		}
		
		if (g_tModH.fAck01H > 0)
		{
			break;			/* 循环NUM次，如果接收到命令则break循环 */
		}
	}
	
	if (g_tModH.fAck01H == 0)
	{
		return 0;
	}
	else 
	{
		return 1;	/* 01H 读成功 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_02H
*	功能说明: 单个参数. 通过发送02H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_02H(uint16_t _reg, uint16_t _num)
{
	int32_t time1;
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		MODH_Send02H (SlaveAddr, _reg, _num);
//		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
		
		while (1)
		{
			bsp_Idle();

//			if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;		/* 通信超时了 */
			}
			
			if (g_tModH.fAck02H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck02H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck02H == 0)
	{
		return 0;
	}
	else 
	{
		return 1;	/* 02H 读成功 */
	}
}
/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_03H
*	功能说明: 单个参数. 通过发送03H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_03H(uint16_t _reg, uint16_t _num)
{
	int32_t time1;
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
//		MODH_Send03H (SlaveAddr, _reg, _num);
//		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
		
		while (1)
		{
			bsp_Idle();

//			if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;		/* 通信超时了 */
			}
			
			if (g_tModH.fAck03H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck03H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck03H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else 
	{
		return 1;	/* 写入03H参数成功 */
	}
}


/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_04H
*	功能说明: 单个参数. 通过发送04H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_04H(uint16_t _reg, uint16_t _num)
{
	int32_t time1;
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		MODH_Send04H (SlaveAddr, _reg, _num);
//		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
		
		while (1)
		{
			bsp_Idle();

//			if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;		/* 通信超时了 */
			}
			
			if (g_tModH.fAck04H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck04H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck04H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else 
	{
		return 1;	/* 04H 读成功 */
	}
}
/*
*********************************************************************************************************
*	函 数 名: MODH_WriteParam_05H
*	功能说明: 单个参数. 通过发送05H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_05H(uint16_t _reg, uint16_t _value)
{
	int32_t time1;
	uint8_t i;

	for (i = 0; i < NUM; i++)
	{
		MODH_Send05H (SlaveAddr, _reg, _value);
//		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
		
		while (1)
		{
			bsp_Idle();
			
			/* 超时大于 TIMEOUT，则认为异常 */
//			if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;	/* 通信超时了 */
			}
			
			if (g_tModH.fAck05H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck05H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck05H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else
	{
		return 1;	/* 05H 写成功 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_WriteParam_06H
*	功能说明: 单个参数. 通过发送06H指令实现，发送之后，等待从机应答。循环NUM次写命令
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_06H(uint16_t _reg, uint16_t _value)
{
	int32_t time1;
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{	
		MODH_Send06H (SlaveAddr, _reg, _value);
//		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
				
		while (1)
		{
			bsp_Idle();
		
//			if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;
			}
			
			if (g_tModH.fAck06H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck06H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck06H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else
	{
		return 1;	/* 写入06H参数成功 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_WriteParam_10H
*	功能说明: 单个参数. 通过发送10H指令实现，发送之后，等待从机应答。循环NUM次写命令
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_10H(uint16_t _reg, uint8_t _num, uint8_t *_buf)
{
	int32_t time1;
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{	
		MODH_Send10H(SlaveAddr, _reg, _num, _buf);
//		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
				
		while (1)
		{
			bsp_Idle();
		
//			if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;
			}
			
			if (g_tModH.fAck10H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck10H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck10H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else
	{
		return 1;	/* 写入10H参数成功 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_ReciveNew
*	功能说明: 串口接收中断服务程序会调用本函数。当收到一个字节时，执行一次本函数。
*	形    参: 
*	返 回 值: 1 表示有数据
*********************************************************************************************************
*/
void MODH_ReciveNew(MODH_T * G_tModH ,uint8_t *_data)
{
	/*
		3.5个字符的时间间隔，只是用在RTU模式下面，因为RTU模式没有开始符和结束符，
		两个数据包之间只能靠时间间隔来区分，Modbus定义在不同的波特率下，间隔时间是不一样的，
		所以就是3.5个字符的时间，波特率高，这个时间间隔就小，波特率低，这个时间间隔相应就大

		4800  = 7.297ms
		9600  = 3.646ms
		19200  = 1.771ms
		38400  = 0.885ms
	*/
	uint32_t timeout;

	g_modh_timeout = 0;
	
	timeout = 35000000 / HBAUD485;		/* 计算超时时间，单位us 35000000*/
	


	G_tModH->RxBufadr = _data[0];
	G_tModH->RxBufgn  = _data[1];
	G_tModH->RxBufnum = _data[2];
	G_tModH->RxBufjuli1= _data[6];
	G_tModH->RxBufjuli2= _data[5];
	G_tModH->RxBufjuli3= _data[4];
	G_tModH->RxBufjuli4= _data[3];
	G_tModH->RxBufjuli5=(_data[6] | (_data[5] << 8)) &0xffff;
	G_tModH->RxBufjuli6=(_data[4] | (_data[3] << 8)) &0xffff;
	G_tModH->RxBufcrcgao =_data[7];
	G_tModH->RxBufcrcdi =_data[8];
}
