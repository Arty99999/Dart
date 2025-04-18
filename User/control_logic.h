#ifndef CONTROLLOGIC_H_
#define CONTROLLOGIC_H_

#include <stm32h7xx_hal.h>
#include "hardware_config.h"
#include <swerve_chassis.h>
#include <check.h>

void TIM14_Task(void);
void TIM13_Task(void);
uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer);
uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer);
void TIM14_Task_send(void);



//����Ϊ������ʱ����
extern float test01;
extern float test02;
extern uint16_t test03;
extern uint16_t test04;


extern uint8_t Auto_state;

		typedef enum {
    RELOAD_IDLE = 0,      // ����״̬
    RELOAD_STEP1,         // ��һ�� �����ת
		RELOAD_STEP1_5, 	//��һ.5����6020�����ת
    RELOAD_STEP2,         // �ڶ�����2006�������
    RELOAD_STEP3,         // ��������2006�������
    RELOAD_STEP4,         // ���Ĳ���6020�����λ
    RELOAD_COMPLETE ,      // ���
			RELOAD_ERROR
} ReloadState_t;

	typedef enum {
    Rubber_IDLE = 0,      
    Rubber_STEP1,     //3508����    
    Rubber_STEP2,     //�����£
		Rubber_STEP3,     //3508����
    Rubber_COMPLETE ,     
			Rubber_ERROR
}
RubberState_t;

uint8_t LCD_callback(uint8_t * recBuffer, uint16_t len);

#endif



