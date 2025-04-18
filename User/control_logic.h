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



//以下为测试临时变量
extern float test01;
extern float test02;
extern uint16_t test03;
extern uint16_t test04;


extern uint8_t Auto_state;

		typedef enum {
    RELOAD_IDLE = 0,      // 空闲状态
    RELOAD_STEP1,         // 第一步 舵机旋转
		RELOAD_STEP1_5, 	//第一.5步：6020电机旋转
    RELOAD_STEP2,         // 第二步：2006电机下移
    RELOAD_STEP3,         // 第三步：2006电机上移
    RELOAD_STEP4,         // 第四步：6020电机复位
    RELOAD_COMPLETE ,      // 完成
			RELOAD_ERROR
} ReloadState_t;

	typedef enum {
    Rubber_IDLE = 0,      
    Rubber_STEP1,     //3508下拉    
    Rubber_STEP2,     //舵机合拢
		Rubber_STEP3,     //3508上拉
    Rubber_COMPLETE ,     
			Rubber_ERROR
}
RubberState_t;

uint8_t LCD_callback(uint8_t * recBuffer, uint16_t len);

#endif



