#ifndef DRIVERTIMER_H
#define DRIVERTIMER_H
#include <tim.h>

typedef void (*TIM_ElapsedCallback)(void); 
typedef struct
{
	TIM_HandleTypeDef* 	Handle;
//	uint32_t ClockTime;             			//< 任务定时器的计数变量
	int32_t ClockTime;
	int32_t Test_Time;
	TIM_ElapsedCallback		ElapCallback;		//< 定时器溢出中断
	uint16_t HolderTime;
	uint16_t ErrorTime;
	uint32_t UI_Time;
}TIM_Object;


void TIMx_Init(TIM_HandleTypeDef* handle,TIM_ElapsedCallback callback);
void TIM_Open(TIM_Object* tim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

extern TIM_Object tim14;
extern TIM_Object tim13;
#endif



