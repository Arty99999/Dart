#include<driver_timer.h>

TIM_Object tim14={
	.ClockTime = 0,
	.Test_Time = 0
};
TIM_Object tim13={
	.ClockTime = 0,

};
void TIMx_Init(TIM_HandleTypeDef* handle,TIM_ElapsedCallback callback)
{
    if(handle->Instance==TIM14)
		{
		  tim14.Handle=handle;
			tim14.ElapCallback=callback;
		}
		    if(handle->Instance==TIM13)
		{
		  tim13.Handle=handle;
			tim13.ElapCallback=callback;
		}
}

void TIM_Open(TIM_Object* tim)
{
	HAL_TIM_Base_Start_IT(tim->Handle);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	
{
	
	if(htim==(&htim14))
	{
		tim14.ElapCallback();
	}
		if(htim==(&htim13))
	{
		tim13.ElapCallback();
	}
	
}

