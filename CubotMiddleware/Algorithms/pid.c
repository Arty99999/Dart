#include<pid.h>
#include "dr16.h"
#include "driver_timer.h"


void BasePID_Init(BasePID_Object* base_pid, float kp, float ki, float kd, float detach)
{
	base_pid->KiPartDetachment = detach;
	
   base_pid->Kp = kp;
   base_pid->Ki = ki;
	 base_pid->Kd = kd;
	
	base_pid->KpPart = 0;
  base_pid->KiPart = 0;
  base_pid->KdPart = 0;

}

/**
  * @brief pitch角度环  双环外环
  */
int32_t BasePID_PitchAngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle)
{
	base_pid->Error = target_angle - feedback_angle;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	
	
	base_pid->KiPartDetachment = 15;
	if((base_pid->Error > base_pid->KiPartDetachment)||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if((base_pid->Error < -(base_pid->KiPartDetachment))||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}

float BasePID_SpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{
	base_pid->Error = target_speed - feedback_speed;
	
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	
	if(base_pid->Error > base_pid->KiPartDetachment)
	{
		base_pid->KiPart = 0;
	}
	else if(base_pid->Error < -(base_pid->KiPartDetachment))
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->Out = base_pid->KpPart + base_pid->KiPart;
	
	return base_pid->Out;
}
//int32_t BasePID_AngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle, float feedback_angle_speed)
//{
//	base_pid->Error = target_angle - feedback_angle;
//	base_pid->KpPart = base_pid->Error * base_pid->Kp;
//	base_pid->KiPart += base_pid->Error * base_pid->Ki;
//	base_pid->KiPartDetachment = 60;
//	if(base_pid->Error > base_pid->KiPartDetachment)
//	{
//		base_pid->KiPart = 0;
//	}
//	else if(base_pid->Error < -(base_pid->KiPartDetachment))
//	{
//		base_pid->KiPart = 0;
//	}
//	
//	base_pid->KdPart = (-1) * base_pid->Kd * feedback_angle_speed;
//	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
//	
//	return base_pid->Out;
//}


/**
  * @brief PID角度环
  */
int32_t BasePID_AngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle)
{
	base_pid->Error = target_angle - feedback_angle;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	base_pid->KiPartDetachment = 15;
	if((base_pid->Error > base_pid->KiPartDetachment)||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if((base_pid->Error < -(base_pid->KiPartDetachment))||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}

/**
  * @brief pitch速度环  双环内环
  */
int32_t BasePID_PitchSpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{
	base_pid->Error = target_speed - feedback_speed;
//	if((base_pid->Error>-0.1)&&(base_pid->Error<0.1))
//		base_pid->Error=0;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	base_pid->KiPartDetachment = 150;
	if(base_pid->Error > base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if((base_pid->Error < -base_pid->KiPartDetachment)||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	return base_pid->Out;
}


/**
  * @brief yaw角度环  双环外环
  */
int32_t BasePID_YawAngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle)
{
	base_pid->Error = target_angle - feedback_angle;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	
	
	
		if (base_pid->Speed_detach!=0 && base_pid->Speed_detach_Negative==0)
	{if (base_pid->KpPart>base_pid->Speed_detach) base_pid->KpPart=base_pid->Speed_detach;
	else if(base_pid->KpPart<-base_pid->Speed_detach) base_pid->KpPart=-base_pid->Speed_detach;}
	else if (base_pid->Speed_detach!=0 && base_pid->Speed_detach_Negative!=0)
		{if (base_pid->KpPart>base_pid->Speed_detach) base_pid->KpPart=base_pid->Speed_detach;
	else if(base_pid->KpPart<base_pid->Speed_detach_Negative) base_pid->KpPart=base_pid->Speed_detach_Negative;}
	
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	base_pid->KiPartDetachment = 15;
	if((base_pid->Error > base_pid->KiPartDetachment)||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if((base_pid->Error < -(base_pid->KiPartDetachment))||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}

/**
  * @brief yaw速度环  双环内环
  */
int32_t BasePID_YawSpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{
	base_pid->Error = target_speed - feedback_speed;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	base_pid->KiPartDetachment = 150;
	if(base_pid->Error > base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if(base_pid->Error < -base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}

/**
  * @brief lifting速度环  双环内环
  */
int32_t BasePID_LiftingSpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{
	base_pid->Error = target_speed - feedback_speed;
//	if((base_pid->Error>-0.1)&&(base_pid->Error<0.1))
//		base_pid->Error=0;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	base_pid->KiPartDetachment = 300;
	if(base_pid->Error > base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if((base_pid->Error < -base_pid->KiPartDetachment)||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	return base_pid->Out;
}


/**
  * @brief lifting角度环  双环外环
  */
int32_t BasePID_LiftingAngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle)
{
	base_pid->Error = target_angle - feedback_angle;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	base_pid->KiPartDetachment = 300;
	if(base_pid->Error > base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if(base_pid->Error < -(base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0))
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}

/**
  * @brief sideway速度环  双环内环
  */
int32_t BasePID_SidewaySpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{
	base_pid->Error = target_speed - feedback_speed;
//	if((base_pid->Error>-0.1)&&(base_pid->Error<0.1))
//		base_pid->Error=0;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	base_pid->KiPartDetachment = 150;
	if(base_pid->Error > base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if((base_pid->Error < -base_pid->KiPartDetachment)||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	return base_pid->Out;
}


/**
  * @brief sideway角度环  双环外环
  */
int32_t BasePID_SidewayAngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle)
{
	base_pid->Error = target_angle - feedback_angle;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	base_pid->KiPartDetachment = 150;
	if(base_pid->Error > base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if(base_pid->Error < -(base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0))
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}

/**
  * @brief 底盘速度环  双环内环
  */
int32_t BasePID_mecaSpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{
	base_pid->Error = target_speed - feedback_speed;
//	if((base_pid->Error>-0.1)&&(base_pid->Error<0.1))
//		base_pid->Error=0;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	base_pid->KiPartDetachment = 150;
	if(base_pid->Error > base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if((base_pid->Error < -base_pid->KiPartDetachment)||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	return base_pid->Out;
}


/**
  * @brief 底盘角度环  双环外环
  */
int32_t BasePID_mecaAngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle)
{
	base_pid->Error = target_angle - feedback_angle;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	base_pid->KiPartDetachment = 150;
	if(base_pid->Error > base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if(base_pid->Error < -(base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0))
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}
