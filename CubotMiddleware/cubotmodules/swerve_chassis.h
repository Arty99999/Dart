#ifndef SWERVECHASSIS_H
#define SWERVECHASSIS_H

#include "stm32h7xx.h"
#include "pid.h"
#include "motor.h"
#include <dr16.h>
typedef struct 
{
	struct
	{
		uint8_t Enable;    						//< ʹ��״̬			
		uint8_t isRefereeUpdating; 		//< ����ϵͳ�Ƿ��ڸ�������
		uint8_t DriveMode;						//< ����ģʽ
		uint8_t ChassisState;					//< ����״̬
	}Flag;
	
	struct 
	{
		float Target_Power_Sum[2];		
		BasePID_Object PowerPID[2];
		float Max_Power;		
    float speedfinalout[4];
    float anglefinalout[4];	
		float bili;
		float bilibili;
	}Power;
	
	struct 
	{
		Motor   motor[4];						//< ���̵���ṹ��
		BasePID_Object RunPID[4];			//< �ٶȿ���PID����
		BasePID_Object FollowPID[1];			//< ���̸���PID����
	}Motors3508;
	
	struct 
	{
		Motor motor[4];						//< ���̵���ṹ��
		BasePID_Object TurnPID[4];			//< ת��Ƕȿ��ƽṹ��
	}Motors6020;
	
	//< ���Ƶ����˶�����Ҫ����������
	struct 
	{
		int16_t Vx;			//< ǰ���˶����ٶ�
		int16_t Vy;		  	//< �����˶����ٶ�
		int16_t Omega;		//< ��ת�Ľ��ٶ�
	  int16_t		DeltaVx;
	  int16_t		DeltaVy;
	  int16_t		DeltaOmega;
		struct 
		{
			float		Vx;
			float		Vy;
			float		Omega;
		}Sensitivity;
		
		int16_t ModuleOfSpeed;		//< �ٶ�������ģֵ
		float  AngleOfSpeed;		//< �ٶ������ĽǶ�
	}Movement;  
	
	struct 
	{
		int16_t Vx[4];
		int16_t Vy[4];
		float Angle[4];	
		float BestAngle[4];
		int16_t TargetEcd[4];
		float FeedbackAngle[4];
		int16_t SpeedNo[4];
		uint8_t SpeedChangeFlag[4];
	}Vectors;
	uint8_t SuperPowerMode;
}SwerveChassis;


void SwerveChassisMotionControl(SwerveChassis* chassis);
void VectorSolve(int16_t vx, int16_t vy, float* vectorAngle, int16_t* vectorModule, uint8_t id);
float EcdtoAngle(int16_t offset, int16_t ecd);
void SwerveChassisInit(SwerveChassis* chassis, BasePID_Object run_pid, BasePID_Object turn_pid, CanNumber canx);
float FindBestTargetAngle(float targetAngle, float feedbackAngle, uint8_t* flag);
void SwerveChassisGetRemoteData(SwerveChassis* chassis, RC_Ctrl* rc_ctrl,float canAngle);
void SwerveChassisMotionControl(SwerveChassis* chassis);
void SwerveChassisOutputControl(SwerveChassis* chassis, RC_Ctrl rcCtrl);

#endif









