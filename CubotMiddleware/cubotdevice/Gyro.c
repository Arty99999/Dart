/**@file Gyro.c
* @brief    �弶֧�ְ������ڹ����������ļ����û��ص��ض���
* @details  ��Ҫ�����������ڹ��������ṩ���ڳ�ʼ�����û��ص��ض���
* @author      RyanJiao  any question please send mail to 1095981200@qq.com

* @date        2021-8-23
* @version     V1.1
* @copyright    Copyright (c) 2021-2121  �й���ҵ��ѧCUBOTս��
**********************************************************************************
* @attention
* Ӳ��ƽ̨: STM32H750VBT \n
* SDK�汾��-++++
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>������ʼ�汾
* </table>
*
**********************************************************************************
*/

/**********************************************************************************
 ==============================================================================
						  How to use this driver
 ==============================================================================

	��� MPU6050.h

	1. ����MPU6050_Init()
  2. �� Gyro_Get_Data() ����1ms���ж�����
  

  ********************************************************************************/


#include "Gyro.h"
#include "mpu6050.h"
#include "driver_timer.h"
#include "kalman.h"
#include "holder.h"
#include "hardware_config.h"
#include "filter.h"
#include "attitude.h"
MpuDebug mpudebug =
{
	.mpuDebugTime=0,
	.DebugBeginFlag=0,
	.DebugFinishFlag=0 
};

/**
  *@brife  usart5���ܴ���������
	*/
uint8_t Usart5_RxBuffer[30]__attribute__((at(0x24002100)));
uint8_t Usart5_TxBuffer[30]__attribute__((at(0x24002130)));

//UART_RxBuffer uart5_buffer={
//	.Data = Usart5_RxBuffer,
//	.Size = 50
//};


gyro_data_t  gyro_data=
{
	.sens_yaw  =0.30f,  // yaw   ���ٶȻ���Ϊ�Ƕȵ�ֵ  ����ģ��ʹ��
	.sens_pitch=0.6783,  // pitch ���ٶȻ���Ϊ�Ƕȵ�ֵ  δ��
  .cqhlp = 10
};

gyro_data_t  gyro_data2=
{
	.sens_yaw  =0.3580,  // yaw   ���ٶȻ���Ϊ�Ƕȵ�ֵ   δ��
	.sens_pitch=0.2951,  // pitch ���ٶȻ���Ϊ�Ƕȵ�ֵ   ����ģ��ʹ��
  .cqhlp = 10
};

void MPU_Init(void)
{
	//MUP structure variable define
	MPU_Region_InitTypeDef MPU_Config;
	
	/*-----------Open FPU--------*///High speed FLOAT calculate
	SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  /* set CP10 and CP11 Full Access */
	/*-----------Open Cache------------*/
	SCB_EnableICache();//ʹ��I-Cache
  SCB_EnableDCache();//ʹ��D-Cache   
	SCB->CACR|=1<<2;   //ǿ��D-Cache͸д,�粻����,ʵ��ʹ���п���������������	
	/*-----------Open MPU------------*/
	HAL_MPU_Disable();
	
	MPU_Config.Enable=MPU_REGION_ENABLE;
	MPU_Config.Number=MPU_REGION_NUMBER1;//��������� 1
	MPU_Config.BaseAddress= 0x24000000;//����������ַ
	MPU_Config.Size=MPU_REGION_SIZE_512KB;//���ñ�����512k
	MPU_Config.SubRegionDisable=0x00;//��ֹ������
	MPU_Config.TypeExtField=MPU_TEX_LEVEL0;//����������չ��Ϊlevel0
	MPU_Config.AccessPermission=MPU_REGION_FULL_ACCESS;//ȫ���ʣ���Ȩ&�û����ɷ��ʣ�
	MPU_Config.DisableExec=MPU_INSTRUCTION_ACCESS_ENABLE;//����ָ�����
	MPU_Config.IsShareable=MPU_ACCESS_SHAREABLE;//������
	MPU_Config.IsCacheable=MPU_ACCESS_CACHEABLE;//����cache
	MPU_Config.IsBufferable=MPU_ACCESS_NOT_BUFFERABLE;//�������� DMAģʽ��Ҫ��Ϊ������
	HAL_MPU_ConfigRegion(&MPU_Config);
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}	


void Gyro_Init(void)
{
	//>�ϵ�
		MPU6050_PowerOn();
	//>��ʼ��
	  MPU6050_Init();
//		MPU6050_Init2();
	//>����Ϊ 0
	  gyro_data.cnt=0;
		gyro_data2.cnt=0;

}


void Gyro_Reset(void)
{
	//>�µ�
	  MPU6050_PowerOff();
	  HAL_Delay(50);
	
	//>�ϵ��ʼ��
    Gyro_Init();

}

/*
 *�������� 
 *yaw��̬���ٶ�   
 *pitch��̬���ٶ� 
 */

//>����1ms�Ķ�ʱ���ж���
float yaw_data_filter_mpu[5]={0};
float pitch_data_filter_mpu[5]={0};
int8_t yaw_filter_mpu_cnt=0;
int8_t pitch_filter_mpu_cnt=0;
void Gyro_Get_Data(void){
	//>���ݸ��´����ۼ�
  gyro_data.cnt++;
	
  //>MPU6050���ݸ���
	MPU6050_RawDataUpdate(&mpuAngle,1);

if(gyro_data.cnt>1000){	
		//>��Ԫ������� pitch�Ƕ�
	
	//>�ǶȻ�Ϊ����  ���ٶȽ��п������˲�  ����һ���޸ģ�
	PrepareForIMU(&sensor,&mpuAngle);
	IMUupdate(&sensor,&mpuAngle,1);
	
	gyro_data.gyro_w_yaw   = mpuAngle.gyroRaw.z-(int16_t)sensor.gyro.quiet.z;
	gyro_data.gyro_w_pitch = mpuAngle.gyroRaw.x-(int16_t)sensor.gyro.quiet.x;

	if(abs(gyro_data.gyro_w_yaw)>100){
			gyro_data.yaw_speed=((gyro_data.gyro_w_yaw)*0.0001*gyro_data.sens_yaw);
			gyro_data.yaw_angle= gyro_data.yaw_angle + gyro_data.yaw_speed;}
	
	if(abs(gyro_data.gyro_w_pitch)>100){		
			gyro_data.pitch_speed = ((gyro_data.gyro_w_pitch)*0.0001f*gyro_data.sens_pitch);
	    gyro_data.pitch_angle= gyro_data.pitch_angle + gyro_data.pitch_speed;}
	if (gyro_data.yaw_angle>=180) gyro_data.yaw_angle-=360;
			else if(gyro_data.yaw_angle<=-180) gyro_data.yaw_angle+=360;
		}
}

void Gyro_Get_Data2(void){
	//>���ݸ��´����ۼ�
  gyro_data2.cnt++;
	
  //>MPU6050���ݸ���
	MPU6050_RawDataUpdate(&mpuAngle2,2);

if(gyro_data2.cnt>1000){	
	
	//>�ǶȻ�Ϊ����  ���ٶȽ��п������˲�  ����һ���޸ģ�
	PrepareForIMU(&sensor2,&mpuAngle2);
	//>��Ԫ������� pitch�Ƕ�
	IMUupdate(&sensor2,&mpuAngle2,2);	
	
	gyro_data2.gyro_w_yaw   = mpuAngle2.gyroRaw.z-(int16_t)sensor2.gyro.quiet.z;
	gyro_data2.gyro_w_pitch = mpuAngle2.gyroRaw.x-(int16_t)sensor2.gyro.quiet.x;

	if(abs(gyro_data2.gyro_w_yaw)>10){
			gyro_data2.yaw_speed=((gyro_data2.gyro_w_yaw)*0.0001*gyro_data2.sens_yaw);
			gyro_data2.yaw_angle= gyro_data2.yaw_angle + gyro_data2.yaw_speed;}
		
	if(abs(gyro_data2.gyro_w_pitch)>10){		
			gyro_data2.pitch_speed = ((gyro_data2.gyro_w_pitch)*0.0001f*gyro_data2.sens_pitch);
	    gyro_data2.pitch_angle= gyro_data2.pitch_angle + gyro_data2.pitch_speed;}
		}
}

void MPU_Get_Data(Holder_t* holder)
{
	Gyro_Get_Data();  //����������
//	Gyro_Get_Data2();   //����IIC������ģ��
	holder->Yaw.MPU6050_Angle=-gyro_data2.yaw_angle;
	holder->Yaw.MPU6050_Angle_speed=gyro_data2.yaw_speed;
  holder->Yaw.MPU6050_Angle_speed1=holder->Yaw.MPU6050_Angle_speed*150;
//holder->Yaw.MPU6050_Angle_speed1=LPFilter(holder->Yaw.MPU6050_Angle_speed1 ,&LPF_yaw_mpu);  //һ�׵�ͨ�˲�
	holder->Pitch.MPU6050_Angle=gyro_data2.pitch_angle;
	holder->Pitch.MPU6050_Angle_speed=gyro_data2.pitch_speed;
	holder->Pitch.MPU6050_Angle_speed1=holder->Pitch.MPU6050_Angle_speed*150;
//holder->Pitch.MPU6050_Angle_speed1=LPFilter(holder->Pitch.MPU6050_Angle_speed1 ,&LPF_pitch_mpu);   //һ�׵�ͨ�˲�
	
	
}

/**
  * @brief   ����5���������ǻص����� 
  * @param[in]  
  */
//uint8_t Gyro_callback(uint8_t * recBuffer, uint16_t len)
//{
////	angle_deal(recBuffer,&Holder);
//	HWT906_Data_Deal(recBuffer,&Holder);
//	return 0;
//}

///**
//  * @brief   ����ԭ�Ӵ��������ǻص����� ,������115200
//  * @param[in]  
//  */
//void angle_deal(uint8_t *angle_receive,Attitude_t *attitude)
//{
//	if(angle_receive[0]==0x55 && angle_receive[1]==0x55){
//		if(angle_receive[15]==0x03){
//			attitude->Pitch = -1*(float) ( (int16_t) (angle_receive[26]<<8) | angle_receive[25]) / 32768 * 2000;// * 0.01745;
//			attitude->Yaw.Uart6050_Angle_speed = -1*(float) ( (int16_t) (angle_receive[28]<<8) | angle_receive[27]) / 32768 * 2000;// * 0.01745;
//		}
//		if(angle_receive[2]==0x02){
//			attitude->q0 = (float)((int16_t)(angle_receive[5]<<8 | angle_receive[4])) / 32768;
//			attitude->q1 = (float)((int16_t)(angle_receive[7]<<8 | angle_receive[6])) / 32768;
//			attitude->q2 = (float)((int16_t)(angle_receive[9]<<8 | angle_receive[8])) / 32768;
//			attitude->q3 = (float)((int16_t)(angle_receive[11]<<8 | angle_receive[10])) / 32768;
//		}
//  		attitude->Yaw.Uart6050_Angle_raw = -atan2(2 * holder->q1 * holder->q2 + 2 * holder->q0* holder->q3, -2 * holder->q2*holder->q2 - 2 * holder->q3 * holder->q3 + 1)*RtA; 
//      attitude->Pitch.Uart6050_Angle_raw = -asin(-2 * holder->q1 * holder->q3 + 2 * holder->q0 * holder->q2)*RtA;
//		}
//	}

//	
///**
//  * @brief   ά�ش��������ǻص�������������915200
//  * @param[in]  
//  */	
//float HWT906_pitch;
//float HWT906_yaw;
//uint16_t count=0;
//float count1=0;
//void HWT906_Data_Deal(uint8_t *data_receive,Holder_t* holder)
//{
//	count++;
//	if(tim14.ClockTime%1000==0)
//	{
//		count1=count;
//		count=0;
//	}
//	if(data_receive[0]==0x55 && data_receive[1]==0x51){
//		holder->Pitch.Uart6050_Angle_acc = (float)((int16_t)(data_receive[3]<<8|data_receive[2]))*32768*16*9.8f;
//		holder->Yaw.Uart6050_Angle_acc = (float)((int16_t)(data_receive[7]<<8|data_receive[6]))*32768*16*9.8f;
//	}
//	if(data_receive[11]==0x55 && data_receive[12]==0x52){
//		holder->Pitch.Uart6050_Angle_speed = (float)((int16_t)(data_receive[14]<<8|data_receive[13]))/32768*2000;
//		holder->Yaw.Uart6050_Angle_speed = (float)((int16_t)(data_receive[18]<<8|data_receive[17]))/32768*2000;
//	}
//	if(data_receive[22]==0x55 && data_receive[23]==0x59){
//		holder->q0 = (float)((int16_t)(data_receive[25]<<8|data_receive[24]))/32768;
//		holder->q1 = (float)((int16_t)(data_receive[27]<<8|data_receive[26]))/32768;
//		holder->q2 = (float)((int16_t)(data_receive[29]<<8|data_receive[28]))/32768;
//		holder->q3 = (float)((int16_t)(data_receive[31]<<8|data_receive[30]))/32768;
//	}
//  	holder->Yaw.Uart6050_Angle_raw = -atan2(2 * holder->q1 * holder->q2 + 2 * holder->q0* holder->q3, -2 * holder->q2*holder->q2 - 2 * holder->q3 * holder->q3 + 1)*RtA; 
//		holder->Pitch.Uart6050_Angle_raw = atan2(2 * holder->q2 * holder->q3 + 2 * holder->q0 * holder->q1, -2 * holder->q1 * holder->q1 - 2 * holder->q2* holder->q2 + 1)* RtA;    // ��Ϊroll�����Ԫ�����㹫ʽ����Ϊ�����ǹ̶������ԭ�򣬽�roll�ǵ���pitch����
////	  holder->Pitch.Uart6050_Angle_raw = -asin(-2 * holder->q1 * holder->q3 + 2 * holder->q0 * holder->q2)*RtA;

//}

//float sens_pitch_change = 1;
///*�����������ǽ��յ���-180��180�ĽǶ�ת�����ۼ�ʽ�ĽǶ�*/
//void HolderAngleChange(Holder_t *holder)
//{
//	if(holder->Pitch.MPU6050_Angle_Change!=0)
//	sens_pitch_change=abs(holder->Pitch.Can_Angle_Change) /abs(holder->Pitch.MPU6050_Angle_Change);

//  if(holder->Yaw.Uart6050_Angle<170 && holder->Yaw.Uart6050_Angle >-170)
//		 holder->Yaw.Uart6050_Angle = holder->Yaw.Uart6050_Angle_raw;
//	else{
//		if(holder->Yaw.Uart6050_Angle_raw<-140 && holder->Yaw.Uart6050_Angle_raw_last>140)
//			holder->Yaw.Uart6050_Angle+=(360+holder->Yaw.Uart6050_Angle_raw - holder->Yaw.Uart6050_Angle_raw_last);
//		else if(holder->Yaw.Uart6050_Angle_raw>140 && holder->Yaw.Uart6050_Angle_raw_last<-140)
//			holder->Yaw.Uart6050_Angle+=(360-holder->Yaw.Uart6050_Angle_raw + holder->Yaw.Uart6050_Angle_raw_last);
//		else
//			holder->Yaw.Uart6050_Angle += (holder->Yaw.Uart6050_Angle_raw - holder->Yaw.Uart6050_Angle_raw_last);
//    	}
//		holder->Yaw.Uart6050_Angle_raw_last=Holder.Yaw.Uart6050_Angle_raw;
//  	holder->Pitch.Uart6050_Angle = Holder.Pitch.Uart6050_Angle_raw; 
//	  if(tim14.ClockTime%1000 == 0)
//		{			
//		holder->Pitch.Can_Angle_Change = Holder.Pitch.Last_Can_Angle - Holder.Pitch.Can_Angle;
//		holder->Pitch.MPU6050_Angle_Change = Holder.Pitch.Last_MPU6050_Angle - Holder.Pitch.MPU6050_Angle;
//		holder->Pitch.Last_Can_Angle = Holder.Pitch.Can_Angle;
//		holder->Pitch.Last_MPU6050_Angle = Holder.Pitch.MPU6050_Angle;
//		}
//}

///**
//  * @brief  У׼�����ǽǶ�
//  */
//uint8_t Mpu6050_senceDebug(MpuDebug* mpuDebug,gyro_data_t* gyro_data)
//{
//if((tim14.ClockTime>1000)&&(mpuDebug->DebugFinishFlag==0)&&(mpuDebug->DebugBeginFlag==1))
//{
//	mpuDebug->mpuDebugTime++;
//	if(mpuDebug->mpuDebugTime==200)
//	{
//		mpuDebug->Yaw.MpuAngle1=Holder.Yaw .MPU6050_Angle;
//		mpuDebug->Yaw.CanAngle1=Holder.Yaw.Can_Angle;
//		mpuDebug->Pitch.MpuAngle1=Holder.Pitch .MPU6050_Angle;
//		mpuDebug->Pitch.CanAngle1=Holder.Pitch.Can_Angle;
//	}
//	if(mpuDebug->mpuDebugTime==400)
//	{
//		mpuDebug->Yaw.MpuAngle2=Holder.Yaw .MPU6050_Angle;
//		mpuDebug->Yaw.CanAngle2=Holder.Yaw.Can_Angle;
//		mpuDebug->Pitch.MpuAngle2=Holder.Pitch .MPU6050_Angle;
//		mpuDebug->Pitch.CanAngle2=Holder.Pitch.Can_Angle;
//	}
//	if(mpuDebug->mpuDebugTime==600)
//	{
//		mpuDebug->Yaw.MpuAngle3=Holder.Yaw .MPU6050_Angle;
//		mpuDebug->Yaw.CanAngle3=Holder.Yaw.Can_Angle;
//		mpuDebug->Pitch.MpuAngle3=Holder.Pitch .MPU6050_Angle;
//		mpuDebug->Pitch.CanAngle3=Holder.Pitch.Can_Angle;
//		mpuDebug->Yaw.MpuAngleChange=((mpuDebug->Yaw.MpuAngle3-mpuDebug->Yaw.MpuAngle2)+(mpuDebug->Yaw.MpuAngle2-mpuDebug->Yaw.MpuAngle1))*0.5;
//		mpuDebug->Yaw.CanAngleChange=((mpuDebug->Yaw.CanAngle3-mpuDebug->Yaw.CanAngle2)+(mpuDebug->Yaw.CanAngle2-mpuDebug->Yaw.CanAngle1))*0.5;
//		mpuDebug->Pitch.MpuAngleChange=((mpuDebug->Pitch.MpuAngle3-mpuDebug->Pitch.MpuAngle2)+(mpuDebug->Pitch.MpuAngle2-mpuDebug->Pitch.MpuAngle1))*0.5;
//		mpuDebug->Pitch.CanAngleChange=((mpuDebug->Pitch.CanAngle3-mpuDebug->Pitch.CanAngle2)+(mpuDebug->Pitch.CanAngle2-mpuDebug->Pitch.CanAngle1))*0.5;
//    mpuDebug->Yaw.SenceBili=abs(mpuDebug->Yaw.CanAngleChange)/abs(mpuDebug->Yaw.MpuAngleChange);
//		mpuDebug->Pitch.SenceBili=abs(mpuDebug->Pitch.CanAngleChange)/abs(mpuDebug->Pitch.MpuAngleChange);
//		if((mpuDebug->Yaw.SenceBili>=2)||(mpuDebug->Yaw.SenceBili<=0))
//			mpuDebug->Yaw.SenceBili=1;
//		if((mpuDebug->Pitch.SenceBili>=2)||(mpuDebug->Pitch.SenceBili<=0))
//			mpuDebug->Pitch.SenceBili=1;
//		gyro_data->sens_yaw=gyro_data->sens_yaw*(mpuDebug->Yaw.SenceBili);
//		gyro_data->sens_pitch=gyro_data->sens_pitch*(mpuDebug->Pitch.SenceBili);
//	}
//	if(mpuDebug->mpuDebugTime>600)
//	{
//		mpuDebug->mpuDebugTime=600;
//	  mpuDebug->DebugFinishFlag=1;
//	}
// }
//}

