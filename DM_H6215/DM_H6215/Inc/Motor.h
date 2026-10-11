#ifndef Motor_H
#define Motor_H

#include "bsp_can.h"
#include "stdbool.h"

typedef enum{

  Motor_Enable,
  Motor_Disable,
  Motor_Save_Zero_Position,
  DM_Motor_CMD_Type_Num,

}DM_Motor_CMD_e;

typedef enum{

  MIT_Mode,
  Position_Velocity_Mode,
  Velocity_Mode,
  DM_Motor_Mode_Type_Num,

}DM_Motor_Mode_e;


typedef struct 
{
  int16_t  State; 	
  uint16_t  P_int;
  uint16_t  V_int;
  uint16_t  T_int;
  float  Position;  
  float  Velocity;  
  float  Torque;  
  float  Temperature_MOS;   
  float  Temperature_Rotor;  
}DM_Motor_Data_Typedef;

typedef struct
{
  uint32_t Master_ID;   
  uint32_t CAN_ID;  
}Motor_CANFrameInfo_typedef;

typedef struct
{
	uint16_t ID;
  Motor_CANFrameInfo_typedef CANFrameInfo;
	DM_Motor_Data_Typedef Data;  
}DM_Motor_Info_Typedef;

typedef struct
{
	float  KP;
	float  KD;
	float  Position; 
  float  Velocity;  	
  float  Torque;  
	
}DM_Motor_Control_Typedef;

extern DM_Motor_Info_Typedef DM_6215_Motor;

#define MAX_MOTORS  4
extern DM_Motor_Info_Typedef DM_Motors[MAX_MOTORS];

extern void DM_Motor_Enable(uint16_t CAN_ID);
extern void DM_Motor_Disable(uint16_t CAN_ID);
extern void DM_Motor_Torque(uint16_t CAN_ID, float torque_Nm);
extern void DM_Motor_Speed(uint16_t CAN_ID, float speed_rpm);      // backward compatible: rpm input
extern void DM_Motor_SpeedRpm(uint16_t CAN_ID, float speed_rpm);   // velocity mode, rpm input
extern void DM_Motor_SpeedRad(uint16_t CAN_ID, float speed_rads);  // velocity mode, rad/s input

extern DM_Motor_Control_Typedef DM_Motor_Control;

extern void DM_Motor_Info_Update(uint8_t *rxBuf,DM_Motor_Info_Typedef *DM_Motor);

extern void DM_Motor_Command(FDCAN_TxFrame_TypeDef *TxFrame,uint16_t CAN_ID,uint8_t CMD);

extern void DM_Motor_Read_Param(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t RID);

extern void DM_Motor_Write_Param(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t RID,uint8_t *Write_Param);
	
extern void DM_Motor_Save_Param(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor);

extern void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t Mode,
	                                             float Postion, float Velocity, float KP, float KD, float Torque);

#endif