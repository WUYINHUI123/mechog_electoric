#include "Motor.h"

// 定义电机数组，按需修改 CAN_ID
DM_Motor_Info_Typedef DM_Motors[MAX_MOTORS] = {
    { .CANFrameInfo = { .CAN_ID = 0x09, .Master_ID = 0x19 } },   // 电机1
    { .CANFrameInfo = { .CAN_ID = 0x10, .Master_ID = 0x20 } },   // 电机2
    { .CANFrameInfo = { .CAN_ID = 0x11, .Master_ID = 0x21 } },   // 电机3
    { .CANFrameInfo = { .CAN_ID = 0x12, .Master_ID = 0x22 } },   // 电机4
};

// 辅助函数：根据 CAN_ID 查找电机结构体指针
static DM_Motor_Info_Typedef* find_motor_by_id(uint16_t can_id)
{
    for (int i = 0; i < MAX_MOTORS; i++) {
        if (DM_Motors[i].CANFrameInfo.CAN_ID == can_id)
            return &DM_Motors[i];
    }
    return NULL;
}

//DM_Motor_Info_Typedef DM_6215_Motor = { //6215电机结构体
// 
//	 .CANFrameInfo = {
//		.CAN_ID = 0x09, 
//	  .Master_ID = 0x19, 
//	 },
// 
//};

#define P_MAX 12.5f
#define V_MAX 45.f
#define T_MAX 10.f


DM_Motor_Control_Typedef DM_Motor_Control;

static HAL_StatusTypeDef DM_CAN_Send(FDCAN_TxFrame_TypeDef *TxFrame)
{
    uint32_t start = HAL_GetTick();

    while (HAL_FDCAN_GetTxFifoFreeLevel(TxFrame->hcan) == 0U)
    {
        if ((HAL_GetTick() - start) > 2U)
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan, &TxFrame->Header, TxFrame->Data);
}

static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

static int float_to_uint(float X_float, float X_min, float X_max, int bits){
    float span = X_max - X_min;
    float offset = X_min;
    return (int) ((X_float-offset)*((float)((1<<bits)-1))/span);
}

void DM_Motor_Command(FDCAN_TxFrame_TypeDef *TxFrame,uint16_t CAN_ID,uint8_t CMD){

	 TxFrame->Header.Identifier = CAN_ID;
  	
	 TxFrame->Data[0] = 0xFF;
   TxFrame->Data[1] = 0xFF;
 	 TxFrame->Data[2] = 0xFF;
	 TxFrame->Data[3] = 0xFF;
	 TxFrame->Data[4] = 0xFF;
	 TxFrame->Data[5] = 0xFF;
	 TxFrame->Data[6] = 0xFF;
	
	 switch(CMD){
		 
		  case Motor_Enable :
	        TxFrame->Data[7] = 0xFC; 
	    break;
      
			case Motor_Disable :
	        TxFrame->Data[7] = 0xFD; 
      break;
      
			case Motor_Save_Zero_Position :
	        TxFrame->Data[7] = 0xFE; 
			break;
			
			default:
	    break;   
	}
	
   DM_CAN_Send(TxFrame);

}


void DM_Motor_Read_Param(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t RID){

   TxFrame->Header.Identifier = 0x7FF;
	 TxFrame->Data[0] = (uint8_t)(DM_Motor->CANFrameInfo.CAN_ID);
	 TxFrame->Data[1] = (uint8_t)(DM_Motor->CANFrameInfo.CAN_ID >> 8);
	 TxFrame->Data[2] = 0x33;
	 TxFrame->Data[3] = RID;
	 TxFrame->Data[4] = 0;
	 TxFrame->Data[5] = 0;
	 TxFrame->Data[6] = 0; 
	 TxFrame->Data[7] = 0;   
	 
 DM_CAN_Send(TxFrame); 
}

void DM_Motor_Write_Param(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t RID,uint8_t *Write_Param){

   TxFrame->Header.Identifier = 0x7FF;
	 TxFrame->Data[0] = (uint8_t)(DM_Motor->CANFrameInfo.CAN_ID);
	 TxFrame->Data[1] = (uint8_t)(DM_Motor->CANFrameInfo.CAN_ID >> 8);
	 TxFrame->Data[2] = 0x55;
	 TxFrame->Data[3] = RID;
	 TxFrame->Data[4] = Write_Param[0];
	 TxFrame->Data[5] = Write_Param[1];
	 TxFrame->Data[6] = Write_Param[2]; 
	 TxFrame->Data[7] = Write_Param[3];   
	 
 DM_CAN_Send(TxFrame); 
}

void DM_Motor_Save_Param(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor){

   TxFrame->Header.Identifier = 0x7FF;
	 TxFrame->Data[0] = (uint8_t)(DM_Motor->CANFrameInfo.CAN_ID);
	 TxFrame->Data[1] = (uint8_t)(DM_Motor->CANFrameInfo.CAN_ID >> 8);
	 TxFrame->Data[2] = 0xAA;
	 TxFrame->Data[3] = 1;
	 TxFrame->Data[4] = 0;
	 TxFrame->Data[5] = 0;
	 TxFrame->Data[6] = 0; 
	 TxFrame->Data[7] = 0;   
	 
 DM_CAN_Send(TxFrame); 
}


void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t Mode,float Postion, float Velocity, float KP, float KD, float Torque){

	 if(Mode > Velocity_Mode) Mode = MIT_Mode;	
		 
	 if(Mode == MIT_Mode) {
		
			 static uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
			 
			 Postion_Tmp  =  float_to_uint(Postion,-P_MAX,P_MAX,16) ;
			 Velocity_Tmp =  float_to_uint(Velocity,-V_MAX,V_MAX,12);
			 Torque_Tmp = float_to_uint(Torque,-T_MAX,T_MAX,12);
			 KP_Tmp = float_to_uint(KP,0,500,12);
			 KD_Tmp = float_to_uint(KD,0,5,12);

			 TxFrame->Header.Identifier = DM_Motor->CANFrameInfo.CAN_ID;
			 
			 TxFrame->Data[0] = (uint8_t)(Postion_Tmp>>8);
			 TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
			 TxFrame->Data[2] = (uint8_t)(Velocity_Tmp>>4);
			 TxFrame->Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (KP_Tmp>>8);
			 TxFrame->Data[4] = (uint8_t)(KP_Tmp);
			 TxFrame->Data[5] = (uint8_t)(KD_Tmp>>4);
			 TxFrame->Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (Torque_Tmp>>8);
			 TxFrame->Data[7] = (uint8_t)(Torque_Tmp);
			
			 DM_CAN_Send(TxFrame);
	 
	 }else if(Mode == Position_Velocity_Mode){
	 
		   KP = 0; KD = 0; Torque = 0;
		 
       uint8_t *Postion_Tmp,*Velocity_Tmp;
		   
		   Postion_Tmp = (uint8_t *)&Postion; 
		   Velocity_Tmp = (uint8_t *)&Velocity; 
		 
	     TxFrame->Header.Identifier = DM_Motor->CANFrameInfo.CAN_ID + 0x100;
			 
			 TxFrame->Data[0] = *(Postion_Tmp);
			 TxFrame->Data[1] = *(Postion_Tmp + 1);
			 TxFrame->Data[2] = *(Postion_Tmp + 2);
			 TxFrame->Data[3] = *(Postion_Tmp + 3);
			 TxFrame->Data[4] = *(Velocity_Tmp);
			 TxFrame->Data[5] = *(Velocity_Tmp + 1);
			 TxFrame->Data[6] = *(Velocity_Tmp + 2);
			 TxFrame->Data[7] = *(Velocity_Tmp + 3);
			
			 DM_CAN_Send(TxFrame);
	 
	 }else if(Mode == Velocity_Mode){
	 
	     Postion = 0;KP = 0; KD = 0; Torque = 0;
		 
       uint8_t *Velocity_Tmp;
		   
		   Velocity_Tmp = (uint8_t *)&Velocity; 
		 
	     TxFrame->Header.Identifier = DM_Motor->CANFrameInfo.CAN_ID + 0x200;
			 
			 TxFrame->Data[0] = *(Velocity_Tmp);
			 TxFrame->Data[1] = *(Velocity_Tmp + 1);
			 TxFrame->Data[2] = *(Velocity_Tmp + 2);
			 TxFrame->Data[3] = *(Velocity_Tmp + 3);
			 TxFrame->Data[4] = 0;
			 TxFrame->Data[5] = 0;
			 TxFrame->Data[6] = 0;
			 TxFrame->Data[7] = 0;
			
			 DM_CAN_Send(TxFrame);
	 
	 }
	 
	 
}



void DM_Motor_Info_Update(uint8_t *Data,DM_Motor_Info_Typedef *DM_Motor)
{
		DM_Motor->ID  = Data[0] & 0x0F;
	  DM_Motor->Data.State = Data[0]>>4;
		DM_Motor->Data.P_int = (uint16_t)(((Data[1]) <<8) | (Data[2]));
		DM_Motor->Data.V_int = (uint16_t)((Data[3]) <<4) | ((Data[4])>>4);
		DM_Motor->Data.T_int = (uint16_t)((Data[4]&0xF) <<8) | ((uint16_t)(Data[5]));
		DM_Motor->Data.Torque=  uint_to_float(DM_Motor->Data.T_int,-T_MAX,T_MAX,12);
		DM_Motor->Data.Position=uint_to_float(DM_Motor->Data.P_int,-P_MAX,P_MAX,16);
    DM_Motor->Data.Velocity=uint_to_float(DM_Motor->Data.V_int,-V_MAX,V_MAX,12);
    DM_Motor->Data.Temperature_MOS   = (float)(Data[6]);
		DM_Motor->Data.Temperature_Rotor = (float)(Data[7]);

}
	 


// ========== 简化接口（不需要传 TxFrame 指针） ==========
extern FDCAN_TxFrame_TypeDef FDCAN1TxFrame;   // 引用全局变量

void DM_Motor_Enable(uint16_t CAN_ID)
{
    DM_Motor_Command(&FDCAN1TxFrame, CAN_ID, Motor_Enable);
}

void DM_Motor_Disable(uint16_t CAN_ID)
{
    DM_Motor_Command(&FDCAN1TxFrame, CAN_ID, Motor_Disable);
}

void DM_Motor_Torque(uint16_t CAN_ID, float torque_Nm)
{
    DM_Motor_Info_Typedef* motor = find_motor_by_id(CAN_ID);
    if (motor == NULL) return;

    if (torque_Nm >  T_MAX) torque_Nm =  T_MAX;
    if (torque_Nm < -T_MAX) torque_Nm = -T_MAX;

    DM_Motor_CAN_TxMessage(&FDCAN1TxFrame, motor, MIT_Mode,
                           0.0f, 0.0f, 0.0f, 0.0f, torque_Nm);
}

void DM_Motor_SpeedRad(uint16_t CAN_ID, float speed_rads)
{
    DM_Motor_Info_Typedef* motor = find_motor_by_id(CAN_ID);
    if (motor == NULL) return;

    if (speed_rads >  V_MAX) speed_rads =  V_MAX;
    if (speed_rads < -V_MAX) speed_rads = -V_MAX;

    /* Velocity Mode frame:
     *   CAN ID = 0x200 + motor_id
     *   Data[0..3] = float velocity, unit rad/s, little-endian on STM32
     */
    DM_Motor_CAN_TxMessage(&FDCAN1TxFrame, motor, Velocity_Mode,
                           0.0f, speed_rads, 0.0f, 0.0f, 0.0f);
}

void DM_Motor_SpeedRpm(uint16_t CAN_ID, float speed_rpm)
{
    /* 1 rpm = 2*pi/60 rad/s = 0.104719755 rad/s */
    DM_Motor_SpeedRad(CAN_ID, speed_rpm * 0.104719755f);
}

void DM_Motor_Speed(uint16_t CAN_ID, float speed_rpm)
{
    /* Backward compatible interface: parameter is rpm. */
    DM_Motor_SpeedRpm(CAN_ID, speed_rpm);
}
