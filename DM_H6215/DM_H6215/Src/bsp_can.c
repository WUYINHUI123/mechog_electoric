#include "fdcan.h"
#include "bsp_can.h"
#include "bsp_can2.h"
#include "Motor.h"

extern DM_Motor_Info_Typedef DM_Motors[MAX_MOTORS];

FDCAN_RxFrame_TypeDef FDCAN1_RxFrame;

FDCAN_TxFrame_TypeDef FDCAN1TxFrame = {
  .hcan = &hfdcan1,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = FDCAN_DLC_BYTES_8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};

void BSP_FDCAN_Init(void){

  FDCAN_FilterTypeDef FDCAN1_FilterConfig;
	
	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID; // ���˱�׼ID������CANֻ�б�׼ID
  FDCAN1_FilterConfig.FilterIndex = 0;           //��������ţ��ü�·CAN����������0��1��2....
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK; //������Maskģʽ �غ�������ID1��ID2������
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//ѡ���ĸ�FIFO�����գ�������CubeMX����������FIFO1�͸ĳ�FDCAN_FILTER_TO_RXFIFO1
  FDCAN1_FilterConfig.FilterID1 = 0x00000000; // ������У�ֻҪID2����0x00000000�Ͳ�����˵��κ�ID
  FDCAN1_FilterConfig.FilterID2 = 0x00000000; //��������
  
  HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig); //���������õ�CAN1
		
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); //����CAN1��ȫ�ֹ��ˣ����ǿ���������
 
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//��FIFO0���������ݽ����жϣ�
  
  HAL_FDCAN_Start(&hfdcan1);//ʹ��CAN1
 	
	
}


static void FDCAN1_RxFifo0RxHandler(uint32_t *Master_ID, uint8_t Data[8])
{
    uint32_t id = *Master_ID;
    for (int i = 0; i < MAX_MOTORS; i++) {
        if (DM_Motors[i].CANFrameInfo.CAN_ID == id) {
            DM_Motor_Info_Update(Data, &DM_Motors[i]);
            break;
        }
    }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == FDCAN1)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxFrame.Header, FDCAN1_RxFrame.Data);
        FDCAN1_RxFifo0RxHandler(&FDCAN1_RxFrame.Header.Identifier, FDCAN1_RxFrame.Data);
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        CAN2_RxFifo0RxHandler(hfdcan, RxFifo0ITs);
    }
}
	

	
	
	
