/**
@file		can.c
@brief		CAN bus routines
@author		Matej Kogovsek (matej@hamradio.si)
@copyright	LGPL 2.1
@note		This file is part of mat-stm32f1-lib
*/

#include <stm32f10x.h>
#include <stm32f10x_can.h>

//-----------------------------------------------------------------------------
//  CAN filter mask helper – forces IDE=0 and RTR=data
//-----------------------------------------------------------------------------
#define CAN_FILTER_ID_SHIFT  21U
#define CAN_FILTER_IDE_MASK  (1UL << 2)
#define CAN_FILTER_RTR_MASK  (1UL << 1)

uint8_t can_init(uint16_t br)
{
	GPIO_InitTypeDef iotd;

	// configure CAN1 IOs
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// configure CAN1 RX pin
	iotd.GPIO_Pin = GPIO_Pin_11;
	iotd.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &iotd);

	// configure CAN1 TX pin
	iotd.GPIO_Pin = GPIO_Pin_12;
	iotd.GPIO_Mode = GPIO_Mode_AF_PP;
	iotd.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &iotd);

	// CAN1 peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	// CAN1 register init
	CAN_DeInit(CAN1);

	// init CAN1
	CAN_InitTypeDef cnis;
	CAN_StructInit(&cnis);
	cnis.CAN_TTCM = DISABLE;
	cnis.CAN_ABOM = ENABLE;      // automatic bus‑off recovery
	cnis.CAN_AWUM = DISABLE;
	cnis.CAN_NART = ENABLE;
	cnis.CAN_RFLM = DISABLE;
	cnis.CAN_TXFP = DISABLE;
	cnis.CAN_Mode = CAN_Mode_Normal;
	cnis.CAN_SJW = CAN_SJW_1tq;
	cnis.CAN_BS1 = CAN_BS1_3tq;
	cnis.CAN_BS2 = CAN_BS2_2tq;
	cnis.CAN_Prescaler = br;

	// Return 0 if init succeeded, 1 otherwise
	return (CAN_Init(CAN1, &cnis) == CAN_InitStatus_Success) ? 0 : 1;
}

uint8_t can_filter(uint32_t id, uint32_t msk, uint8_t canfilnum)
{
	if( canfilnum >= 14 ) return 0;

	// Standard ID lives in bits [31:21] of the 32-bit filter register value.
	// Force IDE bit to 0 and RTR bit to 0 (data frame) so extended and remote
	// frames never match a standard‑ID filter.
	uint32_t idr = id << CAN_FILTER_ID_SHIFT;
	uint32_t mskr = (msk << CAN_FILTER_ID_SHIFT) |
	                CAN_FILTER_IDE_MASK |
	                CAN_FILTER_RTR_MASK;

	CAN_FilterInitTypeDef fitd;
	fitd.CAN_FilterNumber = canfilnum;
	fitd.CAN_FilterMode = CAN_FilterMode_IdMask;
	fitd.CAN_FilterScale = CAN_FilterScale_32bit;
	fitd.CAN_FilterIdHigh = idr >> 16;
	fitd.CAN_FilterIdLow = (uint16_t)idr;
	fitd.CAN_FilterMaskIdHigh = mskr >> 16;
	fitd.CAN_FilterMaskIdLow = (uint16_t)mskr;
	fitd.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	fitd.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&fitd);

	return canfilnum;
}

uint8_t can_tx(CanTxMsg* msg)
{
	uint8_t mailbox = CAN_Transmit(CAN1, msg);
	if (mailbox == CAN_TxStatus_NoMailBox) return 0;
	// Wait for transmission to complete or time out (simple implementation)
	uint32_t timeout = 0xFFFF;
	while (!(CAN_TransmitStatus(CAN1, mailbox) & CAN_TxStatus_Failed) && timeout--) {
		if (CAN_TransmitStatus(CAN1, mailbox) & CAN_TxStatus_Ok) return 1;
	}
	return 0; // timed out or failed
}

uint8_t can_rx(CanRxMsg* msg)
{
	if( CAN_MessagePending(CAN1, CAN_FIFO0) ) {
		CAN_Receive(CAN1, CAN_FIFO0, msg);
		return 1;
	}
	return 0;
}