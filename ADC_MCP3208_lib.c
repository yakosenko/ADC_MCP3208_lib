/*
 * ADC_MCP3208_lib.c
 *
 *  Created on: 16 груд. 2020 р.
 *      Author: Denys.S.Yakosenko
 */

#include "ADC_MCP3208_lib.h"

void MCP3208_vInitPeriph(void);

void MCP3208_vSendData(uint8_t *pu8TxAddress, uint8_t u8Size);
void MCP3208_vReceiveData(uint8_t *pu8RxAddress, uint8_t u8Size);

void MCP3208_vWriteChipSelect(MCP3208_tenChipType enCurrentChip, BitAction BitAct);
void MCP3208_vChooseInput(uint8_t u8NumInput);

MCP3208_tstContextType MCP3208_stContext;

uint32_t MCP3208_u32GetADCInputData(MCP3208_tenChipType enChipNum, uint8_t u8NumInput)
{
	uint32_t u32Output = 0;

	if(enChipNum < MCP3208_enChipsNumber && u8NumInput < MCP3208_NUM_INPUT)
	u32Output = MCP3208_stContext.MCP3208_u16InputValue[enChipNum][u8NumInput];

	return u32Output;
}

void MCP3208_vChooseInput(uint8_t u8NumInput)
{
	uint32_t u32TempTxPacket = 0xFFFFFFFF;
	u32TempTxPacket = (u32TempTxPacket & 0x00060000) | (u8NumInput << 14);

	MCP3208_stContext.MCP3208_TxBuffer[0] = (uint8_t) (u32TempTxPacket >> 16);
	MCP3208_stContext.MCP3208_TxBuffer[1] = (uint8_t) (u32TempTxPacket >> 8);
	MCP3208_stContext.MCP3208_TxBuffer[2] = (uint8_t) (u32TempTxPacket & 0x000000FF);
}

void MCP3208_vInit(void)
{
	MCP3208_vInitPeriph();

	MCP3208_vEnableWork();
}

void MCP3208_vInitPeriph(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_init;

	TIM_TimeBaseInitTypeDef tstTimerInit;
	TIM_TimeBaseStructInit(&tstTimerInit);
//----------------------------- RCC --------------------------------------------
	MCP3208_DMA_RCC_AHBxPeriphClockCmd(MCP3208_RCC_AHBxPeriph_DMAx, ENABLE);
	MCP3208_SPI_PORT_PeriphClockCmd(MCP3208_SPI_PORT_Peripch, ENABLE);

#define	MCP3208_Chips_MakePort(NameADC, CS_GPIO_Port,	CS_GPIO_Pin, Vref) \
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_##CS_GPIO_Port, ENABLE);

		MCP3208_Chips_MakeConfig

#undef MCP3208_Chips_MakePort

	RCC_APB2PeriphClockCmd(	MCP3208_SPI_Peripch_GPIO_SCLK, ENABLE);
	RCC_APB2PeriphClockCmd(	MCP3208_SPI_Peripch_GPIO_MISO, ENABLE);
	RCC_APB2PeriphClockCmd(	MCP3208_SPI_Peripch_GPIO_MOSI, ENABLE);
//--------------------------- GPIO ---------------------------------------------
#define	MCP3208_Chips_MakePort(NameADC, CS_GPIO_Port,	CS_GPIO_Pin, Vref) \
\
		GPIO_InitStructure.GPIO_Pin = CS_GPIO_Pin;\
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;\
		GPIO_Init(CS_GPIO_Port, &GPIO_InitStructure);\
		MCP3208_stContext.pGPIOx_CS[MCP3208_en##NameADC] = CS_GPIO_Port;\
		MCP3208_stContext.u16GPIO_Pin_CS[MCP3208_en##NameADC] = CS_GPIO_Pin;\
		MCP3208_stContext.fRefVolt[MCP3208_en##NameADC] = Vref;\
		MCP3208_vWriteChipSelect(MCP3208_en##NameADC, Bit_SET);\

		MCP3208_Chips_MakeConfig

#undef MCP3208_Chips_MakePort

	GPIO_InitStructure.GPIO_Pin = MCP3208_SPI_PIN_SCLK;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(MCP3208_SPI_GPIO_PORT_SCLK, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MCP3208_SPI_PIN_MISO;
	GPIO_Init(MCP3208_SPI_GPIO_PORT_MISO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MCP3208_SPI_PIN_MOSI;
	GPIO_Init(MCP3208_SPI_GPIO_PORT_MOSI, &GPIO_InitStructure);
//--------------------- SPI ----------------------------------------------------
	SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;//SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA              = SPI_CPHA_1Edge;//SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
	//SPI_InitStructure.SPI_NSS               = SPI_NSS_Hard;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
	//SPI_InitStructure.SPI_CRCPolynomial 	= 7;

	SPI_Init(MCP3208_SPI_CORE, &SPI_InitStructure);
	//SPI_SSOutputCmd(MCP3208_SPI_CORE, ENABLE);
	SPI_Cmd(MCP3208_SPI_CORE, ENABLE);

//------------------------------- DMA ---------------------------------------
	DMA_init.DMA_PeripheralBaseAddr = (uint32_t) &(MCP3208_SPI_CORE->DR);
	DMA_init.DMA_MemoryBaseAddr = (uint32_t) MCP3208_stContext.MPC3208_RxBuffer;
	DMA_init.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_init.DMA_BufferSize = 24;
	DMA_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_init.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_init.DMA_Mode = DMA_Mode_Normal;
	DMA_init.DMA_Priority = DMA_Priority_High;
	DMA_init.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(MCP3208_DMA_CHANNEL_RX, &DMA_init);

	SPI_I2S_DMACmd(MCP3208_SPI_CORE, SPI_I2S_DMAReq_Rx, ENABLE);
//----------------------------- ENABLE -----------------------------------------
	NVIC_EnableIRQ(MCP3208_SPI_IRQn);
	NVIC_EnableIRQ(MCP3208_DMA_CHANNEL_IRQn_RX);

	SPI_I2S_ClearITPendingBit(MCP3208_SPI_CORE, SPI_I2S_IT_TXE);
	//SPI_I2S_ITConfig(MCP3208_SPI_CORE, SPI_I2S_IT_TXE, ENABLE);

	DMA_ClearITPendingBit(MCP3208_DMA_IT_TCIFx_RX);
	DMA_ITConfig(MCP3208_DMA_CHANNEL_RX, DMA_IT_TC, ENABLE);

	//DMA_Cmd(MCP3208_DMA_CHANNEL_RX, ENABLE);

	MCP3208_TIM_ClockBus(MCP3208_TIM_Clock_Name, ENABLE);

	tstTimerInit.TIM_CounterMode = TIM_CounterMode_Up;
	tstTimerInit.TIM_Prescaler = TIM_u32GetClockFrequencyTimer(MCP3208_TIM_Name)/1000000.0f;
	tstTimerInit.TIM_Period = MCP3208_END_TIMER_COUNTER;

	TIM_TimeBaseInit(MCP3208_TIM_Name, &tstTimerInit);

	NVIC_EnableIRQ(MCP3208_TIM_IRQn);
	TIM_ITConfig(MCP3208_TIM_Name, TIM_IT_Update, ENABLE);
	TIM_ClearITPendingBit(MCP3208_TIM_Name, TIM_IT_Update);

	TIM_Cmd(MCP3208_TIM_Name,ENABLE);
}

void MCP3208_TIM_IRQHandler(void)
{
	if (TIM_GetITStatus(MCP3208_TIM_Name, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(MCP3208_TIM_Name, TIM_IT_Update);
		//MCP3208_stContext.bFlagTimerUpdate = 1;
	}
}

void MCP3208_vReceiveData(uint8_t *pu8RxAddress, uint8_t u8Size)
{
	DMA_Cmd(MCP3208_DMA_CHANNEL_RX, DISABLE);
	MCP3208_DMA_CHANNEL_RX->CMAR = (uint32_t) pu8RxAddress;
	MCP3208_DMA_CHANNEL_RX->CNDTR = (uint16_t)u8Size;
	DMA_Cmd(MCP3208_DMA_CHANNEL_RX, ENABLE);
}

void MCP3208_DMA_CHANNEL_IRQHandler_RX(void)
{
	if(DMA_GetITStatus(MCP3208_DMA_IT_TCIFx_RX) == SET)
	{
		DMA_ClearITPendingBit(MCP3208_DMA_IT_TCIFx_RX);
		DMA_Cmd(MCP3208_DMA_CHANNEL_RX, DISABLE);
		MCP3208_stContext.bFlagRxEnd = 1;
	}
}

void MCP3208_vSendData(uint8_t *pu8TxAddress, uint8_t u8Size)
{
		MCP3208_stContext.u8TxIter = 0;
		MCP3208_stContext.pu8CurrentTxBuffer = pu8TxAddress;
		MCP3208_stContext.u8TxBytesNumber = u8Size;

		SPI_I2S_SendData(MCP3208_SPI_CORE, MCP3208_stContext.pu8CurrentTxBuffer[MCP3208_stContext.u8TxIter]);

		SPI_I2S_ITConfig(MCP3208_SPI_CORE, SPI_I2S_IT_TXE, ENABLE);
}

void MCP3208_SPI_Handler(void)
{
	if (SPI_I2S_GetITStatus(MCP3208_SPI_CORE, SPI_I2S_IT_TXE) != RESET)
	{
		SPI_I2S_ClearITPendingBit(MCP3208_SPI_CORE, SPI_I2S_IT_TXE);

		MCP3208_stContext.u8TxIter++;

		if (MCP3208_stContext.u8TxIter == MCP3208_stContext.u8TxBytesNumber)
		{
			SPI_I2S_ITConfig(MCP3208_SPI_CORE, SPI_I2S_IT_TXE, DISABLE);
			MCP3208_stContext.u8TxIter = 0;
			MCP3208_stContext.bFlagTxEnd = 1;
		}
		else
			SPI_I2S_SendData(MCP3208_SPI_CORE, MCP3208_stContext.pu8CurrentTxBuffer[MCP3208_stContext.u8TxIter]);
	}
}

void MCP3208_vWriteChipSelect(MCP3208_tenChipType enCurrentChip, BitAction BitAct)
{
	GPIO_WriteBit(MCP3208_stContext.pGPIOx_CS[enCurrentChip],MCP3208_stContext.u16GPIO_Pin_CS[enCurrentChip], BitAct);
}

void MCP3208_vEnableWork(void)
{
	MCP3208_stContext.bEnableWork = 1;
}

void MCP3208_vDisableWork(void)
{
	MCP3208_stContext.bEnableWork = 0;
}

void MCP3208_vMain(void)
{
	switch(MCP3208_stContext.tenCurrentState)
	{

	case MCP3208_enWait:
	{
		if(MCP3208_stContext.bEnableWork == 1)
		{
			MCP3208_stContext.tenCurrentState = MCP3208_enChipSelect;
		}
	} break;

	case MCP3208_enChipSelect:
	{
		MCP3208_vWriteChipSelect(MCP3208_stContext.tenCurrentChip, Bit_RESET);

		if(MCP3208_stContext.tenCurrentChip == 0)
			MCP3208_stContext.MCP3208_u32CurrentTimeMarker = MCP3208_TIM_Name->CNT;

		MCP3208_vReceiveData(&MCP3208_stContext.MPC3208_RxBuffer[MCP3208_stContext.tenCurrentChip][MCP3208_stContext.u8CurrentInput*3],  3);

		MCP3208_vChooseInput(MCP3208_stContext.u8CurrentInput);

		MCP3208_vSendData(MCP3208_stContext.MCP3208_TxBuffer, 3);

		MCP3208_stContext.tenCurrentState = MCP3208_enWaitTxRx;
	} break;

	case MCP3208_enWaitTxRx:
	{
		if(MCP3208_stContext.bFlagRxEnd == 1)
		{
			MCP3208_stContext.bFlagTxEnd = 0;
			MCP3208_stContext.bFlagRxEnd = 0;

			MCP3208_stContext.u8CurrentInput++;

			MCP3208_vWriteChipSelect(MCP3208_stContext.tenCurrentChip, Bit_SET);

			MCP3208_stContext.MCP3208_u16InputValue[MCP3208_stContext.tenCurrentChip][MCP3208_stContext.u8CurrentInput - 1] = 0x0FFF & ( (MCP3208_stContext.MPC3208_RxBuffer[MCP3208_stContext.tenCurrentChip][(MCP3208_stContext.u8CurrentInput - 1)*3 + 1] << 8) |
																							(MCP3208_stContext.MPC3208_RxBuffer[MCP3208_stContext.tenCurrentChip][(MCP3208_stContext.u8CurrentInput - 1)*3 + 2]) );

			if(MCP3208_stContext.u8CurrentInput == MCP3208_NUM_INPUT)
			{
				MCP3208_stContext.u8CurrentInput = 0;

				MCP3208_stContext.tenCurrentChip++;

				if(MCP3208_stContext.tenCurrentChip == MCP3208_enChipsNumber)
				{
					MCP3208_stContext.tenCurrentChip = 0;

					MCP3208_stContext.bFlagRxDataUpdate = 1;
				}

				MCP3208_stContext.tenCurrentState = MCP3208_enWait;
			}
			else
			{
				MCP3208_stContext.tenCurrentState = MCP3208_enChipSelect;
			}
		}
	} break;

	}
}

