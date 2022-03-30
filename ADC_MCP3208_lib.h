/*
 * ADC_MCP3208_lib.h
 *
 *  Created on: 16 груд. 2020 р.
 *      Author: Denys.S.Yakosenko
 */
// Library for single mode.

#ifndef ADC_MCP3208_LIB_H_
#define ADC_MCP3208_LIB_H_

#include "stm32f10x.h"

#define MCP3208_NUM_INPUT	8
#define MCP3208_END_TIMER_COUNTER	65000
//----------------------------------DMA---------------------------------------------
#define MCP3208_DMA_RCC_AHBxPeriphClockCmd		RCC_AHBPeriphClockCmd
#define MCP3208_RCC_AHBxPeriph_DMAx				RCC_AHBPeriph_DMA1

//#define MCP3208_DMA_CHANNEL_TX				DMA1_Channel_3
//#define MCP3208_DMA_STREAM_IRQn_TX			DMA1_Channel3_IRQn
//#define MCP3208_DMA_STREAM_IRQHandler_TX		DMA1_Channel3_IRQHandler
//#define MCP3208_DMA_IT_TCIFx_TX 				DMA1_IT_TC3

#define MCP3208_DMA_CHANNEL_RX					DMA1_Channel2
#define MCP3208_DMA_CHANNEL_IRQn_RX				DMA1_Channel2_IRQn
#define MCP3208_DMA_CHANNEL_IRQHandler_RX		DMA1_Channel2_IRQHandler
#define MCP3208_DMA_IT_TCIFx_RX 				DMA1_IT_TC2

//----------------------------------SPI---------------------------------------------
#define MCP3208_SPI_CORE						SPI1
#define MCP3208_SPI_IRQn						SPI1_IRQn
#define MCP3208_SPI_Handler						SPI1_IRQHandler
#define MCP3208_SPI_PORT_PeriphClockCmd			RCC_APB2PeriphClockCmd
#define MCP3208_SPI_PORT_Peripch				RCC_APB2Periph_SPI1
#define MCP3208_SPI_CPOL						SPI_CPOL_High
#define MCP3208_SPI_CPHA						SPI_CPHA_2Edge
//-----------------------------------------------------------------------------------

#define	MCP3208_Chips_MakeConfig \
/*          		   			NameADC,				GPIO_Port,	GPIO_Pin,	Vref */ \
		MCP3208_Chips_MakePort(	AC_VoltMeasurement,		GPIOA,		GPIO_Pin_4,	4.096f) \
		MCP3208_Chips_MakePort(	J1772_Measurement,		GPIOC,		GPIO_Pin_4,	4.096f) \


#define	MCP3208_Chips_MakePort(NameADC, CS_GPIO_Port,	CS_GPIO_Pin, Vref) \

#undef MCP3208_Chips_MakePort

//----------------------------------SCLK---------------------------------------------
#define MCP3208_SPI_GPIO_PORT_SCLK				GPIOA
#define MCP3208_SPI_PIN_SCLK					GPIO_Pin_5
#define MCP3208_SPI_PIN_SOURCE_SCLK				GPIO_PinSource5
#define MCP3208_SPI_Peripch_GPIO_SCLK			RCC_APB2Periph_GPIOA
//----------------------------------MOSI---------------------------------------------
#define MCP3208_SPI_GPIO_PORT_MOSI				GPIOA
#define MCP3208_SPI_PIN_MOSI					GPIO_Pin_7
#define MCP3208_SPI_PIN_SOURCE_MOSI				GPIO_PinSource7
#define MCP3208_SPI_Peripch_GPIO_MOSI			RCC_APB2Periph_GPIOA
//----------------------------------MISO---------------------------------------------
#define MCP3208_SPI_GPIO_PORT_MISO				GPIOA
#define MCP3208_SPI_PIN_MISO					GPIO_Pin_6
#define MCP3208_SPI_PIN_SOURCE_MISO				GPIO_PinSource6
#define MCP3208_SPI_Peripch_GPIO_MISO			RCC_APB2Periph_GPIOA

#define MCP3208_TIM_Name		TIM2
#define MCP3208_TIM_ClockBus	RCC_APB1PeriphClockCmd
#define MCP3208_TIM_Clock_Name 	RCC_APB1Periph_TIM2
#define MCP3208_TIM_IRQn		TIM2_IRQn
#define MCP3208_TIM_IRQHandler	TIM2_IRQHandler

typedef enum
{
#define	MCP3208_Chips_MakePort(NameADC, CS_GPIO_Port, CS_GPIO_Pin, Vref) \
		MCP3208_en##NameADC,

	MCP3208_Chips_MakeConfig

	MCP3208_enChipsNumber
}MCP3208_tenChipType;

#undef MCP3208_Chips_MakePort

typedef enum
{
	MCP3208_enWait = 0,
	MCP3208_enChipSelect,
	MCP3208_enWaitTxRx
}MCP3208_tenStatesType;

typedef struct
{
	_Bool bEnableWork;
	_Bool bFlagTxEnd;
	_Bool bFlagRxEnd;
	_Bool bSendBusy;
	_Bool bFlagRxDataUpdate;

	uint8_t u8TxIter;

	uint8_t u8CurrentInput;
	uint8_t *pu8CurrentTxBuffer;
	uint8_t u8TxBytesNumber;

	MCP3208_tenStatesType tenCurrentState;
	MCP3208_tenChipType tenCurrentChip;

	GPIO_TypeDef* pGPIOx_CS[MCP3208_enChipsNumber];
	uint16_t u16GPIO_Pin_CS[MCP3208_enChipsNumber];
	float fRefVolt[MCP3208_enChipsNumber];

	uint8_t MPC3208_RxBuffer[MCP3208_enChipsNumber][MCP3208_NUM_INPUT*3];
	uint8_t MCP3208_TxBuffer[3];

	uint16_t MCP3208_u16InputValue[MCP3208_enChipsNumber][MCP3208_NUM_INPUT];
	uint32_t MCP3208_u32CurrentTimeMarker;
}MCP3208_tstContextType;

void MCP3208_vInit(void);
void MCP3208_vMain(void);
void MCP3208_vEnableWork(void);
void MCP3208_vDisableWork(void);
uint32_t MCP3208_u32GetADCInputData(MCP3208_tenChipType enChipNum, uint8_t u8NumInput);

extern MCP3208_tstContextType MCP3208_stContext;

#endif /* ADC_MCP3208_LIB_H_ */
