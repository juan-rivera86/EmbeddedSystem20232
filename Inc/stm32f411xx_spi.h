/*
 * stm32f411XX_spi.h
 *
 *  Created on: 13/09/2022
 *      Author: juanr
 */

#ifndef INC_STM32F411XX_SPI_H_
#define INC_STM32F411XX_SPI_H_

#include "stm32f411xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx; //This will be the base address of the SPI Peripheral
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer; 	//Stores Tx information to be send
	uint8_t *pRxBuffer;		//Stores the RX buffer address that is received.
	uint32_t TxLen;			//Length of the data to be send
	uint32_t RxLen;			//Length of the data received.
	uint8_t TxState;		// Variable that stores the Tx State
	uint8_t	RxState;		//Variable that stores the Rx state
}SPI_Handle_t;


#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

#define SPI_BUS_CONFIG_FD			1	//Full duplex
#define SPI_BUS_CONFIG_HD			2	//Half duplex
#define SPI_BUS_CONFIG_SI_TXONLY	3	//Simple TX
#define SPI_BUS_CONFIG_SI_RXONLY	4	//SImple Rx

#define SPI_CLK_SPEED_DIV2		0	//Fclk/2
#define SPI_CLK_SPEED_DIV4		1	//Fclk/4
#define SPI_CLK_SPEED_DIV8		2	//Fclk/8
#define SPI_CLK_SPEED_DIV16		3	//Fclk/16
#define SPI_CLK_SPEED_DIV32		4	//Fclk/32
#define SPI_CLK_SPEED_DIV64		5	//Fclk/64
#define SPI_CLK_SPEED_DIV128	6	//Fclk/128
#define SPI_CLK_SPEED_DIV256	7	//Fclk/256

#define SPI_DFF_8BITS			0 //Data format to 8 bits
#define SPI_DFF_16BITS			1 //Data format to 16 bits

#define SPI_CPOL_H				0 //Clock to zero when idle
#define SPI_CPOL_L				1 //Clock to 1 when idle

#define SPI_CPHA_H				0 //Clock to zero when idle
#define SPI_CPHA_L				1 //Clock to 1 when idle

#define SPI_SSM_DISABLE			0 //Disable
#define SPI_SSM_ENABLE			1 //enable

/************************************************************************************
 * 				Bits position definition for SPU Peripheral
 *
 ************************************************************************************/
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE 	15

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define	SPI_CR2_RXNEIE		6
#define	SPI_CR2_TXEIE		7

/******************************************************
 * 			SPI_SR register flags
 *
 ****************************************************/

#define SPI_RXNE_FLAG		(1 << 0)
#define SPI_TXE_FLAG		(1 << 1)
#define SPI_CHSIDE_FLAG		(1 << 2)
#define SPI_UDR_FLAG		(1 << 3)
#define SPI_CRCERROR_FLAG	(1 << 4)
#define SPI_MODF_FLAG		(1 << 5)
#define SPI_OVR_FLAG		(1 << 6)
#define SPI_BSY_FLAG		(1 << 7)
#define SPI_FRE_FLAG		(1 << 8)

/******************************************************
 * 			SPI Application States
 *
 ****************************************************/
#define SPI_READY				0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2

/******************************************************
 * 			SPI Application Events
 *
 ****************************************************/
#define SPI_EVENT_TX_COMPLETE 1
#define SPI_EVENT_RX_COMPLETE 2
#define SPI_EVENT_OVR_ERR 	  3

/*****************************************************
 * 			Enable or disable
 *
 *************************************************/
#define SPI_ENABLE	1
#define SPI_DISABLE	0

/************************************************************************************
 * 				API supported by this driver
 *
 ************************************************************************************/

//Peripherials clock init
// *pGPIOx pointer to Register Definition structure
//	EnableOrDisable, flag that enables or disables
void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx, uint8_t EnableOrDisable);


//Init and De-init

//Function to initialize the GPIO
//*pGPIOx pointer to the handlers structure. (To use it, create a pointer of type GPIO_Handle_t and use it as input of this function)
void SPI_Init(SPI_Handle_t *pSPIHandler);

//Function to a de-initialize the GPIO port. (Use the GPIO Reset/Set register to reset this port)
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//Data send and Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t len);

//Sends data but accomplish all steps related to the send data
//1. Enables peripheral
//2. Calls Send Data
//3. Checks if the SPI is busy or not.
//4. Disables the peripheral
//void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t len);

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t len);

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t len);

//Configures peripheral is open or not
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnableOrDiable);

//Configures the SSI
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnableOrDiable);

//COnfigures the SSIOE register
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnableOrDiable);

//Handles CLear of OVR Flag
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

//CLose Communication
void SPI_CloseTransmission(SPI_Handle_t *pHandle);
void SPI_CloseReception(SPI_Handle_t *pHandle);

//IRQ Configuration
//Handles interrupt configuration
//	IRQNumber Interruption number
//	EnableOrDisable flag that enables or disables the Interrupt
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnableOrDisable);

//Handles Interrupt Priority Configuration
//	IRQPriority IRQ priority
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

//Handles interrupt behavior
//	SPI Handle structure
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/************************************************************************************
 * 				Application Callback
 *
 ************************************************************************************/

void SPI_ApplicationCallBack(SPI_Handle_t *pHandle, uint8_t EventNumber);


#endif /* INC_STM32F411XX_SPI_H_ */
