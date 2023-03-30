/*
 * stm32f411x_i2c.h
 *
 *  Created on: 25/01/2023
 *      Author: juanr
 */

#ifndef INC_STM32F411X_I2C_H_
#define INC_STM32F411X_I2C_H_

#include "stm32f411xx.h"

/***Configuration Structure******/
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_AddressingMode;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;


/******** Handle Structure***********/
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxRxState;	/* !< To store Communication state > */
	uint8_t 		DevAddr;	/* !< To store slave/device address > */
	uint32_t        RxSize;		/* !< To store Rx size  > */
	uint8_t         Sr;			/* !< To store repeated start value  > */
}I2C_Handle_t;

/*
 * I2C Application States
 *
 * **/

#define I2C_READY		0
#define I2C_BUSY_IN_RX	1
#define I2C_BUSY_IN_TX	2


/*
 * I2C_SCLSpeed
 *
 */

#define I2C_SCL_SPEED_SM	100000 //NOrmal mode 100kHz
#define I2C_SCL_SPEED_FM4K	400000 //Fast mode 400kHz
#define I2C_SCL_SPEED_FM2K	200000 //Fast mode 400kHz


/**
 * 	I2c_AckControl
 */

#define I2C_ACK_ENABLE	1 //Acknowledge returned
#define I2C_ACK_DISABLE	0 //no acknowledge returned

/****
 *  i2c for sr
 */

#define I2C_DISABLE_SR	0
#define I2C_ENABLE_SR	1

/****
 *  I2C Events
 *
 */

#define I2C_EVENT_TX_COMPLETE	0
#define I2C_EVENT_RX_COMPLETE	1
#define I2C_EVENT_STOP			2


/**
 * 	I2c_FMDutyCycle
 */

#define I2C_FM_DUTY_CYCLE_2		0
#define I2C_FM_DUTY_CYCLE_16_9	1

/*
 * 	Flags
 *
 */

#define I2C_SR1_SB_FLAG 		0
#define I2C_SR1_ADDR_FLAG 		1
#define I2C_SR1_BTF_FLAG 		2
#define I2C_SR1_ADD10_FLAG 		3
#define I2C_SR1_STOP_FLAG 		4
#define I2C_SR1_RXNE_FLAG 		6
#define I2C_SR1_TXE_FLAG 		7
#define I2C_SR1_BEER_FLAG 		8
#define I2C_SR1_ARLO_FLAG 		9
#define I2C_SR1_AF_FLAG 		10
#define I2C_SR1_OVR_FLAG 		11
#define I2C_SR1_PECERR_FLAG 	12
#define I2C_SR1_TIMEOUT_FLAG 	14
#define I2C_SR1_SMBALERT_FLAG 	15

/***
 * 	Error flags
 *
 */

#define I2C_ERROR_BERR  		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7



/************************************************************************************
 * 				API supported by this driver
 *
 ************************************************************************************/

uint32_t RCC_GetPCLK1Value(void);

//Peripherials clock init
// *pGPIOx pointer to Register Definition structure
//	EnableOrDisable, flag that enables or disables
void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnableOrDisable);


//Init and De-init

//Function to initialize the GPIO
//*pGPIOx pointer to the handlers structure. (To use it, create a pointer of type GPIO_Handle_t and use it as input of this function)
void I2C_Init(I2C_Handle_t *pI2CHandler);

//Function to a de-initialize the GPIO port. (Use the GPIO Reset/Set register to reset this port)
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * 	Data send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandler, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandler, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t sr);

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnableDisable);


/*
 * 	Data send and Receive using interrupts
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandler, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t sr);

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandler, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t sr);

void I2C_CloseSendData(I2C_Handle_t *pI2CHandler);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandler);

/*
 * Other Peripherals Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnableOrDiable);

/*
  * IRQ Configuration
 */

//Handles interrupt configuration
//	IRQNumber Interruption number
//	EnableOrDisable flag that enables or disables the Interrupt
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnableOrDisable);

//Handles Interrupt Priority Configuration
//	IRQPriority IRQ priority
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

void I2C_Event_IRQHandling(I2C_Handle_t *pI2CHandler);
void I2C_Error_IRQHandling(I2C_Handle_t *pI2CHandler);

//Interrutps Enable or disable
void I2C_InterruptsStatud(uint8_t EnableOrDisable);


//Init GPIO pins
//PB6 -> I2C1_SCL
//PB7 -> I2C1_SDA
void I2C_GpioInit(uint32_t gpioBaseAddress, uint8_t scl, uint8_t sda);

/************************************************************************************
 * 				Application Callback
 *
 ************************************************************************************/

void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle, uint8_t EventNumber);




#endif /* INC_STM32F411X_I2C_H_ */
