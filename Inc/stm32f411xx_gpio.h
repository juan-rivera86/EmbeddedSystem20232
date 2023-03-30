/*
 * stm32f411xx_gpio.h
 *
 *  Created on: 27/08/2022
 *      Author: juanr
 *      This GPIO driver will handle the behavior and configuration of any GPIO
 *      for stm32f411xx.h
 */

#ifndef INC_STM32F411XX_GPIO_H_
#define INC_STM32F411XX_GPIO_H_

#include "stm32f411xx.h"

/***********************************************
 * 			GPIO PIN NUMBER
 *
 **********************************************/
#define GPIO_PIN_0	0
#define GPIO_PIN_1	1
#define GPIO_PIN_2	2
#define GPIO_PIN_3	3
#define GPIO_PIN_4	4
#define GPIO_PIN_5	5
#define GPIO_PIN_6	6
#define GPIO_PIN_7	7
#define GPIO_PIN_8	8
#define GPIO_PIN_9	9
#define GPIO_PIN_10	10
#define GPIO_PIN_11	11
#define GPIO_PIN_12	12
#define GPIO_PIN_13	13
#define GPIO_PIN_14	14
#define GPIO_PIN_15	15

/***********************************************
 * 			MACROS FOR PERIPHERAL MODE
 *
 **********************************************/

#define GPIO_MODE_INPUT 		0 //INPUT (Reset State)
#define GPIO_MODE_GP		 	1 //General purpose Output mode
#define GPIO_MODE_AFM 			2 //Alternate Function Mode
#define GPIO_MODE_AM 			3 //Analog mode

#define GPIO_MODE_IT_FT			4//INterrupt configuration for failing edge
#define GPIO_MODE_IT_RT			5//interrupt configuration for rising edge
#define GPIO_MODE_IT_RFT		6//interrupt for failing and rising edge

//When GPIO OUTPUT TYPE REGISTER
#define GPIO_OTYPER_PP 		0 //output push pull
#define GPIO_OTYPER_OD	 	1 //output open drain

//When GPIO Speed
#define GPIO_SPEED_LOW 			0
#define GPIO_SPEED_MEDIUM 		1
#define GPIO_SPEED_FAST 		2
#define GPIO_SPEED_HIGH 		3

//GPIO Pull up/ pull down register
#define GPIO_PUPDR_NO_PU_PD		0
#define GPIO_PUPDR_PULLUP		1
#define GPIO_PUPDR_PULLDOWN		2
#define GPIO_PUPDR_RESERVED		3



//GPIO pin configuration structure
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_Speed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;


//This structure is created to handle the behavior of any of the GPIO peripherals
typedef struct
{
	GPIO_RegDef_t *pGPIOBaseAddress;	//Pointer to hold the base address of the GPIO peripheral
	GPIO_PinConfig_t pGPIOConfiguration; //GPIO pin configuration settings
}GPIO_Handle_t;


/**************************************************************************
 *
 * 					API Supported by this driver
 *
 *************************************************************************/

//Handles peripheral clock
// *pGPIOx pointer to Register Definition structure
//	EnableOrDisable, flag that enables or disables
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnableOrDisable);

//Function to initialize the GPIO
//*pGPIOx pointer to the handlers structure. (To use it, create a pointer of type GPIO_Handle_t and use it as input of this function)
void GPIO_Init(GPIO_Handle_t *pGPIOHandler);

//Function to a de-initialize the GPIO port. (Use the GPIO Reset/Set register to reset this port)
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);



//Handles input pin reading
//	*pGPIOx pointer to GPIO structure
//	PinNumber set pin number
// uint8_t Return value can be 1 or 0
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//Handles input port reading
//	*pGPIOx pointer to GPIO structure
//	uint16_t Return value of 16 bits with information from the port
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

//Handles writing to output pin
//	*pGPIOx pointer to GPIO structure
//	PinNumber set pin number
//  valueArgument input parameter to be sent
void GPIO_WriteToOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t valueArgument);

//Handles Writing to output port
//	*pGPIOx pointer to GPIO structure
//  valueArgument input parameter to be sent
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t valueArgument);

//Handles toggling GPIO specific pin
//	*pGPIOx pointer to GPIO structure
//	PinNumber set pin number
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);




//Handles interrupt configuration
//	IRQNumber Interruption number
//	EnableOrDisable flag that enables or disables the Interrupt
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnableOrDisable);

//Handles interrupt behavior
//	PinNumber, only takes the pin number
void GPIO_IRQHandling(uint8_t PinNumber);

//Handles Interrupt Priority Configuration
//	IRQPriority IRQ priority
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);





#endif /* INC_STM32F411XX_GPIO_H_ */
