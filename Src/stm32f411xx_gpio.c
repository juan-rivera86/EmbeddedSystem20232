#include "stm32f411xx_gpio.h"


/***************************************************
	@GPIO_PeripheralClockControl
	This function will control the clock of the specific GPIO selected

	@GPIO_RegDef_t *pGPIOx pointer to Register Definition structure
	@IsEnable, flag that enables or disables

****************************************************/
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t IsEnable)
{
	if(IsEnable == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DIS();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DIS();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DIS();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DIS();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DIS();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DIS();
		}
	}
}


/***************************************************
	@GPIO INIT
	This function will help  Initialize an specific GPIO

	@GPIO_Handle_t *pGPIOHandler

	@Note
	To use it, create a pointer of type GPIO_Handle_t and use it as input of this function
****************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandler)
{
	//Enables clock
	GPIO_PeripheralClockControl(pGPIOHandler->pGPIOBaseAddress, ENABLE);

	uint32_t temporal = 0;
	uint8_t pinNumber =  pGPIOHandler->pGPIOConfiguration.GPIO_PinNumber;

	//0. Validate if it's mode configuration
	if(pGPIOHandler->pGPIOConfiguration.GPIO_PinMode <= GPIO_MODE_AM)
	{
		temporal = pGPIOHandler->pGPIOConfiguration.GPIO_PinMode << (2 * pGPIOHandler->pGPIOConfiguration.GPIO_PinNumber);
		pGPIOHandler->pGPIOBaseAddress->MODER &= ~(0x3 << pinNumber);//clearing
		//Configures the mode for the specific GPIO pin
		pGPIOHandler->pGPIOBaseAddress->MODER |= temporal;
		temporal = 0;
	}
	else
	{
		//Interrupt configuration
		if(pGPIOHandler->pGPIOConfiguration.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1.Configure the FTSR (Falling trigger selection register)
			EXTI->FTSR |= (1 << pinNumber);
			EXTI->RTSR &= ~(1 << pinNumber);
		}
		else if(pGPIOHandler->pGPIOConfiguration.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1.Configure the RTSR (Rising edge selection register)
			EXTI->RTSR |= (1 << pinNumber);
			EXTI->FTSR &= ~(1 << pinNumber);
		}
		else if(pGPIOHandler->pGPIOConfiguration.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure both registers RTSR (Rising edge selection register) and FTSR (Falling trigger selection register)
			EXTI->FTSR |= (1 << pinNumber);
			EXTI->RTSR |= (1 << pinNumber);
		}

		//2. Configure the GPIO port selection in the SYSCFG_EXTICR register
		uint8_t temp1 = pinNumber / 4;
		uint8_t temp2 = pinNumber % 4;
		uint8_t portcode = GPIO_BASEADDRESS_TO_CODE(pGPIOHandler->pGPIOBaseAddress);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << temp2 * 4;


		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pinNumber);
	}

	//1. Configure mode of the GPIO pin (input / output)

	//2. Configure speed
	temporal = pGPIOHandler->pGPIOConfiguration.GPIO_Speed << (2 * pinNumber ); //moves the configuration to the left side by the number of pins
	pGPIOHandler->pGPIOBaseAddress->OSPEEDR &= ~(0x3 << pinNumber);//clearing
	pGPIOHandler->pGPIOBaseAddress->OSPEEDR |= temporal;
	temporal = 0;

	//3. Configure Pull up and pull down settings
	temporal = pGPIOHandler->pGPIOConfiguration.GPIO_PinPuPdControl << (2 * pinNumber);
	pGPIOHandler->pGPIOBaseAddress->PUPDR &= ~(0x3 << pinNumber);//clearing
	pGPIOHandler->pGPIOBaseAddress->PUPDR |= temporal;
	temporal = 0;

	//4. Configure output type
	temporal = pGPIOHandler->pGPIOConfiguration.GPIO_PinOPType << (pinNumber);
	pGPIOHandler->pGPIOBaseAddress->OTYPER &= ~(0x1 << pinNumber);//clearing
	pGPIOHandler->pGPIOBaseAddress->OTYPER |= temporal;
	temporal = 0;

	//5. Configure alternate functionality if required
	if(pGPIOHandler->pGPIOConfiguration.GPIO_PinMode == GPIO_MODE_AFM)
	{
		uint8_t temporalPinNumber = pinNumber / 8;
		uint8_t temporalPinNumberMode = pinNumber % 8;
		//Configure alternate functionality
		pGPIOHandler->pGPIOBaseAddress->AFR[temporalPinNumber] &= ~(0xF << 4 * temporalPinNumber);//clearing
		pGPIOHandler->pGPIOBaseAddress->AFR[temporalPinNumber] |= pGPIOHandler->pGPIOConfiguration.GPIO_PinAltFunMode << (4 * temporalPinNumberMode);
	}

}

//Function to a de-initialize the GPIO port. (Use the GPIO Reset/Set register to reset this port)
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}



//Handles input pin reading
//	*pGPIOx pointer to GPIO structure
//	PinNumber set pin number
// uint8_t Return value can be 1 or 0
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;
}

//Handles input port reading
//	*pGPIOx pointer to GPIO structure
//	uint16_t Return value of 16 bits with information from the port
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (pGPIOx->IDR);
	return value;
}

//Handles writing to output pin
//	*pGPIOx pointer to GPIO structure
//	PinNumber set pin number
//  valueArgument input parameter to be sent
void GPIO_WriteToOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t valueArgument)
{
	if(valueArgument == GPIO_PIN_RESET)
	{
		pGPIOx->ODR |=  (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &=  (0 << PinNumber);
	}

}

//Handles Writing to output port
//	*pGPIOx pointer to GPIO structure
//  valueArgument input parameter to be sent
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t valueArgument)
{
	pGPIOx->ODR = valueArgument;
}

//Handles toggling GPIO specific pin
//	*pGPIOx pointer to GPIO structure
//	PinNumber set pin number
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber); //Xor operation
}




//Handles interrupt configuration
//	IRQNumber Interruption number
//	EnableOrDisable flag that enables or disables the Interrupt
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnableOrDisable )
{
	if(EnableOrDisable == ENABLE)
	{	//Enables the interrupt
		if(IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		//Disables the interrupt
		if(IRQNumber <= 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

//Priority configuration
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. First find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;
	uint8_t shift_amout = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);//this is because only the upper part of the 8 bits are used

	*(NVIC_IPER0 + iprx) |=  (IRQPriority << shift_amout);
}

//Handles interrupt behavior
//	PinNumber, only takes the pin number
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}

