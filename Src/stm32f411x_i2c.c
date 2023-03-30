/*
 * stm32f411x_i2c.c
 *
 *  Created on: 25/01/2023
 *      Author: juanr
 */

#include <stm32f411xx_gpio.h>
#include <stm32f411x_i2c.h>

uint16_t AHB_PreScale[8] = {2, 4, 8, 16, 64, 128, 512};
uint16_t APB_PreScale[4] = {2, 4, 8, 16};

/*
 * 		PRIVATE FUNCTIONS
 *
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandler);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	uint16_t tempMask = 0;
	tempMask |= (1 << FlagName);

	if(pI2Cx->SR1 & tempMask)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress)
{
	//Shift slave address by 1 bit
	SlaveAddress = SlaveAddress << 1;
	SlaveAddress &= ~(1); //Clear the most right bit for r/w
	pI2Cx->DR = SlaveAddress;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress)
{
	//Shift slave address by 1 bit
	SlaveAddress = SlaveAddress << 1;
	SlaveAddress |= 1; //Clear the most right bit for r/w
	pI2Cx->DR = SlaveAddress;
}

static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandler)
{
	uint8_t dummy_read;
	//1. Check the device mode
	if(pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//Master mode
		if(pI2CHandler->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandler->RxSize == 1)
			{
				//disable Ack
				I2C_ManageAcking(pI2CHandler->pI2Cx, I2C_ACK_DISABLE);

				//Clear ADDR FLag
				dummy_read = pI2CHandler->pI2Cx->SR1;
				dummy_read = pI2CHandler->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else
		{
			//Clear ADDR FLag
			dummy_read = pI2CHandler->pI2Cx->SR1;
			dummy_read = pI2CHandler->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else
	{
		//salve mode
		//Clear ADDR FLag
		dummy_read = pI2CHandler->pI2Cx->SR1;
		dummy_read = pI2CHandler->pI2Cx->SR2;
		(void)dummy_read;
	}
}

/**************************************************************************/


void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnableOrDisable)
{
	if(EnableOrDisable == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DIS();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DIS();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DIS();
		}
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandler)
{
	uint32_t temp = 0;

	//enable the clock for i2c
	I2C_PeripheralClockControl(pI2CHandler->pI2Cx, ENABLE);

	//COnfigure the ACk control bit
	temp |= pI2CHandler->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandler->pI2Cx->CR1 = temp;

	//Sets the FREQ value
	temp = 0;
	temp = RCC_GetPCLK1Value() / 1000000; //divide the value by 1M to only get the main number
	pI2CHandler->pI2Cx->CR2 = (temp & 0x3F);

	//Select the slave address
	temp |= pI2CHandler->I2C_Config.I2C_DeviceAddress << 1;
	temp |= (1 << 14); //this should always be 1 by software

	//Select addressing mode
	if(pI2CHandler->I2C_Config.I2C_AddressingMode == 1)//Case when the address is 10 bits long otherwise is 7 bits
	{
		temp |= pI2CHandler->I2C_Config.I2C_AddressingMode << 15;
		pI2CHandler->pI2Cx->OAR1 = temp;
	}
	pI2CHandler->pI2Cx->OAR1 = temp;

	//CCR (Clock Control Register) calculations
	uint16_t ccr_value = 0;
	temp = 0;
	if(pI2CHandler->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Configure the standard mode
		pI2CHandler->pI2Cx->CCR |= (0 << 15);
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandler->I2C_Config.I2C_SCLSpeed); //F_sys_clock/(2*F_serial_clock)
		temp |= (ccr_value & 0xFFF);
	}
	else
	{
		//configure the fast mode
		temp |= (1 << 15); //mode
		temp |= (pI2CHandler->I2C_Config.I2C_FMDutyCycle << 14); //type of duty cycle to configure
		if(pI2CHandler->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_CYCLE_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandler->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandler->I2C_Config.I2C_SCLSpeed);
		}
		temp |= (ccr_value & 0xFFF);
		pI2CHandler->pI2Cx->CCR |= temp;
	}


	//TRISE configuration
	uint8_t trise;
	if(pI2CHandler->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Configure the standard mode
		trise = (RCC_GetPCLK1Value() /1000000U) + 1;
	}
	else
	{
		trise = (RCC_GetPCLK1Value() *300/1000000000U) + 1;
	}

	pI2CHandler->pI2Cx->TRISE = (trise & 0x3F);
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnableOrDiable)
{
	if(EnableOrDiable == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClock;

	uint8_t clksrc, temp, ahbp, apb1;

	clksrc = (RCC->CFGR >> 2) & 0x3; //Gets the values of the System Clock Configuration

	if(clksrc == 0)
	{
		SystemClock = 16000000;
	}else if(clksrc == 1)
	{
		SystemClock = 25000000;
	}//TODO configur the PLL configurations

	temp = (RCC->CFGR >> 4) & 0xF;

	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScale[temp-8];
	}

	temp = (RCC->CFGR >> 10) & 0x7;

	if(temp < 4)
	{
		apb1 = 1;
	}
	else
	{
		apb1 = AHB_PreScale[temp-4];
	}

	pclk1 = (SystemClock / ahbp) / apb1;

	return pclk1;
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandler, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t sr)
{
	pI2CHandler->TxRxState = I2C_BUSY_IN_TX;

	//1. Generate the Start Condition
	I2C_GenerateStartCondition(pI2CHandler->pI2Cx);

	//2. Confirm that start condition is completed by checking the SB flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_SB_FLAG));

	//3. Send the address of the slave with r/w bit set w(0) (total of 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandler->pI2Cx, SlaveAddress);

	//4, Confirm that address phase is completed by checking the ADDR flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_ADDR_FLAG));

	//5. Clear the ADDR flag according to its software sequence
	// NOTE: Until ADDR is cleared SCL will be stretched (pulled Low)
	I2C_ClearAddrFlag(pI2CHandler);

	//6. Send data until Len becomes 0
	while(Len > 0)
	{
		while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_TXE_FLAG));
		pI2CHandler->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. WHen Len becomes 0 wait for Txe=1 and BTF=1 before generating the Stop condition
	//Note: TxE = 1, BTF = 1, means that both SR and DR are empty and next transmission should begin
	// when BTF=1  SCL will be stretched (pulled Low)
	while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_TXE_FLAG));

	while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_BTF_FLAG));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition
	//NOTE: Generating stop condition, automatically clears the BTF.
	if(sr == I2C_DISABLE_SR)
	{
		I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandler, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t sr)
{
	pI2CHandler->TxRxState = I2C_BUSY_IN_RX;
	pI2CHandler->RxSize = Len;

	//Generate start condition
	I2C_GenerateStartCondition(pI2CHandler->pI2Cx);

	//Confirm that start generation is completed by checking SB flag in SR1
		//Note: Until SB is cleared SCL will be streched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_SB_FLAG));

	// Send the address of the slave with r/w bit set to R(1) (total of 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandler->pI2Cx, SlaveAddress);

	//wait until address phase is completed by checking the ADDR flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_ADDR_FLAG));

	if(Len == 1)
	{
		//Disable ACK
		I2C_ManageAcking(pI2CHandler->pI2Cx, I2C_ACK_DISABLE);

		//Clear the ADDR flag
		I2C_ClearAddrFlag(pI2CHandler);

		//wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_RXNE_FLAG));

		//Generate stop condition
		if(sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
		}

		//read data in buffer
		uint8_t test = pI2CHandler->pI2Cx->DR;
		*pRxBuffer = test;
	}

	//Procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//Clear the ADDR flag
		I2C_ClearAddrFlag(pI2CHandler);

		//read the data until Len becomes zero
		for(uint32_t i = Len; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_RXNE_FLAG));


			if(i == 2) //if last 2 bytes are remaining
			{
				//clear the ack bit
				I2C_ManageAcking(pI2CHandler->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				if(sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
				}
			}

			//read the data from the data register in to buffer
			*pRxBuffer = pI2CHandler->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}

	//re-enable ACKing
	if(pI2CHandler->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandler->pI2Cx,I2C_ACK_ENABLE);
	}
}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnableDisable)
{
	if(EnableDisable == I2C_ACK_ENABLE)
	{
		//Enable
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		//Disable
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

void I2C_GpioInit(uint32_t gpioBaseAddress, uint8_t scl, uint8_t sda)
{
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOBaseAddress = gpioBaseAddress;
	I2CPins.pGPIOConfiguration.GPIO_PinMode = GPIO_MODE_AFM;
	I2CPins.pGPIOConfiguration.GPIO_PinOPType = GPIO_OTYPER_OD;
	I2CPins.pGPIOConfiguration.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP;
	I2CPins.pGPIOConfiguration.GPIO_PinAltFunMode = 4;
	I2CPins.pGPIOConfiguration.GPIO_Speed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.pGPIOConfiguration.GPIO_PinNumber = scl;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.pGPIOConfiguration.GPIO_PinNumber = sda;
	GPIO_Init(&I2CPins);
}

/******************************************************************
 *
 * 			Handling communication with interrupts
 *
 *
 *****************************************************************/

//Handles interrupt configuration
//	IRQNumber Interruption number
//	EnableOrDisable flag that enables or disables the Interrupt
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnableOrDisable)
{
	uint32_t temp = 0;
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
			temp = (IRQNumber % 32);
			*NVIC_ISER1 |= (1 << temp);
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

//Handles Interrupt Priority Configuration
//	IRQPriority IRQ priority
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. First find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;
	uint8_t shift_amout = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);//this is because only the upper part of the 8 bits are used

	*(NVIC_IPER0 + iprx) |=  (IRQPriority << shift_amout);
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandler, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t sr)
{
	uint8_t busystate = pI2CHandler->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandler->pTxBuffer = pTxBuffer;
		pI2CHandler->TxLen = Len;
		pI2CHandler->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandler->DevAddr = SlaveAddress;
		pI2CHandler->Sr = sr;

		//Implement code to Generate START Condition
		//1. Generate the Start Condition
		I2C_GenerateStartCondition(pI2CHandler->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandler, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t sr)
{
	uint8_t busystate = pI2CHandler->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandler->pRxBuffer = pRxBuffer;
		pI2CHandler->RxLen = Len;
		pI2CHandler->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandler->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandler->DevAddr = SlaveAddress;
		pI2CHandler->Sr = sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandler->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandler)
{
	//Implement the code to disable ITBUFEN
	pI2CHandler->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN
	pI2CHandler->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandler->TxRxState = I2C_READY;
	pI2CHandler->pRxBuffer = NULL;
	pI2CHandler->RxLen = 0;
	pI2CHandler->RxSize = 0;
	if(pI2CHandler->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandler->pI2Cx, ENABLE);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandler)
{
	//Implement the code to disable ITBUFEN
	pI2CHandler->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN
	pI2CHandler->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandler->TxRxState = I2C_READY;
	pI2CHandler->pRxBuffer = NULL;
	pI2CHandler->TxLen = 0;
}

void I2C_Event_IRQHandling(I2C_Handle_t *pI2CHandler)//Common for master or slave mode of the device
{
	uint32_t temp1, temp2, temp3;
	//1. HAndle INterrupt generated by SB Event (this flag is only applicable in master mode)
	temp1 = pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = I2C_GetFlagStatus(pI2CHandler->pI2Cx,I2C_SR1_SB);
	if(temp1 && temp3)
	{
		//SB flag is set
		//Executes the Address phase
		if(pI2CHandler->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandler->pI2Cx, pI2CHandler->DevAddr);
		}
		else if(pI2CHandler->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandler->pI2Cx, pI2CHandler->DevAddr);
		}
	}

	//2. Handle for interrupt generated by ADDR event (When MM Address is sent, When Slave Address matched with own address)
	temp3 = I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_ADDR_FLAG);
	if(temp1 && temp3)
	{
		//ADDR flag is set
		I2C_ClearAddrFlag(pI2CHandler);
	}
	//3. Handle for interrupt generated by BTF (Byte transfer finished) Event
	temp3 = I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_BTF_FLAG);
	if(temp1 && temp3)
	{
		//BTF Event flag is set
		if(pI2CHandler->TxRxState == I2C_BUSY_IN_TX && pI2CHandler->TxLen == 0)
		{
			//Validate  that TXE is set
			if(I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_TXE_FLAG))
			{
				//BTF and TXE = 1
				 //1. gENERATE THE STOP CONDITION
				if(pI2CHandler->Sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
				}

				//2. RESET ALL THE MEMBER ELEMENTS OF HANDLE STRUCTURE
				I2C_CloseSendData(pI2CHandler);

				//3. trANSMISSION cOMPLETE
				I2C_ApplicationEventCallback(pI2CHandler, I2C_EVENT_TX_COMPLETE);
			}
		}
	}

	//4. Handle for interrupt genetsred byt STOPF event (Stop detection is only applicable for Slave mode)
	temp3 = I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_STOP_FLAG);
	if(temp1 && temp3)
	{
		//STOP FLAG Event flag is set
		//Clear Stop Flag
			//1. Read SR1
				//was done in the temp3 line definition
			//Write to CR1
		pI2CHandler->pI2Cx->CR1 |= 0x0000;

		//Notify the app that stop was generated by the master
		I2C_ApplicationEventCallback(pI2CHandler, I2C_EVENT_STOP);
	}

	//5. Handle for interrupt generated by TXE event (IS GENERATED WHEN ITEVTEN and ITBUFEN ARE ENABLE)
	temp3 = I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_TXE_FLAG);
	if(temp1 && temp2 && temp3 && (pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_MSL)))
	{
		//txe Event flag is set
		//Do the data transmission
		if(pI2CHandler->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2CHandler->TxLen > 0)
			{
				//Load data into DR
				pI2CHandler->pI2Cx->DR = *(pI2CHandler->pTxBuffer);
				// Decrement TxLen
				pI2CHandler->TxLen--;
				//Increment buffer address
				pI2CHandler->pTxBuffer++;
			}
		}
	}

	//6. Handle for interrupt generated by RXE event
	temp3 = I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_SR1_RXNE_FLAG);
	if(temp1 && temp2 && temp3 && (pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_MSL)))
	{
		//RXE Event flag is set
		//Do the data reception
		if(pI2CHandler->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandler->RxSize == 1)
			{
				*pI2CHandler->pRxBuffer = pI2CHandler->pI2Cx->DR;
				pI2CHandler->RxLen--;
			}

			if(pI2CHandler->RxSize > 1)
			{
				if(pI2CHandler->RxLen == 2)
				{
					I2C_ManageAcking(pI2CHandler->pI2Cx, I2C_ACK_DISABLE);
				}

				*pI2CHandler->pRxBuffer = pI2CHandler->pI2Cx->DR;
				pI2CHandler->pRxBuffer++;
				pI2CHandler->RxLen--;
			}

			if(pI2CHandler->RxLen == 0)
			{
				//Close the data reception
				//1. Generate the stop condition
				if(pI2CHandler->Sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
				}

				//2. Close the I2C Rx
				I2C_CloseReceiveData(pI2CHandler);

				//3. Notify the application
				I2C_ApplicationEventCallback(pI2CHandler, I2C_EVENT_RX_COMPLETE);
			}

		}
	}

}

void I2C_Error_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

	    //Know the status of  ITERREN control bit in the CR2
		temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
		if(temp1  && temp2 )
		{
			//This is Bus error

			//Implement the code to clear the buss error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
		}

	/***********************Check for arbitration lost error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
		if(temp1  && temp2)
		{
			//This is arbitration lost error

			//Implement the code to clear the arbitration lost error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
		}

	/***********************Check for ACK failure  error************************************/

		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
		if(temp1  && temp2)
		{
			//This is ACK failure error

		    //Implement the code to clear the ACK failure error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
		}

	/***********************Check for Overrun/underrun error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
		if(temp1  && temp2)
		{
			//This is Overrun/underrun

		    //Implement the code to clear the Overrun/underrun error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
		}

	/***********************Check for Time out error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
		if(temp1  && temp2)
		{
			//This is Time out error

		    //Implement the code to clear the Time out error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
		}
}

//Interrutps Enable or disable
void I2C_InterruptsStatud(uint8_t EnableOrDisable)
{
	//Enable interrupts for events and for errors
	I2C_IRQInterruptConfig(IRQ_I2C1_ER, EnableOrDisable);
	I2C_IRQInterruptConfig(IRQ_I2C1_EV, EnableOrDisable);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle, uint8_t EventNumber)
{
	if(EventNumber == I2C_EVENT_TX_COMPLETE)
	{
		//Tx is complete
	}else if(EventNumber == I2C_EVENT_RX_COMPLETE)
	{
		//Rx is complete
	}else if(EventNumber == I2C_ERROR_AF)
	{
		//iN MASTER ack FAILURE HAPPENS WHEN SLAVE FAILS TO SEND sck FOR THE BYTE SENT FROM MASTER
		I2C_CloseSendData(&pHandle);

		//Generate the stop condition
		I2C_GenerateStopCondition(pHandle->pI2Cx);
	}
}


