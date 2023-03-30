/*
 * stm32f411xx.h
 *
 *  Created on: Jul 28, 2022
 *      Author: Juan Camilo Rivera Medina
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>
#include <stddef.h>

/*********************************************************************************************************************/
/*
 * Processor Specific details ARM Cortex Processor
 */
/*
 *
 * NVIC ISER registers
 */

#define NVIC_ISER0	(volatile uint32_t*)0xE000E100
#define NVIC_ISER1	(volatile uint32_t*)0xE000E104
#define NVIC_ISER2	(volatile uint32_t*)0xE000E108
#define NVIC_ISER3	(volatile uint32_t*)0xE000E10C

/*
 * NVIC ICER Registers
 */

#define NVIC_ICER0	(volatile uint32_t*)0xE000E180
#define NVIC_ICER1	(volatile uint32_t*)0xE000E184
#define NVIC_ICER2	(volatile uint32_t*)0xE000E188
#define NVIC_ICER3	(volatile uint32_t*)0xE000E18C

/*
 * NVIC IPER Registers
 */

#define NVIC_IPER0	(volatile uint32_t*)0xE000E400
#define NVIC_IPER1	(volatile uint32_t*)0xE000E404
#define NVIC_IPER2	(volatile uint32_t*)0xE000E408
#define NVIC_IPER3	(volatile uint32_t*)0xE000E40C

#define NO_PR_BITS_IMPLEMENTED	4 //Definition of the real number of priority bits implemented, this is a parameter specific to the microcontroller

/********************************************************************************/


#define FLASH_BASEADDRESS					0x08000000U
#define SRAM1_BASEADDRESS					0x20000000U //Embedded SRAM
#define ROM 								0x1FFF0000U
#define SRAM 								SRAM1_BASEADDRESS //Main SRAM

//bus domains
#define PERIPHERAL_BASE						0x40000000U // Base address of the APB1 bus
#define APB1_PERIPHERAL_BASE				PERIPHERAL_BASE //same as peripehral base for this micro
#define APB2_PERIPHERAL_BASE				0x40010000U //Base address of peripheral 2
#define AHB1_PERIPHERAL_BASE				0x40020000U //Base address AHB1 bus
#define AHB2_PERIPHERAL_BASE				0x50000000U //Base address AHB2 bus


/**Peripheral that depend from the AHB1 bus*/

#define GPIOA_OFFSET 						0x0000U
#define GPIOA_BASE_ADDRESS					(AHB1_PERIPHERAL_BASE + GPIOA_OFFSET) //Offset 0x0000 because is first register
#define GPIOB_BASE_ADDRESS					(AHB1_PERIPHERAL_BASE + 0x0400) 	//offset 0x0400
#define GPIOC_BASE_ADDRESS					(AHB1_PERIPHERAL_BASE + 0x0800)		//offset 0x0800
#define GPIOD_BASE_ADDRESS					(AHB1_PERIPHERAL_BASE + 0x0C00)		//offset 0x0C00
#define GPIOE_BASE_ADDRESS					(AHB1_PERIPHERAL_BASE + 0x1000)		//offset 0x1000
#define GPIOH_BASE_ADDRESS					(AHB1_PERIPHERAL_BASE + 0x1C00)		//offset 0x1C00
#define RCC_BASE_ADDRESS					(AHB1_PERIPHERAL_BASE + 0x3800) //RCC base address


/**Peripheral that depend from the APB1 bus*/

#define TIM2_BASE_ADDRESS					(APB1_PERIPHERAL_BASE + 0x0000)
#define TIM3_BASE_ADDRESS					(APB1_PERIPHERAL_BASE + 0x0400)
#define TIM4_BASE_ADDRESS					(APB1_PERIPHERAL_BASE + 0x0800)
#define TIM5_BASE_ADDRESS					(APB1_PERIPHERAL_BASE + 0x0C00)
#define I2SEXT_BASE_ADDRESS					(APB1_PERIPHERAL_BASE + 0x4000)
#define SPI2_I2S2_BASE_ADDRESS				(APB1_PERIPHERAL_BASE + 0x3800)
#define SPI3_I2S3_BASE_ADDRESS				(APB1_PERIPHERAL_BASE + 0x3C00)
#define USART2_BASE_ADDRESS					(APB1_PERIPHERAL_BASE + 0x4400)
#define I2C1_BASE_ADDRESS					(APB1_PERIPHERAL_BASE + 0x5400)
#define I2C2_BASE_ADDRESS					(APB1_PERIPHERAL_BASE + 0x5800)
#define I2C3_BASE_ADDRESS					(APB1_PERIPHERAL_BASE + 0x5C00)

/**Peripheral that depend from the APB2 bus*/

#define TIM1_BASE_ADDRESS					(APB2_PERIPHERAL_BASE + 0x0000)
#define USART1_BASE_ADDRESS					(APB2_PERIPHERAL_BASE + 0x1000)
#define USART6_BASE_ADDRESS					(APB2_PERIPHERAL_BASE + 0x1400)
#define ADC1_BASE_ADDRESS					(APB2_PERIPHERAL_BASE + 0x2000)

#define SPI1_I2S1_BASE_ADDRESS				(APB2_PERIPHERAL_BASE + 0x3000)
#define SPI4_I2S4_BASE_ADDRESS				(APB2_PERIPHERAL_BASE + 0x3400)
#define SPI5_I2S5_BASE_ADDRESS				(APB2_PERIPHERAL_BASE + 0x5000)

#define TIM9_BASE_ADDRESS					(APB2_PERIPHERAL_BASE + 0x4000)
#define TIM10_BASE_ADDRESS					(APB2_PERIPHERAL_BASE + 0x4400)
#define TIM11_BASE_ADDRESS					(APB2_PERIPHERAL_BASE + 0x4800)
#define EXTI_BASE_ADDRESS					(APB2_PERIPHERAL_BASE + 0x3C00)
#define SYSCFG_BASE_ADDRESS					(APB2_PERIPHERAL_BASE + 0x3800)





/***************************PERIPHERAL REGITER DEFINITION STRUCTURES*****************/
//the order of the register in the structure must be the same as the ones with the offset structure
typedef struct
{
	volatile uint32_t MODER; 	//GPIO port mode register
	volatile uint32_t OTYPER; 	//GPIO port output type register
	volatile uint32_t OSPEEDR; 	//GPIO port output speed register
	volatile uint32_t PUPDR; 	//GPIO port pull-up/pull-down register
	volatile uint32_t IDR; 		//GPIO port input data register
	volatile uint32_t ODR; 		//GPIO port output data register
	volatile uint32_t BSRR; 	//GPIO port bit set/reset register
	volatile uint32_t LCKR; 	//GPIO port configuration lock register
	volatile uint32_t AFR[2];	//AFRL and AFRH GPIO alternate function register
}GPIO_RegDef_t;

//Exti peripheral structure
typedef struct
{
	volatile uint32_t IMR; 		//Address offset 0x00
	volatile uint32_t EMR;		//Address offset 0x04
	volatile uint32_t RTSR;		//Address offset 0x08
	volatile uint32_t FTSR;		//Address offset 0x0C
	volatile uint32_t SWIER;	//Address offset 0x10
	volatile uint32_t PR;		//Address offset 0x14
}EXTI_RegDef_t;

//SYCFG register structure
typedef struct
{
	volatile uint32_t MEMRMP;	//Offset 0x00 just available for first two bits
	volatile uint32_t PMC; 		//Offset 0x04, enable for bit 16
	volatile uint32_t EXTICR[4];	//Offset 0x08, enable for fir 15 bits
	volatile uint32_t CMPCR; 	//offset 0x20, enable for bits 0 and 6
}SYSCFG_RegDef_t;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	uint32_t RESERVED1[2];
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED2[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	uint32_t RESERVED3[2];
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED4[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	uint32_t RESERVED5[2];
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED6[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED7[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t DCKCFGR;
}RCC_RegDef_t;


/***************************************************
 *
 	 	 	 Structure for SPIx COnfiguration
 **************************************************/

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXRCR;
	volatile uint32_t TXRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;

//Peripheral definition
#define SPI1			((SPI_RegDef_t*)SPI1_I2S1_BASE_ADDRESS)
#define SPI2			((SPI_RegDef_t*)SPI2_I2S2_BASE_ADDRESS)
#define SPI3			((SPI_RegDef_t*)SPI3_I2S3_BASE_ADDRESS)
#define SPI4			((SPI_RegDef_t*)SPI4_I2S4_BASE_ADDRESS)

/***Clock enable macros for SPIx  peripherals***/

#define SPI1_PCLK_EN()	(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()	(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()	(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()	(RCC->APB2ENR |= (1 << 13))

/***Clock disable macros for SPIx  peripherals***/
#define SPI1_PCLK_DIS()	(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DIS()	(RCC->APB2ENR &= ~(1 << 13))

/**********************************************************************/


/**Peripherals definition**/

#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASE_ADDRESS)
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASE_ADDRESS)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASE_ADDRESS)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASE_ADDRESS)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASE_ADDRESS)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASE_ADDRESS)

#define RCC 		((RCC_RegDef_t*)RCC_BASE_ADDRESS)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASE_ADDRESS)
#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDRESS)

/***Clock enable macros for GPIOx peripherals***/

#define GPIOA_PCLK_EN()	( RCC->AHB1ENR |= (1 << 0)) //BIT IS MOVED THREE POSITIONS
#define GPIOB_PCLK_EN()	( RCC->AHB1ENR |= (1 << 1)) //BIT IS MOVED four POSITIONS
#define GPIOC_PCLK_EN()	( RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	( RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	( RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()	( RCC->AHB1ENR |= (1 << 7))

/***Clock disable macros for GPIOx peripherals***/

#define GPIOA_PCLK_DIS()	( RCC->APB2ENR &= ~(1 << 0)) //BIT IS MOVED THREE POSITIONS
#define GPIOB_PCLK_DIS()	( RCC->APB2ENR &= ~(1 << 1)) //BIT IS MOVED four POSITIONS
#define GPIOC_PCLK_DIS()	( RCC->APB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DIS()	( RCC->APB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DIS()	( RCC->APB2ENR &= ~(1 << 4))
#define GPIOH_PCLK_DIS()	( RCC->APB2ENR &= ~(1 << 7))

/***Reset macros for GPIOx peripherals***/

#define GPIOA_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 0)); ( RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 1)); ( RCC->AHB1RSTR &= ~(1 << 1));	}while(0)
#define GPIOC_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 2)); ( RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 3)); ( RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 4)); ( RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 7)); ( RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

#define GPIO_BASEADDRESS_TO_CODE(x)		((x == GPIOA) ? 0 :	(x == GPIOB) ? 1 : (x == GPIOC) ? 2 : (x == GPIOD) ? 3 : (x == GPIOE) ? 4 :(x == GPIOH) ? 7 :0)

/***Clock enable macros for USARTx  peripherals***/

#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))

/***Clock disable macros for USARTx  peripherals***/

#define USART2_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 14))

/***Clock enable macros for TIMERx  peripherals***/

#define TIMER2_PCLK_EN() (RCC->APB1ENR |= (1 << 0))
#define TIMER3_PCLK_EN() (RCC->APB1ENR |= (1 << 1))
#define TIMER4_PCLK_EN() (RCC->APB1ENR |= (1 << 2))
#define TIMER5_PCLK_EN() (RCC->APB1ENR |= (1 << 3))

/***Clock disable macros for TIMERx  peripherals***/

#define TIMER2_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 0))
#define TIMER3_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 1))
#define TIMER4_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 2))
#define TIMER5_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 3))

/***Clock enable macros for SYSCFG  peripherals***/

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/***Clock enable macros for SYSCFG  peripherals***/

#define SYSCFG_PCLK_DIS() (RCC->APB2ENR  &= ~(1 << 14))

/***************************************************
 *
 	 	 	 Structure for I2Cx COnfiguration
 **************************************************/

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
}I2C_RegDef_t;

/**** Peripheral Definition******************/

#define I2C1	((I2C_RegDef_t*)I2C1_BASE_ADDRESS)
#define I2C2	((I2C_RegDef_t*)I2C2_BASE_ADDRESS)
#define I2C3	((I2C_RegDef_t*)I2C3_BASE_ADDRESS)

/***Clock enable macros for i2c  peripherals***/
#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR |= (1 << 23))

/***Clock disable macros for i2c  peripherals***/
#define I2C1_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 23))

/**** Bit position definition ***********/
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWSRT		15

#define I2C_CR2_FREQ		0 //initial position, this is for 5 bits
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8 //goes from 8:15 positions

/*****************************************************
 * 			GENERAL MACROS
 *
 ****************************************************/

#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	ENABLE
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define	FLAG_SET		ENABLE

//Interrupts Request Numbers if stm32f411xx microcontroller
//This is coming from Reference Manual Vector Table
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define	IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

//IRQ number specific for SPI registers
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84
#define IRQ_NO_SPI5			85

//IRQ Number specific for I2C registers
#define IRQ_I2C1_EV			31
#define IRQ_I2C1_ER			32
#define IRQ_I2C2_EV			33
#define IRQ_I2C2_ER			34
#define IRQ_I2C3_EV			72
#define IRQ_I2C3_ER			73



//Macros for all possible Priority levels
#define NVIC_IRQ_PRIO15	15

#include "stm32f411xx_gpio.h"
#include "stm32f411xx_spi.h"
#include "stm32f411x_i2c.h"


#endif /* INC_STM32F411XX_H_ */
