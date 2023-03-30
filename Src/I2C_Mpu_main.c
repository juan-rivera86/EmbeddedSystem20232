/*
 * I2C_Mpu_main.c
 *
 *  Created on: 14/02/2023
 *      Author: juanr
 */
#include <string.h>
#include <stdio.h>

#include "stm32f411xx.h"
#include "stm32f411xx_gpio.h"
#include "stm32f411xx_spi.h"
#include "stm32f411x_i2c.h"
#include "mpu9250_i2c.h"

#define TRUE 	1
#define FALSE	0

I2C_Handle_t I2C_MpuHandler;
MpuSensorData ImuData;

int main (void)
{
	//Init Mpu Connection
	I2C_MpuInit(&I2C_MpuHandler);

	//Configure mpu registers
	I2C_MpuConfig(&I2C_MpuHandler);

	//Enable interrupts for events and for errors
	//I2C_InterruptsStatud(ENABLE);

	while(TRUE)
	{
		I2C_MpuReadData(&I2C_MpuHandler, &ImuData);
		//CalculateRealAcceleration(&ImuData);
	}

}

void I2C1_EV_IRQHandler(void)
{
	I2C_Event_IRQHandling(&I2C_MpuHandler);
}
void I2C1_ER_IRQHandler()
{
	I2C_Error_IRQHandling(&I2C_MpuHandler);
}
