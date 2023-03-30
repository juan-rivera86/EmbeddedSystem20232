/*
 * mpu9250_i2c.c
 *
 *  Created on: 13/02/2023
 *      Author: juanr
 */
#include <string.h>

#include "stm32f411xx.h"
#include "stm32f411x_i2c.h"
#include "mpu9250_i2c.h"

static void I2C_ModuleConfiguration(uint32_t i2c, I2C_Handle_t *pI2CHandler);

static void I2C_MpuConfiguration(uint8_t Register,  uint8_t payload, I2C_Handle_t *pI2CHandler);

static void Delay(uint16_t delay);

static uint8_t WhoAmI();


static void I2C_ModuleConfiguration(uint32_t i2c, I2C_Handle_t *pI2CHandler)
{
	pI2CHandler->pI2Cx = I2C1;
	pI2CHandler->I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	pI2CHandler->I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_FM4K;
	pI2CHandler->I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_CYCLE_2;

	I2C_Init(pI2CHandler);
}

void I2C_MpuInit(I2C_Handle_t *pI2CHandler)
{
	//Initialize pins that will be used for I2C Communication
	I2C_GpioInit(GPIOB, GPIO_PIN_6, GPIO_PIN_7);

	//Initialize I2C
	I2C_ModuleConfiguration(I2C1, pI2CHandler);

	//Enable the peripheral
	I2C_PeripheralControl(pI2CHandler->pI2Cx, ENABLE);
}

static void I2C_MpuConfiguration(uint8_t Register,  uint8_t payload, I2C_Handle_t *pI2CHandler)
{
	uint8_t parameterToSend[2] = {Register, payload };
	I2C_Mpu9250WriteData(parameterToSend, 2, pI2CHandler);
	uint8_t parameterToReceive = 0;
	I2C_Mpu9250ReadData(&parameterToReceive, Register, 1, pI2CHandler);

	if(parameterToReceive != parameterToSend[1])
	{
		return;
	}
}

static void Delay(uint16_t delay)
{
	uint16_t counter=0;
	while(counter < delay)
	{
		counter++;
	}
}

static uint8_t WhoAmI(I2C_Handle_t *pI2CHandler){
	uint8_t parameterToReceive = 0;
	I2C_Mpu9250ReadData(&parameterToReceive, WHO_AM_I, 1, pI2CHandler);
	if(parameterToReceive == 0x71)
	{
		return 1;
	}
	return 0;

}

/****
 *
 * 	Api to configure MPU 9250 with Blocking I2C communication
 *
 */
void I2C_MpuConfig(I2C_Handle_t *pI2CHandler)
{
	//1. Write to power management register. Reset Mpu
	I2C_MpuConfiguration(PWR_MGMT_1, 0x00, pI2CHandler);
	Delay(100);

	//2. Clock source to be PLL
	I2C_MpuConfiguration(PWR_MGMT_1, 0x01, pI2CHandler);

	//3. Gyro bandwith 250Hs
	I2C_MpuConfiguration(CONFIG, 0x00, pI2CHandler); //gYRO BANDWITH 250Hz

	//4. Sample rate
	I2C_MpuConfiguration(SMPLRT_DIV, 0x00, pI2CHandler);

	//5. select accel scale
	I2C_MpuConfiguration(ACCEL_CONFIG, 0x00, pI2CHandler);

	//6. Select gyro scale & low pass filter to Fs=8kHz, Delay 0.97ms
	I2C_MpuConfiguration(GYRO_CONFIG, 0x03, pI2CHandler);

	//7. turn on low pass filter for accel
	I2C_MpuConfiguration(ACCEL_CONFIG2, 0x03, pI2CHandler);

	//8. Enables FIFO mode
	I2C_MpuConfiguration(FIFO_EN, 0x08, pI2CHandler);

	//9. Enable Bypass
	I2C_MpuConfiguration(INT_PIN_CFG, 0x02, pI2CHandler);

	//10. Enable interrupts
	I2C_MpuConfiguration(INT_ENABLE, 0x01, pI2CHandler);

	//Validate whoamI
	if(WhoAmI(pI2CHandler) == 0)
	{
		return;
	}

}

//Write data to MPU9250
void I2C_Mpu9250WriteData(uint8_t *pTxDataBuffer, uint8_t Len, I2C_Handle_t *pI2CHandler)
{
	I2C_MasterSendData(pI2CHandler, pTxDataBuffer, Len, MPU9250_SLAVE_ADDRESS_0, I2C_ENABLE_SR);
}

//Read data to MPU9250
void I2C_Mpu9250ReadData(uint8_t *pRxBuffer, uint8_t registerAddress, uint8_t Len, I2C_Handle_t *pI2CHandler)
{
	uint8_t *pTxDataBuffer = registerAddress;

	I2C_MasterSendData(pI2CHandler, pTxDataBuffer, Len, MPU9250_SLAVE_ADDRESS_0, I2C_ENABLE_SR);
	I2C_MasterReceiveData(pI2CHandler, pRxBuffer, 1, MPU9250_SLAVE_ADDRESS_0, I2C_ENABLE_SR);
}

void I2C_MpuReadData(I2C_Handle_t *pI2CHandler, MpuSensorData *pImuData)
{
	//Call accel_H
	uint8_t registerAddress = ACCEL_XOUT_H;
	uint8_t receivedData = 0;

	for(uint8_t memoryPosition = 0; memoryPosition < 6; memoryPosition++ )
	{
		I2C_Mpu9250ReadData(&receivedData, registerAddress, 1, pI2CHandler);
		pImuData->AccelRawData[memoryPosition] = receivedData;
		registerAddress++;
		receivedData = 0;
	}
}


void CalculateRealAcceleration(MpuSensorData *pImuData)
{
	int16_t temp = 0;
	temp = ((pImuData->AccelRawData[0] << 8) + pImuData->AccelRawData[1]);
	pImuData->AccelData[0] = temp*ACCEL_SENSITIVITY_G;

	temp = ((pImuData->AccelRawData[2] << 8) + pImuData->AccelRawData[3]);
	pImuData->AccelData[1] = temp*ACCEL_SENSITIVITY_G;

	temp = ((pImuData->AccelRawData[4] << 8) + pImuData->AccelRawData[5]);
	pImuData->AccelData[2] = temp*ACCEL_SENSITIVITY_G;
}



/****
 *
 * 	MPU 9250 with interrupts
 *
 */


void I2C_MpuConfigIT(I2C_Handle_t *pI2CHandler)
{
	//COnfig I2C Peripheral

	//1. Write to power management register
	uint8_t parameterToSend[2] = {PWR_MGMT_1, 0x01 };
	I2C_Mpu9250WriteDataIT(parameterToSend, 2, &pI2CHandler);
	uint8_t *parameterToReceive;
	I2C_Mpu9250ReadDataIT(parameterToReceive, PWR_MGMT_1, 1, &pI2CHandler);

	if(*parameterToReceive != parameterToSend[1])
	{
		return;
	}

	//2. select accel scale
	parameterToSend[0] = ACCEL_CONFIG;
	parameterToSend[1] = 0x00;
	I2C_Mpu9250WriteDataIT(parameterToSend, 2, &pI2CHandler);
	I2C_Mpu9250ReadDataIT(parameterToReceive, ACCEL_CONFIG,1, &pI2CHandler);
	if(*parameterToReceive != parameterToSend[1])
	{
		return;
	}

	//3. Select gyro scale & low pass filter to Fs=8kHz, Delay 0.97ms
	parameterToSend[0] = GYRO_CONFIG;
	parameterToSend[1] = 0x03;
	I2C_Mpu9250WriteDataIT(parameterToSend, 2, &pI2CHandler);
	I2C_Mpu9250ReadDataIT(parameterToReceive, GYRO_CONFIG,1, &pI2CHandler);

	if(*parameterToReceive != parameterToSend[1])
	{
		return;
	}

	//4. turn on low pass filter for accel
	parameterToSend[0] = ACCEL_CONFIG2;
	parameterToSend[1] = 0x10;
	I2C_Mpu9250WriteDataIT(parameterToSend, 2, &pI2CHandler);
	I2C_Mpu9250ReadDataIT(parameterToReceive, ACCEL_CONFIG2, 1, &pI2CHandler);

	if(*parameterToReceive != parameterToSend[1])
	{
		return;
	}

	//6. disable I2C master interface.Precondition to enable bypass multiplexer of the I2C master interface

	//7. enable I2C master interface bypass multiplexer

	//8. Setup the magnetometer:Fuse ROM access mode and 16 bit output

	//9. read the sensitivity adjustment values

	//10. reset the magnetometer to power down mode

	//11. enable chip to continuous mode 2(100Hz) and 16-bit output

}


//Write data to MPU9250
void I2C_Mpu9250WriteDataIT(uint8_t *pTxDataBuffer, uint8_t Len, I2C_Handle_t *pI2CHandler)
{
	while(I2C_MasterSendDataIT(&pI2CHandler, pTxDataBuffer, Len, MPU9250_SLAVE_ADDRESS_0, I2C_ENABLE_SR) != I2C_READY);
}

//Read data to MPU9250
void I2C_Mpu9250ReadDataIT(uint8_t *pRxBuffer, uint8_t registerAddress, uint8_t Len, I2C_Handle_t *pI2CHandler)
{
	uint8_t *pTxDataBuffer;
	*pTxDataBuffer = registerAddress;

	while(I2C_MasterSendDataIT(&pI2CHandler, pTxDataBuffer, Len, MPU9250_SLAVE_ADDRESS_0, I2C_ENABLE_SR) != I2C_READY);
	while(I2C_MasterReceiveDataIT(&pI2CHandler, pRxBuffer, 1, MPU9250_SLAVE_ADDRESS_0, I2C_DISABLE_SR) != I2C_READY);
}


/***
 *
 * 	Reads Mpu9250 using Interrupts
 *
 */

void I2C_MpuReadDataWithInterrupts(I2C_Handle_t *pI2CHandler,MpuSensorData *pImuData)
{
	//Call accel_H
	uint8_t registerAddress = 0x3B;

	for(uint8_t memoryPosition = 0; memoryPosition < 6; memoryPosition++ )
	{
		uint8_t *pReceivedData;
		I2C_Mpu9250ReadDataIT(pReceivedData, registerAddress, 1, &pI2CHandler);
		pImuData->AccelRawData[memoryPosition] = *pReceivedData;
		registerAddress++;
	}

}




