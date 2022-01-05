/*
 * soft_spi.cpp
 *
 *  Created on: Oct 25, 2021
 *      Author: Dominik
 */
#include "main.h"
#include "iostream"
#include "soft_spi.h"



#define SCK_0 HAL_GPIO_WritePin(SCK_GPIOx, sck_pin, GPIO_PIN_RESET)
#define SCK_1 HAL_GPIO_WritePin(SCK_GPIOx, sck_pin, GPIO_PIN_SET)
#define MOSI_0 HAL_GPIO_WritePin(MOSI_GPIOx, mosi_pin, GPIO_PIN_RESET)
#define MOSI_1 HAL_GPIO_WritePin(MOSI_GPIOx, mosi_pin, GPIO_PIN_SET)
#define CS_0 HAL_GPIO_WritePin(CS_GPIOx, cs_pin, GPIO_PIN_RESET)
#define CS_1 HAL_GPIO_WritePin(CS_GPIOx, cs_pin, GPIO_PIN_SET)


// **************************** class SoftSPI ********************************

SoftSPI::SoftSPI()
{
	this->MOSI_GPIOx = GPIOA;
	this->mosi_pin = GPIO_PIN_0;
	this->SCK_GPIOx = GPIOA;
	this->sck_pin = GPIO_PIN_1;
}
SoftSPI::SoftSPI(GPIO_TypeDef* MOSI_GPIOx, uint16_t mosi_pin, GPIO_TypeDef* SCK_GPIOx, uint16_t sck_pin)
{
	this->MOSI_GPIOx = MOSI_GPIOx;
	this->mosi_pin = mosi_pin;
	this->SCK_GPIOx = SCK_GPIOx;
	this->sck_pin = sck_pin;
}
void SoftSPI::SoftSPIInit(GPIO_TypeDef *MOSI_GPIOx, uint16_t mosi_pin, GPIO_TypeDef *SCK_GPIOx, uint16_t sck_pin)
{
	this->MOSI_GPIOx = MOSI_GPIOx;
	this->mosi_pin = mosi_pin;
	this->SCK_GPIOx = SCK_GPIOx;
	this->sck_pin = sck_pin;

	this->MOSI_GPIOx->MODER |= (0x01 << this->mosi_pin*2);
	this->SCK_GPIOx->MODER |= (0x01 << this->sck_pin*2);
}

//**************************** class SPIDevice *******************************

SPIDevice::SPIDevice() : SoftSPI()
{
	this->CS_GPIOx = GPIOA;
	this->cs_pin = GPIO_PIN_2;
}

void SPIDevice::DeviceSPIInit(GPIO_TypeDef *MOSI_GPIOx, uint16_t mosi_pin, GPIO_TypeDef *SCK_GPIOx, uint16_t sck_pin, GPIO_TypeDef *CS_GPIOx, uint16_t cs_pin)
{
	this->CS_GPIOx = CS_GPIOx;
	this->cs_pin = cs_pin;

	this->SoftSPIInit(MOSI_GPIOx, mosi_pin, SCK_GPIOx, sck_pin);

	this->CS_GPIOx->MODER |= (0x01 << this->cs_pin*2);
}

void SPIDevice::SendSoftSpi(uint8_t byte)
{
	uint8_t cnt = 0x80;

	SCK_0;

	while(cnt)
	{
		if(byte & cnt) MOSI_1;
		else MOSI_0;
		SCK_1;
		SCK_0;

		cnt >>= 1;
	}
	CS_1;
	CS_0;
}



