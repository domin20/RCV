/*
 * sensor.cpp
 *
 *  Created on: Nov 15, 2021
 *      Author: Dominik
 */
#include "main.h"
#include "sensor.h"
#include "soft_spi.h"


extern uint8_t lightRegister;
extern SPIDevice* spiLightRegister;

extern int8_t temperature;

void TrigPinSensor(uint8_t onoff)
{
	if(onoff == ON) lightRegister |= (1 << 7);
	else if(onoff == OFF) lightRegister &= ~(1<<7);

	spiLightRegister->SendSoftSpi(lightRegister);
}
void CalculateTemperature(uint16_t data)
{
	int16_t diff = 0;
	diff = data - 2048;
	temperature = 20 + diff / 10;	// x / 10 = (x / 20) * 2 -> specjalnie *2 by miec dokladnosc 0.5
}


