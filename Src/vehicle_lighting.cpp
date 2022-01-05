/*
 * vehicle_lighting.cpp
 *
 *  Created on: Nov 14, 2021
 *      Author: Dominik
 */
#include "main.h"
#include "vehicle_lighting.h"
#include "soft_spi.h"

extern uint8_t lightRegister;
extern uint8_t softStartLightsCnt;
extern uint8_t cntPwm;
extern SPIDevice* spiLightRegister;

void SoftStartOfLights()
{
	if(softStartLightsCnt >= cntPwm) lightRegister |= 0x3F;
	else lightRegister &= ~0x3F;

	spiLightRegister->SendSoftSpi(lightRegister);
}



