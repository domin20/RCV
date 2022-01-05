/*
 * supply.cpp
 *
 *  Created on: Nov 15, 2021
 *      Author: Dominik
 */
#include "main.h"
#include "supply.h"


extern uint16_t bufferADC[3];
extern uint16_t mainSupplyVoltage;
extern uint16_t dcEngineSupplyVoltage;


void CalculateSupplyVoltage()
{
	mainSupplyVoltage = bufferADC[0] * 185 / 4095;
	dcEngineSupplyVoltage = bufferADC[1] * 368 / 4095;
}

