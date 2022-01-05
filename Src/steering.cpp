/*
 * steering.cpp
 *
 *  Created on: Nov 29, 2021
 *      Author: Dominik
 */
#include "main.h"
#include "soft_spi.h"
#include "steering.h"
#include "maps.h"
#include "math.h"


extern volatile uint16_t distance;

extern volatile float pwmValue[4];
extern volatile uint8_t cntPwm;
extern volatile uint8_t engineRegister;
extern SPIDevice* spiEngineRegister;

extern uint8_t drivingDirection;
extern uint8_t speedJoystickDeflexion;

extern uint8_t steeringDirection;
extern uint8_t steeringJoystickDeflexion;
extern float steeringValue;

extern uint8_t cruiseControlFlag;
extern float speedForCruiseControl;

extern uint8_t AVBSActivated;
extern float speed;

void HBridgeSteering()
{
	if(pwmValue[0] > cntPwm) engineRegister |= 0x02;
	else engineRegister &= 0x02;
	if(pwmValue[1] > cntPwm) engineRegister |= 0x04;
	else engineRegister &= 0x04;
	if(pwmValue[2] > cntPwm) engineRegister |= 0x08;
	else engineRegister &= 0x08;
	if(pwmValue[3] > cntPwm) engineRegister |= 0x10;
	else engineRegister &= 0x10;

	spiEngineRegister->SendSoftSpi(engineRegister);
}

void CalculateFinalValuesOfSteering()
{
	uint16_t rowIdx = GetDrivingModeIdx(speed, speedJoystickDeflexion);
	uint16_t colIdx = rowIdx & 0xFF;
	rowIdx >>= 8;

	// PART OF CODE RESPONSIBLE FOR STEERING FRONT WHEELS
	float steeringValueFromSteeringMap = GetValueFromSteeringMap(speed);

	steeringValue = steeringJoystickDeflexion * MAX_STEERING_VALUE / 100;
	steeringValue *= steeringValueFromSteeringMap;
	if(steeringDirection == 0) steeringValue = -steeringValue;

	// PART OF CODE RESPONSIBLE FOR CALCULATING VALUES FOR DC ENGINE
	float mapValue = GetValueFromDrivingModeMap(rowIdx, colIdx);
	if(drivingDirection)
	{
		pwmValue[3] = 100.0f * mapValue;
		pwmValue[1] = pwmValue[2] = 0.0f;
		pwmValue[0] = 100.0f;
	}
	else
	{
		pwmValue[0] = pwmValue[3] = 0.0f;
		pwmValue[2] = 100.0f * mapValue;
		pwmValue[1] = 100.0f;
	}

	// PART OF CODE RESPONSIBLE FOR CHANGE VALUE WITH AVBS
	if(AVBSActivated)
	{
		uint16_t rowIdx = GetAVBSIdx(speed, distance);
		uint16_t colIdx = rowIdx & 0xFF;
		rowIdx >>= 8;

		float avbsValue = GetValueFromAVBSMap(rowIdx, colIdx);

		if(drivingDirection) pwmValue[3] *= avbsValue;
		else pwmValue[2] *= avbsValue;
	}
}

