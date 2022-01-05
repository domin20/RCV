/*
 * data_handling.cpp
 *
 *  Created on: 13 lis 2021
 *      Author: Dominik
 */
#include "main.h"
#include "data_handling.h"
#include "steering.h"
#include "vehicle_lighting.h"
#include "soft_spi.h"
#include "radio.h"

extern HC12* radio;
extern volatile uint8_t radioFlag;

extern uint8_t drivingMode;
extern uint8_t drivingDirection;
extern uint8_t speedJoystickDeflexion;

extern uint8_t steeringDirection;
extern uint8_t steeringJoystickDeflexion;

extern int8_t temperature;
extern uint8_t lightRegister;
extern volatile uint8_t softStartLightsFlag;
extern volatile uint8_t softStartLightsCnt;
extern SPIDevice* spiLightRegister;

extern uint8_t cruiseControlFlag;
extern float speedForCruiseControl;
extern uint8_t AVBSActivated;

extern uint16_t speed;
extern uint16_t rpm;

extern uint8_t mainSupplyVoltage;
extern uint16_t dcEngineSupplyVoltage;

extern uint8_t vehicleToPilotDataBuffer[8];

extern uint8_t serviceModeFlag;

void ExecuteCommand(uint8_t* data)
{
	switch(data[0])
	{
		case DATA_OF_STEERING:
			drivingDirection = data[2] >> 7;
			speedJoystickDeflexion = data[2] & 0x7F;
			steeringDirection = data[3] >> 7;
			steeringJoystickDeflexion = data[3] & 0x7F;
			break;

		case DATA_OF_LIGHT_REGISTER:
			if(!LIGHTS_ON){
				lightRegister = data[2];
				if(LIGHTS_ON) {
					softStartLightsFlag = 1;
					softStartLightsCnt = 0;
				}
			}
			else lightRegister = data[2];
			spiLightRegister->SendSoftSpi(lightRegister);
			break;

		case DATA_OF_DRIVING_MODE:
			drivingMode = data[2];
			break;

		case SET_CRUISE_CONTROL:
			cruiseControlFlag = 1;
			speedForCruiseControl = data[2];
			break;

		case RESET_CRUISE_CONTROL:
			cruiseControlFlag = 0;
			break;

		case SET_SPEED_FOR_CRUISE_CONTROL:
			speedForCruiseControl = data[2];
			break;

		case AVBS_SYSTEM:
			if(AVBSActivated) AVBSActivated = 0;
			else AVBSActivated = 1;
			SendAVBSConfirmation(&AVBSActivated);
			break;

		case SERVICE_MODE:
			if(serviceModeFlag)
			{
				serviceModeFlag = 0;
				SERVICE_MODE_LED_OFF;
			}
			else
			{
				serviceModeFlag = 1;
				SERVICE_MODE_LED_ON;
			}
			break;

		case QUESTION_FOR_INITIAL_DATA:
			vehicleToPilotDataBuffer[0] = drivingMode;
			vehicleToPilotDataBuffer[1] = serviceModeFlag;
			vehicleToPilotDataBuffer[2] = AVBSActivated;
			vehicleToPilotDataBuffer[3] = lightRegister;
			radio->SendRadioFrameIT(vehicleToPilotDataBuffer, 4, ANSWER_CONTAINING_INITIAL_DATA);
			break;

		case ANSWER_CONTAINING_INITIAL_DATA:
			drivingMode = data[2];
			serviceModeFlag = data[3];
			if(serviceModeFlag) SERVICE_MODE_LED_ON;
			else SERVICE_MODE_LED_OFF;
			AVBSActivated = data[4];
			lightRegister = data[5];

			spiLightRegister->SendSoftSpi(lightRegister);
			break;
	}
	radioFlag = 0;
}

void SendAVBSConfirmation(uint8_t* AVBSState)
{
	for(uint8_t i = 0; i < 3; i++)
	{
		radio->SendRadioFrameIT(AVBSState, 1, AVBS_CONFIRMATION);
	}
}

void SendStandardDataToPilot()
{
	vehicleToPilotDataBuffer[0] = speed >> 8;
	vehicleToPilotDataBuffer[1] = speed;
	vehicleToPilotDataBuffer[2] = rpm >> 8;
	vehicleToPilotDataBuffer[3] = rpm;
	vehicleToPilotDataBuffer[4] = dcEngineSupplyVoltage >> 8;
	vehicleToPilotDataBuffer[5] = dcEngineSupplyVoltage;
	vehicleToPilotDataBuffer[6] = mainSupplyVoltage;
	vehicleToPilotDataBuffer[7] = drivingMode;
	vehicleToPilotDataBuffer[8] = temperature;

	radio->SendRadioFrameIT(vehicleToPilotDataBuffer, 9, STANDARD_FRAME_VEHICLE_TO_PILOT);
}
