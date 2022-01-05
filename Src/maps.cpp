/*
 * maps.cpp
 *
 *  Created on: 30 paź 2021
 *      Author: Dominik
 */
#include "main.h"
#include "maps.h"

extern uint8_t drivingMode;

float STEERING_map [6] = {

/* >30kmh */	0.40f,
/* >25kmh */	0.50f,
/* >20kmh */	0.60f,
/* >15kmh */	0.70f,
/* >10kmh */	0.80f,
/* >=0kmh */	1.00f
};


float AVBS_map [7][9] = {
		//[cm]   >300   >270   >210   >150   >100   >50    >20    >10    >= 0
/* >30 kmh*/	{1.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f},
/* >25 kmh*/	{1.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f},
/* >20 kmh*/	{1.00f, 0.00f, 0.10f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f},
/* >15 kmh*/	{1.00f, 0.20f, 0.10f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f},
/* >10 kmh*/	{1.00f, 0.70f, 0.50f, 0.40f, 0.30f, 0.20f, 0.00f, 0.00f, 0.00f},
/* >0 kmh*/		{1.00f, 1.00f, 1.00f, 0.80f, 0.50f, 0.30f, 0.10f, 0.00f, 0.00f},
/* = 0kmh*/		{1.00f, 1.00f, 0.70f, 0.50f, 0.30f, 0.20f, 0.10f, 0.05f, 0.00f}
};


float SNOW_MODE_map [6][10] = {
// 		pos [%]  > 90  	> 80   > 70   > 60   > 50   > 40   > 30   > 20   > 10   <= 10
/* >15kmh */	{0.65f, 0.65f, 0.55f, 0.45f, 0.35f, 0.30f, 0.25f, 0.20f, 0.20f, 0.00f},
/* >12kmh */	{0.65f, 0.60f, 0.50f, 0.40f, 0.30f, 0.25f, 0.25f, 0.20f, 0.15f, 0.00f},
/* >9kmh */		{0.55f, 0.45f, 0.40f, 0.35f, 0.25f, 0.25f, 0.25f, 0.15f, 0.15f, 0.00f},
/* >6kmh */		{0.45f, 0.40f, 0.35f, 0.35f, 0.25f, 0.25f, 0.20f, 0.15f, 0.10f, 0.00f},
/* >3kmh */		{0.35f, 0.30f, 0.30f, 0.30f, 0.25f, 0.20f, 0.20f, 0.15f, 0.10f, 0.00f},
/* >=0kmh*/		{0.25f, 0.25f, 0.25f, 0.20f, 0.20f, 0.20f, 0.20f, 0.15f, 0.10f, 0.00f}
};
float EKO_MODE_map [6][10] = {
// 		pos [%]  > 90  	> 80   > 70   > 60   > 50   > 40   > 30   > 20   > 10   <= 10
/* >20kmh */ 	{0.75f, 0.65f, 0.65f, 0.55f, 0.45f, 0.35f, 0.30f, 0.25f, 0.15f, 0.00f},
/* >15kmh */ 	{0.70f, 0.65f, 0.60f, 0.50f, 0.40f, 0.30f, 0.25f, 0.25f, 0.15f, 0.00f},
/* >10kmh */ 	{0.60f, 0.55f, 0.45f, 0.40f, 0.35f, 0.25f, 0.25f, 0.25f, 0.15f, 0.00f},
/* >7kmh */  	{0.50f, 0.45f, 0.40f, 0.35f, 0.35f, 0.25f, 0.25f, 0.20f, 0.10f, 0.00f},
/* >3kmh */  	{0.40f, 0.35f, 0.30f, 0.30f, 0.30f, 0.25f, 0.20f, 0.20f, 0.10f, 0.00f},
/* >=0kmh */  	{0.30f, 0.25f, 0.25f, 0.25f, 0.20f, 0.20f, 0.20f, 0.20f, 0.15f, 0.00f}
};

float COMFORT_MODE_map [6][10] = {
// 		pos [%]  > 90  	> 80   > 70   > 60   > 50   > 40   > 30   > 20   > 10   <= 10
/* >30kmh */ 	{1.00f, 0.90f, 0.80f, 0.75f, 0.65f, 0.55f, 0.45f, 0.40f, 0.30f, 0.00f},
/* >20kmh */ 	{1.00f, 0.80f, 0.70f, 0.65f, 0.60f, 0.50f, 0.40f, 0.35f, 0.25f, 0.00f},
/* >15kmh */	{0.80f, 0.60f, 0.60f, 0.55f, 0.50f, 0.40f, 0.35f, 0.30f, 0.25f, 0.00f},
/* >10kmh */	{0.60f, 0.50f, 0.50f, 0.45f, 0.40f, 0.35f, 0.25f, 0.25f, 0.25f, 0.00f},
/* >5kmh */		{0.50f, 0.50f, 0.45f, 0.40f, 0.35f, 0.30f, 0.25f, 0.20f, 0.20f, 0.00f},
/* >=0kmh */	{0.50f, 0.45f, 0.40f, 0.35f, 0.30f, 0.25f, 0.25f, 0.20f, 0.15f, 0.00f}
};

float SPORT_MODE_map [6][10] = {
// 		pos [%]  > 90   > 80   > 70   > 60   > 50   > 40   > 30   > 20   > 10   <= 10
/* >30kmh */	{1.00f, 1.00f, 1.00f, 0.90f, 0.80f, 0.70f, 0.60f, 0.50f, 0.40f, 0.00f},
/* >20kmh */	{1.00f, 1.00f, 0.90f, 0.80f, 0.70f, 0.60f, 0.60f, 0.50f, 0.40f, 0.00f},
/* >15kmh */	{1.00f, 1.00f, 0.80f, 0.70f, 0.60f, 0.60f, 0.50f, 0.50f, 0.40f, 0.00f},
/* >10kmh */	{1.00f, 0.90f, 0.70f, 0.60f, 0.60f, 0.50f, 0.50f, 0.40f, 0.40f, 0.00f},
/* >5kmh */		{1.00f, 0.90f, 0.70f, 0.60f, 0.50f, 0.50f, 0.40f, 0.40f, 0.30f, 0.00f},
/* >=0kmh */	{1.00f, 0.80f, 0.70f, 0.60f, 0.50f, 0.40f, 0.40f, 0.40f, 0.30f, 0.00f}
};

float DRIFT_MODE_map [1][10] = {
// 		pos [%]  > 90  	> 80   > 70   > 60   > 50   > 40   > 30   > 20   > 10   <= 10
				{1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 0.90f, 0.80f, 0.70f, 0.00f}
};

uint16_t GetAVBSIdx(float speed, uint16_t distance)
{
	uint16_t SpeedDistanceIdx = 0;
	if(speed > 30.0f) SpeedDistanceIdx |= (0x00 << 8);
	else if(speed > 25.0f) SpeedDistanceIdx |= (0x01 << 8);
	else if(speed > 20.0f) SpeedDistanceIdx |= (0x02 << 8);
	else if(speed > 15.0f) SpeedDistanceIdx |= (0x03 << 8);
	else if(speed > 10.0f) SpeedDistanceIdx |= (0x04 << 8);
	else if(speed >= 0.0f) SpeedDistanceIdx |= (0x05 << 8);

	if(distance > 300) SpeedDistanceIdx |= 0x00;
	else if(distance > 270) SpeedDistanceIdx |= 0x01;
	else if(distance > 210) SpeedDistanceIdx |= 0x02;
	else if(distance > 150) SpeedDistanceIdx |= 0x03;
	else if(distance > 100) SpeedDistanceIdx |= 0x04;
	else if(distance > 50) SpeedDistanceIdx |= 0x05;
	else if(distance > 20) SpeedDistanceIdx |= 0x06;
	else if(distance >= 0) SpeedDistanceIdx |= 0x07;

	return SpeedDistanceIdx;
}
uint16_t GetDrivingModeIdx(float speed, uint8_t joystick_value)
{
	uint16_t DrivingModeIdx = 0;
	if(drivingMode == SNOW_MODE)
	{
			if(speed > 15.0f) DrivingModeIdx |= (0x00 << 8);
			else if(speed > 12.0f) DrivingModeIdx |= (0x01 << 8);
			else if(speed > 9.0f) DrivingModeIdx |= (0x02 << 8);
			else if(speed > 6.0f)  DrivingModeIdx |= (0x03 << 8);
			else if(speed > 3.0f)  DrivingModeIdx |= (0x04 << 8);
			else if(speed >= 0.0f) DrivingModeIdx |= (0x05 << 8);
	}
	else if(drivingMode == EKO_MODE)
	{
			if(speed > 20.0f) DrivingModeIdx |= (0x00 << 8);
			else if(speed > 15.0f) DrivingModeIdx |= (0x01 << 8);
			else if(speed > 10.0f) DrivingModeIdx |= (0x02 << 8);
			else if(speed > 7.0f)  DrivingModeIdx |= (0x03 << 8);
			else if(speed > 3.0f)  DrivingModeIdx |= (0x04 << 8);
			else if(speed >= 0.0f) DrivingModeIdx |= (0x05 << 8);
	}
	else if((drivingMode == COMFORT_MODE) || (drivingMode == SPORT_MODE))
	{
			if(speed > 30.0f) DrivingModeIdx |= (0x00 << 8);
			else if(speed > 20.0f) DrivingModeIdx |= (0x01 << 8);
			else if(speed > 15.0f) DrivingModeIdx |= (0x02 << 8);
			else if(speed > 10.0f) DrivingModeIdx |= (0x03 << 8);
			else if(speed > 5.0f)  DrivingModeIdx |= (0x04 << 8);
			else if(speed >= 0.0f) DrivingModeIdx |= (0x05 << 8);
	}
	else if(drivingMode == DRIFT_MODE) DrivingModeIdx |= (0x00 << 8);

	if(joystick_value > 90) DrivingModeIdx |= 0x00;
	else if(joystick_value > 80) DrivingModeIdx |= 0x01;
	else if(joystick_value > 70) DrivingModeIdx |= 0x02;
	else if(joystick_value > 60) DrivingModeIdx |= 0x03;
	else if(joystick_value > 50) DrivingModeIdx |= 0x04;
	else if(joystick_value > 40) DrivingModeIdx |= 0x05;
	else if(joystick_value > 30) DrivingModeIdx |= 0x06;
	else if(joystick_value > 20) DrivingModeIdx |= 0x07;
	else if(joystick_value > 10) DrivingModeIdx |= 0x08;
	else if(joystick_value >= 0) DrivingModeIdx |= 0x09;

	return DrivingModeIdx;
}
float GetValueFromDrivingModeMap(uint8_t rowIdx, uint8_t colIdx)
{
	switch(drivingMode)
	{
	case SNOW_MODE:
		return SNOW_MODE_map[rowIdx][colIdx];
		break;

	case EKO_MODE:
		return EKO_MODE_map[rowIdx][colIdx];
		break;

	case COMFORT_MODE:
		return COMFORT_MODE_map[rowIdx][colIdx];
		break;

	case SPORT_MODE:
		return SPORT_MODE_map[rowIdx][colIdx];
		break;

	case DRIFT_MODE:
		return DRIFT_MODE_map[rowIdx][colIdx];
		break;

	default: return 0;
	}
}
float GetValueFromAVBSMap(uint8_t rowIdx, uint8_t colIdx)
{
	return AVBS_map[rowIdx][colIdx];
}
uint8_t GetSteeringIdx(float speed)
{
	if(speed > 30.0f) return 0x00;
	else if(speed > 25.0f) return 0x01;
	else if(speed > 20.0f) return 0x02;
	else if(speed > 15.0f) return 0x03;
	else if(speed > 10.0f) return 0x04;
	else if(speed >= 0.0f) return 0x05;

	else return 0;
}

float GetValueFromSteeringMap(float speed)
{
	uint8_t rowIdx = GetSteeringIdx(speed);
	return STEERING_map[rowIdx];
}
