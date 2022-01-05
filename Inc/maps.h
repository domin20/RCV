/*
 * maps.h
 *
 *  Created on: 30 paź 2021
 *      Author: Dominik
 */

#ifndef INC_MAPS_H_
#define INC_MAPS_H_

#define SNOW_MODE 1
#define EKO_MODE 2
#define COMFORT_MODE 3
#define SPORT_MODE 4
#define DRIFT_MODE 5


uint16_t GetAVBSIdx(float speed, uint16_t distance);
uint16_t GetDrivingModeIdx(float speed, uint8_t joystick_value);
uint8_t GetSteeringIdx(float speed);
float GetValueFromDrivingModeMap(uint8_t rowIdx, uint8_t colIdx);
float GetValueFromAVBSMap(uint8_t rowIdx, uint8_t colIdx);
float GetValueFromSteeringMap(float speed);


#endif /* INC_MAPS_H_ */
