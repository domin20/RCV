/*
 * sensor.h
 *
 *  Created on: Nov 15, 2021
 *      Author: Dominik
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_


#define ON 1
#define OFF 0


void TrigPinSensor(uint8_t onoff);
void CalculateTemperature(uint16_t data);


#endif /* INC_SENSOR_H_ */
