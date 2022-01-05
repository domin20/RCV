/*
 * vehicle_lighting.h
 *
 *  Created on: Nov 14, 2021
 *      Author: Dominik
 */

#ifndef INC_VEHICLE_LIGHTING_H_
#define INC_VEHICLE_LIGHTING_H_

#define LIGHTS_ON ((lightRegister & 0x3F) == 0x3F)

void SoftStartOfLights();



#endif /* INC_VEHICLE_LIGHTING_H_ */
