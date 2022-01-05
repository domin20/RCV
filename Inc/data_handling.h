/*
 * data_handling.h
 *
 *  Created on: 13 lis 2021
 *      Author: Dominik
 */

#ifndef INC_DATA_HANDLING_H_
#define INC_DATA_HANDLING_H_

#define STANDARD_FRAME_VEHICLE_TO_PILOT 0
#define DATA_OF_STEERING 1
#define DATA_OF_LIGHT_REGISTER 2
#define DATA_OF_DRIVING_MODE 3
#define SET_CRUISE_CONTROL 4
#define RESET_CRUISE_CONTROL 5
#define SET_SPEED_FOR_CRUISE_CONTROL 6
#define AVBS_SYSTEM 7
#define SERVICE_MODE 8

#define AVBS_CONFIRMATION 10
#define QUESTION_FOR_INITIAL_DATA 11
#define ANSWER_CONTAINING_INITIAL_DATA 12

#define SERVICE_MODE_LED_ON HAL_GPIO_WritePin(SM_LED_GPIO_Port, SM_LED_Pin, GPIO_PIN_SET)
#define SERVICE_MODE_LED_OFF HAL_GPIO_WritePin(SM_LED_GPIO_Port, SM_LED_Pin, GPIO_PIN_RESET)


void ExecuteCommand(uint8_t* data);
void SendAVBSConfirmation(uint8_t* AVBSState);
void SendStandardDataToPilot();


#endif /* INC_DATA_HANDLING_H_ */
