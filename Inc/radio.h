/*
 * radio.h
 *
 *  Created on: 27 kwi 2021
 *      Author: Dominik
 */

#ifndef RADIO_COMMUNICATION_RADIO_H_
#define RADIO_COMMUNICATION_RADIO_H_

class HC12 {
private:
	UART_HandleTypeDef* uart;
	uint8_t* bufferOfReceivedData;
	uint8_t* byte;
	uint8_t* radioFlag;
public:
	HC12();
	HC12(UART_HandleTypeDef*);

	void HC12Init(UART_HandleTypeDef* uart, uint8_t* bufferOfReceivedData, uint8_t* byte, uint8_t* radioFlag);
	void SendRadioFrameIT(uint8_t *data_bytes, uint8_t size, uint8_t command);
	void GetRadioFrameIT();

	~HC12(){};
};


#endif /* RADIO_COMMUNICATION_RADIO_H_ */
