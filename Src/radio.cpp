/*
 * radio.c
 *
 *  Created on: 27 kwi 2021
 *      Author: Dominik
 */
#include "main.h"
#include "radio.h"



HC12::HC12(){
	uart = nullptr;
	bufferOfReceivedData = nullptr;
	byte = nullptr;
	radioFlag = nullptr;
}
HC12::HC12(UART_HandleTypeDef* uart){
	this->uart = uart;
	bufferOfReceivedData = nullptr;
	byte = nullptr;
	radioFlag = nullptr;
}
void HC12::HC12Init(UART_HandleTypeDef* uart, uint8_t* bufferOfReceivedData, uint8_t* byte, uint8_t* radioFlag)
{
	this->uart = uart;
	this->bufferOfReceivedData = bufferOfReceivedData;
	this->byte = byte;
	this->radioFlag = radioFlag;
}

void HC12::SendRadioFrameIT(uint8_t *data, uint8_t size, uint8_t command)
{
	static uint8_t frame[20];
	uint16_t checkBytes = 0;
	frame[0] = 255;
	frame[1] = command;
	frame[2] = size;
	for(uint8_t i = 3; i < size + 3; i++)
	{
		frame[i] = *data++;
		if(frame[i] == 255) frame[i] = 254;
		checkBytes += frame[i];
	}
	for(uint8_t i = 0; i < 3; i++) checkBytes += frame[i];

	if((checkBytes & 0xFF) == 255) checkBytes <<= 1;
	frame[size+3] = checkBytes>>8;
	frame[size+4] = checkBytes;

	HAL_UART_Transmit_IT(uart, frame, size + 5);
}

void HC12::GetRadioFrameIT()
{
	static uint8_t cnt=0;
	static uint8_t size, frame[20];
	uint16_t sum = 0, checkBytes = 0;

	if((*byte) == 255)
	{
		frame[0] = (*byte);
		cnt = 1;
		return;
	}
	if(cnt == 1)
	{
		frame[1] = (*byte);
		cnt = 2;
		return;
	}
	if(cnt == 2)
	{
		frame[2] = (*byte);
		size = (*byte);
		cnt = 3;
		return;
	}

	if((cnt >= 3) && (cnt < size + 5))
	{
		frame[cnt] = (*byte);
		cnt++;
		if(cnt == size + 5)
			{
				checkBytes = frame[cnt-2];
				checkBytes <<= 8;
				checkBytes |= frame[cnt-1];

				for(uint8_t i = 0; i < size + 3; i++)
				{
					sum += frame[i];
				}

				if((sum & 0xFF) == 255) sum <<= 1;
				if(sum == checkBytes)
				{
					cnt=0;
					for(uint8_t i = 1; i < size + 3; i++)
					{
						bufferOfReceivedData[i-1] = frame[i];
					}
					(*radioFlag) |= 0x01;
				}
				else {cnt = 0; (*radioFlag) &= ~ 0x01;}
			}
	}
}

