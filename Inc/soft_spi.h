/*
 * soft_spi.h
 *
 *  Created on: Oct 25, 2021
 *      Author: Dominik
 */

#ifndef INC_SOFT_SPI_H_
#define INC_SOFT_SPI_H_


class SoftSPI {
protected:
	GPIO_TypeDef *MOSI_GPIOx;
	GPIO_TypeDef *SCK_GPIOx;

	uint16_t mosi_pin;
	uint16_t sck_pin;
public:

	SoftSPI();
	SoftSPI(GPIO_TypeDef* MOSI_GPIOx, uint16_t mosi_pin, GPIO_TypeDef* SCK_GPIOx, uint16_t sck_pin);
	void SoftSPIInit(GPIO_TypeDef *MOSI_GPIOx, uint16_t mosi_pin, GPIO_TypeDef *SCK_GPIOx, uint16_t sck_pin);

	~SoftSPI(){};
};

class SPIDevice : public SoftSPI {
private:
	GPIO_TypeDef *CS_GPIOx;
	uint16_t cs_pin;

public:
	SPIDevice();
	void DeviceSPIInit(GPIO_TypeDef *MOSI_GPIOx, uint16_t mosi_pin, GPIO_TypeDef *SCK_GPIOx, uint16_t sck_pin, GPIO_TypeDef *CS_GPIOx, uint16_t cs_pin);
	void SendSoftSpi(uint8_t byte);
	~SPIDevice(){};
};

#endif /* INC_SOFT_SPI_H_ */
