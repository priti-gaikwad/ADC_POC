/*
 * ads1271.h
 *
 *  Created on: Jun 7, 2024
 *      Author: priti
 */

#ifndef INC_ADS1271_H_
#define INC_ADS1271_H_
#include "main.h"

#define CS1_LOW 	(1 << 19)        	// port pin no.19 RESET
#define CS1_HIGH	(1 << 3)			// port pin no.3
#define SPI1_TXF	(SPI1->SR & (1<<1))
#define SPI1_RXF	(SPI1->SR & (1<<0))
#define ADC_DRDY	(GPIOE->IDR & (1<<2))

void SPI_rxtx(uint8_t* Txdata,uint8_t* Rxdata,uint16_t Size);

#endif /* INC_ADS1271_H_ */
