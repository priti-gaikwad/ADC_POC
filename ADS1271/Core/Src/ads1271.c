/*
 * ads1271.c
 *
 *  Created on: Jun 7, 2024
 *      Author: priti
 */

#include "ads1271.h"
//extern int count;


extern SPI_HandleTypeDef hspi1;
void SPI_rxtx(uint8_t* Txdata,uint8_t* Rxdata,uint16_t Size)
{



//HAL_Delay(1);


	while(ADC_DRDY == 0) //wait for DRDY high --> until conversion completes
	{
		GPIOE->BSRR |= CS1_LOW;	//
		for(int i = 0 ; i<1000;i++);
	SPI1->CR1 &= ~SPI_CR1_SPE;				// spi1 disable
	SPI1->CR2 = Size;					// tsize
	SPI1->CR1 |= SPI_CR1_SPE;				// spi enable
	SPI1->CR1 |=SPI_CR1_CSTART;				// cstart

	for(int i = 0; i< Size;i++)
	{
		while((SPI1_TXF)== 0){};	// wait for txp =1
		SPI1->TXDR = *Txdata;
//		while((SPI1_RXF)==0){};	// wait for rxp = 1
//		*Rxdata++ = SPI1->RXDR;
		Txdata++;
	}


	SPI1->IFCR |= SPI_IFCR_TXTFC | SPI_IFCR_EOTC ;			// set EOTC
	SPI1->CR1 &= ~SPI_CR1_SPE;					// disable spi
	GPIOE->BSRR |= CS1_HIGH;
	for(int i = 0 ; i<1000;i++);
	}
//HAL_Delay(1);


}
