/*
 * enc28j60.c
 *
 *  Created on: Jan 8, 2022
 *      Author: khorton
 */
#include "enc28j60.h"

extern SPI_HandleTypeDef hspi1;


uint8_t ENC28_readOp(uint8_t oper, uint8_t addr)
{
	uint8_t spiData[2];
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
	spiData[0] = (oper << 5) | addr;
	HAL_SPI_Transmit(&hspi1, spiData, 1, 100);
	HAL_SPI_Receive(&hspi1, &spiData[1], 1, 100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

	return spiData[1];
}

void ENC28_writeOp(uint8_t oper, uint8_t addr, uint8_t data)
{

}

uint8_t ENC28_readReg8(uint8_t oper, uint8_t addr)
{
	return 0;
}

void ENC28_writeReg8(uint8_t oper, uint8_t addr, uint8_t data)
{

}

uint8_t ENC28_readReg16(uint8_t oper, uint8_t addr)
{
	return 0;

}

void ENC28_writeReg16(uint8_t oper, uint8_t addr, uint8_t data)
{

}

void ENC28_setBank(uint8_t bankNo)
{

}
