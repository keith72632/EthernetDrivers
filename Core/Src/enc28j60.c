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
	uint8_t spiData[2];
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
	spiData[0] = (oper << 5) | addr;
	spiData[1] = data;
	HAL_SPI_Transmit(&hspi1, spiData, 2, 100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
}

uint8_t ENC28_readReg8(uint8_t addr)
{
	ENC28_setBank(addr);
	return ENC28_readOp(ENC28_READ_CTRL_REG, addr);
}

void ENC28_writeReg8(uint8_t addr, uint8_t data)
{
	ENC28_setBank(addr);
	ENC28_writeOp(ENC28_WRITE_CTRL_REG, addr, data);
}

uint8_t ENC28_readReg16(uint8_t addr)
{
	uint16_t data16;
	data16 = ENC28_readReg8(addr);
	data16 += ENC28_readReg8(addr+1) << 8;
	return data16;
}

void ENC28_writeReg16(uint8_t addr, uint8_t data)
{
	ENC28_writeReg8(addr, data&0xFF);
	ENC28_writeReg8(addr+1, (data >> 8)&0xFF);
}

void ENC28_setBank(uint8_t addr)
{
	uint8_t Enc28_Bank = 0;
	if ((addr & BANK_MASK) != Enc28_Bank)
	{
		ENC28_writeOp(ENC28_BIT_FIELD_CLR, ECON1, ECON2_BSEL1|ECON1_BSEL0);
		ENC28_writeOp(ENC28_BIT_FIELD_SET, ECON1, Enc28_Bank >> 5);
		Enc28_Bank = addr & BANK_MASK;
	}
}

void ENC28_Init()
{

}
