/*
 * enc28j60.c
 *
 *  Created on: Jan 8, 2022
 *      Author: khorton
 */
#include "enc28j60.h"

extern SPI_HandleTypeDef hspi1;

static uint8_t Enc28_Bank;


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

uint8_t ENC28_readReg16(uint16_t addr)
{
	uint16_t data16;
	data16 = ENC28_readReg8(addr);
	data16 += ENC28_readReg8(addr+1) << 8;
	return data16;
}

void ENC28_writeReg16(uint16_t addr, uint16_t data)
{
	ENC28_writeReg8(addr, data&0xFF);
	ENC28_writeReg8(addr+1, (data >> 8)&0xFF);
}

void ENC28_setBank(uint8_t addr)
{
	if ((addr & BANK_MASK) != Enc28_Bank)
	{
		ENC28_writeOp(ENC28_BIT_FIELD_CLR, ECON1, ECON2_BSEL1|ECON1_BSEL0);
		ENC28_writeOp(ENC28_BIT_FIELD_SET, ECON1, Enc28_Bank >> 5);
		Enc28_Bank = addr & BANK_MASK;
	}
}

void ENC28_Init()
{
	//1: Disable chip select pin
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
	//2. Perform soft reset to the ENC28J60 module
	ENC28_writeOp(ENC28_SOFT_RESET, 0x1F, 0x00);
	HAL_Delay(2); //Must wait for 2 milliseconds
	//3. Wait until clock is ready
	while(!ENC28_readOp(ENC28_READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY);
	//4. Initialize RX buffer size. Page 20 of datasheet
	ENC28_writeReg16(ERXST, RXSTART_INIT);
	ENC28_writeReg16(ERXND, RXSTOP_INIT);
	//5. Initialize TX buffer size. Page 20 of datasheet
	ENC28_writeReg16(ETXST, TXSTART_INIT);
	ENC28_writeReg16(ETXND, TXSTOP_INIT);
	//6. Set RX Read and RX write pointers to zero position. When package receieved from router, write to this location
	ENC28_writeReg16(ERXRDPT, RXSTART_INIT);
	ENC28_writeReg16(ERXWRPT, RXSTART_INIT);
	//7. Receiver buffer filters. Unicast
	ENC28_writeReg16(ERXFCON, ERXFCON_UCEN|ERXFCON_ANDOR|ERXFCON_CRCEN);
	//8. MAC Control Register 1. Page 36 datasheet
	ENC28_writeReg8(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS|MACON1_PASSALL);
	//9. MAC Control Register 3
	ENC28_writeOp(ENC28_BIT_FIELD_SET, MACON3, MACON3_PADFG0|MACON3_TXCREN|MACON3_FRMLNEN);
	//10.NON/Back to back gap
	ENC28_writeReg16(MAIPG, 0x0C12);
	//11/ Back to back gap
	ENC28_writeReg8(MABBIPG, 0x12);
	//12. Maximum frame length. Any packets larger than this will be discarded
	ENC28_writeReg16(MAMXFL, MAX_FRAMELEN);
	//13. Set the MAC address of the device. This is defined in software
	ENC28_writeReg8(MAADR6, MAC_6);
	ENC28_writeReg8(MAADR5, MAC_5);
	ENC28_writeReg8(MAADR4, MAC_4);
	ENC28_writeReg8(MAADR3, MAC_3);
	ENC28_writeReg8(MAADR2, MAC_2);
	ENC28_writeReg8(MAADR1, MAC_1);

	//***************Advanced Initializations***********************//


}
