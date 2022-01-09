/*
 * enc28j60.h
 *
 *  Created on: Jan 8, 2022
 *      Author: khorton
 */

#ifndef ENC28J60_H_
#define ENC28J60_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

uint8_t ENC28_readOp(uint8_t oper, uint8_t addr);
void ENC28_writeOp(uint8_t oper, uint8_t addr, uint8_t data);

uint8_t ENC28_readReg8(uint8_t oper, uint8_t addr);
void ENC28_writeReg8(uint8_t oper, uint8_t addr, uint8_t data);

uint8_t ENC28_readReg16(uint8_t oper, uint8_t addr);
void ENC28_writeReg16(uint8_t oper, uint8_t addr, uint8_t data);

void ENC28_setBank(uint8_t bankNo);

#endif /* ENC28J60_H_ */
