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

//Operations defines
#define ENC28_READ_CTRL_REG  0x00
#define ENC28_WRITE_CTRL_REG 0x02
#define ENC28_BIT_FIELD_CLR  0x05
#define ENC28_BIT_FIELD_SET  0x04

//Masks and some constants
#define ADDR_MASK           0x1F
#define BANK_MASK           0x60  //0110 0000

//Bank0 - control registers and addresses
#define ERDPT               0x00
#define EWRPT               0x02
#define ETXST               0x04
#define ETXND               0x06
#define ERXST               0x08
#define ERXND               0x0A
#define ERXRDPT             0x0C
#define ERXWRPT             0x0E

//Bank1 - control registers and addresses
#define ERXFCON             0x18|0x20
#define EPMMO               0x08|0x20
#define EPMCS               0x10|0x20
#define EPKTCNT             0x19|0x20

//Bank2 - control registers and addresses
#define MACON1              0x00|0x40
#define MACON2              0x01|0x40
#define MACON3              0x02|0x40
#define MAIPG               0x06|0x40
#define MABBIPG             0x04|0x40
#define MAMXFL              0x0A|0x40
#define MIREGADR            0x14|0x40
#define MIWR                0x16|0x40
#define MICMD               0x12|0x40
#define MIRD                0x18|0x40

//Bank3 - registers and addresses
#define MAADR1              0x04|0x60
#define MAADR2              0x05|0x60
#define MAADR3              0x02|0x60
#define MAADR4              0x03|0x60
#define MAADR5              0x00|0x60
#define MAADR6              0x01|0x60
#define MISTAT              0x0A|0x60
#define EREVID              0x12|0x60

//Common Registers
#define EIE                 0x1B
#define EIR                 0x1C
#define ESTAT               0x1D
#define ECON1               0x1F
#define ECON2               0x1E

//Bitfeild Defines
#define ECON1_BSEL0         0x01
#define ECON2_BSEL1         0x02


uint8_t ENC28_readOp(uint8_t oper, uint8_t addr);
void ENC28_writeOp(uint8_t oper, uint8_t addr, uint8_t data);

uint8_t ENC28_readReg8(uint8_t addr);
void ENC28_writeReg8(uint8_t addr, uint8_t data);

uint8_t ENC28_readReg16(uint8_t addr);
void ENC28_writeReg16(uint8_t addr, uint8_t data);

void ENC28_setBank(uint8_t addr);

void ENC28_Init();

#endif /* ENC28J60_H_ */
