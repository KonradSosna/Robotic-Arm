/*
 * zyroskop.h
 *
 *  Created on: 7 gru 2015
 *      Author: PC
 */

#include <avr/io.h>

#ifndef ZYROSKOP_H_
#define ZYROSKOP_H_

#define WHO_AM_I 0x0F
#define gyroscope_W 0xD6
#define gyroscope_R 0xD7 // SA0 to + VDD through 10kOhm resistor, read

#define CTRL1 0x20
#define CTRL1REG 0x0F // 12,5 Hz (ODR), n.a. (cut-off[hz]), power mode normal, all axis enabled
#define CTRL2 0x21

#define CTRL3 0x22
#define CTRL4 0x23

#define CTRL5 0x24
#define CTRL6 0x25

#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

#define FIFO_CTRL 0x2E
#define FIFO_SRC 0x2F


void initgyro(void);
void Inicjacja(void);
void Start(void);
void Write (uint8_t address);
void Read(void);
void Stop(void);
void TWI_write_register(uint8_t address, uint8_t reg, uint8_t val);
uint8_t TWI_read_register(uint8_t address, uint8_t reg);


#endif /* ZYROSKOP_H_ */
