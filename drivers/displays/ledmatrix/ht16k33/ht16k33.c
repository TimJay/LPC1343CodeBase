/**************************************************************************/
/*!
 @file ht16k33.c
 @author Tim Jagenberg (tim@jagenberg.info)

 @brief Driver for HOLTEK HT16K33 16*8 LED Controller Driver

 @section DESCRIPTION

 The HT16K33 is a memory mapping and multi-function LED controller driver.
 The max. Display segment numbers in the device is 128 patterns (16 segments
 and 8 commons) with a 13*3 (MAX.) matrix key scan circuit.

 @section LICENSE

 Software License Agreement (BSD License)

 Copyright (c) 2013, Tim Jagenberg
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 1. Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 3. Neither the name of the copyright holders nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**************************************************************************/
#include "ht16k33.h"

extern volatile uint8_t I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t I2CReadLength, I2CWriteLength;

static bool _ht16k33_inited = false;
static bool rotated = false;
static bool mirroredH = false;
static bool mirroredV = false;
static uint8_t buffer[HT16K33_BUFFER_LENGTH];

/**************************************************************************/
/*!
 @brief Clears the I2CMasterBuffer
 */
/**************************************************************************/
static void clear_write_buffer(void) {
	uint32_t i;
	for (i = 0; i < I2C_BUFSIZE; i++) {
		I2CMasterBuffer[i] = 0x00;
	}
}

/**************************************************************************/
/*!
 @brief Issue command without data
 */
/**************************************************************************/
static void i2c_cmd(uint8_t register_address) {
	clear_write_buffer();
	I2CWriteLength = 2;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = HT16K33_SA;
	I2CMasterBuffer[1] = register_address;
	i2cEngine();
}

/**************************************************************************/
/*!
 @brief write n bytes to register_address
 */
/**************************************************************************/
static void i2c_write_nbytes(uint8_t register_address, uint8_t n,
		uint8_t values[]) {
	clear_write_buffer();
	if (2 + n > I2C_BUFSIZE) {
		// TODO proper error handling
		return;
	}
	I2CWriteLength = 2 + n;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = HT16K33_SA;
	I2CMasterBuffer[1] = register_address;
	uint8_t i;
	for (i = 0; i < n; i++) {
		I2CMasterBuffer[i + 2] = values[i];
	}
	i2cEngine();
}

/**************************************************************************/
/*!
 @brief Initialise I2C, no state change in HT16K33
 */
/**************************************************************************/
int ht16k33_init(void) {
	// Initialise I2C
	if (i2cInit(I2CMASTER) == false) {
		/* Fatal error */
		return -1;
	}

	// clear buffer
	ht16k33_clearBuffer();

	// turn oscillator on
	i2c_cmd(HT16K33_SYSSETUP_CBA_W | HT16K33_SYSSETUP_OSC_ON);

	// turn blinking off and display on
	i2c_cmd(
			HT16K33_DISPSETUP_CBA_W | HT16K33_DISPSETUP_BLINK_OFF
					| HT16K33_DISPSETUP_STATUS_ON);

	// set full intensity
	i2c_cmd(HT16K33_DIMSET_CBA_W | HT16K33_DIMSET_16);

	/* Set initialisation flag */
	_ht16k33_inited = true;

	return 0;
}

void ht16k33_clearBuffer(void) {
	uint8_t i;
	for (i = 0; i < HT16K33_BUFFER_LENGTH; i++) {
		buffer[i] = 0x00;
	}
}

void ht16k33_setRotated(bool rotate) {
	if (!_ht16k33_inited)
		ht16k33_init();
	rotated = rotate;
}

void ht16k33_setMirroredH(bool mirrorH) {
	if (!_ht16k33_inited)
		ht16k33_init();
	mirroredH = mirrorH;
}

void ht16k33_setMirroredV(bool mirrorV) {
	if (!_ht16k33_inited)
		ht16k33_init();
	mirroredV = mirrorV;
}

void ht16k33_setLED(uint8_t row, uint8_t col, bool on, uint8_t color,
		bool update) {
	if (!_ht16k33_inited)
		ht16k33_init();
	if (rotated) {
		uint8_t tmp = row;
		row = col;
		col = 7 - tmp;
	}
	if (mirroredH) {
		row = 7 - row;
	}
	if (mirroredV) {
		col = 7 - col;
	}
	if ((row >= 0) && (row <= 7) && (col >= 0) && (col <= 7)) {
		int buf_col = col;
		if ((color >= 0) && (color <= 1)) {
			int buf_row = (row * 2) + color;
			if (on) {
				buffer[buf_row] |= 1 << buf_col;
				if (update) {
					ht16k33_updateLEDs();
				}
			} else {
				buffer[buf_row] &= ~(1 << buf_col);
				if (update) {
					ht16k33_updateLEDs();
				}
			}
		} else {
			ht16k33_setLED(row, col, on, 0, false);
			ht16k33_setLED(row, col, on, 1, false);
			ht16k33_updateLEDs();
		}
	}
}

void ht16k33_updateLEDs(void) {
	if (!_ht16k33_inited)
		ht16k33_init();
	i2c_write_nbytes(HT16K33_DISPLAYDATA_CBA_RW, HT16K33_BUFFER_LENGTH, buffer);
}
