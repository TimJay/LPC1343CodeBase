/**************************************************************************/
/*!
 @file ht16k33.h
 @author Tim Jagenberg (tim@jagenberg.info)

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

#include "projectconfig.h"
#include "core/i2c/i2c.h"

#ifndef BITTOOLS
#define BITTOOLS
/* a=target variable, b=bit number to act upon 0-n */
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))
/* x=target variable, y=mask */
#define BITMASK_SET(x,y) ((x) |= (y))
#define BITMASK_CLEAR(x,y) ((x) &= (~(y)))
#define BITMASK_FLIP(x,y) ((x) ^= (y))
#define BITMASK_CHECK(x,y) ((x) & (y))
#endif

#ifndef HT16K33_H_
#define HT16K33_H_

/*=========================================================================
 I2C SLAVE ADDRESS/COMMAND BASE ADDRESSES
 -----------------------------------------------------------------------*/
#define        HT16K33_SA						0xe0

#define        HT16K33_BUFFER_LENGTH			16

#define        HT16K33_DISPLAYDATA_CBA_RW		0x00
#define        HT16K33_DISPLAYDATA_MASK			0x0f
#define        HT16K33_DISPLAYDATA_SB			3
#define        HT16K33_DISPLAYDATA_BL			4
#define        HT16K33_DISPLAYDATA_GREEN		0
#define        HT16K33_DISPLAYDATA_RED			1
#define        HT16K33_DISPLAYDATA_YELLOW		2

#define        HT16K33_SYSSETUP_CBA_W			0x20
#define        HT16K33_SYSSETUP_OSC_MASK		0x01
#define        HT16K33_SYSSETUP_OSC_SB			0
#define        HT16K33_SYSSETUP_OSC_BL			1
#define        HT16K33_SYSSETUP_OSC_OFF			0x00
#define        HT16K33_SYSSETUP_OSC_ON			0x01

#define        HT16K33_KEYDATA_CBA_R			0x40
#define        HT16K33_KEYDATA_MASK				0x07
#define        HT16K33_KEYDATA_SB				2
#define        HT16K33_KEYDATA_BL				3

#define        HT16K33_INTFLAG_CBA_R			0x60

#define        HT16K33_DISPSETUP_CBA_W			0x80
#define        HT16K33_DISPSETUP_STATUS_MASK	0x01
#define        HT16K33_DISPSETUP_STATUS_SB		0
#define        HT16K33_DISPSETUP_STATUS_BL		1
#define        HT16K33_DISPSETUP_STATUS_OFF		0x00
#define        HT16K33_DISPSETUP_STATUS_ON		0x01
#define        HT16K33_DISPSETUP_BLINK_MASK		0x06
#define        HT16K33_DISPSETUP_BLINK_SB		2
#define        HT16K33_DISPSETUP_BLINK_BL		2
#define        HT16K33_DISPSETUP_BLINK_OFF		0x00
#define        HT16K33_DISPSETUP_BLINK_2HZ		0x02
#define        HT16K33_DISPSETUP_BLINK_1HZ		0x04
#define        HT16K33_DISPSETUP_BLINK_05HZ		0x06

#define        HT16K33_ROWINTSET_CBA_W			0xa0
#define        HT16K33_ROWINTSET_MASK			0x03
#define        HT16K33_ROWINTSET_SB				1
#define        HT16K33_ROWINTSET_BL				2
#define        HT16K33_ROWINTSET_ROW			0x00
#define        HT16K33_ROWINTSET_INTLOW			0x01
#define        HT16K33_ROWINTSET_INTHIGH		0x03

#define        HT16K33_DIMSET_CBA_W				0xe0
#define        HT16K33_DIMSET_MASK				0x0f
#define        HT16K33_DIMSET_SB				3
#define        HT16K33_DIMSET_BL				4
#define        HT16K33_DIMSET_1					0x00
#define        HT16K33_DIMSET_2					0x01
#define        HT16K33_DIMSET_3					0x02
#define        HT16K33_DIMSET_4					0x03
#define        HT16K33_DIMSET_5					0x04
#define        HT16K33_DIMSET_6					0x05
#define        HT16K33_DIMSET_7					0x06
#define        HT16K33_DIMSET_8					0x07
#define        HT16K33_DIMSET_9					0x08
#define        HT16K33_DIMSET_10				0x09
#define        HT16K33_DIMSET_11				0x0a
#define        HT16K33_DIMSET_12				0x0b
#define        HT16K33_DIMSET_13				0x0c
#define        HT16K33_DIMSET_14				0x0d
#define        HT16K33_DIMSET_15				0x0e
#define        HT16K33_DIMSET_16				0x0f

#define        HT16K33_TESTMODE_CBA_W			0xd9

int ht16k33_init(void);
void ht16k33_clearBuffer(void);
void ht16k33_setRotated(bool rotate);
void ht16k33_setMirroredH(bool mirrorH);
void ht16k33_setMirroredV(bool mirrorV);
void ht16k33_setLED(uint8_t row, uint8_t col, bool on, uint8_t color,
		bool update);
void ht16k33_updateLEDs(void);

#endif /* HT16K33_H_ */
