/**************************************************************************/
/*!
 @file     ds1621.h
 @author   Tim Jagenberg (tim@jagenberg.info)

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

#ifndef DS1621_H_
#define DS1621_H_

/*=========================================================================
    I2C SLAVE ADDRESS/REGISTER ADDRESSES
    -----------------------------------------------------------------------*/
#define	DS1621_SA						0x90 // 1001 000 << 1 with A0-A2 = 0
#define	DS1621_RA_TEMP					0xaa
#define	DS1621_RA_TH					0xa1
#define	DS1621_RA_TL					0xa2
#define	DS1621_RA_CONFIG				0xac
#define	DS1621_RA_COUNT					0xa8
#define	DS1621_RA_SLOPE					0xa9
#define	DS1621_RA_STARTCONV				0xee
#define	DS1621_RA_STOPCONV				0x22

/*=========================================================================
    I2C FIELD START BITS/BIT LENGTHS
    -----------------------------------------------------------------------*/
#define	DS1621_FR_TEMP_TEMP_SB			15
#define	DS1621_FR_TEMP_TEMP_BL			16
#define	DS1621_FRW_TH_TH_SB				15
#define	DS1621_FRW_TH_TH_BL				16
#define	DS1621_FRW_TL_TL_SB				15
#define	DS1621_FRW_TL_TL_BL				16
#define	DS1621_FRW_CONFIG_DONE_SB		7
#define	DS1621_FRW_CONFIG_DONE_BL		1
#define	DS1621_FRW_CONFIG_THF_SB		6
#define	DS1621_FRW_CONFIG_THF_BL		1
#define	DS1621_FRW_CONFIG_TLF_SB		5
#define	DS1621_FRW_CONFIG_TLF_BL		1
#define	DS1621_FRW_CONFIG_NVB_SB		4
#define	DS1621_FRW_CONFIG_NVB_BL		1
#define	DS1621_FRW_CONFIG_POL_SB		1
#define	DS1621_FRW_CONFIG_POL_BL		1
#define	DS1621_FRW_CONFIG_ONESHOT_SB	0
#define	DS1621_FRW_CONFIG_ONESHOT_BL	1
#define	DS1621_FR_COUNT_COUNT_SB		7
#define	DS1621_FR_COUNT_COUNT_BL		8
#define	DS1621_FR_SLOPE_SLOPE_SB		7
#define	DS1621_FR_SLOPE_SLOPE_BL		8

int ds1621_init(void);
uint16_t ds1621_read_temp_lores(void);
float ds1621_read_temp_hires(void);
uint16_t ds1621_read_th(void);
void ds1621_write_th(uint16_t);
uint16_t ds1621_read_tl(void);
void ds1621_write_tl(uint16_t);
bool ds1621_is_config_done(void);
bool ds1621_is_config_thf(void);
void ds1621_reset_config_thf(void);
bool ds1621_is_config_tlf(void);
void ds1621_reset_config_tlf(void);
bool ds1621_is_config_nvb(void);
bool ds1621_is_config_pol(void);
void ds1621_set_config_pol(bool);
bool ds1621_is_config_oneshot(void);
void ds1621_set_config_oneshot(bool);
uint8_t ds1621_read_count(void);
uint8_t ds1621_read_slope(void);
void ds1621_start_conv(void);
void ds1621_stop_conv(void);

#endif /* DS1621_H_ */
