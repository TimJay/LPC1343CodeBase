/**************************************************************************/
/*!
 @file     ds1621.c
 @author   Tim Jagenberg (tim@jagenberg.info)

 @brief    Drivers for maxim integrated DS1621 Thermometer and Thermostat

 @section DESCRIPTION

 The DS1621 Digital Thermometer and Thermostat provides 9-bit temperature
 readings, which indicate the temperature of the device. The thermal alarm
 output, TOUT, is active when the temperature of the device exceeds a
 user-defined temperature TH. The output remains active until the temperature
 drops below user defined temperature TL, allowing for any hysteresis
 necessary.

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
#include "ds1621.h"
#include "core/systick/systick.h"

extern volatile uint8_t I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t I2CReadLength, I2CWriteLength;

static bool _ds1621_inited = false;

/**************************************************************************/
/*!
 @brief  Clears the I2CMasterBuffer
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
 @brief  Read one byte from register_address
 */
/**************************************************************************/
static uint8_t i2c_read_byte(uint8_t register_address) {
	clear_write_buffer();
	I2CWriteLength = 2;
	I2CReadLength = 1;
	I2CMasterBuffer[0] = DS1621_SA;
	I2CMasterBuffer[1] = register_address;
	// WHY?!
	I2CMasterBuffer[2] = DS1621_SA | RD_BIT;
	i2cEngine();
	return I2CSlaveBuffer[0];
}

/**************************************************************************/
/*!
 @brief  Read two bytes/one word from register_address
 most significant byte first
 */
/**************************************************************************/
static uint16_t i2c_read_word_msbfirst(uint8_t register_address) {
	clear_write_buffer();
	I2CWriteLength = 2;
	I2CReadLength = 2;
	I2CMasterBuffer[0] = DS1621_SA;
	I2CMasterBuffer[1] = register_address;
	// WHY?!
	I2CMasterBuffer[2] = DS1621_SA | RD_BIT;
	i2cEngine();
	return (I2CSlaveBuffer[0] << 8) | (I2CSlaveBuffer[1]);
}

/**************************************************************************/
/*!
 @brief  Issue command without data
 */
/**************************************************************************/
static void i2c_cmd(uint8_t register_address) {
	clear_write_buffer();
	I2CWriteLength = 2;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = DS1621_SA;
	I2CMasterBuffer[1] = register_address;
	i2cEngine();
}

/**************************************************************************/
/*!
 @brief  Write one byte to register_address
 */
/**************************************************************************/
static void i2c_write_byte(uint8_t register_address, uint8_t value) {
	clear_write_buffer();
	I2CWriteLength = 3;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = DS1621_SA;
	I2CMasterBuffer[1] = register_address;
	I2CMasterBuffer[2] = value;
	i2cEngine();
}

/**************************************************************************/
/*!
 @brief  write two bytes/one word to register_address
 most significant byte first
 */
/**************************************************************************/
static void i2c_write_word_msbfirst(uint8_t register_address, uint16_t value) {
	clear_write_buffer();
	I2CWriteLength = 4;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = DS1621_SA;
	I2CMasterBuffer[1] = register_address;
	I2CMasterBuffer[2] = ((value >> 8) & 0xff);
	I2CMasterBuffer[3] = (value & 0xff);
	i2cEngine();
}

/**************************************************************************/
/*!
 @brief  Initialise I2C, no state change in DS1621
 */
/**************************************************************************/
int ds1621_init(void) {
	// Initialise I2C
	if (i2cInit(I2CMASTER) == false) {
		/* Fatal error */
		return -1;
	}

	/* Set initialisation flag */
	_ds1621_inited = true;

	return 0;
}

/**************************************************************************/
/*!
 @brief  Read raw low-resolution temp (see p. 4 DS1621 doc)
 */
/**************************************************************************/
uint16_t ds1621_read_temp_lores(void) {
	if (!_ds1621_inited)
		ds1621_init();
	return i2c_read_word_msbfirst(DS1621_RA_TEMP);
}

/**************************************************************************/
/*!
 @brief  Read high-resolution temp (see p. 4 DS1621 doc)
 */
/**************************************************************************/
float ds1621_read_temp_hires(void) {
	if (!_ds1621_inited)
		ds1621_init();
	uint16_t temp_raw = ds1621_read_temp_lores();
	int8_t temp_read = temp_raw >> 8;
	uint8_t count_remain = ds1621_read_count();
	uint8_t count_per_c = ds1621_read_slope();
	return (float) temp_read - 0.25f
			+ ((float) count_per_c - (float) count_remain) / (float) count_per_c;
}

/**************************************************************************/
/*!
 @brief  Read raw high temperature
 */
/**************************************************************************/
uint16_t ds1621_read_th(void) {
	if (!_ds1621_inited)
		ds1621_init();
	return i2c_read_word_msbfirst(DS1621_RA_TH);
}

/**************************************************************************/
/*!
 @brief  Write raw high temperature
 */
/**************************************************************************/
void ds1621_write_th(uint16_t th) {
	if (!_ds1621_inited)
		ds1621_init();
	i2c_write_word_msbfirst(DS1621_RA_TH, th);
}

/**************************************************************************/
/*!
 @brief  Read raw low temperature
 */
/**************************************************************************/
uint16_t ds1621_read_tl(void) {
	if (!_ds1621_inited)
		ds1621_init();
	return i2c_read_word_msbfirst(DS1621_RA_TL);
}

/**************************************************************************/
/*!
 @brief  Write raw low temperature
 */
/**************************************************************************/
void ds1621_write_tl(uint16_t tl) {
	if (!_ds1621_inited)
		ds1621_init();
	i2c_write_word_msbfirst(DS1621_RA_TL, tl);
}

/**************************************************************************/
/*!
 @brief  Check if conversion is done
 */
/**************************************************************************/
bool ds1621_is_config_done(void) {
	if (!_ds1621_inited)
		ds1621_init();
	return BIT_CHECK(i2c_read_byte(DS1621_RA_CONFIG), DS1621_FRW_CONFIG_DONE_SB);
}

/**************************************************************************/
/*!
 @brief  Check temperature high flag
 */
/**************************************************************************/
bool ds1621_is_config_thf(void) {
	if (!_ds1621_inited)
		ds1621_init();
	return BIT_CHECK(i2c_read_byte(DS1621_RA_CONFIG), DS1621_FRW_CONFIG_THF_SB);
}

/**************************************************************************/
/*!
 @brief  Reset temperature high flag to 0
 */
/**************************************************************************/
void ds1621_reset_config_thf(void) {
	if (!_ds1621_inited)
		ds1621_init();
	uint8_t original_value = i2c_read_byte(DS1621_RA_CONFIG);
	i2c_write_byte(DS1621_RA_CONFIG,
			BIT_CLEAR(original_value, DS1621_FRW_CONFIG_THF_SB));
}

/**************************************************************************/
/*!
 @brief  Check temperature low flag
 */
/**************************************************************************/
bool ds1621_is_config_tlf(void) {
	if (!_ds1621_inited)
		ds1621_init();
	return BIT_CHECK(i2c_read_byte(DS1621_RA_CONFIG), DS1621_FRW_CONFIG_TLF_SB);
}

/**************************************************************************/
/*!
 @brief  Reset temperature low flag
 */
/**************************************************************************/
void ds1621_reset_config_tlf(void) {
	if (!_ds1621_inited)
		ds1621_init();
	uint8_t original_value = i2c_read_byte(DS1621_RA_CONFIG);
	i2c_write_byte(DS1621_RA_CONFIG,
			BIT_CLEAR(original_value, DS1621_FRW_CONFIG_TLF_SB));
}

/**************************************************************************/
/*!
 @brief  Check if non-volatile memory bus is active
 */
/**************************************************************************/
bool ds1621_is_config_nvb(void) {
	if (!_ds1621_inited)
		ds1621_init();
	return BIT_CHECK(i2c_read_byte(DS1621_RA_CONFIG), DS1621_FRW_CONFIG_NVB_SB);
}

/**************************************************************************/
/*!
 @brief  Check output polarity bit
 */
/**************************************************************************/
bool ds1621_is_config_pol(void) {
	if (!_ds1621_inited)
		ds1621_init();
	return BIT_CHECK(i2c_read_byte(DS1621_RA_CONFIG), DS1621_FRW_CONFIG_POL_SB);
}

/**************************************************************************/
/*!
 @brief  Set output polarity bit
 */
/**************************************************************************/
void ds1621_set_config_pol(bool pol) {
	if (!_ds1621_inited)
		ds1621_init();
	uint8_t original_value = i2c_read_byte(DS1621_RA_CONFIG);
	if (pol) {
		i2c_write_byte(DS1621_RA_CONFIG,
				BIT_SET(original_value, DS1621_FRW_CONFIG_POL_SB));
	} else {
		i2c_write_byte(DS1621_RA_CONFIG,
				BIT_CLEAR(original_value, DS1621_FRW_CONFIG_POL_SB));
	}
}

/**************************************************************************/
/*!
 @brief  Check if one shot mode is active
 */
/**************************************************************************/
bool ds1621_is_config_oneshot(void) {
	if (!_ds1621_inited)
		ds1621_init();
	return BIT_CHECK(i2c_read_byte(DS1621_RA_CONFIG),
			DS1621_FRW_CONFIG_ONESHOT_SB);
}

/**************************************************************************/
/*!
 @brief  Set one shot mode
 */
/**************************************************************************/
void ds1621_set_config_oneshot(bool oneshot) {
	if (!_ds1621_inited)
		ds1621_init();
	uint8_t original_value = i2c_read_byte(DS1621_RA_CONFIG);
	if (oneshot) {
		i2c_write_byte(DS1621_RA_CONFIG,
				BIT_SET(original_value, DS1621_FRW_CONFIG_ONESHOT_SB));
	} else {
		i2c_write_byte(DS1621_RA_CONFIG,
				BIT_CLEAR(original_value, DS1621_FRW_CONFIG_ONESHOT_SB));
	}
}

/**************************************************************************/
/*!
 @brief  Read count_remain
 */
/**************************************************************************/
uint8_t ds1621_read_count(void) {
	if (!_ds1621_inited)
		ds1621_init();
	return i2c_read_byte(DS1621_RA_COUNT);
}

/**************************************************************************/
/*!
 @brief  Read count_per_c
 */
/**************************************************************************/
uint8_t ds1621_read_slope(void) {
	if (!_ds1621_inited)
		ds1621_init();
	return i2c_read_byte(DS1621_RA_SLOPE);
}

/**************************************************************************/
/*!
 @brief  Start one conversion (one shot mode) or continuous conversion
 */
/**************************************************************************/
void ds1621_start_conv(void) {
	if (!_ds1621_inited)
		ds1621_init();
	i2c_cmd(DS1621_RA_STARTCONV);
}

/**************************************************************************/
/*!
 @brief  Stop continuous conversion
 */
/**************************************************************************/
void ds1621_stop_conv(void) {
	if (!_ds1621_inited)
		ds1621_init();
	i2c_cmd(DS1621_RA_STOPCONV);
}
