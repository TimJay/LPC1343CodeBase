/**************************************************************************/
/*!
 @file     r0ket.c
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

#include "r0ket.h"
#include "core/gpio/gpio.h"

void r0ketInit() {
	// Configure r0ket specific stuff
	// power to 3.3v POWER_GOOD
	gpioSetDir(1, 8, gpioDirection_Output);
	gpioSetValue(1, 8, 0);
	// set up PIO0_11 for GPIO/LED function
	IOCON_JTAG_TDI_PIO0_11 &= ~(IOCON_JTAG_TDI_PIO0_11_FUNC_MASK | IOCON_JTAG_TDI_PIO0_11_ADMODE_MASK);
	IOCON_JTAG_TDI_PIO0_11 |= IOCON_JTAG_TDI_PIO0_11_FUNC_GPIO | IOCON_JTAG_TDI_PIO0_11_ADMODE_DIGITAL;
	// set up PIO0_1 for GPIO/BUTTON function
	IOCON_PIO0_1 &= ~(IOCON_PIO0_1_FUNC_MASK);
	IOCON_PIO0_1 |= IOCON_PIO0_1_FUNC_GPIO;
	// configure LEDs
	gpioSetDir(CFG_LED_TOPLEFT_PORT, CFG_LED_TOPLEFT_PIN, gpioDirection_Output);
	gpioSetValue(CFG_LED_TOPLEFT_PORT, CFG_LED_TOPLEFT_PIN, CFG_LED_OFF);
	gpioSetDir(CFG_LED_TOPRIGHT_PORT, CFG_LED_TOPRIGHT_PIN, gpioDirection_Output);
	gpioSetValue(CFG_LED_TOPRIGHT_PORT, CFG_LED_TOPRIGHT_PIN, CFG_LED_OFF);
	gpioSetDir(CFG_LED_BOTTOMLEFT_PORT, CFG_LED_BOTTOMLEFT_PIN, gpioDirection_Output);
	gpioSetValue(CFG_LED_BOTTOMLEFT_PORT, CFG_LED_BOTTOMLEFT_PIN, CFG_LED_OFF);
	gpioSetDir(CFG_LED_BOTTOMRIGHT_PORT, CFG_LED_BOTTOMRIGHT_PIN, gpioDirection_Output);
	gpioSetValue(CFG_LED_BOTTOMRIGHT_PORT, CFG_LED_BOTTOMRIGHT_PIN, CFG_LED_OFF);
}
