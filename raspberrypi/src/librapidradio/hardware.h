/*
	The rapidradio project
	
	Author: Michal Okulski, The RapidRadio Team
	Website: http://rapidradio.pl
	Email: michal@rapidradio.pl
	
	Inspired by AVR's RFM70 libraries. 
	
	------------------------------------------------------------------------------------
	The MIT License (MIT)

	Copyright (c) 2015 Michal Okulski (micas.pro)

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
	------------------------------------------------------------------------------------
*/

#ifndef HARDWARE_H
#define HARDWARE_H

#include <config.h>

#define RAPIDRADIO_PLATFORM
#define RPI_BPLUS

#ifndef RAPIDRADIO_PLATFORM
	#error "RAPIDRADIO_PLATFORM is not defined. Did you run ./configure before?"
#endif

#ifdef RPI_BPLUS

#include <bcm2835.h>

#define CE								RPI_BPLUS_GPIO_J8_15
#define CSN								RPI_BPLUS_GPIO_J8_13
#define IRQ								RPI_BPLUS_GPIO_J8_11
#define WIRINGPI_IRQ					0

#define CE_LOW							bcm2835_gpio_clr(CE)
#define CE_HIGH							bcm2835_gpio_set(CE)

#define CSN_LOW							bcm2835_gpio_clr(CSN)
#define CSN_HIGH						bcm2835_gpio_set(CSN)

#define PROGMEM
#define pgm_read_byte(x)				(*x)
#define _delay_ms(x)					usleep((x)*1000)

#endif

#endif

