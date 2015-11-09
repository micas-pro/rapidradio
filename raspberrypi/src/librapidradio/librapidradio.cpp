/*
	The rapidradio project
	
	Author: Michal Okulski, the rapidradio team
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


#include <cstring>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <algorithm>
#include "rapidradio.h"

using namespace std;

namespace rapidradio
{

bool initRegisters();

//commands
const uint8_t PROGMEM RFM7x_cmd_adrRX0[] = { (0x20 | 0x0A), 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
const uint8_t PROGMEM RFM7x_cmd_adrTX[]  = { (0x20 | 0x10), 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
const uint8_t PROGMEM RFM7x_cmd_adrRX1[] = { (0x20 | 0x0B), 0x35, 0x43, 0x10, 0x10, 0x02};
const uint8_t PROGMEM RFM7x_cmd_switch_cfg[] = { 0x50, 0x53 }; // 
const uint8_t PROGMEM RFM7x_cmd_flush_rx[] = { 0xe2, 0x00 }; // flush RX FIFO
const uint8_t PROGMEM RFM7x_cmd_flush_tx[] = { 0xe1, 0x00 }; // flush TX FIFO
const uint8_t PROGMEM RFM7x_cmd_activate[] = { 0x50, 0x73 }; // 
const uint8_t PROGMEM RFM7x_cmd_tog1[] = { (0x20 | 0x04), 0xd9 | 0x06, 0x9e, 0x86, 0x0b };
const uint8_t PROGMEM RFM7x_cmd_tog2[] = { (0x20 | 0x04), 0xd9 & ~0x06, 0x9e, 0x86, 0x0b}; 

//RFM73
//************ Bank0 register initialization commands
const uint8_t PROGMEM RFM7x_bank0Init[][2] = {
  // address data
  { (0x20 | 0x00), 0x03 }, //Disable CRC ,CRC=1byte, POWER UP, RX
  { (0x20 | 0x01), 0x3F }, //Enable auto acknowledgement data pipe0-5
  { (0x20 | 0x02), 0x3F }, //Enable RX Addresses pipe0-5
  { (0x20 | 0x03), 0x02 }, //RX/TX address field width 4byte
  { (0x20 | 0x04), 0xFF }, //4ms, 15 retries
  { (0x20 | 0x05), 0x17 }, //channel = 0x17
  { (0x20 | 0x06), 0x0F }, //init register 6 for RFM73 (2M, LNA gain high, 5dBM) //2F
  { (0x20 | 0x07), 0x07 }, //
  { (0x20 | 0x08), 0x00 }, //
  { (0x20 | 0x09), 0x00 }, //
  { (0x20 | 0x0C), 0xc3 }, //LSB Addr pipe 2
  { (0x20 | 0x0D), 0xc4 }, //LSB Addr pipe 3
  { (0x20 | 0x0E), 0xc5 }, //LSB Addr pipe 4
  { (0x20 | 0x0F), 0xc6 }, //LSB Addr pipe 5
  { (0x20 | 0x11), 0x20 }, //Payload len pipe0
  { (0x20 | 0x12), 0x20 }, //Payload len pipe0
  { (0x20 | 0x13), 0x20 }, //Payload len pipe0
  { (0x20 | 0x14), 0x20 }, //Payload len pipe0
  { (0x20 | 0x15), 0x20 }, //Payload len pipe0
  { (0x20 | 0x16), 0x20 }, //Payload len pipe0
  { (0x20 | 0x17), 0x20 }, //Payload len pipe0
  { (0x20 | 0x1C), 0x3F }, //Enable dynamic payload legth data pipe0-5
  { (0x20 | 0x1D), 0x07 } //Enables Dynamic Payload Length,Enables Payload with ACK
};

//************ Bank1 register initialization commands
const uint8_t PROGMEM RFM7x_bank1Init[][5] = {
  { (0x20 | 0x00), 0x40, 0x4B, 0x01, 0xE2 },
  { (0x20 | 0x01), 0xC0, 0x4B, 0x00, 0x00 },
  { (0x20 | 0x02), 0xD0, 0xFC, 0x8C, 0x02 },
  { (0x20 | 0x03), 0x99, 0x00, 0x39, 0x41 },
  { (0x20 | 0x04), 0xDB, 0x82, 0x96, 0xF9 },
  { (0x20 | 0x05), 0xB6, 0x0F, 0x06, 0x24 },
  { (0x20 | 0x06), 0x00, 0x00, 0x00, 0x00 },
  { (0x20 | 0x07), 0x00, 0x00, 0x00, 0x00 },
  { (0x20 | 0x08), 0x00, 0x00, 0x00, 0x00 },
  { (0x20 | 0x09), 0x00, 0x00, 0x00, 0x00 },
  { (0x20 | 0x0a), 0x00, 0x00, 0x00, 0x00 },
  { (0x20 | 0x0b), 0x00, 0x00, 0x00, 0x00 },
  { (0x20 | 0x0C), 0x00, 0x12, 0x73, 0x00 },
  { (0x20 | 0x0D), 0x36, 0xb4, 0x80, 0x00 }
};

//Bank1 register 14
const uint8_t PROGMEM RFM7x_bank1R0EInit[] = {
  (0x20 | 0x0E), 0x41, 0x20, 0x08, 0x04, 0x81, 0x20, 0xCF, 0xF7, 0xFE, 0xFF, 0xFF
};


void SPI_MasterInit(void)
{	
	//Setup SPI pins
	bcm2835_spi_begin();
	
	//Set CS pins polarity to low
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, 0);
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, 0);
	
	//Set SPI clock speed
	//	BCM2835_SPI_CLOCK_DIVIDER_65536 = 0,       ///< 65536 = 262.144us = 3.814697260kHz (total H+L clock period) 
	//	BCM2835_SPI_CLOCK_DIVIDER_32768 = 32768,   ///< 32768 = 131.072us = 7.629394531kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_16384 = 16384,   ///< 16384 = 65.536us = 15.25878906kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_8192  = 8192,    ///< 8192 = 32.768us = 30/51757813kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_4096  = 4096,    ///< 4096 = 16.384us = 61.03515625kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_2048  = 2048,    ///< 2048 = 8.192us = 122.0703125kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_1024  = 1024,    ///< 1024 = 4.096us = 244.140625kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_512   = 512,     ///< 512 = 2.048us = 488.28125kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_256   = 256,     ///< 256 = 1.024us = 976.5625MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_128   = 128,     ///< 128 = 512ns = = 1.953125MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_64    = 64,      ///< 64 = 256ns = 3.90625MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_32    = 32,      ///< 32 = 128ns = 7.8125MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_16    = 16,      ///< 16 = 64ns = 15.625MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_8     = 8,       ///< 8 = 32ns = 31.25MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_4     = 4,       ///< 4 = 16ns = 62.5MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_2     = 2,       ///< 2 = 8ns = 125MHz, fastest you can get
	//	BCM2835_SPI_CLOCK_DIVIDER_1     = 1,       ///< 1 = 262.144us = 3.814697260kHz, same as 0/65536
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32);

	//Set SPI data mode
	//	BCM2835_SPI_MODE0 = 0,  // CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
	//	BCM2835_SPI_MODE1 = 1,  // CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
	//	BCM2835_SPI_MODE2 = 2,  // CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
	//	BCM2835_SPI_MODE3 = 3,  // CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);

	//Set with CS pin to use for next transfers
	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
}

bool init()
{
	bcm2835_gpio_fsel(CE, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(CSN, BCM2835_GPIO_FSEL_OUTP);
	
	bcm2835_gpio_fsel(IRQ, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_set_pud(IRQ, BCM2835_GPIO_PUD_UP);
	
	// doesn't work on my RPi with Raspbian
	//bcm2835_gpio_fen(IRQ);
	
	CE_LOW;
	CSN_HIGH;
	
	SPI_MasterInit();
	
	_delay_ms(RFM7x_BEGIN_INIT_WAIT_MS);
	return initRegisters();
}

bool initRegisters()
{
	// init bank 0 registers
	selectBank(0);

	// !! The last two regs in the bank0Init list will be handled later
	for (int i = 0; i < 20; i++)
	writeRegVal(pgm_read_byte(&RFM7x_bank0Init[i][0]), pgm_read_byte(&RFM7x_bank0Init[i][1]));

	// init address registers in bank 0
	writeRegPgmBuf((uint8_t *)RFM7x_cmd_adrRX0, sizeof(RFM7x_cmd_adrRX0));
	writeRegPgmBuf((uint8_t *)RFM7x_cmd_adrRX1, sizeof(RFM7x_cmd_adrRX1));
	writeRegPgmBuf((uint8_t *)RFM7x_cmd_adrTX, sizeof(RFM7x_cmd_adrTX));

	// activate Feature register
	if(!readRegVal(RFM7x_REG_FEATURE))
	writeRegPgmBuf((uint8_t *)RFM7x_cmd_activate, sizeof(RFM7x_cmd_activate));

	// now set Registers 1D and 1C
	writeRegVal(pgm_read_byte(&RFM7x_bank0Init[22][0]), pgm_read_byte(&RFM7x_bank0Init[22][1]));
	writeRegVal(pgm_read_byte(&RFM7x_bank0Init[21][0]), pgm_read_byte(&RFM7x_bank0Init[21][1]));

	// init bank 1 registers
	selectBank(1);

	for (int i=0; i < 14; i++)
	writeRegPgmBuf((uint8_t *)RFM7x_bank1Init[i], sizeof(RFM7x_bank1Init[i]));

	// set ramp curve
	writeRegPgmBuf((uint8_t *)RFM7x_bank1R0EInit, sizeof(RFM7x_bank1R0EInit));

	// do we have to toggle some bits here like in the example code?
	writeRegPgmBuf((uint8_t *)RFM7x_cmd_tog1, sizeof(RFM7x_cmd_tog1));
	writeRegPgmBuf((uint8_t *)RFM7x_cmd_tog2, sizeof(RFM7x_cmd_tog2));


	_delay_ms(RFM7x_END_INIT_WAIT_MS);

	//Check the ChipID
	if (readRegVal(0x08) != 0x63)
	{
		return false;
	}

	selectBank(0);
	setModeRX();
	
	return true;
}

uint8_t transmitSPI(uint8_t val)
{	
	return bcm2835_spi_transfer(val);
}

void selectBank(uint8_t bank) 
{
	uint8_t tmp = readRegVal(0x07) & 0x80;
	if(bank) 
	{
		if(!tmp)
			writeRegPgmBuf((uint8_t *)RFM7x_cmd_switch_cfg, sizeof(RFM7x_cmd_switch_cfg));
	} 
	else 
	{
		if(tmp)
			writeRegPgmBuf((uint8_t *)RFM7x_cmd_switch_cfg, sizeof(RFM7x_cmd_switch_cfg));
	}
}

void setModeRX(void)
{
	uint8_t val;
	writeRegPgmBuf((uint8_t *)RFM7x_cmd_flush_rx, sizeof(RFM7x_cmd_flush_rx)); 
	val = readRegVal(RFM7x_REG_STATUS);
	writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_STATUS, val); 
	CE_LOW;
	val=readRegVal(RFM7x_REG_CONFIG);
	val |= RFM7x_PIN_PRIM_RX;
	writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_CONFIG, val); 
	CE_HIGH;
}


void setModeTX(void) 
{
	uint8_t val;
	writeRegPgmBuf((uint8_t *)RFM7x_cmd_flush_tx, sizeof(RFM7x_cmd_flush_tx));
	val = readRegVal(RFM7x_REG_STATUS); 
	writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_STATUS, val);
	CE_LOW;
	val=readRegVal(RFM7x_REG_CONFIG);
	val &= ~RFM7x_PIN_PRIM_RX;
	writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_CONFIG, val); 
	
	// Don't turn the radio on to save power
	//CE_HIGH;
}

uint8_t getMode(void) 
{
	return readRegVal(RFM7x_REG_CONFIG) & RFM7x_PIN_PRIM_RX;
}

void setChannel(uint8_t cnum) 
{
	writeRegVal( RFM7x_CMD_WRITE_REG | RFM7x_REG_RF_CH, cnum);
}

uint8_t getChannel(void) 
{
	return readRegVal(RFM7x_REG_RF_CH);
}

void setPower(uint8_t pwr) 
{
	if (pwr > 3) return;
	uint8_t tmp = readRegVal(RFM7x_REG_RF_SETUP);
	tmp &= 0xF9;
	tmp |= pwr << 1;
	
	writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_RF_SETUP, tmp);
}

uint8_t readRegVal(uint8_t cmd)  
{
	uint8_t res;
	CSN_LOW;
	_delay_ms(RFM7x_CS_DELAY);
	transmitSPI(cmd);
	res = transmitSPI(0);
	CSN_HIGH;
	_delay_ms(RFM7x_CS_DELAY);
	return res;
}

uint8_t writeRegVal(uint8_t cmd, uint8_t val) 
{	
	CSN_LOW;
	_delay_ms(RFM7x_CS_DELAY);
	transmitSPI(cmd);
	transmitSPI(val);
	CSN_HIGH;
	_delay_ms(RFM7x_CS_DELAY);
	return 1;
}

void readRegBuf(uint8_t reg, uint8_t * buf, uint8_t len) 
{
	uint8_t byte_ctr;
	CSN_LOW;
	_delay_ms(RFM7x_CS_DELAY);
	transmitSPI(reg); 
	for(byte_ctr = 0; byte_ctr < len; byte_ctr++)
		buf[byte_ctr] = transmitSPI(0); 
	CSN_HIGH;
	_delay_ms(RFM7x_CS_DELAY);
}



uint8_t writeRegPgmBuf(uint8_t * cmdbuf, uint8_t len) 
{
	CSN_LOW;
	_delay_ms(RFM7x_CS_DELAY);
	while(len--) transmitSPI(pgm_read_byte(cmdbuf++));
	CSN_HIGH;
	_delay_ms(RFM7x_CS_DELAY);
	return 1;
}

uint8_t writeRegCmdBuf(uint8_t cmd, uint8_t * buf, uint8_t len)
{

	CSN_LOW;
	_delay_ms(RFM7x_CS_DELAY);
	transmitSPI(cmd);
	while(len--) transmitSPI(*(buf++));	
	CSN_HIGH;
	_delay_ms(RFM7x_CS_DELAY);
	return 1;
}

// Important! adr has to be 5-bytes length even if MSB bytes are unused
uint8_t configRxPipe(uint8_t pipe_nr, uint8_t *adr, uint8_t plLen, uint8_t en_aa)
{

	uint8_t tmp;
	uint8_t nr = pipe_nr -1;

	if(plLen > 32 || nr > 5 || en_aa > 1)
		return 0;

	// write address
	if(nr<2)      // full length for rx pipe 0 an 1
		writeRegCmdBuf(RFM7x_CMD_WRITE_REG | (RFM7x_REG_RX_ADDR_P0 + nr), adr, 5);
	else // only LSB for pipes 2..5
		writeRegVal(RFM7x_CMD_WRITE_REG | (RFM7x_REG_RX_ADDR_P0 + nr), adr[0]); 

	// static
	if (plLen) {
		// set payload len
		writeRegVal(RFM7x_CMD_WRITE_REG | (RFM7x_REG_RX_PW_P0 + nr), plLen);
		// set EN_AA bit
		tmp = readRegVal(RFM7x_REG_EN_AA);
		if (en_aa)
		tmp |= 1 << nr;
		else
		tmp &= ~(1 << nr);
		writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_EN_AA, tmp);
		// clear DPL bit
		tmp = readRegVal(RFM7x_REG_DYNPD);
		tmp &= ~(1 << nr);
		writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_DYNPD, tmp);        
		// set Enable pipe bit
		enableRxPipe(nr);
	}
	// dynamic
	else 
	{
		// set payload len to default
		writeRegVal(RFM7x_CMD_WRITE_REG | (RFM7x_REG_RX_PW_P0 + nr), 0x20);
		// set EN_AA bit
		tmp = readRegVal(RFM7x_REG_EN_AA);
		tmp |= 1 << nr;
		writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_EN_AA, tmp);
		// set DPL bit
		tmp = readRegVal(RFM7x_REG_DYNPD);
		tmp |= 1 << nr;
		writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_DYNPD, tmp);
		// set Enable pipe bit
		enableRxPipe(nr);
	}
	return 1;
}

void enableRxPipe(uint8_t pipe_nr) 
{
	uint8_t nr = pipe_nr - 1;
	if (nr > 5) return;
	uint8_t tmp;
	// set Enable pipe bit
	tmp = readRegVal(RFM7x_REG_EN_RXADDR);
	tmp |= 1 << nr;
	writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_EN_RXADDR, tmp);
}

void disableRxPipe(uint8_t pipe_nr) 
{
	uint8_t nr = pipe_nr - 1;
	if (nr > 5) return;
	uint8_t tmp;
	// set Enable pipe bit
	tmp = readRegVal(RFM7x_REG_EN_RXADDR);
	tmp &= ~(1 << nr);
	writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_EN_RXADDR, tmp);
}

void configTxPipe(uint8_t * adr, uint8_t pltype) 
{
	// write TX address
	writeRegCmdBuf(RFM7x_CMD_WRITE_REG | RFM7x_REG_TX_ADDR, adr, 5);
	// write RX0 address
	writeRegCmdBuf(RFM7x_CMD_WRITE_REG | RFM7x_REG_RX_ADDR_P0, adr, 5);
	// set static or dynamic payload
	uint8_t tmp;
	tmp = readRegVal(RFM7x_REG_DYNPD);
	if(pltype == TX_DPL) // dynamic
	tmp |= 1;
	else  
	tmp &= ~(1 << 0);
	writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_DYNPD, tmp);
}

uint8_t sendPayload(const uint8_t * payload, const uint8_t _len, const uint8_t toAck)
{
	// turn on the radio
	CE_HIGH;
	
	// check TX_FIFO
	uint8_t status;
	status = readRegVal(RFM7x_REG_FIFO_STATUS); 
	if (status & RFM7x_FIFO_STATUS_TX_FULL) 
	{
		return 0;
	}

	// send payload
	CSN_LOW;
	_delay_ms(RFM7x_CS_DELAY);
	if(toAck == -1)
		transmitSPI(RFM7x_CMD_W_ACK_PAYLOAD);
	else if (toAck == 0)
		transmitSPI(RFM7x_CMD_W_TX_PAYLOAD_NOACK);
	else
		transmitSPI(RFM7x_CMD_WR_TX_PLOAD);
		
	uint8_t len = _len;
	while(len--) transmitSPI(*(payload++));
	CSN_HIGH;
	_delay_ms(RFM7x_CS_DELAY);
	return 1;
}

uint8_t receivePayload(uint8_t *payload)
{
	uint8_t len;
	// check RX_FIFO
	uint8_t status;
	status = readRegVal(RFM7x_REG_STATUS);
	if (status & RFM7x_IRQ_STATUS_RX_DR) { // RX_DR
		
		uint8_t fifo_sta;
		len = readRegVal(RFM7x_CMD_RX_PL_WID); // Payload width
		readRegBuf(RFM7x_CMD_RD_RX_PLOAD, payload, len);
		fifo_sta = readRegVal(RFM7x_REG_FIFO_STATUS);
		
		if (fifo_sta & RFM7x_FIFO_STATUS_RX_EMPTY) {
			status|= 0x40 & 0xCF; // clear status bit rx_dr
			writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_STATUS, status); 
		}
		return len;
	}
	else
	{		
		return 0;
	}
}

void flushTxFIFO() 
{
	writeRegPgmBuf((uint8_t *)RFM7x_cmd_flush_tx, sizeof(RFM7x_cmd_flush_tx));
}

void flushRxFIFO() 
{
	writeRegPgmBuf((uint8_t *)RFM7x_cmd_flush_rx, sizeof(RFM7x_cmd_flush_rx));
}

void setCE()
{
	CE_HIGH;
}

void resetCE()
{
	CE_LOW;
}

void setCSN()
{
	CSN_HIGH;
}

void resetCSN()
{
	CSN_LOW;
}

void turnOn()
{
	uint8_t status = readRegVal(RFM7x_REG_STATUS);
	status |= PWR_BIT;
	writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_STATUS, status);
}

void turnOff()
{
	uint8_t status = readRegVal(RFM7x_REG_STATUS);
	status &= ~PWR_BIT;
	writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_STATUS, status);
	CE_LOW;
}

bool checkStatusForMissingIRQ(uint8_t &status)
{
	status = readRegVal(RFM7x_REG_STATUS);
	return status != 14;
}

TransmitResult internalSendPacket(const uint8_t *buff, const size_t &length, const uint32_t maxTimeoutUs, bool requestAck)
{
	const size_t toSendLength = MIN(length, RFM7x_MAX_PACKET_LEN);
	sendPayload(buff, toSendLength, requestAck ? 1 : 0);
	
	TransmitResult result;
	
	uint32_t i=0;
	uint8_t status = 0;
	bool readStatus = true;
	while (bcm2835_gpio_lev(IRQ) != LOW && i++ < maxTimeoutUs)
	{		
		delayMicroseconds(1);
		if (i >= 10 && i % 5 == 0)
		{
			if (checkStatusForMissingIRQ(status)) 
			{
				readStatus = false;
				break;
			}
		}
	}
	
	if (readStatus)
	{
		status = readRegVal(RFM7x_REG_STATUS);
	}
	
	if (status & RFM7x_IRQ_STATUS_TX_DS)
	{
		result.status = Success;
		result.bytesSent = toSendLength;
	}
	else if (status & RFM7x_IRQ_STATUS_MAX_RT)
	{
		result.status = MaxRT;
		result.bytesSent = 0;
	}
	else if (status & RFM7x_IRQ_STATUS_TX_FULL)
	{
		result.status = FifoFull;
		result.bytesSent = 0;
	}
	
	if (i >= maxTimeoutUs)
	{
		fprintf(stderr, "Result %u %u %s\n", status, i, bcm2835_gpio_lev(IRQ) == LOW ? "LOW" : "HIGH");
		result.status = Unknown;
		result.bytesSent = 0;
	}
	
	//if (result.status != Success) it's better to always clear the error flags
	{
		// Reset error flags
		writeRegVal(RFM7x_CMD_WRITE_REG | RFM7x_REG_STATUS, status);
	}
	
	return result;
}

TransmitResult& operator +=(TransmitResult &a, const TransmitResult b)
{
	a.status = b.status;	
	a.bytesSent += b.bytesSent;
	return a;
}

TransmitResult internalSend(const uint8_t *buff, const size_t &length, bool requestAck, bool usePacketNumber, const uint8_t firstPacketNumber)
{	
	uint8_t setupRetr = readRegVal(RFM7x_REG_SETUP_RETR);
	uint32_t maxTimeoutUs = (uint32_t)(((setupRetr & 0b11110000) >> 4) + 1) * 250UL * (uint32_t)(setupRetr & 0b00001111);
	maxTimeoutUs += maxTimeoutUs/2UL;
	
	if (maxTimeoutUs == 0) maxTimeoutUs = 250;

	CE_HIGH;
	
	TransmitResult result;
	result.status = Success;
	result.bytesSent = 0;
	
	const int maxPacketAttempts = 50;
	const int packetLength = usePacketNumber ? RFM7x_MAX_PACKET_LEN - 1 : RFM7x_MAX_PACKET_LEN;
	uint8_t packetNr = firstPacketNumber;
	for (size_t i=0; i<length; i+=packetLength)
	{				
		int packetAttempts = maxPacketAttempts;
		while (packetAttempts--)
		{
			if (result.status != Success)
			{
				fprintf(stderr, "Packet send error %u, retrying (%u/%u)...\n", (int)result.status, maxPacketAttempts-packetAttempts, maxPacketAttempts);
			}
		
			const uint8_t *buffToSend;
			
			if (usePacketNumber)
			{			
				uint8_t tmpBuff[RFM7x_MAX_PACKET_LEN];
				tmpBuff[0] = packetNr++;
				memcpy(tmpBuff + 1, buff + i, MIN(length - i, packetLength));
				buffToSend = tmpBuff;
			}
			else
			{
				buffToSend = buff + i;
			}
			
			const size_t bytesToSend = MIN(length - i + (usePacketNumber ? 1 : 0), RFM7x_MAX_PACKET_LEN);
			result += internalSendPacket(buffToSend, bytesToSend, maxTimeoutUs, requestAck);
			
			if (result.status == Success)
			{
				break;
			}
			else if (result.status == FifoFull)
			{
				flushTxFIFO();
			}
						
			usleep(100000);
		}
		
		if (result.status != Success)
		{
			break;
		}
	}

	CE_LOW;
	return result;
}

// if length is greater than 32 bytes, then buff will be splitted into 32-bytes long packets
TransmitResult send(const uint8_t *buff, const size_t &length, bool requestAck, bool usePacketNumber, const uint8_t firstPacketNumber)
{
	if (getMode() != MODE_PTX)
	{
		setModeTX();
	}
	
	return internalSend(buff, length, requestAck, usePacketNumber, firstPacketNumber);
}

// if length is greater than 32 bytes, then buff will be splitted into 32-bytes long packets
TransmitResult send(const uint32_t &targetAddress, const uint8_t *buff, const size_t &length, bool requestAck, bool usePacketNumber, const uint8_t firstPacketNumber)
{
	if (getMode() != MODE_PTX)
	{
		setModeTX();
	}
	
	uint8_t adr[5];
	memcpy(adr, &targetAddress, 4);
	adr[4] = 0;
	configTxPipe(adr, TX_DPL);
	
	return internalSend(buff, length, requestAck, usePacketNumber, firstPacketNumber);
}

// if length is greater than 32 bytes, then buff will be splitted into 32-bytes long packets
TransmitResult send(const uint8_t channel, const uint32_t &targetAddress, const uint8_t *buff, const size_t &length, bool requestAck, bool usePacketNumber, const uint8_t firstPacketNumber)
{
	if (getMode() != MODE_PTX)
	{
		setModeTX();
	}
	
	uint8_t adr[5];
	memcpy(adr, &targetAddress, 4);
	adr[4] = 0;
	configTxPipe(adr, TX_DPL);
	
	setChannel(channel);
	
	return internalSend(buff, length, requestAck, usePacketNumber, firstPacketNumber);
}

void prepareForListening(const uint32_t &localAddress)
{
	if (getMode() != MODE_PRX)
	{
		setModeRX();
	}
	
	uint8_t adr[5];
	memcpy(adr, &localAddress, 4);
	adr[4] = 0;
	if (!configRxPipe(1, adr, 0, 1))
	{
		fprintf(stderr, "Can't configure Rx Pipe properly!\n");
	}
}

void startListening(const uint32_t &localAddress)
{
	prepareForListening(localAddress);
	
	CE_HIGH;
}

void startListening(const uint8_t channel, const uint32_t &localAddress)
{
	prepareForListening(localAddress);
	
	setChannel(channel);
	
	CE_HIGH;
}

// buff has to be at least 32 bytes!
bool received(uint8_t *buff, uint8_t &length)
{
	length = receivePayload(buff);

	if (length > 0)
	{
		return true;
	}
	else
	{
		delayMicroseconds(1);
		return false;
	}
}

}
