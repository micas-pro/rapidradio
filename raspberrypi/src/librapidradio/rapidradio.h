/*
	The rapidradio project
	
	Author: Michal Okulski, The RapidRadio Team
	Website: www.rapidradio.pl
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

#ifndef RAPIDRADIO_H
#define RAPIDRADIO_H
	
#include "hardware.h"
#include <inttypes.h>
#include <cstring>

namespace rapidradio
{

// High level functions ----------------------------------------------------------------------

bool init();
void turnOn();
void turnOff();

#define DBMM10       0x00 // parameter for setPower(pwr): -10 dBm
#define DBMM5        0x01 // parameter for setPower(pwr): -5 dBm
#define DBM0         0x02 // parameter for setPower(pwr): 0 dBm
#define DBM5         0x03 // parameter for setPower(pwr): +5 dBm
void setPower(uint8_t pwr);

typedef enum
{
	Success = 0,
	FifoFull = 1,
	MaxRT = 2,
	Unknown = 3
} TransmitStatus;

typedef struct
{
	TransmitStatus status;
	size_t bytesSent;
} TransmitResult;

// if length is greater than 32 bytes, then buff will be splitted into 32-bytes long packets
TransmitResult send(const uint8_t *buff, const size_t &length, bool requestAck = true, bool usePacketNumber = false, const uint8_t firstPacketNumber = 1);
TransmitResult send(const uint32_t &targetAddress, const uint8_t *buff, const size_t &length, bool requestAck = true, bool usePacketNumber = false, const uint8_t firstPacketNumber = 1);
TransmitResult send(const uint8_t channel, const uint32_t &targetAddress, const uint8_t *buff, const size_t &length, bool requestAck = true, bool usePacketNumber = false, const uint8_t firstPacketNumber = 1);

template <class Iterator>
TransmitResult send(const Iterator begin, const Iterator end, bool requestAck = true, bool usePacketNumber = false, const uint8_t firstPacketNumber = 1);

template <class Iterator>
TransmitResult send(const uint32_t &targetAddress, const Iterator begin, const Iterator end, bool requestAck = true, bool usePacketNumber = false, const uint8_t firstPacketNumber = 1);

template <class Iterator>
TransmitResult send(const uint8_t channel, const uint32_t &targetAddress, const Iterator begin, const Iterator end, bool requestAck = true, bool usePacketNumber = false, const uint8_t firstPacketNumber = 1);

void startListening(const uint32_t &localAddress);
void startListening(const uint8_t channel, const uint32_t &localAddress);

// buff has to be at least 32 bytes!
bool received(uint8_t *buff, uint8_t &length);

// Low level functions and definitions -------------------------------------------------------

#define WITH_ACK     0x01 // parameter for sendPayload(..): send with ack expectation
#define NO_ACK       0x00 // parameter for sendPayload(..): send without ack expectation
#define MODE_PTX     0x00 // parameter for setMode(mode): set to transmitter
#define MODE_PRX     0x01 // parameter for setMode(mode): set to receiver
#define EN_AA        0x01 // parameter for configRxPipe(..): enable pipe auto ack
#define NO_AA        0x00 // parameter for configRxPipe(..): disable pipe auto ack
#define TX_DPL       0x01 // parameter for configTxPipe(..): enable dynamic payload for PTX
#define TX_SPL       0x00 // parameter for configTxPipe(..): enable static payload for PTX
#define CRC0         0x00 // parameter for configCRC(crc): disable CRC
#define CRC1         0x01 // parameter for configCRC(crc): 1 byte CRC
#define CRC2         0x02 // parameter for configCRC(crc): 2 byte CRC
#define MBPS1        0x01 // parameter for configSpeed(speed): 1Mbps
#define MBPS2        0x02 // parameter for configSpeed(speed): 2Mbps
#define PWR_BIT      0x02 // parameter for configSpeed(speed): 2Mbps

#define ADR_WIDTH3   0x03 // parameter for confAdrWidth(width): 3 byte
#define ADR_WIDTH4   0x03 // parameter for confAdrWidth(width): 4 byte
#define ADR_WIDTH5   0x03 // parameter for confAdrWidth(width): 5 byte

#define RFM7x_MAX_PACKET_LEN 32// max value is 32
#define RFM7x_BEGIN_INIT_WAIT_MS 0 // pause before Init Registers: 3000ms after RFM73 power up, but Raspberry Pi boots longer than 3s so can be set to 0ms here
#define RFM7x_END_INIT_WAIT_MS 100 // pause after init registers
#define RFM7x_CS_DELAY 0 // wait ms after CS pin state change

// SPI commands
#define RFM7x_CMD_READ_REG 0x00 // Define read command to register
#define RFM7x_CMD_WRITE_REG 0x20 // Define write command to register
#define RFM7x_CMD_RD_RX_PLOAD 0x61 // Define RX payload command
#define RFM7x_CMD_WR_TX_PLOAD 0xA0 // Define TX payload command
#define RFM7x_CMD_FLUSH_TX 0xE1 // Define flush TX register command
#define RFM7x_CMD_FLUSH_RX 0xE2 // Define flush RX register command
#define RFM7x_CMD_REUSE_TX_PL 0xE3 // Define reuse TX payload register command
#define RFM7x_CMD_W_TX_PAYLOAD_NOACK 0xb0 // Define TX payload NOACK command
#define RFM7x_CMD_W_ACK_PAYLOAD 0xa8 // Define Write ack command
#define RFM7x_CMD_ACTIVATE 0x50 // Define feature activation command
#define RFM7x_CMD_RX_PL_WID 0x60 // Define received payload width command
#define RFM7x_CMD_NOP_NOP 0xFF // Define No Operation, might be used to read status register

// Register addresses
#define RFM7x_REG_CONFIG 0x00 // 'Config' register address
#define RFM7x_REG_EN_AA 0x01 // 'Enable Auto Acknowledgment' register address
#define RFM7x_REG_EN_RXADDR 0x02 // 'Enabled RX addresses' register address
#define RFM7x_REG_SETUP_AW 0x03 // 'Setup address width' register address
#define RFM7x_REG_SETUP_RETR 0x04 // 'Setup Auto. Retrans' register address
#define RFM7x_REG_RF_CH 0x05 // 'RF channel' register address
#define RFM7x_REG_RF_SETUP 0x06 // 'RF setup' register address
#define RFM7x_REG_STATUS 0x07 // 'Status' register address
#define RFM7x_REG_OBSERVE_TX 0x08 // 'Observe TX' register address
#define RFM7x_REG_CD 0x09 // 'Carrier Detect' register address
#define RFM7x_REG_RX_ADDR_P0 0x0A // 'RX address pipe0' register address
#define RFM7x_REG_RX_ADDR_P1 0x0B // 'RX address pipe1' register address
#define RFM7x_REG_RX_ADDR_P2 0x0C // 'RX address pipe2' register address
#define RFM7x_REG_RX_ADDR_P3 0x0D // 'RX address pipe3' register address
#define RFM7x_REG_RX_ADDR_P4 0x0E // 'RX address pipe4' register address
#define RFM7x_REG_RX_ADDR_P5 0x0F // 'RX address pipe5' register address
#define RFM7x_REG_TX_ADDR 0x10 // 'TX address' register address
#define RFM7x_REG_RX_PW_P0 0x11 // 'RX payload width, pipe0' register address
#define RFM7x_REG_RX_PW_P1 0x12 // 'RX payload width, pipe1' register address
#define RFM7x_REG_RX_PW_P2 0x13 // 'RX payload width, pipe2' register address
#define RFM7x_REG_RX_PW_P3 0x14 // 'RX payload width, pipe3' register address
#define RFM7x_REG_RX_PW_P4 0x15 // 'RX payload width, pipe4' register address
#define RFM7x_REG_RX_PW_P5 0x16 // 'RX payload width, pipe5' register address
#define RFM7x_REG_FIFO_STATUS 0x17 // 'FIFO Status Register' register address
#define RFM7x_REG_DYNPD 0x1c // 'Enable dynamic payload length' register address
#define RFM7x_REG_FEATURE 0x1d // 'Feature' register address

// Interrupts
#define RFM7x_IRQ_STATUS_RX_DR 0x40 // Status bit RX_DR IRQ
#define RFM7x_IRQ_STATUS_TX_DS 0x20 // Status bit TX_DS IRQ
#define RFM7x_IRQ_STATUS_MAX_RT 0x10 // Status bit MAX_RT IRQ
#define RFM7x_IRQ_STATUS_TX_FULL 0x01 
#define RFM7x_PIN_PRIM_RX 0x01
#define RFM7x_PIN_POWER 0x02

// FIFO status
#define RFM7x_FIFO_STATUS_TX_REUSE 0x40
#define RFM7x_FIFO_STATUS_TX_FULL 0x20
#define RFM7x_FIFO_STATUS_TX_EMPTY 0x10
#define RFM7x_FIFO_STATUS_RX_FULL 0x02
#define RFM7x_FIFO_STATUS_RX_EMPTY 0x01

void setModeTX(void);
void setModeRX(void);
uint8_t getMode(void);

void setChannel(uint8_t cnum);
uint8_t getChannel(void);

uint8_t transmitSPI(uint8_t val);
uint8_t readRegVal(uint8_t cmd);
uint8_t writeRegVal(uint8_t cmd, uint8_t val);
uint8_t writeRegPgmBuf(uint8_t * cmdbuf, uint8_t len);
void readRegBuf(uint8_t reg, uint8_t * buf, uint8_t len);
void selectBank(uint8_t bank);

// adr has to be 5 bytes!
uint8_t configRxPipe(uint8_t pipe_nr, uint8_t *adr, uint8_t plLen, uint8_t en_aa);
void enableRxPipe(uint8_t pipe_nr);
uint8_t writeRegCmdBuf(uint8_t cmd, uint8_t * buf, uint8_t len);
void disableRxPipe(uint8_t pipe_nr);

// adr has to be 5 bytes!
void configTxPipe(uint8_t * adr, uint8_t pltype);
void flushTxFIFO();
void flushRxFIFO();
uint8_t receivePayload(uint8_t *payload);
uint8_t sendPayload(const uint8_t * payload, const uint8_t _len, const uint8_t toAck);

void SPI_MasterInit(void);
void SPI_MasterTransmit(char cData);

void setCE();
void resetCE();
void setCSN();
void resetCSN();

#define MIN(a,b)								((a) < (b) ? (a) : (b))

template <class Iterator>
TransmitResult send(const Iterator begin, const Iterator end, bool requestAck, bool usePacketNumber, const uint8_t firstPacketNumber)
{	
	uint8_t *buff = new uint8_t[end-begin];
	copy(begin, end, buff);
	
	TransmitResult result = send(buff, end-begin, requestAck, usePacketNumber, firstPacketNumber);
	delete [] buff;
	
	return result;
}

template <class Iterator>
TransmitResult send(const uint32_t &targetAddress, const Iterator begin, const Iterator end, bool requestAck, bool usePacketNumber, const uint8_t firstPacketNumber)
{	
	uint8_t *buff = new uint8_t[end-begin];
	copy(begin, end, buff);
	
	TransmitResult result = send(targetAddress, buff, end-begin, requestAck, usePacketNumber, firstPacketNumber);
	delete [] buff;
	
	return result;
}

template <class Iterator>
TransmitResult send(const uint8_t channel, const uint32_t &targetAddress, const Iterator begin, const Iterator end, bool requestAck, bool usePacketNumber, const uint8_t firstPacketNumber)
{	
	uint8_t *buff = new uint8_t[end-begin];
	copy(begin, end, buff);
	
	TransmitResult result = send(channel, targetAddress, buff, end-begin, requestAck, usePacketNumber, firstPacketNumber);
	delete [] buff;
	
	return result;
}

}

#endif

