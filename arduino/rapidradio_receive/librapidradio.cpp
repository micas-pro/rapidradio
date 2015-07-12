/*
	The rapidradio project

	Author: Michal Okulski, the rapidradio team
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


#include <string.h>
#include "rapidradio.h"
#include <SPI.h>



SIGNAL(PCINT2_vect) {
  rapidradio::irq = true;
}

namespace rapidradio
{

volatile bool irq = false;

bool initRegisters();

//commands
const uint8_t PROGMEM RFM73_cmd_adrRX0[] = { (0x20 | 0x0A), 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
const uint8_t PROGMEM RFM73_cmd_adrTX[]  = { (0x20 | 0x10), 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
const uint8_t PROGMEM RFM73_cmd_adrRX1[] = { (0x20 | 0x0B), 0x35, 0x43, 0x10, 0x10, 0x02};
const uint8_t PROGMEM RFM73_cmd_switch_cfg[] = { 0x50, 0x53 }; // zmiana rejestru
const uint8_t PROGMEM RFM73_cmd_flush_rx[] = { 0xe2, 0x00 }; // flush RX FIFO
const uint8_t PROGMEM RFM73_cmd_flush_tx[] = { 0xe1, 0x00 }; // flush TX FIFO
const uint8_t PROGMEM RFM73_cmd_activate[] = { 0x50, 0x73 }; // aktywacja
const uint8_t PROGMEM RFM73_cmd_tog1[] = { (0x20 | 0x04), 0xd9 | 0x06, 0x9e, 0x86, 0x0b };
const uint8_t PROGMEM RFM73_cmd_tog2[] = { (0x20 | 0x04), 0xd9 & ~0x06, 0x9e, 0x86, 0x0b}; //set1[4]

//RFM73
//************ Bank0 register initialization commands
const uint8_t PROGMEM RFM73_bank0Init[][2] = {
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
const uint8_t PROGMEM RFM73_bank1Init[][5] = {
  { (0x20 | 0x00), 0x40, 0x4B, 0x01, 0xE2 },
  { (0x20 | 0x01), 0xC0, 0x4B, 0x00, 0x00 },
  { (0x20 | 0x02), 0xD0, 0xFC, 0x8C, 0x02 },
  { (0x20 | 0x03), 0x99, 0x00, 0x39, 0x41 },
  { (0x20 | 0x04), 0xD9, 0x96, 0x82, 0x1B },
  { (0x20 | 0x05), 0x24, 0x06, 0x7F, 0xA6 },
  { (0x20 | 0x06), 0x00, 0x00, 0x00, 0x00 },
  { (0x20 | 0x07), 0x00, 0x00, 0x00, 0x00 },
  { (0x20 | 0x08), 0x00, 0x00, 0x00, 0x00 },
  { (0x20 | 0x09), 0x00, 0x00, 0x00, 0x00 },
  { (0x20 | 0x0a), 0x00, 0x00, 0x00, 0x00 },
  { (0x20 | 0x0b), 0x00, 0x00, 0x00, 0x00 },
  { (0x20 | 0x0C), 0x00, 0x12, 0x73, 0x00 },
  { (0x20 | 0x0D), 0x46, 0xb4, 0x80, 0x00 }
};

//Bank1 register 14
const uint8_t PROGMEM RFM73_bank1R0EInit[] = {
  (0x20 | 0x0E), 0x41, 0x20, 0x08, 0x04, 0x81, 0x20, 0xCF, 0xF7, 0xFE, 0xFF, 0xFF
};


void SPI_MasterInit(void)
{
  // set pin 10 as the slave select for the digital pot:
  const int slaveSelectPin = 10;
  // set the slaveSelectPin as an output:
  pinMode (slaveSelectPin, OUTPUT);
  // initialize SPI:
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);
}

bool init()
{
  pinMode(CE, OUTPUT);
  pinMode(CSN, OUTPUT);
  pinMode(IRQ, INPUT_PULLUP);

  // IRQ pin 7 is PCINT23
  PCICR |= _BV(PCIE2);
  PCMSK2 |= _BV(PCINT23);

  CE_LOW;
  CSN_HIGH;

  SPI_MasterInit();

  _delay_ms(RFM73_BEGIN_INIT_WAIT_MS);
  return initRegisters();
}

bool initRegisters()
{
  // init bank 0 registers
  selectBank(0);

  // !! The last two regs in the bank0Init list will be handled later
  for (int i = 0; i < 20; i++)
    writeRegVal(pgm_read_byte(&RFM73_bank0Init[i][0]), pgm_read_byte(&RFM73_bank0Init[i][1]));

  // init address registers in bank 0
  writeRegPgmBuf((uint8_t *)RFM73_cmd_adrRX0, sizeof(RFM73_cmd_adrRX0));
  writeRegPgmBuf((uint8_t *)RFM73_cmd_adrRX1, sizeof(RFM73_cmd_adrRX1));
  writeRegPgmBuf((uint8_t *)RFM73_cmd_adrTX, sizeof(RFM73_cmd_adrTX));

  // activate Feature register
  if (!readRegVal(RFM73_REG_FEATURE))
    writeRegPgmBuf((uint8_t *)RFM73_cmd_activate, sizeof(RFM73_cmd_activate));

  // now set Registers 1D and 1C
  writeRegVal(pgm_read_byte(&RFM73_bank0Init[22][0]), pgm_read_byte(&RFM73_bank0Init[22][1]));
  writeRegVal(pgm_read_byte(&RFM73_bank0Init[21][0]), pgm_read_byte(&RFM73_bank0Init[21][1]));

  // init bank 1 registers
  selectBank(1);

  for (int i = 0; i < 14; i++)
    writeRegPgmBuf((uint8_t *)RFM73_bank1Init[i], sizeof(RFM73_bank1Init[i]));

  // set ramp curve
  writeRegPgmBuf((uint8_t *)RFM73_bank1R0EInit, sizeof(RFM73_bank1R0EInit));

  // do we have to toggle some bits here like in the example code?
  writeRegPgmBuf((uint8_t *)RFM73_cmd_tog1, sizeof(RFM73_cmd_tog1));
  writeRegPgmBuf((uint8_t *)RFM73_cmd_tog2, sizeof(RFM73_cmd_tog2));


  _delay_ms(RFM73_END_INIT_WAIT_MS);

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
  return SPI.transfer(val);
}

void selectBank(uint8_t bank)
{
  uint8_t tmp = readRegVal(0x07) & 0x80;
  if (bank)
  {
    if (!tmp)
      writeRegPgmBuf((uint8_t *)RFM73_cmd_switch_cfg, sizeof(RFM73_cmd_switch_cfg));
  }
  else
  {
    if (tmp)
      writeRegPgmBuf((uint8_t *)RFM73_cmd_switch_cfg, sizeof(RFM73_cmd_switch_cfg));
  }
}

void setModeRX(void)
{
  uint8_t val;
  writeRegPgmBuf((uint8_t *)RFM73_cmd_flush_rx, sizeof(RFM73_cmd_flush_rx));
  val = readRegVal(RFM73_REG_STATUS);
  writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_STATUS, val);
  CE_LOW;
  val = readRegVal(RFM73_REG_CONFIG);
  val |= RFM73_PIN_PRIM_RX;
  writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_CONFIG, val);
  CE_HIGH;
}


void setModeTX(void)
{
  uint8_t val;
  writeRegPgmBuf((uint8_t *)RFM73_cmd_flush_tx, sizeof(RFM73_cmd_flush_tx));
  val = readRegVal(RFM73_REG_STATUS);
  writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_STATUS, val);
  CE_LOW;
  val = readRegVal(RFM73_REG_CONFIG);
  val &= ~RFM73_PIN_PRIM_RX;
  writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_CONFIG, val);

  // Don't turn the radio on to save power
  //CE_HIGH;
}

uint8_t getMode(void)
{
  return readRegVal(RFM73_REG_CONFIG) & RFM73_PIN_PRIM_RX;
}

void setChannel(uint8_t cnum)
{
  writeRegVal( RFM73_CMD_WRITE_REG | RFM73_REG_RF_CH, cnum);
}

uint8_t getChannel(void)
{
  return readRegVal(RFM73_REG_RF_CH);
}

void setPower(uint8_t pwr)
{
  if (pwr > 3) return;
  uint8_t tmp = readRegVal(RFM73_REG_RF_SETUP);
  tmp &= 0xF9;
  tmp |= pwr << 1;

  writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_RF_SETUP, tmp);
}

uint8_t readRegVal(uint8_t cmd)
{
  uint8_t res;
  CSN_LOW;
  _delay_ms(RFM73_CS_DELAY);
  transmitSPI(cmd);
  res = transmitSPI(0);
  CSN_HIGH;
  _delay_ms(RFM73_CS_DELAY);
  return res;
}

uint8_t writeRegVal(uint8_t cmd, uint8_t val)
{
  CSN_LOW;
  _delay_ms(RFM73_CS_DELAY);
  transmitSPI(cmd);
  transmitSPI(val);
  CSN_HIGH;
  _delay_ms(RFM73_CS_DELAY);
  return 1;
}

void readRegBuf(uint8_t reg, uint8_t * buf, uint8_t len)
{
  uint8_t byte_ctr;
  CSN_LOW;
  _delay_ms(RFM73_CS_DELAY);
  transmitSPI(reg);
  for (byte_ctr = 0; byte_ctr < len; byte_ctr++)
    buf[byte_ctr] = transmitSPI(0);
  CSN_HIGH;
  _delay_ms(RFM73_CS_DELAY);
}



uint8_t writeRegPgmBuf(uint8_t * cmdbuf, uint8_t len)
{
  CSN_LOW;
  _delay_ms(RFM73_CS_DELAY);
  while (len--) transmitSPI(pgm_read_byte(cmdbuf++));
  CSN_HIGH;
  _delay_ms(RFM73_CS_DELAY);
  return 1;
}

uint8_t writeRegCmdBuf(uint8_t cmd, uint8_t * buf, uint8_t len)
{

  CSN_LOW;
  _delay_ms(RFM73_CS_DELAY);
  transmitSPI(cmd);
  while (len--) transmitSPI(*(buf++));
  CSN_HIGH;
  _delay_ms(RFM73_CS_DELAY);
  return 1;
}

// Important! adr has to be 5-bytes length even if MSB bytes are unused
uint8_t configRxPipe(uint8_t pipe_nr, uint8_t *adr, uint8_t plLen, uint8_t en_aa)
{

  uint8_t tmp;
  uint8_t nr = pipe_nr - 1;

  if (plLen > 32 || nr > 5 || en_aa > 1)
    return 0;

  // write address
  if (nr < 2)   // full length for rx pipe 0 an 1
    writeRegCmdBuf(RFM73_CMD_WRITE_REG | (RFM73_REG_RX_ADDR_P0 + nr), adr, 5);
  else // only LSB for pipes 2..5
    writeRegVal(RFM73_CMD_WRITE_REG | (RFM73_REG_RX_ADDR_P0 + nr), adr[0]);

  // static
  if (plLen) {
    // set payload len
    writeRegVal(RFM73_CMD_WRITE_REG | (RFM73_REG_RX_PW_P0 + nr), plLen);
    // set EN_AA bit
    tmp = readRegVal(RFM73_REG_EN_AA);
    if (en_aa)
      tmp |= 1 << nr;
    else
      tmp &= ~(1 << nr);
    writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_EN_AA, tmp);
    // clear DPL bit
    tmp = readRegVal(RFM73_REG_DYNPD);
    tmp &= ~(1 << nr);
    writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_DYNPD, tmp);
    // set Enable pipe bit
    enableRxPipe(nr);
  }
  // dynamic
  else
  {
    // set payload len to default
    writeRegVal(RFM73_CMD_WRITE_REG | (RFM73_REG_RX_PW_P0 + nr), 0x20);
    // set EN_AA bit
    tmp = readRegVal(RFM73_REG_EN_AA);
    tmp |= 1 << nr;
    writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_EN_AA, tmp);
    // set DPL bit
    tmp = readRegVal(RFM73_REG_DYNPD);
    tmp |= 1 << nr;
    writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_DYNPD, tmp);
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
  tmp = readRegVal(RFM73_REG_EN_RXADDR);
  tmp |= 1 << nr;
  writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_EN_RXADDR, tmp);
}

void disableRxPipe(uint8_t pipe_nr)
{
  uint8_t nr = pipe_nr - 1;
  if (nr > 5) return;
  uint8_t tmp;
  // set Enable pipe bit
  tmp = readRegVal(RFM73_REG_EN_RXADDR);
  tmp &= ~(1 << nr);
  writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_EN_RXADDR, tmp);
}

void configTxPipe(uint8_t * adr, uint8_t pltype)
{
  // write TX address
  writeRegCmdBuf(RFM73_CMD_WRITE_REG | RFM73_REG_TX_ADDR, adr, 5);
  // write RX0 address
  writeRegCmdBuf(RFM73_CMD_WRITE_REG | RFM73_REG_RX_ADDR_P0, adr, 5);
  // set static or dynamic payload
  uint8_t tmp;
  tmp = readRegVal(RFM73_REG_DYNPD);
  if (pltype == TX_DPL) // dynamic
    tmp |= 1;
  else
    tmp &= ~(1 << 0);
  writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_DYNPD, tmp);
}

uint8_t sendPayload(const uint8_t * payload, const uint8_t _len, const uint8_t toAck)
{
  // turn on the radio
  CE_HIGH;

  // check TX_FIFO
  uint8_t status;
  status = readRegVal(RFM73_REG_FIFO_STATUS);
  if (status & RFM73_FIFO_STATUS_TX_FULL)
  {
    return 0;
  }

  // send payload
  CSN_LOW;
  _delay_ms(RFM73_CS_DELAY);
  if (toAck == -1)
    transmitSPI(RFM73_CMD_W_ACK_PAYLOAD);
  else if (toAck == 0)
    transmitSPI(RFM73_CMD_W_TX_PAYLOAD_NOACK);
  else
    transmitSPI(RFM73_CMD_WR_TX_PLOAD);

  uint8_t len = _len;
  while (len--) transmitSPI(*(payload++));
  CSN_HIGH;
  _delay_ms(RFM73_CS_DELAY);
  return 1;
}

uint8_t receivePayload(uint8_t *payload)
{
  uint8_t len;
  // check RX_FIFO
  uint8_t status;
  status = readRegVal(RFM73_REG_STATUS);
  if (status & RFM73_IRQ_STATUS_RX_DR) { // RX_DR

    uint8_t fifo_sta;
    len = readRegVal(RFM73_CMD_RX_PL_WID); // Payload width
    readRegBuf(RFM73_CMD_RD_RX_PLOAD, payload, len);
    fifo_sta = readRegVal(RFM73_REG_FIFO_STATUS);

    if (fifo_sta & RFM73_FIFO_STATUS_RX_EMPTY) {
      status |= 0x40 & 0xCF; // clear status bit rx_dr
      writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_STATUS, status);
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
  writeRegPgmBuf((uint8_t *)RFM73_cmd_flush_tx, sizeof(RFM73_cmd_flush_tx));
}

void flushRxFIFO()
{
  writeRegPgmBuf((uint8_t *)RFM73_cmd_flush_rx, sizeof(RFM73_cmd_flush_rx));
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
  uint8_t status = readRegVal(RFM73_REG_STATUS);
  status |= PWR_BIT;
  writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_STATUS, status);
}

void turnOff()
{
  uint8_t status = readRegVal(RFM73_REG_STATUS);
  status &= ~PWR_BIT;
  writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_STATUS, status);
  CE_LOW;
}

bool checkStatusForMissingIRQ(uint8_t &status)
{
  status = readRegVal(RFM73_REG_STATUS);
  return status != 14;
}

TransmitResult internalSendPacket(const uint8_t *buff, const size_t &length, const uint32_t maxTimeoutUs, bool requestAck)
{
  irq = false;
  const size_t toSendLength = MIN(length, RFM73_MAX_PACKET_LEN);
  sendPayload(buff, toSendLength, requestAck ? 1 : 0);

  TransmitResult result;

  uint32_t i = 0;
  uint8_t status = 0;
  bool readStatus = true;
  while (!irq && i++ < maxTimeoutUs)
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
    status = readRegVal(RFM73_REG_STATUS);
  }

  if (status & RFM73_IRQ_STATUS_TX_DS)
  {
    result.status = Success;
    result.bytesSent = toSendLength;
  }
  else if (status & RFM73_IRQ_STATUS_MAX_RT)
  {
    result.status = MaxRT;
    result.bytesSent = 0;
  }
  else if (status & RFM73_IRQ_STATUS_TX_FULL)
  {
    result.status = FifoFull;
    result.bytesSent = 0;
  }

  if (i >= maxTimeoutUs)
  {
    result.status = Unknown;
    result.bytesSent = 0;
  }

  // Reset error flags
  writeRegVal(RFM73_CMD_WRITE_REG | RFM73_REG_STATUS, status);

  return result;
}

TransmitResult& operator +=(TransmitResult &a, const TransmitResult b)
{
  a.status = b.status;
  a.bytesSent += b.bytesSent;
  return a;
}

TransmitResult internalSend(const uint8_t *buff, const size_t &length, bool requestAck)
{
  Serial.print("Start sending ");
  Serial.print(length);
  Serial.println('B');

  uint8_t setupRetr = readRegVal(RFM73_REG_SETUP_RETR);
  uint32_t maxTimeoutUs = (uint32_t)(((setupRetr & 0b11110000) >> 4) + 1) * 250UL * (uint32_t)(setupRetr & 0b00001111);
  maxTimeoutUs += maxTimeoutUs / 2UL;

  if (maxTimeoutUs == 0) maxTimeoutUs = 250;

  CE_HIGH;

  TransmitResult result;
  result.status = Success;
  result.bytesSent = 0;

  const int maxPacketAttempts = 50;
  for (size_t i = 0; i < length; i += RFM73_MAX_PACKET_LEN)
  {
    int packetAttempts = maxPacketAttempts;
    while (packetAttempts--)
    {
      if (result.status != Success)
      {
        Serial.print("Packet send error: ");
        Serial.print((int)result.status);
        Serial.print(", retrying (");
        Serial.print(maxPacketAttempts - packetAttempts);
        Serial.print('/');
        Serial.print(maxPacketAttempts);
        Serial.println(")...");
      }

      result += internalSendPacket(buff + i, MIN(length - i, RFM73_MAX_PACKET_LEN), maxTimeoutUs, requestAck);

      if (result.status == Success)
      {
        break;
      }
      else if (result.status == FifoFull)
      {
        flushTxFIFO();
      }

      _delay_ms(100);
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
TransmitResult send(const uint8_t *buff, const size_t &length, bool requestAck)
{
  if (getMode() != MODE_PTX)
  {
    setModeTX();
  }

  return internalSend(buff, length, requestAck);
}

// if length is greater than 32 bytes, then buff will be splitted into 32-bytes long packets
TransmitResult send(const uint32_t &targetAddress, const uint8_t *buff, const size_t &length, bool requestAck)
{
  if (getMode() != MODE_PTX)
  {
    setModeTX();
  }

  uint8_t adr[5];
  memcpy(adr, &targetAddress, 4);
  adr[4] = 0;
  configTxPipe(adr, TX_DPL);

  return internalSend(buff, length, requestAck);
}

// if length is greater than 32 bytes, then buff will be splitted into 32-bytes long packets
TransmitResult send(const uint8_t channel, const uint32_t &targetAddress, const uint8_t *buff, const size_t &length, bool requestAck)
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

  return internalSend(buff, length, requestAck);
}



void startListening(const uint32_t &localAddress)
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
    Serial.println("ERROR: cannot config RX Pipe");
  }
  else
  {
    // Enable radio unit in the RFM73
    CE_HIGH;
  }
}

void startListening(const uint8_t channel, const uint32_t &localAddress)
{
  setChannel(channel);
  startListening(localAddress);
}

// buff has to be at least 32 bytes!
bool received(uint8_t *buff, uint8_t &length)
{
  length = receivePayload(buff);
  
  if (!length)
  {
    // no more data to read - reset interrupt flag
    irq = false;
  }
  
  return length > 0;
}

}
