/*
	The rapidradio project
	
	rapidradio command line tool
	
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

#include <errno.h>
#include "rapidradio.h"
#include <cstring>
#include <iostream>
#include <vector>
#include <signal.h>
#include <stdio.h>
#include <utility>
#include <cstddef>


using namespace std;
using namespace rapidradio;

const uint8_t transmissionEndToken[] = {0xb3, 0x67, 0x2b, 0x26, 0xa2, 0x33, 0x43, 0x25, 0x80, 0x50, 0x80, 0xd5, 0x6e, 0xad, 0x94, 0x85,
										0x0c, 0xec, 0xf8, 0x0f, 0x57, 0x8c, 0x48, 0xda, 0x95, 0x2d, 0x09, 0xbf, 0xc0, 0x04, 0xd5, 0x40};
sig_atomic_t signaled = 0;

static volatile bool _irq = false;

static uint8_t singlePacket[32];
static uint8_t singlePacketLength = 0;

void irq (void)
{
	fprintf(stderr, "IRQ!\n");
	_irq = true;
}

void my_handler (int param)
{
	signaled = 1;
}

struct Settings
{
	uint8_t channel;
	uint32_t targetAddress;
	bool listen;
	bool discard;
	vector<pair<uint8_t, uint8_t> > registers;
	bool ack;
	bool verbose;
	string byteSpecifier;
	bool newlineAfterPacket;
	bool packetNumbering;
};

bool parseParams(const int argc, const char **argv, Settings &settings)
{
	settings.channel = 55;
	settings.targetAddress = 0xAABB0001UL;
	settings.listen = false;
	settings.discard = false;
	settings.ack = true;
	settings.verbose = false;
	settings.byteSpecifier = "%c";
	settings.newlineAfterPacket = false;
	settings.packetNumbering = true;
	
	for (int i=1; i<argc; i++)
	{		
		if (argv[i] == string("-l"))
		{
			settings.listen = true;
		}
		else if (string(argv[i]).substr(0, 3) == "-a=")
		{
			string sadr = string(argv[i]).substr(3);
			uint32_t adr = 0xCCCD1102UL;
			if (sscanf(sadr.c_str(), "%x", &adr))
			{
				settings.targetAddress = adr;
			}
		}
		else if (argv[i] == string("-i"))
		{
			settings.packetNumbering = false;
		}
		else if (string(argv[i]).substr(0, 3) == "-c=")
		{
			string schannel = string(argv[i]).substr(3);
			int channel = 0;
			if (sscanf(schannel.c_str(), "%i", &channel))
			{
				if (channel < 0) channel = 0;
				if (channel > 83) channel = 83;
				settings.channel = (uint8_t)channel;
			}
		}
		else if (argv[i] == string("-nack"))
		{
			settings.ack = false;
		}
		else if (argv[i] == string("-ld"))
		{
			settings.listen = true;
			settings.discard = true;
		}
		else if (argv[i] == string("-v"))
		{
			settings.verbose = true;
		}
		else if (argv[i] == string("-x"))
		{
			settings.byteSpecifier = "%.2X ";
		}
		else if (argv[i] == string("-n"))
		{
			settings.newlineAfterPacket = true;
		}
		else if (argv[i] == string("-?") || argv[i] == string("--help"))
		{
			return false;
		}
		else if (strlen(argv[i]) > 2 && argv[i][0] == '-' && argv[i][1] == 'r')
		{
			string cmd(argv[i] + 2);
			size_t j = cmd.find('=');
			if (j != string::npos)
			{
				string reg = cmd.substr(0, j);
				string val = cmd.substr(j + 1);
				
				int ireg, ival;
				if (sscanf(reg.c_str(), "%i", &ireg) && sscanf(val.c_str(), "%i", &ival))
				{
					settings.registers.push_back(make_pair((uint8_t)ireg, (uint8_t)ival));
				}
			}
		}
		else if (string(argv[i]).substr(0, 3) == string("-p="))
		{
			string sbytes(string(argv[i]).substr(3));
			if (sbytes.length() > 64)
			{
				printf("Single packet cannot be longer than 32 bytes (64 hex characters).\n");
				return false;
			}
			
			if (sbytes.length() % 2)
			{
				printf("Single packet is formatted as hex, so it must contain even number of hex characters (two per each byte).\n");
				return false;
			}
			
			for (uint8_t i=0;i<sbytes.length()/2;i++)
			{
				string ch(sbytes.substr(i*2, 2));
				unsigned int x;
				if (sscanf(ch.c_str(), "%2X", &x))
				{
					singlePacket[i] = (uint8_t)x;
				}
				else
				{
					printf("Unable to parse %s as hex byte.\n", ch.c_str());
					return false;
				}
			}
			
			singlePacketLength = (uint8_t)(sbytes.length() / 2);
		}
		else
		{
			printf("Unknown parameter: %s\n", argv[i]);
			return false;
		}
	}
	
	return true;
}

void usage()
{
	printf("rapidradio command-line tool\n\n");
	printf("Usage:\n");
	printf("sudo ./rapidradio [-l] [-ld] [-v] [-x] [-n] [-i] [-a=4_byte_hex_address] [-c=channel] [-nack] [-rN=1_byte_value] [-p=single_hex_packet]\n\n");
	printf("-l\t\t\tListen mode - rapidradio will listen for incomming packets send to address specified by -a and channel set by -c.\n");
	printf("-ld\t\t\tListen mode with discarded output - rapidradio will just print a dot '.' every ~10KiB received.\n");
	printf("-v\t\t\tVerbose output.\n");
	printf("-x\t\t\tPrint received bytes as 2-character hex - useful when debugging or sniffing received radio packets.\n");
	printf("-n\t\t\tPrint a newline after each received packet. Useful together with the -x option.\n");
	printf("-i\t\t\tIgnore packet number byte. Use for real raw transmissions.\n");
	printf("-a=address\t\tIt's a 4-byte hex address of target device (when sending) or this device address (when listening). Example: -a=123456AB\n");
	printf("-c=channel\t\tRadio channel number, a value from range: 1 to 83.\n");
	printf("-nack\t\t\tDon't expect ACKs when sending. Speeds up transmission but gives no guarantee that all packets reach the target device.\n");
	printf("-rN=value\t\tManual setting for particular RFM's register. E.g. usage: to change auto-retry behavior to just 2 retries and 4ms interval use: -r4=242 which means: put 242 (decimal) value into the RFM's register no. 4 (decimal).\n");
	printf("-p=single_hex_packet\tSends just one single packet. It must contain 1-32 bytes (hex formatted). Example: -p=AA1100FF will send 4-bytes long packet.\n");
	printf("\n");
}

void listen(Settings &settings)
{
	if (settings.verbose) fprintf(stderr, "Listening...\n");
	startListening(settings.channel, settings.targetAddress);
	int32_t packetCount = -1;
	uint8_t packetNumber = 0;
	bool stop = false;
	while (!signaled && !stop)
	{
		// active wait for IRQ signal - it's faster than any user level interrupt implementation... ;(
		while (bcm2835_gpio_lev(IRQ) != LOW && !signaled)
		{
			delayMicroseconds(1);
		}
		
		_irq = false;
		
		if (signaled) break;
		
		uint8_t buff[32];
		uint8_t length = 0;
		
		// process all packets (if any)
		// 'while' loop instead of just 'if' statement because the RFM7x can buffer up to 3 received packets
		while (received(buff, length) && length)
		{
			++packetCount;
			if (!settings.discard)
			{
				// check if the packet is the end transmission indicator
				bool isEndPacket = true;
				for(uint8_t i=0; i<length; i++)
				{
					if (buff[i] != transmissionEndToken[i])
					{
						isEndPacket = false;
						break;
					}
				}
				
				if (isEndPacket)
				{
					if (settings.verbose) fprintf(stderr, "Received transmission end packet.\n");
					stop = true;
					break;
				}
				
				if (settings.packetNumbering)
				{
					// check packet number if packets are missing 										
					if (buff[0] > packetNumber)
					{				
						if (settings.verbose) fprintf(stderr, "Missing packet(s). Got packet %u, expected %u\n", buff[0], packetNumber);
						packetNumber = buff[0];
					}				
					// check packet number to discard duplicated packets (rare, but possible when ACK is used)										
					else if (buff[0] < packetNumber)
					{				
						if (settings.verbose) fprintf(stderr, "Duplicated packet %u, expected %u\n", buff[0], packetNumber);
						packetCount--;
						continue;
					}
				}
				
				// skip first byte
				for(uint8_t i = settings.packetNumbering ? 1 : 0; i<length; i++)
				{
					printf(settings.byteSpecifier.c_str(), buff[i]);
				}
				
				if (settings.newlineAfterPacket)
				{
					printf("\n");
				}
				
				packetNumber++;
			}	
			
			// print dot '.' every ~10 KiB
			if (settings.discard && (packetCount % 330U) == 0)
			{
				fprintf(stderr, ".");
			}				
		}			
	}
}

void transmit(Settings &settings)
{
	freopen(NULL, "rb", stdin);
	uint32_t totalSent = 0;
	
	vector<uint8_t> input;
	while (!feof(stdin))
	{
		uint8_t buff[1024];
		size_t read = fread(buff, 1, 1024, stdin);
		input.insert(input.end(), buff, buff + read);						
		totalSent += (uint32_t)read;
	}
	
	if (settings.verbose) fprintf(stderr, "Read %uB (%uKiB, %uMiB) from input.\n", input.size(), input.size()/1024U, input.size()/(1024U*1024U));	
	
	TransmitResult result = send(settings.channel, settings.targetAddress, input.begin(), input.end(), settings.ack, settings.packetNumbering, 0);
	if (result.status != Success)
	{
		fprintf(stderr, "An error occured while sending! (%u)\n", (int)result.status);
	}
	
	if (result.status == Success)
	{
		totalSent = result.bytesSent;
		// send transmission end special packet
		result = send(settings.channel, settings.targetAddress, transmissionEndToken, sizeof(transmissionEndToken), settings.ack);
		if (result.status != Success)
		{
			fprintf(stderr, "An error occured while sending last packet! (%u)\n", (int)result.status);
		}				
	}
	
	if (settings.verbose) fprintf(stderr, "Sent %uB (%uKiB, %uMiB)\n", totalSent, totalSent/1024U, totalSent/(1024U*1024U));
}

int main(const int argc, const char **argv)
{
	Settings settings;
	if (!parseParams(argc, argv, settings))
	{
		usage();
		return 1;
	}

	void (*prev_handler)(int);
	prev_handler = signal (SIGINT, my_handler);
	
	if (!bcm2835_init())
	{
	    fprintf(stderr, "BCM2835 library init failed.\n");
	    return 1;
	}		
	
	if (!init())
	{
		fprintf(stderr, "rapidradio init failed.\n");
	    return 1;
	}
	
	// use custom RFM7x register settings (if any)
	for (size_t i=0;i<settings.registers.size();i++)
	{
		if (settings.verbose) fprintf(stderr, "REG%u = %.2X\n", settings.registers[i].first, settings.registers[i].second);
		writeRegVal(RFM7x_CMD_WRITE_REG | settings.registers[i].first, settings.registers[i].second); 
	}
				
	if (settings.verbose) fprintf(stderr, "Address = %.8X\nChannel = %u\nPacket numbering %s\n", settings.targetAddress, settings.channel, settings.packetNumbering ? "enabled" : "disabled");
		
	turnOn();
	
	if (singlePacketLength)
	{
		if (settings.verbose) fprintf(stderr, "Sending single packet, packet numbering and transmission end packet skipped.\n");
		TransmitResult result = send(settings.channel, settings.targetAddress, singlePacket, singlePacket + singlePacketLength, settings.ack, false, 0);
		if (result.status != Success)
		{
			fprintf(stderr, "An error occured while sending! (%u)\n", (int)result.status);
		}
	}
	else 
	{
		if (settings.listen)
		{
			listen(settings);
		}
		else
		{
			transmit(settings);
		}
	}
	
	turnOff();
	bcm2835_spi_end();	
	bcm2835_close();

    return 0;
}

