#include <SPI.h>
#include "rapidradio.h"

#define ADDRESS                                  0xCCCD1102UL
#define CHANNEL                                  1

bool initialized = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.println("Initializing rapidradio...");
  if (rapidradio::init())
  {
    initialized = true;
    Serial.println("OK");
  }
  else
  {
    Serial.println("ERROR");
  }

  // start listening
  rapidradio::startListening(CHANNEL, ADDRESS);
  
  interrupts();
}

void loop() {
  // put your main code here, to run repeatedly:

  if (!initialized) return;
  
  Serial.println("Waiting for IRQ...");

  // simple active wait for an interrupt
  while (!rapidradio::irq);

  // single packet could be up to 32 bytes long
  uint8_t buff[32];
  uint8_t length;

  // rapidradio can buffer up to 3 packets, so call receive in a loop to take them all
  while (rapidradio::received(buff, length))
  {
    if (Serial)
    {
      Serial.print("Received packet: [");
      Serial.write(buff, length);
      Serial.println("]");
    }
  }
}
