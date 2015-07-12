#include <SPI.h>

#include "rapidradio.h"

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
}

void loop() {
  // put your main code here, to run repeatedly:

  if (!initialized) return;

  const char data[] = "Hello!";

  // don't send string null terminator (0x00 character) to make it nice printable
  rapidradio::send(1, 0xCCCD1102UL, (uint8_t*)data, sizeof(data) - 1, true);

  delay(300);
}
