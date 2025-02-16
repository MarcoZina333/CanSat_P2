#include <Arduino.h>

#define LED 6

#include "LoRa/LoRa.h"

#define FREQ 868300E3
#define BW 125E3

void sendLoRaMessage(char *buffer, size_t len)
{
  while (LoRa.beginPacket() == 0)
  {
    delayMicroseconds(10);
  }

  // send in async / non-blocking mode
  LoRa.beginPacket();

  for (size_t i = 0; i < len; i++)
  {
    LoRa.write(buffer[i]);
  }

  LoRa.endPacket(); // true = async / non-blocking mode
}

void setup()
{
  // --- Setup iniziale -------------------------------------------------

  if (!LoRa.begin(FREQ))
  {
    pinMode(LED, OUTPUT);
    digitalWrite(LED, true);
    while (1)
      ;
  }
  LoRa.setTxPower(20);
  LoRa.setSignalBandwidth(BW);
  LoRa.sleep();

  // counter per raccolta dati da PMS
  uint8_t i_PMS = 0;

  char tx_buffer[256];
}

void loop()
{
  // put your main code here, to run repeatedly:
}
