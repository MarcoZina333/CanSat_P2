#include <Arduino.h>
#include <string.h>

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO) // 2 ^ GPIO_NUMBER in hex

#define PHOTO_PIN GPIO_NUM_2
#define HALL_PIN GPIO_NUM_4
#define WAKE_UP_TIMEOUT 2000

#define LED_PIN 33

#include "LoRa/LoRa.h"

#define FREQ 868300E3
#define BW 125E3

#define TX_BUFFER_LEN 200
char tx_buffer[TX_BUFFER_LEN];

#include <Positioning/UWB/DW1000Ng.hpp>
#include <Positioning/UWB/DW1000NgUtils.hpp>
#include <Positioning/UWB/DW1000NgRanging.hpp>
#include <Positioning/UWB/DW1000NgRTLS.hpp>
#include <Positioning/Trilateration/Trilateration.h>
#include <Positioning/Trilateration/Position.h>

// connection pins
#if defined(ESP8266)
const uint8_t PIN_SS = 15;
#else
const uint8_t PIN_RST = 17;
const uint8_t PIN_SS = 5; // spi select pin
#endif

// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
const char EUI[] = "AA:BB:CC:DD:EE:FF:00:01";

const Pos3d position_self(0, 0, 0);
const Pos3d position_B(0, 2, 0);
const Pos3d position_C(2, 0, 0);
const Pos3d position_D(-1.25, 6, 1.25);

Pos3d position_tag(0, 0, 0);

double range_self = 0;
double range_B = 0;
double range_C = 0;
double range_D = 0;

PosAndDistance3dVec beacons;

Trilateration tri;

boolean received_B = false;
boolean received_C = false;

byte target_eui[8];
byte tag_shortAddress[] = {0x05, 0x00};

byte anchor_b[] = {0x02, 0x00};
uint16_t next_anchor = 2;
byte anchor_c[] = {0x03, 0x00};
byte anchor_d[] = {0x04, 0x00};

const uint16_t antenna_delay = 16415; // main
// const uint16_t antenna_delay = 16403; // B
// const uint16_t antenna_delay = 16433; // C
// const uint16_t antenna_delay = 16417; // D

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::DECAWAVE_SFD,
    Channel::CHANNEL_1,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_2048,
    PreambleCode::CODE_2};

frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    true /* This allows blink frames */
};

void setup()
{

  Serial.begin(115200);

  // LoRa setup
  if (!LoRa.begin(FREQ))
  {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, true);
    while (1)
    {
      delay(1000);
      if (LoRa.begin(FREQ))
      {
        digitalWrite(LED_PIN, false);
        break;
      }
    }
  }
  // LoRa.setTxPower(20);
  // LoRa.setSignalBandwidth(BW);

  // put the radio into receive mode
  // LoRa.receive();

  // LoRa.sleep();*/
  /*
  Serial.println(F("### DW1000Ng-arduino-ranging-anchorMain ###"));
// initialize the driver
#if defined(ESP8266)
  DW1000Ng::initializeNoInterrupt(PIN_SS);
#else
  DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
#endif
  Serial.println(F("DW1000Ng initialized ..."));
  // general configuration
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);

  DW1000Ng::setEUI(EUI);

  DW1000Ng::setPreambleDetectionTimeout(64);
  DW1000Ng::setSfdDetectionTimeout(2048 + 16 + 1);
  DW1000Ng::setReceiveFrameWaitTimeoutPeriod(5000);

  DW1000Ng::setNetworkId(RTLS_APP_ID);
  DW1000Ng::setDeviceAddress(1);

  DW1000Ng::setAntennaDelay(antenna_delay);

  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000Ng::getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: ");
  Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: ");
  Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: ");
  Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg);
  Serial.print("Device mode: ");
  Serial.println(msg);
  beacons.clear();*/
}

void loop()
{
  // Receive
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available())
    {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
  /*
  if (DW1000NgRTLS::receiveFrame())
  {
    size_t recv_len = DW1000Ng::getReceivedDataLength();
    byte recv_data[recv_len];
    DW1000Ng::getReceivedData(recv_data, recv_len);

    if (recv_data[0] == BLINK)
    {
      DW1000NgRTLS::transmitRangingInitiation(&recv_data[2], tag_shortAddress);
      DW1000NgRTLS::waitForTransmission();

      RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::RANGING_CONFIRM, next_anchor);
      if (!result.success)
        return;
      range_self = result.range;

      String rangeString = "Range: ";
      rangeString += range_self;
      rangeString += " m";
      rangeString += "\t RX power: ";
      rangeString += DW1000Ng::getReceivePower();
      rangeString += " dBm";
      Serial.println(rangeString);
    }
    else if (recv_data[9] == 0x60)
    {
      double range = static_cast<double>(DW1000NgUtils::bytesAsValue(&recv_data[10], 2) / 1000.0);
      String rangeReportString = "Range from: ";
      rangeReportString += recv_data[7];
      rangeReportString += " = ";
      rangeReportString += range;
      Serial.println(rangeReportString);
      if (!received_B && recv_data[7] == anchor_b[0] && recv_data[8] == anchor_b[1])
      {
        range_B = range;
        received_B = true;
      }
      else if (received_B && !received_C && recv_data[7] == anchor_c[0] && recv_data[8] == anchor_c[1])
      {
        range_C = range;
        received_C = true;
      }
      else if (received_B && received_C && recv_data[7] == anchor_d[0] && recv_data[8] == anchor_d[1])
      {
        range_D = range;
        beacons.push_back(PosAndDistance3d(position_self, range_self));
        beacons.push_back(PosAndDistance3d(position_B, range_B));
        beacons.push_back(PosAndDistance3d(position_C, range_C));
        beacons.push_back(PosAndDistance3d(position_D, range_D));

        tri.CalculateLocation3d(beacons, position_tag);
        String positioning = "Found position - x: ";
        positioning += position_tag(0);
        positioning += " y: ";
        positioning += position_tag(1);
        positioning += " z: ";
        positioning += position_tag(2);
        Serial.println(positioning);
        received_B = false;
        received_C = false;

        beacons.clear();
      }
      else
      {
        received_B = false;
        received_C = false;
      }
    }
  }
  */
}
