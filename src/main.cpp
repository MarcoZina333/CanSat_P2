#include <Arduino.h>
#include <string.h>

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO) // 2 ^ GPIO_NUMBER in hex

#define PHOTO_PIN GPIO_NUM_2
#define HALL_PIN GPIO_NUM_4

#define LED_PIN 33

#include "LoRa/LoRa.h"

#define FREQ 868300E3
#define BW 125E3

#include "Accel/MPU6050.h"

#define LORA_ACC_MSG "Acceleration -> X: %.2f mg Y: %.2f mg Z: %.2f mg\nGyro rate -> X: %.1f deg/s Y: %.1f deg/s Z: %.1f deg/s\nTemperature: %.2f deg C"
#define ACC_RATE_MS 100

#define TX_BUFFER_LEN 200
char tx_buffer[TX_BUFFER_LEN];

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
    Serial.printf("%c", buffer[i]);
  }

  LoRa.endPacket(); // true = async / non-blocking mode
}

// Accel Pin definitions
// int intPin = 12;                 // This can be changed, 2 and 3 are the Arduinos ext int pins
int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;                // Stores the real accel value in g's
int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output
float gyrox, gyroy, gyroz;       // Stores the real gyro value in degrees per seconds
float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
int16_t tempCount;               // Stores the internal chip temperature sensor output
float temperature;               // Scaled_PIN temperature in degrees Celsius
float SelfTest[6];               // Gyro and accelerometer self-test sensor output
uint32_t count = 0;
float aRes, gRes; // scale resolutions per LSB for the sensors
MPU6050lib mpu;

void setup()
{

  Serial.begin(115200);

  Wire.begin();

  // Accel Setup

  // Set up the interrupt pin, its set as active high, push-pull
  // pinMode(intPin, INPUT);
  // digitalWrite(intPin, LOW);

  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050); // Read WHO_AM_I register for MPU-6050

  if (c == MPU6050_ADDRESS) // WHO_AM_I should always be 0x68
  {

    mpu.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values

    if (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f)
    {
      mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
      mpu.initMPU6050();
    }
    else
    {
      pinMode(LED_PIN, OUTPUT);
      digitalWrite(LED_PIN, true);
      while (1)
        ; // Loop forever if communication doesn't happen
    }
  }
  else
  {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, true);
    while (1)
      ; // Loop forever if communication doesn't happen
  }

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
  LoRa.setTxPower(20);
  LoRa.setSignalBandwidth(BW);

  // put the radio into receive mode
  // LoRa.receive();

  LoRa.sleep();

  // --- Setup iniziale -------------------------------------------------

  // Sleep Setup
  pinMode(HALL_PIN, INPUT);
  pinMode(PHOTO_PIN, INPUT);

  uint64_t bitmask = BUTTON_PIN_BITMASK(PHOTO_PIN);

  esp_sleep_enable_ext1_wakeup(bitmask, ESP_EXT1_WAKEUP_ALL_LOW);

  esp_sleep_enable_ext0_wakeup(HALL_PIN, HIGH);

  uint32_t start = millis();
  while (millis() - start < 1000)
  {
    if (!digitalRead(HALL_PIN) && digitalRead(PHOTO_PIN))
    {
      mpu.LowPowerAccelOnlyMPU6050();
      esp_deep_sleep_start();
    }
  }
}

void loop()
{
  // If data ready bit set, all data registers have new data
  if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01)
  { // check if data ready interrupt

    mpu.readAccelData(accelCount); // Read the x/y/z adc values
    aRes = mpu.getAres();

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes - accelBias[1];
    az = (float)accelCount[2] * aRes - accelBias[2];

    mpu.readGyroData(gyroCount); // Read the x/y/z adc values
    gRes = mpu.getGres();

    // Calculate the gyro value into actual degrees per second
    gyrox = (float)gyroCount[0] * gRes - gyroBias[0]; // get actual gyro value, this depends on scale being set
    gyroy = (float)gyroCount[1] * gRes - gyroBias[1];
    gyroz = (float)gyroCount[2] * gRes - gyroBias[2];

    tempCount = mpu.readTempData();                  // Read the x/y/z adc values
    temperature = ((float)tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
  }

  uint32_t deltat = millis() - count;
  if (deltat > ACC_RATE_MS)
  {

    sprintf(tx_buffer,
            LORA_ACC_MSG,
            1000 * ax, // meas in mg
            1000 * ay, // meas in mg
            1000 * az, // meas in mg
            gyrox,
            gyroy,
            gyroz,
            temperature);

    // Transmit
    sendLoRaMessage(tx_buffer, strnlen(tx_buffer, TX_BUFFER_LEN));

    /* // Debug print
    Serial.printf(LORA_ACC_MSG,
                  1000 * ax, // meas in mg
                  1000 * ay, // meas in mg
                  1000 * az, // meas in mg
                  gyrox,
                  gyroy,
                  gyroz,
                  temperature);*/

    count = millis();
  }

  /*delay(1000);
  sprintf(tx_buffer, "cacca");
  sendLoRaMessage(tx_buffer, strnlen(tx_buffer, TX_BUFFER_LEN));*/

  /*//Receive
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
  }*/
}
