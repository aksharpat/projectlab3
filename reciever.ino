#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "HCPCA9685.h"
#define I2CAdd 0x40 // server driver I2C Address
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message
{
  int flex1;
  int flex2;
  int flex3;
  int flex4;
  int flex5;
} struct_message;

Servo servo1;

HCPCA9685 HCPCA9685(I2CAdd);
struct_message myData;

// new ESP Address
uint8_t newMACAddress[] = {0xAA, 0xAB, 0x03, 0x23, 0xB1, 0xBA};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  HCPCA9685.Servo(0, myData.flex1); // index finger
  HCPCA9685.Servo(3, myData.flex4); // pinky finger
  HCPCA9685.Servo(1, myData.flex2); // middle finger
  HCPCA9685.Servo(2, myData.flex3); // ring finger
}

// test each finger on startup to make sure they all work
void handInit()
{
  int thumb, index, middle, ring, pinky;
  // stagger close and open (change i to 240 when adding thumb)
  for (int i = 0; i <= 210; i++)
  {

    // Get staggered values between 0 and 60 to map to each finger
    // int th = constrain(abs(i - 180), 0, 60);
    int in = constrain(abs(i - 150), 0, 60);
    int mi = constrain(abs(i - 120), 0, 60);
    int ri = constrain(abs(i - 90), 0, 60);
    int pi = constrain(abs(i - 60), 0, 60);
    // map each 0-60 value to max/min servo value
    index = map(in, 60, 0, 268, 10);
    middle = map(mi, 60, 0, 285, 40);
    ring = map(ri, 60, 0, 55, 300);
    pinky = map(pi, 60, 0, 100, 340);
    // move servos
    HCPCA9685.Servo(0, index);
    HCPCA9685.Servo(3, pinky);
    HCPCA9685.Servo(1, middle);
    HCPCA9685.Servo(2, ring);
    // 250 loop cycles = about 2 seconds with delay of 8
    delay(8);
  }
  delay(500); // wait a moment before doing the next test

  // Simultaneous close and open to test all fingers at once & current draw
  for (int i = 0; i <= 120; i++)
  {
    // absolute value function that goes from 60 -> 0 -> 60
    int j = abs(60 - i);
    // map function to servo max/min servo values
    // thumb = map(j, 60, 0, );
    index = map(j, 60, 0, 268, 10);
    middle = map(j, 60, 0, 285, 40);
    ring = map(j, 60, 0, 55, 300);
    pinky = map(j, 60, 0, 100, 340);
    // move servos
    HCPCA9685.Servo(0, index);
    HCPCA9685.Servo(3, pinky);
    HCPCA9685.Servo(1, middle);
    HCPCA9685.Servo(2, ring);

    // closes in about half a second, stops for a quarter second, and opens in about half a second
    if (j == 0)
    {
      delay(250);
    }
    else
    {
      delay(8);
    }
  }
}

void setup()
{
  // Initialize Serial Monitor
  Serial.begin(115200);
  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // change MAC address
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  handInit();

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
}
