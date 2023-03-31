#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "HCPCA9685.h"
#define  I2CAdd 0x40
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int flex1;
  int flex2;
  int flex3;
  int flex4;
  int flex5;
} struct_message;

Servo servo1;

HCPCA9685 HCPCA9685(I2CAdd);
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Read: ");
  Serial.println(myData.flex1);
  //servo1.write(myData.flex1);
  HCPCA9685.Servo(0, myData.flex1);
  HCPCA9685.Servo(3, myData.flex4);
  HCPCA9685.Servo(1, myData.flex2);
  HCPCA9685.Servo(2, myData.flex3);
}
 
//new ESP Address
uint8_t newMACAddress[] = {0xAA, 0xAB, 0x03, 0x23, 0xB1, 0xBA};

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  // Init ESP-NOW
  servo1.attach(18);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

}