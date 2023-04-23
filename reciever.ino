#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "HCPCA9685.h"
#define I2CAdd 0x40 // server driver I2C Address
// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int flex1;
  int flex2;
  int flex3;
  int flex4;
  int flex5;
  int flex6;
  int steps; // + is right, - is left
  int base;
  int wristRot;
  int wristBend; 
} struct_message;

Servo servo1;
// data storage for arm positioning
int steps = 0;
int elbowDest = 150;
int elbowPos = 150;
int base_angle;
int base_raw;
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

  int thumb1 = constrain(map(myData.flex5, 13800, 15800, 90, 340), 100, 330); 
  int thumb2 = constrain(map(myData.flex5, 13800, 15800, 260, 90), 100, 250);
  HCPCA9685.Servo(4, thumb1); // fishing line
  HCPCA9685.Servo(5, thumb2); // palm joint

  elbowDest = myData.flex6;
  //HCPCA9685.Servo(8, myData.flex6); // elbow (without smoothing)
  
  // Wrist control
  HCPCA9685.Servo(9, myData.wristRot); // wrist rotation
  HCPCA9685.Servo(10, myData.wristBend); // wrist bend

  steps = myData.steps;
  
}

// test each finger on startup to make sure they all work
void handInit()
{
  int thumb1, thumb2, index, middle, ring, pinky;
  // stagger close and open 
  for (int i = 0; i <= 240; i++)
  {

    // Get staggered values between 0 and 60 to map to each finger
    int th = constrain(abs(i - 180), 0, 60);
    int in = constrain(abs(i - 150), 0, 60);
    int mi = constrain(abs(i - 120), 0, 60);
    int ri = constrain(abs(i - 90), 0, 60);
    int pi = constrain(abs(i - 60), 0, 60);
    // map each 0-60 value to max/min servo value
    thumb1 = map(th, 60, 0, 100, 330); // fishing line
    thumb2 = map(th, 60, 0, 250, 100); // palm joint
    index = map(in, 60, 0, 268, 10);
    middle = map(mi, 60, 0, 285, 40);
    ring = map(ri, 60, 0, 55, 300);
    pinky = map(pi, 60, 0, 100, 340);
    // move servos
    HCPCA9685.Servo(0, index);
    HCPCA9685.Servo(3, pinky);
    HCPCA9685.Servo(1, middle);
    HCPCA9685.Servo(2, ring);
    HCPCA9685.Servo(4, thumb1);
    HCPCA9685.Servo(5, thumb2);
    // 250 loop cycles = about 2 seconds with delay of 8
    delay(5);
  }
  delay(500); // wait a moment before doing the next test

  // Simultaneous close and open to test all fingers at once & current draw
  for (int i = 0; i <= 120; i++)
  {
    // absolute value function that goes from 60 -> 0 -> 60
    int j = abs(60 - i);
    // map function to servo max/min servo values
    thumb1 = map(j, 60, 0, 100, 330); // fishing line
    thumb2 = map(j, 60, 0, 250, 100); // palm joint
    index = map(j, 60, 0, 268, 10);
    middle = map(j, 60, 0, 285, 40);
    ring = map(j, 60, 0, 55, 300);
    pinky = map(j, 60, 0, 100, 340);
    // move servos
    HCPCA9685.Servo(0, index);
    HCPCA9685.Servo(3, pinky);
    HCPCA9685.Servo(1, middle);
    HCPCA9685.Servo(2, ring);
    HCPCA9685.Servo(4, thumb1);
    HCPCA9685.Servo(5, thumb2);

    // closes in about half a second, stops for a quarter second, and opens in about half a second
    if (j == 0)
    {
      delay(500);
    }
    else
    {
      delay(5);
    }
  }
}

void openHand(){
  HCPCA9685.Servo(0, 268);
  HCPCA9685.Servo(3, 100);
  HCPCA9685.Servo(1, 285);
  HCPCA9685.Servo(2, 55);
  HCPCA9685.Servo(4, 100);
  HCPCA9685.Servo(5, 250);
}

void closeHand(){
  HCPCA9685.Servo(0, 10);
  HCPCA9685.Servo(3, 340);
  HCPCA9685.Servo(1, 40);
  HCPCA9685.Servo(2, 300);
  HCPCA9685.Servo(4, 330);
  HCPCA9685.Servo(5, 100);
}

void setup()
{
  // Initialize Serial Monitor
  Serial.begin(115200);
  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);
  base_angle = 175;
  base_raw = 17500;
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

  // Initialize stepper motor pins (8 is direction and 9 is stepping)
  
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  digitalWrite(18, 0);
  digitalWrite(19, LOW);
  
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  
  if(steps > 0){ // rotate right
    digitalWrite(18, 0); // Set stepper turn direction
    digitalWrite(19,HIGH); //Trigger one step forward
    delayMicroseconds(1);
    digitalWrite(19,LOW); //Pull step pin low so it can be triggered again
    delayMicroseconds(1);
    
  }
  else if (steps < 0){ // rotate left
    digitalWrite(18, 1); // Set stepper turn direction
    digitalWrite(19,HIGH); //Trigger one step forward
    delayMicroseconds(1);
    digitalWrite(19,LOW); //Pull step pin low so it can be triggered again
    delayMicroseconds(1);
    
  }

  if(myData.base == 1 && base_raw < 25000){
    base_raw += 10;
  } else if(myData.base == -1 && base_raw > 10000){
    base_raw -= 10;
  } 
  base_raw = constrain(base_raw, 10000, 25000);

  base_angle = map(base_raw, 10000,25000, 100,250);
  HCPCA9685.Servo(6, base_angle + 20); // base joint 1
  HCPCA9685.Servo(7, 410 - base_angle); // base joint 2
  Serial.println(myData.wristRot);

  // smoothing function for elbow movement
  
  if(abs(elbowDest - elbowPos) < 6){
    elbowPos = elbowDest;
    HCPCA9685.Servo(8, elbowPos); 
  }
  else{
    elbowPos += (elbowDest - elbowPos) * .15;
    HCPCA9685.Servo(8, elbowPos);
  }

}
