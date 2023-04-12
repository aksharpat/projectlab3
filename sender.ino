#include <esp_now.h>
#include <WiFi.h>
#include <MPU6050.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xAA, 0xAB, 0x03, 0x23, 0xB1, 0xBA};//{0x70, 0xB8, 0xF6, 0x5D, 0x64, 0x48};

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


const int flexPin4 = 32;
const int flexPin1 = 33;
const int flexPin3 = 34;
const int flexPin2 = 35;
const int flexPin5 = 39;
const int flexPin6 = 36;

#define  I2CAdd 0x40
#define VRX_PIN  26 // analog in pin for x direction 
#define VRY_PIN  27 // analog in pin for y direction 

// storage for base joint data (up/down joystick)
int basePos = 175;
int rightleft1 = 0;
int rightleft2 = 0;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void readData(){
  int position;
  int servoposition;
  
  position = analogRead(flexPin1); //index finger
  servoposition = map(position, 2940, 3700, 268, 10);
  servoposition = constrain(servoposition, 10, 268);
  //Serial.println(servoposition);
  myData.flex1 = servoposition;
  
  position = analogRead(flexPin4); //pinky finger
  servoposition = map(position, 2940, 3700, 100, 340);
  servoposition = constrain(servoposition, 100, 340);
  //Serial.println(servoposition);
  myData.flex4 = servoposition;
  
  position = analogRead(flexPin2); //middle finger
  servoposition = map(position, 3500, 4100, 285, 40);
  servoposition = constrain(servoposition, 40, 285);
  //Serial.println(servoposition);
  myData.flex2 = servoposition;
  
  position = analogRead(flexPin3); //ring finger
  servoposition = map(position, 2940, 3700, 55, 300);
  servoposition = constrain(servoposition, 55, 300);
  //Serial.println(position);
  myData.flex3 = servoposition;

  // thumb is sent over as raw analog instead of premapped due to 2 thumb servos
  myData.flex5 = analogRead(flexPin5);

  // change these values before implementation
  position = analogRead(flexPin6); //elbow
  servoposition = map(position, 1700, 3300, 360, 100);
  servoposition = constrain(servoposition, 360, 100);
  //Serial.println(position);
  myData.flex6 = servoposition;

  //joystick values
  int valueX = round(analogRead(VRX_PIN)/1000);
  int valueY = round(analogRead(VRY_PIN)/1000);
 
  // stepper motor control
  if (valueX > 1) {
    myData.steps = -10;
  }
  else if (valueX < 1) {
    myData.steps = 10;
  }
  // base joint control
  if (valueY > 1) {
    if(rightleft1 >= 5){
      if(basePos <= 330){
        basePos = basePos + 1;
        myData.base = basePos;
      }
      rightleft2 = 0;
    }else{
      rightleft1++;
    }
  }
  else if (valueY < 1) {
    if(rightleft2 >= 5){
      if (basePos >= 20){
        basePos = basePos - 1;
        myData.base = basePos;
      }
      rightleft1 = 0;
    } else{
      rightleft2++;
    }
  }
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // Set values to send
  readData(); //call function to read sensor data
  delay(50); //small delay
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  
}
