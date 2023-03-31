
#include <esp_now.h>
#include <WiFi.h>
//70:B8:F6:5D:64:48
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
} struct_message;

const int flexPin1 = 33;
const int flexPin4 = 32;
const int flexPin2 = 35;
const int flexPin3 = 34;
 
// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void readData(){
  //delay(50);
  int position;
  int servoposition;
  position = analogRead(flexPin1);
  servoposition = map(position, 2940, 3700, 268, 10);
  servoposition = constrain(servoposition, 10, 268);
  //Serial.println(servoposition);
  myData.flex1 = servoposition;
  position = analogRead(flexPin4);
  servoposition = map(position, 2940, 3700, 100, 340);
  servoposition = constrain(servoposition, 100, 340);
  //Serial.println(servoposition);
  myData.flex4 = servoposition;
  position = analogRead(flexPin2);
  servoposition = map(position, 3500, 4100, 285, 40);
  servoposition = constrain(servoposition, 40, 285);
  //Serial.println(servoposition);
  myData.flex2 = servoposition;
  position = analogRead(flexPin3);
  servoposition = map(position, 2940, 3700, 55, 300);
  servoposition = constrain(servoposition, 55, 300);
  Serial.println(position);
  myData.flex3 = servoposition;
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial.println(analogRead(flexPin1));
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println(analogRead(flexPin1));

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println(analogRead(flexPin1));

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  Serial.println(analogRead(flexPin1));

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println(analogRead(flexPin1));

}
 
void loop() {
  // Set values to send

  int servoposition;

  readData();
  delay(50);
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  
}