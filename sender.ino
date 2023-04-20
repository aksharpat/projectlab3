#include <Adafruit_ADS1X15.h>

#include <MPU6050.h>
#include <esp_now.h>
#include <WiFi.h>
//#include <Wire.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xAA, 0xAB, 0x03, 0x23, 0xB1, 0xBA};//{0x70, 0xB8, 0xF6, 0x5D, 0x64, 0x48};

Adafruit_ADS1115 ads; //adc extension
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


const int flexPin4 = 32; //pinky
const int flexPin1 = 1; //adc ext pin; index
const int flexPin3 = 34; //ring
const int flexPin2 = 35; //middle
const int flexPin5 = 0; //adc ext pin; thumb
const int flexPin6 = 33; //elbow

//#define  I2CAdd 0x40
#define VRX_PIN  2 // adc ext pin for x direction 
#define VRY_PIN  3 //adc ext pin for y direction 

// storage for base joint data (up/down joystick)
int basePos = 175;
int rightleft1 = 0;
int rightleft2 = 0;

// wrist accelerometer data
MPU6050 accel;
int16_t ax, ay, az;
int16_t gx, gy, gz;

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
  int16_t adc;
  //WiFi.stop();

  position = ads.readADC_SingleEnded(flexPin1); //index finger
  servoposition = map(position, 13450, 16000, 268, 10);
  servoposition = constrain(servoposition, 10, 268);
  //Serial.println(servoposition);
  myData.flex1 = servoposition;
  
  position = analogRead(flexPin4); //pinky finger
  servoposition = map(position, 2940, 3500, 100, 340);
  servoposition = constrain(servoposition, 100, 340);
  //Serial.println(servoposition);
  myData.flex4 = servoposition;
  
  position = analogRead(flexPin2); //middle finger
  servoposition = map(position, 3770, 4095, 285, 30);
  servoposition = constrain(servoposition, 40, 285);
  Serial.println(servoposition);
  myData.flex2 = servoposition;
  
  position = analogRead(flexPin3); //ring finger
  servoposition = map(position, 3100, 3700, 55, 300);
  servoposition = constrain(servoposition, 55, 300);
  //Serial.println(servoposition);
  myData.flex3 = servoposition;

  // thumb is sent over as raw analog instead of premapped due to 2 thumb servos
  myData.flex5 = ads.readADC_SingleEnded(flexPin5);
  //Serial.println(myData.flex5);

  position = analogRead(flexPin6); //elbow
  servoposition = map(position, 1700, 3100, 360, 100);
  servoposition = constrain(servoposition, 100, 360);
  //Serial.println(position);
  myData.flex6 = servoposition;
  //Serial.println(myData.flex6);
  Serial.println(" ");
  // Wrist values from accelerometer
  accel.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  myData.wristBend = map(ax, -17000, 17000, 0, 350);
  myData.wristRot = map(ay, -17000, 17000, 0, 350);
  //Serial.println(myData.wristBend);

  //joystick values
  int valueX = round(ads.readADC_SingleEnded(VRX_PIN)/1000);
  int valueY = round(ads.readADC_SingleEnded(VRY_PIN)/1000);
  //Serial.println(valueY);
  // stepper motor control
  if (valueX > 8) {
    myData.steps = -10;
  }
  else if (valueX < 8) {
    myData.steps = 10;
  } else{
    myData.steps = 0;
  }
  // base joint control
  if (valueY > 8) {
    if(rightleft1 >= 2){      
      myData.base = 1;
      rightleft2 = 0;
    }else{
      rightleft1++;
    }
  }
  else if (valueY < 8) {
    if(rightleft2 >= 2){
      myData.base = -1;
      rightleft1 = 0;
    } else{
      rightleft2++;
    }
  } else{
    myData.base = 0;
  }
  //WiFi.begin();
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  //Wire.begin(21, 22);
  // begin ads1115
  ads.begin();
  // Set device as a Wi-Fi Station
  accel.initialize();
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  myData.base = 0;
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
  // Send message via ESP-NOW
  delay(4); //small delay
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
}
