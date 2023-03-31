#include <ESP32Servo.h>
/* Include the HCPCA9685 library */
#include "HCPCA9685.h"
//#include "Servo.h" 
/* I2C slave address for the device/module. For the HCMODU0097 the default I2C address
   is 0x40 */
#define  I2CAdd 0x40
 
Servo thumb_servo;
/*
Start Posistion Angle:
Max Contract Angle:
Max Expand Angle:
*/
Servo index_servo;
/*
Start Posistion Angle:268
Max Contract Angle:10
Max Expand Angle:268
*/
Servo middle_servo;
/*
Start Posistion Angle:285
Max Contract Angle:40
Max Expand Angle:285
*/
Servo ring_servo;
/*
Start Posistion Angle:55
Max Contract Angle:300
Max Expand Angle:55
*/
Servo pinkie_servo;
/*
Start Posistion Angle:100
Max Contract Angle:340
Max Expand Angle:100
*/

/* Create an instance of the library */
HCPCA9685 HCPCA9685(I2CAdd);
int f = 0;
 
void setup() 
{
  /* Initialise the library and set it to 'servo mode' */ 
  HCPCA9685.Init(SERVO_MODE);
 //HCPCA9685.Servo(2, 100);
  /* Wake the device up */
  HCPCA9685.Sleep(false);
}
 
 
void loop() 
{
   
  // VALUES FOR BOTH PARTS OF THE THUMB
  //SERVO 4 IS THE FISHING LINE
  //SERVO 5 IS THE SERVO HOUSED INSIDE THE HAND
  delay(2000);
  HCPCA9685.Servo(4, 330);
  HCPCA9685.Servo(5, 100);

  delay(1500);
  HCPCA9685.Servo(4, 100);
  HCPCA9685.Servo(5, 250);


}
