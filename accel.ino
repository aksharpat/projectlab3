#include <MPU6050.h>
/* Include the HCPCA9685 library */
#include "HCPCA9685.h"

/* I2C slave address for the device/module. For the HCMODU0097 the default I2C address
   is 0x40 */
#define I2CAdd 0x40

MPU6050 sensor;

int16_t ax, ay, az;

int16_t gx, gy, gz;

/* Create an instance of the library */
HCPCA9685 HCPCA9685(I2CAdd);

void setup()
{
    Serial.begin(115200);
    /* Initialise the library and set it to 'servo mode' */
    HCPCA9685.Init(SERVO_MODE);
    sensor.initialize();
    /* Wake the device up */
    HCPCA9685.Sleep(false);
    int Pos;

    for (Pos = -100; Pos < 300; Pos++)
    {

        HCPCA9685.Servo(0, Pos);
        delay(10);
    }

    for (Pos = 300; Pos >= 200; Pos--)
    {
        HCPCA9685.Servo(0, Pos);
        delay(10);
    }
}

int temp = 200;

void loop()
{
    int Pos;

    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax = map(ax, -17000, 17000, 0, 350);
    HCPCA9685.Servo(0, ax);
    ay = map(ay, -17000, 17000, 0, 350);
    HCPCA9685.Servo(2, ay); // wrist joint AY
    // Serial.println(gz);
    gz = map(gz, -17000, 17000, -20, 20);
    temp = temp + gz;
    Pos = map(temp, -5000, 6000, 0, 350);
    Serial.println(temp);
    HCPCA9685.Servo(1, temp);
    delay(5);
}
