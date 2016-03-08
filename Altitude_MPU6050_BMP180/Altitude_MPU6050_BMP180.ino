#define gamma 0.95
#define beta 0.95
#define alt_reference 170                                   //ground reference altitude..run example code for bmp180 and update the value

//Barometer stuff
#include <Wire.h>
#include <BMP180.h>

BMP180 barometer;
float seaLevelPressure = 101325;
double vz_est_baro,h_est_baro,prev_h_est_baro;
bool reference=false;
//MPU6050 stuff
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

int i=200;

#define LED_PIN 13                                                            // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool dmpReady = false;                                                        // set true if DMP init was successful
uint8_t mpuIntStatus;                                                         // holds actual interrupt status byte from MPU
uint8_t devStatus;                                                            // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                                                          // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                                                           // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];                                                       // FIFO storage buffer

Quaternion q;                                                                 // [w, x, y, z]         quaternion container
VectorInt16 aa;                                                               // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;                                                           // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;                                                          // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;                                                          // [x, y, z]            gravity vector
float euler[3];                                                               // [psi, theta, phi]    Euler angle container
float ypr[3];                                                                 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


volatile bool mpuInterrupt = false;                                         // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {mpuInterrupt = true;}
double az,az_prev;                                                        //Kalman filter stuff..right on it.
double vz_est_acc,h_est_acc;
double prev_vz_est_acc,prev_h_est_acc;
double vz_est,prev_vz_est,h_est,prev_h_est;                               //1D implementation of kalman filter..estimating altitude from barometer and accelerometer

uint32_t spudnut,donut;
double dt;

void setup()
{
  Wire.begin();
  TWBR = 12;
  Serial.begin(250000);
  initialize_stuff();
  while(i!=0) { get_imu(); get_bmp(); i--; Serial.print("\nCalibrating");}
  h_est_acc= h_est_baro;
  Serial.print(h_est_acc);
}

void loop()
{
  calc_dt();
  //get_data();
  complementry_filter();
  //kalman_filter();
  //display_stuff();
}
