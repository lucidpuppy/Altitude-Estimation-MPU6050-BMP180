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


//going old school baabyy!-----
MPU6050 mpu;
int i=200;
#define LED_PIN 13                                                            // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
int16_t a[3];  //accelerations from mpu6050
int16_t g[3];  //gyration rates from mpu6050
float A[3],G[3],lastA[3]={0,0,0},lastG[3]={0,0,0};
float offsetA[3] = {60,-29,24},offsetG[3] = {947,-1703,1599};//offset values taken from IMU.ino(original) 
float T[2]; //x=0,y=1,z=2, T=tilt.
//We're doing this my way. Why? because The DMP uses the same kalman that you were using, which means it has the same flaw that
//the normal kalman has. Except I'm not gonna be using a Kalman here. I'll be using a psuedo Kalman(Yes I made that up on the spot).

double az,az_prev;                                                        //Kalman filter stuff..right on it.
double vz_est_acc,h_est_acc;
double prev_vz_est_acc,prev_h_est_acc;
double vz_est,prev_vz_est,h_est,prev_h_est;                               //1D implementation of kalman filter..estimating altitude from barometer and accelerometer

uint32_t spudnut,donut;
double dt;

void setup()
{
  Wire.begin();
  Wire.setClock(800000);//why? because portability matters. Also because max speed boi.
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
