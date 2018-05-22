void setup_imu()                                                                      //Real world Accel with gravity removed
{
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

    // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  pinMode(LED_PIN, OUTPUT);
  for(int j=0;j<2000;j++)   //taking 2000 samples for finding initial orientation,takes about 0.8 seconds  
  {
    mpu.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);//i hope the mpu_6Axis library has this or else we're fucked hardy.
    for(i=0;i<2;i++)                                      
    {
      A[i]=a[i];         //transfer value
      A[i]-=offsetA[i]; //subtracting offset
      A[i]*=0.0006103;    //convert to real world value
      lastA[i]+=A[i];    //store in lastA[i]
    }
  }
  A[0]=lastA[0]*0.0005;   //take average of 2000 readings
  A[1]=lastA[1]*0.0005;
  lastA[0]=A[0];  //lastA[i] was used here only as a place holder for a "sum" variable. it's purpose as a sum variable
  lastA[1]=A[1];  // has been fullfilled, therefore it will now be restored to A[i] so that it can be used for it's origianl purpose

  T[0]=(+57.3*asin(A[1]*0.102));  //initial orientation 
  T[1]=(-57.3*asin(A[0]*0.102));
}

inline __attribute__((always_inline)) float my_asin(float a)//works pretty damn well for a large range of angles.
{
  return a*(1+(0.5*a*a)); //35us
}

void orientationUpdate()    
{
  
  T[0]+=(float(G[0]*dt) -bias[0]) ;  //T[0]=pitch,T[1]=roll,T[2]=yaw.    //~30us 
  T[1]+=(float(G[1]*dt) -bias[1]);//remove bias
  //yaw compensation for pitch and roll
  /*actual formula- 
   * T[0] += T[1]*sin(G[2]*0.0000436);
   * T[1] -= T[0]*sin(G[2]*0.0000436);
   * even this is an approximation from sin(pitch)=sin(roll)*sin(yaw) as the pitch vs yaw curve is approximately a sin function.
   * Grandma would not be happy with all the approximations i make everyday. 
  */
  T[0] += T[1]*(G[2]*0.0000436); //0.0000436=dt/57.3 because sin takes radian not degrees,also, sin(x)=x as x->0 .this saves me fucking 450us
  T[1] -= T[0]*(G[2]*0.0000436);  //because sin() takes fucking 255 us(max, min=180us) to execute. fuck that i can live with approximations
  //These guys are basically my estimates. T[0] is pitch, T[1] is roll.
  float TotalAccel = sqrt(A[0]*A[0] + A[1]*A[1] + A[2]*A[2]);
  float roll = -57.3*asin(A[0]/TotalAccel);
  float pitch = 57.3*asin(A[1]/TotalAccel);
  TotalAccel -= 9.81; //No i m not removing gravity here, I m simply centering the TotalAccel variable at 9.81 .
  float Trust_Factor = 0.05/(1+TotalAccel*TotalAccel); //Remember the notch function? this is his brother spiky boi.
  //visualisation : http://www.wolframalpha.com/input/?i=1%2F(1%2Bx%5E2) . Max trust on Accelero = 0.05 .
  bias[0] = bias[0] + Trust_Factor*(T[0]-pitch); //this is an adhoc implementation. Its called psuedo Kalman filter for a reason
  T[0] = (1-Trust_Factor)*T[0] + Trust_Factor*pitch;
  bias[1] = bias[1] + Trust_Factor*(T[1]-roll);
  T[1] = (1-Trust_Factor)*T[1] + Trust_Factor*roll;
  //I have the pitch and roll now. I am invincible.
}

void get_imu()
{
  //k so lets read the mpu first. 
  mpu.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);//this bad boy takes about 310us on 8 bit 16mhz arduino.
  while(a[0]==0&&a[1]==0&&a[2]==0&&g[0]==0&&g[1]==0&&g[2]==0)//this is in case the accelgyro dicks me over and goes dead, 
  {                     
    mpu.initialize();//keep trying to re-initialize the mpu
    mpu.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);
  }
  else
  {
    for(i=0;i<3;i++)
    {
      A[i]=a[i];
      A[i]-=offsetA[i]; //subtracting offset
      A[i]*=0.0006103; //mapping to real world values in m/s^2 , instead of dividing by 1638.4 i multiply by 1/1638.4 to save on time as multiplication is faster 
      
      A[i]= (0.5*A[i])+(0.5*lastA[i]); //applying the steady state assumption that the Acceleration can't change too much within 2.5 ms 
      lastA[i]=A[i]; 
  
      G[i]=g[i];
      G[i]-=offsetG[i]; // subtracting offset
      G[i]*=0.030516; //mapping to degrees per second for FS_1000. Change to whatever constant you have for whatever scaling you have.
      G[i]=(0.7*G[i])+(0.3*lastG[i]);  //buffer filter,same as that for accel.
      lastG[i]=G[i]; 
    }   //243us in this function.
    //-----EXTRACTION OF ACCEL-GYRO DATA ENDS-----
    orientationUpdate();   //~240us function
    //I solved this a long time ago for implementing altitude hold in my quadcopter. I m not sure where i kept the 
    //derivation so you'll just have to trust me. 
    float tanSqTheta = 0.0003045*(T[0]*T[0] + T[1]*T[1]); //35us. quick math.
    float cosSqTheta = 1 - tanSqTheta + (tanSqTheta*tanSqTheta); //20us . quick math.
    float cosTheta = sqrt(cosSqTheta);//30us
    //Theta is the angle made by the Z axis of the mpu with the vertical. Az*cos(theta) = actual vertical acceleration. 
    if(A[2]<1) //implementing these if conditions because the Accelgyro has an asymetric range of measurement.
    {           //the nominal value of Az is at 10 not 0. Therefore the max value of Az that can be measured is 20 while 
      A[2] = 1;  //the min value is -20. The downward gap is more than the upward gap. This becomes problematic on systems
    }         // where you have a lot of vibrations.
    if(A[2]>19)
    {
      A[2] = 19;
    }
    az = A[2]*cosTheta - 9.8*cosSqTheta; // 20us. Trust me, this is the final formula. I ll change it if i figure its wrong.
                                          //For now, deal with it.
  }
}
