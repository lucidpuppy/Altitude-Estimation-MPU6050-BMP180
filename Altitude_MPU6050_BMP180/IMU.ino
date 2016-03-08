void setup_imu()                                                                      //Real world Accel with gravity removed
{
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

    // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
    mpu.setXGyroOffset(60);                                           //calculate your own offsets and write put them here
    mpu.setYGyroOffset(-29);
    mpu.setZGyroOffset(24);
    mpu.setXAccelOffset(947);
    mpu.setYAccelOffset(-1703);
    mpu.setZAccelOffset(1599);
  
  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else 
    {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
    pinMode(LED_PIN, OUTPUT);
}


void get_imu()
{
  prick:
  if (!dmpReady) return;
   while (!mpuInterrupt && fifoCount < packetSize) {}
   mpuInterrupt = false;
   mpuIntStatus = mpu.getIntStatus();

   fifoCount = mpu.getFIFOCount();

   if ((mpuIntStatus & 0x10) || fifoCount == 1024)
   {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & 0x02)
    {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
        
     fifoCount -= packetSize;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            /*
            Serial.print("\naworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
           */
           
        az=(double) -(aaWorld.z * 9.8)/16384;
        //del_altitude_accel=velocity_accel*dt;
        // blink LED to indicate activite
    }
   else goto prick;
}
