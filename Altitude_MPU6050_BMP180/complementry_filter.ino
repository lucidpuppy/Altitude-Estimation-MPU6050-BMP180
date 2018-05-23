void complementry_filter()
{
  float H[3],baro_SD=0;
  for(int i = 0;i<3;i++)
  {
    H[i] = barometer.GetAltitude(seaLevelPressure)-alt_reference;
    h_est_baro += H[i];
  }
  h_est_baro/=3; // mean
  //Standard deviation (aka error for us) 
  for(i = 0;i<3;i++)
  {
    baro_SD += pow((H[i] - h_est_baro),2);
  }
  baro_SD = sqrt(baro_SD/2)/h_est_baro); //get SD in fractions
  h_est_baro=  0.05* h_est_baro +  0.95*prev_h_est_baro;                                            //LOW pass filter
  //now we have the height estimate and it's standard deviation.
  
  get_imu();
  float dummyA = az,Accel_SD;
  az= 0.05 * az + 0.95 *az_prev;                                                                    //LOW pass filter
  az_prev=az;
  Accel_SD = abs(dummyA - az)/az; //adhoc way of finding variance in acceleration. just take the difference between the 
                              //low pass value and the current value. if they differ by a lot, then it means there is a lot of error
  //this is basically like using a gaussian filter. If you're getting your exam answer scripts back and everyone up to your roll number 
  //had their marks around 60-65 then you'd expect your marks to be near 60-65 too. if you get 10, that would indicate that there's been
  // some problem. (if you had 90+ marks you'd probably shut up and take the sheet home with you so that the teacher can't even look at it 
  //again).
  
  vz_est_acc=(double) prev_vz_est + az * dt;                  //updating velocity estimate  from acceleration data
  vz_est_baro=(double) (h_est_baro-prev_h_est_baro)/dt;       //velocity estimate from barometer..fluctuates alot
  prev_h_est_baro= h_est_baro;
  
  /*since we don't have the covariance matrix nonsense here, its not a true kalman filter, 
    Psuedo Kalman gain  = error in the estimate(Accel_SD)
                  --------------------------------
       (error in estimate(Accel_SD) + error in measurement(baro_SD))
  */
  float PKG = Accel_SD/(Accel_SD + Baro_SD);
  
  vz_est= (1-PKG)*(vz_est_acc) + PKG*(vz_est_baro);           //Psuedo Kalman filter
  vz_est= 0.05*vz_est + 0.95*prev_vz_est;                   //LOW pass filter
  prev_vz_est=(double)vz_est;
  // h_est_acc=(double) prev_h_est + prev_vz_est * dt + 0.5*az*dt*dt;  this line is gave me cancer.
  /*
    We just put so much effort into filtering the vertical velocity and now you're bringing this unfiltered scum (az) back 
    into the equation. Why bro. 
  */
  h_est_acc=(double) prev_h_est + vz_est * dt;                                    //s= s_prev + new_velocity*dt  
  h_est_acc= 0.05*h_est_acc + 0.95*prev_h_est_acc;                              //LOW pass filter
  prev_h_est_acc=h_est_acc;
  
  /*look, I already used up Accel_SD. so lemme use the spiky boy I introduced in the imu.ino
    beta is given as 0.95 so i already got half the stuff covered.
    however, this spiky boi has a slightly more spiky behaviour.
    here, see for yoself - http://www.wolframalpha.com/input/?i=0.05%2F(1%2B%7Cx%7C%5E0.5)
    let the trust factor be lawda_lehsun
    lawda_lehsun = 0.05/(1+sqrt(baro_SD)) 
  */
  float lawda_lehsun = 0.05/(1+sqrt(baro_SD)); 
  
  h_est=(double) (1-lawda_lehsun)*(h_est_acc) + lawda_lehsun*(h_est_baro);  //complemntry filter
  prev_h_est=(double)h_est;

  
  Serial.print("\nv_est_baro v_est_acc v_est h_est_baro h_est_acc h_est\t");
  Serial.print(vz_est_baro);
  Serial.print("\t");
  Serial.print(vz_est_acc);
  Serial.print("\t");
  Serial.print(vz_est);
  Serial.print("\t");
  Serial.print(h_est_baro);
  Serial.print("\t");
  Serial.print(h_est_acc);
  Serial.print("\t");
  Serial.print(h_est);
}

