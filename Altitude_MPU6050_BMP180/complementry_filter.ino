void complementry_filter()
{
  h_est_baro = barometer.GetAltitude(seaLevelPressure)-alt_reference;
  h_est_baro=  0.05* h_est_baro +  0.95*prev_h_est_baro;                                            //LOW pass filter
  
  get_imu();
  az= 0.05 * az + 0.95 *az_prev;                                                                    //LOW pass filter
  az_prev=az;
  
  vz_est_acc=(double) prev_vz_est + az * dt;                                                        //updating velocity estimate  from acceleration data
  vz_est_baro=(double) (h_est_baro-prev_h_est_baro)/dt;                                             //velocity estimate from barometer..fluctuates alot
  prev_h_est_baro= h_est_baro;
  
  
  vz_est= gamma*(vz_est_acc) + (1-gamma)*(vz_est_baro);                                                //complmentry filter
  vz_est= 0.05*vz_est + 0.95*prev_vz_est;                                                             //LOW pass filter
  prev_vz_est=(double)vz_est;
  
  h_est_acc=(double) prev_h_est + prev_vz_est * dt + 0.5*az*dt*dt;                                    //s= s_prev+ ut+ 0.5 a dt^2
  h_est_acc= 0.05*h_est_acc + 0.95*prev_h_est_acc;                                                    //LOW pass filter
  prev_h_est_acc=h_est_acc;
  
  h_est=(double) beta*(h_est_acc) + (1-beta)*(h_est_baro);                                            //complemntry filter
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

