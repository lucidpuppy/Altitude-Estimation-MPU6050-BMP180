void get_bmp()
{
  h_est_baro = barometer.GetAltitude(seaLevelPressure)-alt_reference;
}

void setup_bmp()
{
  barometer = BMP180();

  if(barometer.EnsureConnected())
  { 
    Serial.println("Connected to BMP180.");
    barometer.SoftReset();
    barometer.Initialize();
  }
  else Serial.println("Nagar Palika ko bulao.");
}
/*
void running_avg()
{
  for(byte i=24; i>0; i--) { alt[i]=alt[i-1]; }
  alt[0]=altitude_bmp;
  
  //display_past_alt();
  
  if(count==25) run_avg_status=true;
  else altitude_bmp=alt[0];

  if(run_avg_status==true)
  {
    float dummy_altitude=0;
    //Serial.print("\nRunning avg running");
    for(byte j=0; j<25; j++) { dummy_altitude=dummy_altitude + alt[j]; }
    altitude_bmp=dummy_altitude/25;
  }
  //display_past_alt();
  if(count<25) count++;
}

void display_past_alt()
{
  Serial.print("\nPast_altitude\t");
  for(byte k=0; k<25; k++)
  {
    Serial.print(alt[k]);
    Serial.print("  ");
  }
}
*/
