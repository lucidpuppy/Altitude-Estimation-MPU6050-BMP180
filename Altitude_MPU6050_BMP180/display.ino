void display_stuff()
{
  display_bmp();
  display_imu();
}

void display_bmp()
{
    Serial.print("\nAltitude Velocity BMP\t");
    Serial.print(h_est_baro);
    Serial.print(" m");
    //Serial.print("\t");
    //Serial.print(velocity_bmp);
    //Serial.print(" m/s");
}

void display_imu()
{
  Serial.print("\nAz");
  Serial.print(az);
}
