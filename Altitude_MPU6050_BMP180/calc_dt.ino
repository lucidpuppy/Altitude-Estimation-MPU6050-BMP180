void calc_dt()
{
  spudnut=micros();
  dt=(double)(spudnut-donut) * 0.000001;
  donut=spudnut;
  Serial.print("\nFrequency\t");
  Serial.print(1/dt);
}
