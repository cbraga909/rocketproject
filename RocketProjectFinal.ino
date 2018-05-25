//#include <Wire.h>
#include "Rocket.h"
bool trigger=false;
void setup()
{
  Wire.begin();
  PayloadInitialization();
  bmp180Calibration(); 
  speaker();
}
void loop()
{
  sensordata(acc, gyro, mag, temp, ypr);
  temperature = bmp180GetTemperature(bmp180ReadUT());
  *pres=pressure = bmp180GetPressure(bmp180ReadUP());
  *alt=altitude = (float)44330 * (1 - pow(((float) pressure/p0), 0.190295));
  //*alt=Alt();
  
  logger = sdlogger(logger, acc);
  //trigger once based on acc
  if (logger && !settime && !trigger)
  {
    FileFirstLine();
    timeStamp = millis();
    settime = true;
    trigger=true;
  }
  // stop logging while certen altitude
  if(altitude<trigerDistance&&settime)
  {
    settime=false;
  }
  //
  if (settime)
  {
    //sddata
    DataWrite( alt,  pres,  temp,  acc,  gyro,  mag,   ypr,  timeStamp);

  }
  
//  Serial.print(acc[0]);
//  Serial.print(",");
//  Serial.print(acc[1]);
//  Serial.print(",");
//  Serial.println(acc[2]);
}

