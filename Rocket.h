#ifndef _ROCKETH_
#define _ROCKETH_

#include "quaternionFilters.h"
#include "MPU9250.h"
//#define AHRS true         // Set to false for basic data read
//#define SerialDebug true  // Set to true to get Serial output for debugging
//#include <SD.h>
#include <SdFat.h>
SdFat SD;
//#include <Wire.h>
#include "BMP180Lib.c"
#define BMP180_ADDRESS 0x77
const int CS_PIN = 10;
const float p0 = 101325; // Pressure at sea level (Pa)
float altitude;
short temperature;
long pressure;
// Pin definitions
int intPin = 7;  // These can be changed, 2 and 3 are the Arduinos ext int pins

#define trigerDistance 880
#define trigerAcc 1500
MPU9250 myIMU;

double acc[3];
double gyro[3];
int mag[3];
float temp[1];
float ypr[3];
float alt[1];
long pres[1];

bool logger = false;
bool settime = false;
long timeStamp;

void PayloadInitialization()
{

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  //Serial.println("MPU9250 is online...");
  // Start by performing self test and reporting values
  //myIMU.MPU9250SelfTest(myIMU.SelfTest);
  // Calibrate gyro and accelerometers, load biases in bias registers
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initMPU9250();
  // Initialize device for active mode read of acclerometer, gyroscope, and
  // temperature
  //Serial.println("MPU9250 initialized for active data mode....");
  // Get magnetometer calibration from AK8963 ROM
  myIMU.initAK8963(myIMU.magCalibration);
  // Initialize device for active mode read of magnetometer
  //Serial.println("AK8963 initialized for active data mode....");
  //Serial.println("Initializing Card");
  //CS pin is an output
  pinMode(10, OUTPUT);
  //  if(!SD.begin(CS_PIN)) //If SD.begin fails, return error message and end
  //  {
  //    Serial.println("Card Failure");
  //    return;
  //  }
  //  Serial.println("Card Ready"); //Otherwise, execute program

}

//David Pan
void sensordata(double *acc, double *gyro, int *mag, float *temp, float *ypr)
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    //myIMU.getAres();



    myIMU.ax = (float)myIMU.accelCount[0] * 16.0 / 32768.0; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * 16.0 / 32768.0; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * 16.0 / 32768.0; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    //myIMU.getGres();

    myIMU.gx = (float)myIMU.gyroCount[0] * 500.0 / 32768.0;
    myIMU.gy = (float)myIMU.gyroCount[1] * 500.0 / 32768.0;
    myIMU.gz = (float)myIMU.gyroCount[2] * 500.0 / 32768.0;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    //   myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * 10.*4912. / 8190. * myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1] * 10.*4912. / 8190. * myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2] * 10.*4912. / 8190. * myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  // myIMU.delt_t = millis() - myIMU.count;

  // update LCD once per half-second independent of read rate
  //  if (myIMU.delt_t > 500)
  //  {



  acc[0] = (int)1000 * myIMU.ax;
  acc[1] = (int)1000 * myIMU.ay;
  acc[2] = (int)1000 * myIMU.az;
  gyro[0] = myIMU.gx;
  gyro[1] = myIMU.gy;
  gyro[2] = myIMU.gz;
  mag[0] = (int)myIMU.mx;
  mag[1] = (int)myIMU.my;
  mag[2] = (int)myIMU.mz;


  myIMU.tempCount = myIMU.readTempData();  // Read the adc values


  *temp = ((float)myIMU.tempCount) / 333.87 + 21.0;



  myIMU.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                            *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1) * *(getQ() + 1)
                    - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
  myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                              *(getQ() + 2)));
  myIMU.roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
                             *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1)
                     - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.yaw *= RAD_TO_DEG;
  // Declination of SparkFun Electronics (40Â°05'26.6"N 105Â°11'05.9"W) is
  //   8Â° 30' E  Â± 0Â° 21' (or 8.5Â°) on 2016-07-19
  // - http://www.ngdc.noaa.gov/geomag-web/#declination
  myIMU.yaw -= 8.5;
  myIMU.roll *= RAD_TO_DEG;


  ypr[0] = myIMU.yaw;
  ypr[1] = myIMU.pitch;
  ypr[2] = myIMU.roll;
  // myIMU.count = millis();


  //}
}

//This function writes data to the SD Card
//Christina Braga














void FileFirstLine()
{
  File dataFile = SD.open("rocket.csv", FILE_WRITE);
  if (dataFile)
  {
    dataFile.print("Time");
    dataFile.print(",");
    dataFile.print("Altitude");
    dataFile.print(",");
    dataFile.print("Pressure");
    dataFile.print(",");
    dataFile.print("Acceleration X");
    dataFile.print(",");
    dataFile.print("Acceleration Y");
    dataFile.print(",");
    dataFile.print("Acceleration Z");
    dataFile.print(",");
    dataFile.print("Gyro X");
    dataFile.print(",");
    dataFile.print("Gyro Y");
    dataFile.print(",");
    dataFile.print("Gyro Z");
    dataFile.print(",");
    dataFile.print("Yaw");
    dataFile.print(",");
    dataFile.print("Pitch");
    dataFile.print(",");
    dataFile.print("Roll");
    dataFile.print(",");
    dataFile.print("Mag X");
    dataFile.print(",");
    dataFile.print("Mag Y");
    dataFile.print(",");
    dataFile.print("Mag Z");
    dataFile.println("Temperature");
    dataFile.close();
  }
  else
  {
    //tone(5, 440);
    digitalWrite(5, HIGH);
  }
}


void DataWrite(float *alt, long *pres, float *temp, double *acc, double *gyro, int *mag,  float *ypr, long timeStamp)
{
  //Serial.println("sd");
  File dataFile = SD.open("rocket.csv", FILE_WRITE);
  if (dataFile)
  {
    dataFile.print(millis() - timeStamp);
    dataFile.print(",");
    dataFile.print(*alt);
    dataFile.print(",");
    dataFile.print(*pres);
    dataFile.print(",");

    for (int i = 0; i < 3; i++)
    {
      dataFile.print(acc[i]);
      dataFile.print(",");
    }
    for (int i = 0; i < 3; i++)
    {
      dataFile.print(gyro[i]);
      dataFile.print(",");
    }
    for (int i = 0; i < 3; i++)
    {
      dataFile.print(ypr[i]);
      dataFile.print(",");
    }
    for (int i = 0; i < 3; i++)
    {
      dataFile.print(mag[i]);
      dataFile.print(",");
    }
    dataFile.println(*temp);

    dataFile.close();
  }
  else
  {
    //tone(5, 440);
    digitalWrite(5, HIGH);
  }
}

//void SDInitialization();
////This function initializes the SD card
////Christina Braga
//void SDInitialization()
//{
//  Serial.println("Initializing Card");
//  //CS pin is an output
//  pinMode(10, OUTPUT);
//  if(!SD.begin(CS_PIN)) //If SD.begin fails, return error message and end
//  {
//    Serial.println("Card Failure");
//    return;
//  }
//  Serial.println("Card Ready"); //Otherwise, execute program
//}

char bmp180Read(unsigned char address)
{
  unsigned char data;
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDRESS, 1);
  while (!Wire.available());
  return Wire.read();
}
// Read 2 bytes from the BMP180
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp180ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP180_ADDRESS, 2);
  while (Wire.available() < 2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb << 8 | lsb;
}
void bmp180Calibration()
{


  ac1  = 9648;
  ac2 = -1295;
  ac3 = -14686;
  ac4 = 33716;
  ac5 = 25159;
  ac6 = 18036;
  b1  = 6515;
  b2  = 56;
  mb  = -32768;
  mc  = -11786;
  md  = 2723;

}
// Read the uncompensated temperature value
unsigned int bmp180ReadUT()
{
  unsigned int ut;
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  // Wait at least 4.5ms
  delay(5);
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp180ReadInt(0xF6);
  return ut;
}
// Read the uncompensated pressure value
unsigned long bmp180ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS << 6));
  Wire.endTransmission();
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3 << OSS));
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDRESS, 3);
  // Wait for data to become available
  while (Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8 - OSS);
  return up;
}

short Temp()
{
  temperature = bmp180GetTemperature(bmp180ReadUT());
  return temperature;
}

float Alt()
{
  altitude = (float)44330 * (1 - pow(((float) pressure / p0), 0.190295));
  return altitude;
}

long Press()
{
  pressure = bmp180GetPressure(bmp180ReadUP());
  return pressure;
}









//Gomzalo

void speaker()
{

  for (int i = 0; i < 5; i++)
  {
    digitalWrite(5, HIGH);
    delay(500);
    digitalWrite(5, LOW);
    delay(500);
  }
}


bool sdlogger(bool logger, double *alt)
{
  if (acc[0] > trigerAcc || acc[1] > trigerAcc || acc[2] > trigerAcc)
  {
    logger = true;
  }
  return logger;
}



#endif
