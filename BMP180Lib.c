// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
// b5 is calculated in bmp180GetTemperature(...), this variable is also used in bmp180GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5;
// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
const unsigned char OSS = 2; // Oversampling Setting
short bmp180GetTemperature(unsigned int ut)
{
 long x1, x2;

 x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
 x2 = ((long)mc << 11)/(x1 + md);
 b5 = x1 + x2;
 return ((b5 + 8)>>4);
}
// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp180GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp180GetPressure(unsigned long up)
{
 long x1, x2, x3, b3, b6, p;
 unsigned long b4, b7;
 b6 = b5 - 4000;
 // Calculate B3
 x1 = (b2 * (b6 * b6)>>12)>>11;
 x2 = (ac2 * b6)>>11;
 x3 = x1 + x2;
 b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

 // Calculate B4
 x1 = (ac3 * b6)>>13;
 x2 = (b1 * ((b6 * b6)>>12))>>16;
 x3 = ((x1 + x2) + 2)>>2;
 b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

 b7 = ((unsigned long)(up - b3) * (50000>>OSS));
 if (b7 < 0x80000000)
 p = (b7<<1)/b4;
 else
 p = (b7/b4)<<1;
 x1 = (p>>8) * (p>>8);
 x1 = (x1 * 3038)>>16;
 x2 = (-7357 * p)>>16;
 p += (x1 + x2 + 3791)>>4;
 return p;
}
