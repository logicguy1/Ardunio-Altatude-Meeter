 /* ********************************************************************** 
 *  Micro SD card Adapter
 *
 * Circuit:
 * 
 *  |                              |
 *  | GND  VCC  MISO MOSI  SCK  CS |
 *  |--|----|----|----|----|----|--| 
 *     |    |    |    |    |    |  
 *    GND   5V  D12  D11  D13  D10
 *
 * Pmod NAV
 * 
 * Circuit:
 * 
 *  |                              |
 *  | 3V3  GND  SCK       SDI      |
 *  |--|----|----|----|----|----|--| 
 *     |    |    |    |    |    |  
 *    3V3  GND   A5        A4  
 ****************************************************************************/

// The earth's magnetic field varies according to its location.
// Add or subtract a constant to get the right value
// of the magnetic field using the following site
// http://www.ngdc.noaa.gov/geomag-web/#declination

#define DECLINATION -3.73 // declination (in degrees) in Cluj-Napoca (Romania).
#define PRINT_CALCULATED  //print calculated values

// Call of libraries
#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include <SimpleTimer.h>
#include <SPI.h>
#include <SD.h>
#include <LPS25HBSensor.h>

#if defined(ARDUINO_SAM_DUE)
#define DEV_I2C Wire1   //Define which I2C bus is used. Wire1 for the Arduino Due
#define SerialPort Serial
#else
#define DEV_I2C Wire    //Or Wire
#define SerialPort Serial
#endif
// defining module addresses
#define LSM9DS1_M 0x1E  //magnetometer
#define LSM9DS1_AG 0x6B //accelerometer and gyroscope

// Components.
LPS25HBSensor PressTemp(&DEV_I2C);


LSM9DS1 imu; // Creation of the object
File myFile;

int readingNr = 0;
int ONEplay = 0;

unsigned long myTime;

void setup(void)
{
  Serial.begin(9600); // initialization of serial communication
  
  // Initialize I2C bus.
  DEV_I2C.begin();

  // Initialize components.
  PressTemp.begin();
  PressTemp.Enable();

  
  Wire.begin();     //initialization of the I2C communication
  imu.settings.device.commInterface = IMU_MODE_I2C; // initialization of the module
  imu.settings.device.mAddress = LSM9DS1_M;        //setting up addresses
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin()) //display error message if that's the case
  {
    Serial.println("Communication problem.");
    while (1);
  }

}

void loop()
{

  myTime = millis();

  //measure 1.
  if ( imu.gyroAvailable() )
  {
    imu.readGyro(); //measure with the gyroscope
  }
  if ( imu.accelAvailable() )
  {
    imu.readAccel(); //measure with the accelerometer
  }
  if ( imu.magAvailable() )
  {
    imu.readMag(); //measure with the magnetometer
  }

  //display data
 
  calc(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
    
}

//display additional calculated values
void calc(float ax, float ay, float az, float mx, float my, float mz)
{
readingNr++;

// Read pressure and temperature.
  float pressure, temperature;
  PressTemp.GetPressure(&pressure);
  PressTemp.GetTemperature(&temperature);

float gX = imu.calcGyro(imu.gx);
float gY = imu.calcGyro(imu.gy);
float gZ = imu.calcGyro(imu.gz);

float aX = imu.calcAccel(imu.ax);
float aY = imu.calcAccel(imu.ay);
float aZ = imu.calcAccel(imu.az);


float mX = imu.calcMag(imu.mx);
float mY = imu.calcMag(imu.my);
float mZ = imu.calcMag(imu.mz);

  float roll = atan2(ay, az); //calculate roll
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));  //calculate pitch
  float heading;  //variable for hading

  //calculate heading
  if (my == 0) {
    heading = (mx < 0) ? PI : 0;
  }
  else {
    heading = atan2(mx, my);
  }

  //correct heading according to declination
  heading -= DECLINATION * PI / 180;
  if (heading > PI) {
    heading -= (2 * PI);
  }
  else if (heading < -PI) {
    heading += (2 * PI);
  }
  else if (heading < 0) {
    heading += 2 * PI;
  }

  //convert values in degree
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;

  if(readingNr == 1){write_to_file("#,Tid,hPa,Temp,gX,gY,gZ,aX,aY,aZ,mX,mY,mZ,Pitch,Roll,Heading");}
String test = String(readingNr)+","+String(myTime)+","+String(pressure)+","+String(temperature)
                +","+String(gX)+","+String(gY)+","+String(gZ)
                +","+String(aX)+","+String(aY)+","+String(aZ)
                +","+String(mX)+","+String(mY)+","+String(mZ)
                +","+String(pitch)+","+String(roll)+","+String(heading)+";";

Serial.println(test);
write_to_file(test);
 delay(250); //Normal 1000ms

}
void write_to_file(String data) {
  if (!SD.begin(10)) { // Initialize the sd card 
    Serial.println("initialization failed!");
    while (1);
  }

  myFile = SD.open("data.txt", FILE_WRITE); // Open the file 
  
  if (!myFile) {
    // if the file didn't open, print an error:
    Serial.println("error opening data.txt");
    while (1);
  }

  myFile.println(data);
  myFile.close();
  //Serial.println("Done.");
}
