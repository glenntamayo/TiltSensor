/*  *****************************************
 *  ADXL345_Calibration
 *  Modified by Glenn Gil David Tamayo from
 *  Sparkfun's ADXL345 Hook Up Guide Calibration Example 
 *  E.Robert @ SparkFun Electronics
 *  Created: Jul 13, 2016
 *  Updated: Sep 13, 2016
 *  *****************************************/

//#define ACCEL_ADXL345
#define ACCEL_MPU6050

#ifdef ACCEL_ADXL345
  #include <SparkFun_ADXL345.h>
    /*********** COMMUNICATION SELECTION ***********/
  /*    Comment Out The One You Are Not Using    */
  //ADXL345 adxl = ADXL345(10);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);
  ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION
  const int ScaleFactor = 256;
#endif
#ifdef ACCEL_MPU6050
  #include "I2Cdev.h"
  #include "MPU6050.h"
  
  // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
  // is used in I2Cdev.h
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      #include "Wire.h"
  #endif
  MPU6050 accelgyro(0x69); // <-- use for AD0 high
  const int ScaleFactor = 16384;
#endif

#include <EEPROM.h>


/****************** VARIABLES ******************/
/*                                             */
int AccelMinX = 0;
int AccelMaxX = 0;
int AccelMinY = 0;
int AccelMaxY = 0;
int AccelMinZ = 0;
int AccelMaxZ = 0; 

int accX = 0;
int accY = 0;
int accZ = 0;

byte response;

/************** DEFINED VARIABLES **************/
/*                                             */
int offsetX = 0;
int offsetY = 0;
int offsetZ = 0;

float gainX = 0;
float gainY = 0;
float gainZ = 0;

struct adxl345Data {
  char node[16];
  int xOffset;
  int yOffset;
  int zOffset;
  float xGain;
  float yGain;
  float zGain;
};

char deviceNameChar[16];
int x,y,z;  
/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void setup()
{
  Serial.begin(115200);                 // Start the serial terminal
  Serial.println("SparkFun ADXL345 Accelerometer Breakout Calibration");
  Serial.println();
  #ifdef ACCEL_ADXL345
    adxl.powerOn();                     // Power on the ADXL345

    adxl.writeTo(ADXL345_FIFO_CTL, 0x0);
    adxl.writeTo(ADXL345_BW_RATE, ADXL345_BW_50);
  
    adxl.setRangeSetting(2);           // Give the range settings
                                        // Accepted values are 2g, 4g, 8g or 16g
                                        // Higher Values = Wider Measurement Range
                                        // Lower Values = Greater Sensitivity
                                        
    adxl.setSpiBit(0);                // Configure the device: 4 wire SPI mode = '0' or 3 wire SPI mode = 1
                                        // Default: Set to 1
                                      // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 
  #endif
  #ifdef ACCEL_MPU6050
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    accelgyro.initialize();
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  #endif
  
}

/****************** MAIN CODE ******************/
/*  Accelerometer Readings and Min/Max Values  */
void loop() {

  while (Serial.available()) {
    response = Serial.read();                    
    //Serial.println(response);
  }
  

  if((response == 110) || (response == 78)) {
    calibrate();  
  }

  if((response == 89) || (response == 121)) {
    Serial.print("Enter device name: ");
    while (!Serial.available()){}       // Waiting for character to be sent to Serial
    char character;
    String deviceName = "";
    delay(500);
    while(Serial.available()) {
      character = Serial.read();
      deviceName.concat(character);
    }
    if (deviceName != "") {
      Serial.println(deviceName);
    }                
    deviceName.toCharArray(deviceNameChar, deviceName.length()+1);
    writeOffsetsToEEPROM();  
  }
  
  Serial.println("Send: 'N' to display values,    'Y' to accept and write offset and gain values.");
  while (!Serial.available()){}       // Waiting for character to be sent to Serial
}

void calibrate() {
   // Get the Accelerometer Readings
  #ifdef ACCEL_ADXL345
    adxl.readAccel(&x, &y, &z);
  #endif
  #ifdef ACCEL_MPU6050
    accelgyro.getAcceleration(&x, &y, &z);
  #endif  

  if(x < AccelMinX) AccelMinX = x;
  if(x > AccelMaxX) AccelMaxX = x;

  if(y < AccelMinY) AccelMinY = y;
  if(y > AccelMaxY) AccelMaxY = y;

  if(z < AccelMinZ) AccelMinZ = z;
  if(z > AccelMaxZ) AccelMaxZ = z;

  Serial.print("Accel Minimums: "); Serial.print(AccelMinX); Serial.print("  ");Serial.print(AccelMinY); Serial.print("  "); Serial.print(AccelMinZ); Serial.println();
  Serial.print("Accel Maximums: "); Serial.print(AccelMaxX); Serial.print("  ");Serial.print(AccelMaxY); Serial.print("  "); Serial.print(AccelMaxZ); Serial.println();
  
  /* Note: Must perform offset and gain calculations prior to seeing updated results
  /  Refer to SparkFun ADXL345 Hook Up Guide: https://learn.sparkfun.com/tutorials/adxl345-hookup-guide
  /  offsetAxis = 0.5 * (Acel+1g + Accel-1g)
  /  gainAxis = 0.5 * ((Acel+1g - Accel-1g)/1g) */

  offsetX = 0.5 * (AccelMaxX + AccelMinX);
  offsetY = 0.5 * (AccelMaxY + AccelMinY);
  offsetZ = 0.5 * (AccelMaxZ + AccelMinZ);
  Serial.print("Offsets: "); Serial.print(offsetX); Serial.print("  "); Serial.print(offsetY); Serial.print("  ");  Serial.println(offsetZ);
  gainX = 0.5 * (AccelMaxX - AccelMinX) / ScaleFactor;
  gainY = 0.5 * (AccelMaxY - AccelMinY) / ScaleFactor;
  gainZ = 0.5 * (AccelMaxZ - AccelMinZ) / ScaleFactor;
  Serial.print("Gains: "); Serial.print(gainX); Serial.print("  "); Serial.print(gainY); Serial.print("  ");  Serial.println(gainZ);;
  Serial.println();
}

void writeOffsetsToEEPROM() {
  adxl345Data accelData;
  strcpy(accelData.node, deviceNameChar);
  accelData.xOffset = offsetX;
  accelData.yOffset = offsetY;
  accelData.zOffset = offsetZ;
  accelData.xGain = gainX;
  accelData.yGain = gainY;
  accelData.zGain = gainZ;

  EEPROM.put(0, accelData);
  delay(1000);
  
  adxl345Data getAccelData;
  EEPROM.get(0, getAccelData);
  Serial.println("Successfully written to EEPROM:");
  Serial.println(getAccelData.node);
  Serial.println(getAccelData.xOffset);
  Serial.println(getAccelData.yOffset);
  Serial.println(getAccelData.zOffset);
  Serial.println(getAccelData.xGain);
  Serial.println(getAccelData.yGain);
  Serial.println(getAccelData.zGain);
}
