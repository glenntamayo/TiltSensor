//Created by Glenn Gil David Tamayo
//#define ACCEL_ADXL345
#define ACCEL_MPU6050

#ifdef ACCEL_ADXL345
  #include <SparkFun_ADXL345.h>
#endif

#include <SPI.h>
#include <SdFat.h>
#include <RTClib.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>

#define ADXL_POWERCTL_WAKEUP          0x0
#define ADXL_POWERCTL_SLEEP           0x4
#define ADXL_POWERCTL_MEASURE         0x8

#define ADXL345_FIFOMODE_BYPASS       0x0
#define ADXL345_FIFOMODE_FIFO         0x5F
#define ADXL345_FIFOMODE_STREAM       0x9F
#define ADXL345_FIFOMODE_TRIGGER      0xDF

const char lblDEVID[] PROGMEM = "Device ID                         :";
const char lblOFSX[] PROGMEM = "X-Axis Offset                     :";
const char lblOFSY[] PROGMEM = "Y-Axis Offset                     :";
const char lblOFSZ[] PROGMEM = "Z-Axis Offset                     :";
const char lblBWRATE[] PROGMEM = "Data Rate and Power Mode Control  :";
const char lblPOWERCTL[] PROGMEM = "Power-Saving Features Control     :";
const char lblINTENABLE[] PROGMEM = "Interrupt Enable Control          :";
const char lblINTMAP[] PROGMEM = "Interrupt Mapping Control         :";
const char lblINTSOURCE[] PROGMEM = "Source of Interrupts              :";
const char lblDATAFORMAT[] PROGMEM = "Data Format                       :";
const char lblFIFOCTL[] PROGMEM = "FIFO Control                      :";
const char lblFIFOSTATUS[] PROGMEM = "FIFO Status                       :";
const char lblxOffset[] PROGMEM = "Manual X-Axis Offset              :";
const char lblyOffset[] PROGMEM = "Manual Y-Axis Offset              :";
const char lblzOffset[] PROGMEM = "Manual Z-Axis Offset              :";
const char lblxGain[] PROGMEM = "Manual X-Axis Gain                :";
const char lblyGain[] PROGMEM = "Manual Y-Axis Gain                :";
const char lblzGain[] PROGMEM = "Manual Z-Axis Gain                :";
const char lblNode[] PROGMEM = "Node                              :";

const char* const string_table[] PROGMEM = {lblDEVID, lblOFSX,
lblOFSY, lblOFSZ, lblBWRATE, lblPOWERCTL, lblINTENABLE,
lblINTMAP, lblINTSOURCE, lblDATAFORMAT, lblFIFOCTL, lblFIFOSTATUS,
lblxOffset, lblyOffset, lblzOffset, lblxGain, lblyGain, lblzGain,
lblNode
};

char lblBuffer[40];

/***************** ARDUINO PIN CONFIGURATION ****************/
const int chipSelectSD = 4;
/***************** ARDUINO PIN CONFIGURATION ****************/

/******************** CLASS CONSTRUCTORS ********************/
SdFat SD;
File myFile;
RTC_DS1307 RTC;
#ifdef ACCEL_ADXL345
  ADXL345 adxl = ADXL345();
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
#endif
/******************** CLASS CONSTRUCTORS ********************/

/********************* GLOBAL VARIABLES *********************/
const unsigned long delayStart = 1000;
String fileName;
char fileNameChar[10];

const float pi = 3.141592653589793;

struct eepromData {
  char node[16];
  int xOffset;
  int yOffset;
  int zOffset;
  float xGain;
  float yGain;
  float zGain;  
};
eepromData accData;
String node;
int xOffset = 0;
int yOffset = 0;
int zOffset = 0;
float xGain = 1;
float yGain = 1;
float zGain = 1;
int xAdj = 0;
int yAdj = 0;
int zAdj = 0;

int16_t x, y, z;
double angle[3];

unsigned long previousSdFlushMillis = 0;
unsigned long previousMillis = 0;
//const unsigned long readInterval = 600000;
//const unsigned long SdFlushInterval = 600000;
const unsigned long readInterval = 1000;
const unsigned long SdFlushInterval = 1000;

/********************* GLOBAL VARIABLES *********************/

void setup(){
  Serial.begin(115200);
  getEEPROMdata();    
  #ifdef ACCEL_ADXL345
    ADXL345Setup(ADXL345_BW_50, ADXL345_FIFOMODE_BYPASS);
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
  SDModuleSetup(chipSelectSD);
  RTC.begin();
  DateTime now = RTC.now();
  fileSetup(now);
  #ifdef ACCEL_ADXL345
    ADXL345ReadConfiguration();
  #endif
  delay(delayStart);
  #ifdef ACCEL_ADXL345
    adxl.writeTo(ADXL345_POWER_CTL, ADXL_POWERCTL_MEASURE);
  #endif
}

void loop(){
  unsigned long currentMillis = millis();

  if(currentMillis - previousMillis > readInterval) {
    adjAccel();
    double modulusG = sqrt(pow(xAdj, 2) + pow(yAdj, 2) + pow(zAdj, 2));
    angle[0] = acos(xAdj / modulusG) * 180/pi;
    angle[1] = acos(yAdj / modulusG) * 180/pi;
    angle[2] = acos(zAdj / modulusG) * 180/pi;

    DateTime now = RTC.now();
    /*
    myFile.print(now.month());
    myFile.print('-');
    myFile.print(now.day());
    myFile.print('-');
    myFile.print(now.year());
    myFile.print('-');
    myFile.print(now.hour());
    myFile.print(':');
    myFile.print(now.minute());
    myFile.print(':');
    myFile.print(now.second());
    myFile.print(", ");

    writeToFile(angle[0], angle[1], angle[2]);
    */
    readingsToSerial(currentMillis, angle[0], angle[1], angle[2]);
    previousMillis = currentMillis;
  }
} 

void adjAccel() {
  #ifdef ACCEL_ADXL345
    adxl.readAccel(&x, &y, &z);
  #endif
  #ifdef ACCEL_MPU6050
    accelgyro.getAcceleration(&x, &y, &z);
  #endif
  xAdj = (float)(x - xOffset) / xGain;
  yAdj = (float)(y - yOffset) / yGain;
  zAdj = (float)(z - zOffset) / zGain;  
}



#ifdef ACCEL_ADXL345
  void ADXL345Setup(byte rate, byte FIFOMode) {
    
    adxl.writeTo(ADXL345_BW_RATE, rate);
  
    adxl.writeTo(ADXL345_FIFO_CTL, FIFOMode);
  
    //adxl.setAxisOffset(accData.xOffset, accData.yOffset, accData.zOffset);
    
    adxl.setInterruptMapping(1, 0);
    adxl.setInterrupt(1 ,1);
  
    // Give the range settings
    // Accepted values are 2g, 4g, 8g or 16g
    // Higher Values = Wider Measurement Range
    // Lower Values = Greater Sensitivity
    adxl.setRangeSetting(2);
  
    // Set to activate movement detection in the axes 
    //"adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
    adxl.setActivityXYZ(0, 0, 0);
  
    // Set activity   // Inactivity thresholds (0-255)
    adxl.setActivityThreshold(75); // 62.5mg per increment   
  
    // Set to detect inactivity in all the axes 
    //"adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
    adxl.setInactivityXYZ(0, 0, 0);     
  
    // Set inactivity // Inactivity thresholds (0-255)
    adxl.setInactivityThreshold(75);    // 62.5mg per increment   
    adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?
  
    // Detect taps in the directions turned ON
    //"adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
    adxl.setTapDetectionOnXYZ(0, 0, 0); 
   
    // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
    adxl.setTapThreshold(50);           // 62.5 mg per increment
    adxl.setTapDuration(15);            // 625 μs per increment
    adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
    adxl.setDoubleTapWindow(200);       // 1.25 ms per increment
   
    // Set values for what is considered FREE FALL (0-255)
    adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
    adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment
   
    // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
    adxl.InactivityINT(0);
    adxl.ActivityINT(0);
    adxl.FreeFallINT(0);
    adxl.doubleTapINT(0);
    adxl.singleTapINT(0);
  
  }

  void ADXL345ReadConfiguration() {
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[18])));
    writeToFile(lblBuffer, node);
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[0])));
    writeToFile(lblBuffer, ADXL345_DEVID, "HEX");
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[1])));
    writeToFile(lblBuffer, ADXL345_OFSX, "HEX");
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[2])));
    writeToFile(lblBuffer, ADXL345_OFSY, "HEX");
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[3])));
    writeToFile(lblBuffer, ADXL345_OFSZ, "HEX");
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[4])));
    writeToFile(lblBuffer, ADXL345_BW_RATE, "HEX");
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[5])));
    writeToFile(lblBuffer, ADXL345_POWER_CTL, "HEX");
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[6])));
    writeToFile(lblBuffer, ADXL345_INT_ENABLE, "HEX");
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[7])));
    writeToFile(lblBuffer, ADXL345_INT_MAP, "HEX");
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[8])));
    writeToFile(lblBuffer, ADXL345_INT_SOURCE, "HEX");
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[9])));
    writeToFile(lblBuffer, ADXL345_DATA_FORMAT, "HEX");
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[10])));
    writeToFile(lblBuffer, ADXL345_FIFO_CTL, "HEX");
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[11])));
    writeToFile(lblBuffer, ADXL345_FIFO_STATUS, "HEX");
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[12])));
    writeToFile(lblBuffer, (String)xOffset);
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[13])));
    writeToFile(lblBuffer, (String)yOffset);
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[14])));
    writeToFile(lblBuffer, (String)zOffset);
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[15])));
    writeToFile(lblBuffer, (String)xGain);
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[16])));
    writeToFile(lblBuffer, (String)yGain);
    strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[17])));
    writeToFile(lblBuffer, (String)zGain);
  }
#endif

void fileSetup(DateTime now) {
  long timeSince = (long) now.month() * 2592000 + (long) now.day() * 86400 + 
      (long) now.hour() * 3600 + (long) now.minute() * 60 + (long) now.second();
  fileName = String(timeSince, DEC);
  fileName.trim();
  fileName.concat(".TXT");
  fileName.toCharArray(fileNameChar, fileName.length()+1);
  myFile = SD.open(fileNameChar, FILE_WRITE);
  if (myFile) {
    myFile.print(now.year());
    myFile.print('/');
    myFile.print(now.month());
    myFile.print('/');
    myFile.print(now.day());
    myFile.print(' ');
    myFile.print(now.hour());
    myFile.print(':');
    myFile.print(now.minute());
    myFile.print(':');
    myFile.println(now.second());
    myFile.flush();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }  
}

void getEEPROMdata() {
  EEPROM.get(0, accData);
  node = accData.node;
  xOffset = accData.xOffset;
  yOffset = accData.yOffset;
  zOffset = accData.zOffset;
  xGain = accData.xGain;
  yGain = accData.yGain;
  zGain = accData.zGain;  
}

void Piezometer(int _piezoPin, unsigned long period, int dutyCycle) {
  digitalWrite(_piezoPin, LOW);
  delay(period);
  analogWrite(_piezoPin, dutyCycle);
  delay(period);
}

void readingsToSerial(unsigned long readingMillis, int x, int y, int z) {
  Serial.print(readingMillis);
  Serial.print(", ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.println();
}

void readingsToSerial(double x, double y, double z) {
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.println();
}

void SDModuleSetup(int SDpin) {
  //Initialize SD card
  Serial.print("Initializing SD card...");

  if (!SD.begin(SDpin)) {
    Serial.println("initialization failed!");

  } else {
    Serial.println("initialization done.");
  }
}

void writeToFile(unsigned long currentMillis, int x, int y, int z) {
  myFile.print(currentMillis);
  myFile.print(", ");
  myFile.print(x);
  myFile.print(", ");
  myFile.print(y);
  myFile.print(", ");
  myFile.print(z);
  myFile.print("\n");
}

void writeToFile(double x, double y, double z) {
  myFile.print(x);
  myFile.print(", ");
  myFile.print(y);
  myFile.print(", ");
  myFile.print(z);
  myFile.print("\n");
}

void writeToFile(String parameter, int address, String outputDataType) {
  byte value;
  #ifdef ACCEL_ADXL345
    adxl.readFrom(address, 1, &value);
  #endif
  myFile.print(parameter);
  if (outputDataType == "HEX") {
    myFile.print("0x");
    myFile.println(value, HEX);
  } else if(outputDataType == "DEC") {
    myFile.println(value, DEC);
  }
  myFile.flush();
}

void writeToFile(String parameter, String value) {
  myFile.print(parameter);
  myFile.println(value);
  myFile.flush();
}
