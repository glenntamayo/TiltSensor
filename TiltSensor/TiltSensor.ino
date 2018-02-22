//Created by Glenn Gil David Tamayo

//#define ACCELEROMETERADXL345
#define ACCELEROMETERMMA8451
//#define ACCELEROMETERMPU6050

#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <RTClib.h>
#include <Accelerometer_ggt.h>

/***************** ARDUINO PIN CONFIGURATION ****************/
const int chipSelectSD = 4;
/***************** ARDUINO PIN CONFIGURATION ****************/

/******************** CLASS CONSTRUCTORS ********************/
SdFat SD;
File myFile;
RTC_DS1307 RTC;
#ifdef ACCELEROMETERADXL345
  Accelerometer_ggt accelerometer = Accelerometer_ggt("adxl345");
#endif

#ifdef ACCELEROMETERMMA8451
  Accelerometer_ggt accelerometer = Accelerometer_ggt("mma8451");
#endif

#ifdef ACCELEROMETERMPU6050
  Accelerometer_ggt accelerometer = Accelerometer_ggt("mpu6050");
#endif

/******************** CLASS CONSTRUCTORS ********************/

/********************* GLOBAL VARIABLES *********************/
const unsigned long delayStart = 1000;
String fileName;
char fileNameChar[10];

const float pi = 3.141592653589793;

#ifdef ACCELEROMETERADXL345
  const byte adxlFifoMode = ADXL345_FIFOMODE_BYPASS;
#endif

unsigned long previousMillisWrite = 0;
unsigned long previousMillisRead = 0;

const unsigned long readInterval = 1000;
const unsigned long writeInterval = 600000;

/********************* GLOBAL VARIABLES *********************/

void setup(){
  //Serial.begin(115200);
  SDModuleSetup(chipSelectSD);
  RTC.begin();
  DateTime now = RTC.now();
  fileSetup(now);
  #ifdef ACCELEROMETERADXL345  
    AdxlSetup(ADXL345_BW_1_56);
  #endif

  #ifdef ACCELEROMETERMMA8451
    MmaSetup();
  #endif

  #ifdef ACCELEROMETERMPU6050
    Mpu6050Setup();
  #endif
}

void loop(){
  
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillisRead > readInterval) {
    accelerometer.filter(0.9);
    previousMillisRead = currentMillis;
    
  }
  if(currentMillis - previousMillisWrite > writeInterval) {
    String readingEntry;
    DateTime now = RTC.now();
    if (now.hour() < 10) {
      readingEntry = "0";
      readingEntry.concat(now.hour());
    } else {
      readingEntry = String(now.hour());
    }
    readingEntry.concat(':');
    if (now.minute() < 10) {
      readingEntry.concat("0");
      readingEntry.concat(now.minute());
    } else {
      readingEntry.concat(now.minute());
    }
    readingEntry.concat(':');
    if (now.second() < 10) {
      readingEntry.concat("0");
      readingEntry.concat(now.second());
    } else {
      readingEntry.concat(now.second());
    }
    readingEntry.concat('\t');
    readingEntry.concat(accelerometer.xAngleFil);
    readingEntry.concat('\t');
    readingEntry.concat(accelerometer.yAngleFil);
    readingEntry.concat('\t');
    readingEntry.concat(accelerometer.zAngleFil);         
    //Serial.println(readingEntry);
    myFile.println(readingEntry);
    myFile.flush();

    previousMillisWrite = currentMillis;
  }
  
} 

void fileSetup(DateTime now) {
  long timeSince = (long) now.month() * 2592000 + (long) now.day() * 86400 + 
      (long) now.hour() * 3600 + (long) now.minute() * 60 + (long) now.second();
  fileName = String(timeSince, DEC);
  fileName.trim();
  fileName.concat(".TXT");
  fileName.toCharArray(fileNameChar, fileName.length()+1);
  myFile = SD.open(fileNameChar, FILE_WRITE);
  if (myFile) {
    String strToSD = "";
    strToSD.concat(now.year());
    strToSD.concat('/');
    strToSD.concat(now.month());
    strToSD.concat('/');
    strToSD.concat(now.day());
    strToSD.concat(' ');
    strToSD.concat(now.hour());
    strToSD.concat(':');
    strToSD.concat(now.minute());
    strToSD.concat(':');
    strToSD.concat(now.second());
    myFile.println(strToSD);
    myFile.flush();
  } else {
    // if the file didn't open, print an error:
//    Serial.println("error opening file");
  }  
}

void SDModuleSetup(int SDpin) {
  //Initialize SD card
  //Serial.print("Initializing SD card...");

  if (!SD.begin(SDpin)) {
    //Serial.println("initialization failed!");
  
  } else {
    //Serial.println(F("initialization done."));
  }
}

#ifdef ACCELEROMETERADXL345

  void AdxlSetup(byte rate){
    accelerometer.adxl.writeTo(ADXL345_BW_RATE, rate);
    accelerometer.adxl.writeTo(ADXL345_FIFO_CTL, adxlFifoMode);
    accelerometer.adxl.setRangeSetting(2);           
    accelerometer.adxl.writeTo(ADXL345_POWER_CTL, 0x8); //MEASURE MODE  
  }
#endif

#ifdef ACCELEROMETERMMA8451
  void MmaSetup() {
    accelerometer.mma.begin();
    accelerometer.mma.setRange(MMA8451_RANGE_2_G);
    accelerometer.mma.setDataRate(MMA8451_DATARATE_6_25HZ);
  }
#endif

#ifdef ACCELEROMETERMPU6050
  void Mpu6050Setup() {
    accelerometer.mpu.initialize();
  }
#endif
