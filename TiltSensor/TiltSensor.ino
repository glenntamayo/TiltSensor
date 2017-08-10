//Created by Glenn Gil David Tamayo

#include <SparkFun_ADXL345.h>
#include <SPI.h>
#include <SdFat.h>
#include <RTClib.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

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
const int interruptPin = 2;
const int chipSelectSD = 4;
const int LED1pin = 16;
const int rockerPin = 15;
const int buzzerPin = 17;
const int softRX = 5;
const int softTX = 6;
/***************** ARDUINO PIN CONFIGURATION ****************/

/******************** CLASS CONSTRUCTORS ********************/
SdFat SD;
File myFile;
RTC_DS1307 RTC;
ADXL345 adxl = ADXL345();
SoftwareSerial HC05(softRX, softTX);
/******************** CLASS CONSTRUCTORS ********************/

/********************* GLOBAL VARIABLES *********************/
const unsigned long delayStart = 1000;
volatile bool fifoFull = 1;
String fileName;
char fileNameChar[10];

bool ROCKER = 0;

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
float xGain = 0;
float yGain = 0;
float zGain = 0;
int xAdj = 0;
int yAdj = 0;
int zAdj = 0;

unsigned long previousSdFlushMicros = 0;
const unsigned long SdFlushInterval = 5000000;
unsigned long previousMillisHC05Reading = 0;
/********************* GLOBAL VARIABLES *********************/

void setup(){
pinMode(rockerPin, INPUT_PULLUP);
if(digitalRead(rockerPin) == 0) {
  ROCKER = 0;
} else {
  ROCKER = 1;
}
DateTime now = RTC.now(); //outside switch due to cross-initialization

switch(ROCKER) {
  case 0:
    //Serial.begin(9600);
    pinMode(LED1pin, OUTPUT);
    digitalWrite(LED1pin, LOW);
    getEEPROMdata();    
    ADXL345Setup(ADXL345_BW_50, ADXL345_FIFOMODE_FIFO);
    SDModuleSetup(chipSelectSD);
    RTC.begin();
    fileSetup(now);
    attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);
    ADXL345ReadConfiguration();
    delay(delayStart);
    adxl.writeTo(ADXL345_POWER_CTL, ADXL_POWERCTL_MEASURE);
    break;
  case 1:
    //Serial.begin(9600);
    HC05.begin(9600);
    getEEPROMdata();    
    ADXL345Setup(ADXL345_BW_50, ADXL345_FIFOMODE_BYPASS);
    //ADXL345ReadConfiguration();
    delay(delayStart);
    adxl.writeTo(ADXL345_POWER_CTL, ADXL_POWERCTL_MEASURE);  
    break;  
  }
}

void loop(){
  switch(ROCKER) {
  case 0:
    while(1) {
      unsigned long currentMicros = micros();
      if (fifoFull) {
        int x,y,z;
        for (int i=0; i<31; i++) {
          adjAccel(x, y, z);
          writeToFile(currentMicros, xAdj, yAdj, zAdj);
        }
        fifoFull = 0; 
        //readingsToSerial(micros(), xAdj, yAdj, zAdj);
        //readingsToSerial(micros(), x, y, z);
      }
      
      if(currentMicros - previousSdFlushMicros > SdFlushInterval) {
        digitalWrite(LED1pin, HIGH);
        myFile.flush();
        digitalWrite(LED1pin, LOW);
        previousSdFlushMicros = currentMicros;
      }
    }
  case 1:
    while(1) {
      unsigned long currentMillis = millis();
      /*
      if (HC05.available() > 0) {
        Serial.write(HC05.read());
      }
      */
      if (currentMillis - previousMillisHC05Reading > 3000) {
        //HC05.println("Hello android kups");
        int x,y,z;
        adjAccel(x, y, z);
        HC05.print(currentMillis);
        HC05.print(", ");
        HC05.print(xAdj);
        HC05.print(", ");
        HC05.print(yAdj);
        HC05.print(", ");
        HC05.println(zAdj);
        previousMillisHC05Reading = currentMillis;
      }
    }
  }
} 

void abortWarning(int millisOn, int millisOff) {
  while(1) {
    Piezometer(buzzerPin, 500, 255); 
  } 
}

void adjAccel(int x, int y, int z) {
  adxl.readAccel(&x, &y, &z);
  xAdj = (float)(x - xOffset) / xGain;
  yAdj = (float)(y - yOffset) / yGain;
  zAdj = (float)(z - zOffset) / zGain;  
}

void ADXL_ISR() {
  fifoFull = 1;
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
    abortWarning(1000,1000);
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
  analogWrite(_piezoPin, dutyCycle);
  delay(period);
  digitalWrite(_piezoPin, LOW);
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

void SDModuleSetup(int SDpin) {
  //Initialize SD card
  Serial.print("Initializing SD card...");

  if (!SD.begin(SDpin)) {
    Serial.println("initialization failed!");
    abortWarning(1000,1000); 
  } else {
    Serial.println("initialization done.");
  }
}

void writeToFile(unsigned long currentMicros, int x, int y, int z) {
  myFile.print(currentMicros);
  myFile.print(", ");
  myFile.print(x);
  myFile.print(", ");
  myFile.print(y);
  myFile.print(", ");
  myFile.print(z);
  myFile.print("\n");
}

void writeToFile(String parameter, int address, String outputDataType) {
  byte value;
  adxl.readFrom(address, 1, &value);
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
