//Created by Glenn Gil David Tamayo

#include <SparkFun_ADXL345.h>
#include <SPI.h>
#include <SdFat.h>
#include <RTClib.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <DigitalIO.h>

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

const char* const string_table[] PROGMEM = {lblDEVID, lblOFSX,
lblOFSY, lblOFSZ, lblBWRATE, lblPOWERCTL, lblINTENABLE,
lblINTMAP, lblINTSOURCE, lblDATAFORMAT, lblFIFOCTL, lblFIFOSTATUS,
lblxOffset, lblyOffset, lblzOffset, lblxGain, lblyGain, lblzGain
};

char lblBuffer[40];

/***************** ARDUINO PIN CONFIGURATION ****************/
const int interruptPin = 2;
const int chipSelectSD = 4;
const int chipSelectRadio = 5;
const int csnRadio = 9;
/***************** ARDUINO PIN CONFIGURATION ****************/

/******************** CLASS CONSTRUCTORS ********************/
SdFat SD;
File myFile;
RTC_DS1307 RTC;
ADXL345 adxl = ADXL345();
RF24 radio(chipSelectRadio, csnRadio);
/******************** CLASS CONSTRUCTORS ********************/

/********************* GLOBAL VARIABLES *********************/
const unsigned long delayStart = 1000;
volatile bool fifoFull = 1;
String fileName;
char fileNameChar[10];
const int LED1pin = 9;

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
int xOffset = 0;
int yOffset = 0;
int zOffset = 0;
float xGain = 0;
float yGain = 0;
float zGain = 0;
int xAdj = 0;
int yAdj = 0;
int zAdj = 0;

char SendPayLoad[32] = "";
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL,0xF0F0F0F0D2LL };
/********************* GLOBAL VARIABLES *********************/

void setup(){
  Serial.begin(9600);

  pinMode(LED1pin, OUTPUT);
  digitalWrite(LED1pin, LOW);

  EEPROM.get(0, accData);
  xOffset = accData.xOffset;
  yOffset = accData.yOffset;
  zOffset = accData.zOffset;
  xGain = accData.xGain;
  yGain = accData.yGain;
  zGain = accData.zGain;
  
  ADXL345Setup();

  SDModuleSetup(chipSelectSD);
  
  RTC.begin();
  DateTime now = RTC.now();
  
  FileSetup(now);
  
  attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);

  ADXL345ReadConfiguration();

  //nRF24 configurations
  radio.begin();
  radio.setChannel(0x4c);
  radio.setAutoAck(1);
  radio.setRetries(15,15);
  radio.setDataRate(RF24_250KBPS);
  radio.setPayloadSize(32);
  radio.openReadingPipe(1,pipes[0]);
  radio.openWritingPipe(pipes[1]);

  delay(delayStart);
  
  adxl.writeTo(ADXL345_POWER_CTL, ADXL_POWERCTL_MEASURE);
}

void loop(){
  int x,y,z;
  if (fifoFull) {
    unsigned long currentMicros = micros(); 
    int numEntries = adxl.getFIFOStatus();
    for (int i=0; i<numEntries; i++) {
      digitalWrite(LED1pin, HIGH);
      adxl.readAccel(&x, &y, &z);
      xAdj = (float)(x - xOffset) / xGain;
      yAdj = (float)(y - yOffset) / yGain;
      zAdj = (float)(z - zOffset) / zGain;
      writeToFile(currentMicros, xAdj, yAdj, zAdj);
    }
    digitalWrite(LED1pin, LOW);
    fifoFull = 0; 
    //readingsToSerial(micros(), xAdj, yAdj, zAdj);
    //readingsToSerial(micros(), x, y, z);

    
    strcpy(SendPayLoad, "Ang baho mo");
    bool ok = radio.write(&SendPayLoad,strlen(SendPayLoad));
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
  myFile.flush();  
}

void abortLedWarning(int millisOn, int millisOff) {
  /*while(1) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(millisOn);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(millisOff);   
  } 
  */
}

void ADXL345Setup() {
  
  adxl.writeTo(ADXL345_BW_RATE, ADXL345_BW_12_5);

  adxl.writeTo(ADXL345_FIFO_CTL, ADXL345_FIFOMODE_FIFO);

  //adxl.setAxisOffset(accData.xOffset, accData.yOffset, accData.zOffset);
  
  adxl.setInterruptMapping(1, 0);
  adxl.setInterrupt(1 ,1);

  adxl.setRangeSetting(2);            // Give the range settings
                                      // Accepted values are 2g, 4g, 8g or 16g
                                      // Higher Values = Wider Measurement Range
                                      // Lower Values = Greater Sensitivity
   
  adxl.setActivityXYZ(0, 0, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
 
  adxl.setInactivityXYZ(0, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?

  adxl.setTapDetectionOnXYZ(0, 0, 0); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
 
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

void SDModuleSetup(int SDpin) {
  //Initialize SD card
  Serial.print("Initializing SD card...");

  if (!SD.begin(SDpin)) {
    Serial.println("initialization failed!");
    abortLedWarning(0,500); 
  } else {
    Serial.println("initialization done.");
  }
}

void FileSetup(DateTime now) {
  long timeSince = (long) now.hour() * 3600 + (long) now.minute() * 60 + (long) now.second();
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
    abortLedWarning(0,250);
  }  
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

void ADXL_ISR() {
  fifoFull = 1;
}

void writeToOutputFile(String parameter, int address) {
  byte value;
  adxl.readFrom(address, 1, &value);
  //write to output file
  myFile.print(parameter);
  myFile.println(value, HEX);
  myFile.flush();
}

void ADXL345ReadConfiguration() {
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[0])));
  writeToOutputFile(lblBuffer, ADXL345_DEVID);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[1])));
  writeToOutputFile(lblBuffer, ADXL345_OFSX);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[2])));
  writeToOutputFile(lblBuffer, ADXL345_OFSY);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[3])));
  writeToOutputFile(lblBuffer, ADXL345_OFSZ);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[4])));
  writeToOutputFile(lblBuffer, ADXL345_BW_RATE);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[5])));
  writeToOutputFile(lblBuffer, ADXL345_POWER_CTL);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[6])));
  writeToOutputFile(lblBuffer, ADXL345_INT_ENABLE);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[7])));
  writeToOutputFile(lblBuffer, ADXL345_INT_MAP);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[8])));
  writeToOutputFile(lblBuffer, ADXL345_INT_SOURCE);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[9])));
  writeToOutputFile(lblBuffer, ADXL345_DATA_FORMAT);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[10])));
  writeToOutputFile(lblBuffer, ADXL345_FIFO_CTL);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[11])));
  writeToOutputFile(lblBuffer, ADXL345_FIFO_STATUS);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[12])));
  writeToOutputFile(lblBuffer, xOffset);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[13])));
  writeToOutputFile(lblBuffer, yOffset);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[14])));
  writeToOutputFile(lblBuffer, zOffset);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[15])));
  writeToOutputFile(lblBuffer, xGain);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[16])));
  writeToOutputFile(lblBuffer, yGain);
  strcpy_P(lblBuffer, (char*)pgm_read_word(&(string_table[17])));
  writeToOutputFile(lblBuffer, zGain);
}
