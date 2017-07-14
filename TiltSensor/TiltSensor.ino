//Created by Glenn Gil David Tamayo

#include <SparkFun_ADXL345.h>
#include <SPI.h>
#include <SdFat.h>
#include <RTClib.h>

SdFat SD;
File myFile;
RTC_DS1307 RTC;
ADXL345 adxl = ADXL345();

int interruptPin = 2;                 // Setup pin 2 to be the interrupt pin (for most Arduino Boards)

const int chipSelectSD = 4;
const unsigned long delayStart = 1000;
unsigned long previousMicros = 0;
unsigned long currentMicrosTemp = 0;
//const int FIFOentries = 8;
bool fifoFull = 1;

String fileName;
char fileNameChar[10];


void abortLedWarning(int millisOn, int millisOff) {
  while(1) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(millisOn);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(millisOff);   
  } 
}

void ADXL345Setup() {
  adxl.powerOn();                     // Power on the ADXL345

  adxl.writeTo(ADXL345_BW_RATE, ADXL345_BW_12_5);

  adxl.setFIFOMode("FIFO");
  adxl.setInterruptMapping(1, 0);
  adxl.setInterrupt(1 ,1);

  adxl.setRangeSetting(2);           // Give the range settings
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

void SDModuleSetup() {
  //Initialize SD card
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelectSD)) {
    Serial.println("initialization failed!");
    abortLedWarning(500,500); 
  } else {
    Serial.println("initialization done.");
  }
}

void setup(){
  Serial.begin(250000);

  ADXL345Setup();

  SDModuleSetup();
  
  RTC.begin();
  DateTime now = RTC.now();
  
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
    myFile.print(now.second());
    myFile.print(':');
    myFile.flush();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
    abortLedWarning(500,250);
  }
  attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);   // Attach Interrupt
  
  delay(delayStart);
}

void loop(){
  int x,y,z;
  if (fifoFull) {
    unsigned long currentMicros = micros(); 
    int numEntries = adxl.getFIFOStatus();
    /***** DEBUGGING checking data read rates
    Serial.print(numEntries);
    Serial.print(", ");
    *****/
    for (int i=0; i<numEntries; i++) {
      adxl.readAccel(&x, &y, &z);
      if (myFile) {
        myFile.print(currentMicros);
        myFile.print(", ");
        myFile.print(x);
        myFile.print(", ");
        myFile.print(y);
        myFile.print(", ");
        myFile.print(z);
        myFile.print("\n");
        myFile.flush();
      } else {
        // if the file didn't open, print an error:
        Serial.println("error opening file");
        abortLedWarning(500,250);
      }
    }
    fifoFull = 0;
    /***** DEBUGGING checking data read rates
    currentMicrosTemp = micros();
    Serial.print(1000000 / ((float)(currentMicrosTemp - previousMicros)/numEntries));
    Serial.print(", ");
    Serial.println(adxl.getFIFOStatus());
    previousMicros = currentMicrosTemp;
    *****/ 
  }
}
void ADXL_ISR() {
  fifoFull = 1;
}
