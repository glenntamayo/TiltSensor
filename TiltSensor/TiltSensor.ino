//Created by Glenn Gil David Tamayo

#include <SparkFun_ADXL345.h>
#include <SPI.h>
#include <SdFat.h>
#include <RTClib.h>

SdFat SD;
File myFile;
RTC_DS1307 RTC;
ADXL345 adxl = ADXL345();

const int chipSelectSD = 4;
unsigned long delayStart = 1000;
String readingEntry = "";
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

  adxl.setRate(50);
  adxl.set_bw(25);

  adxl.setFIFOMode("FIFO");

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
  Serial.begin(9600);

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
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
    abortLedWarning(500,250);
  }

  delay(delayStart);
}

void loop(){
  // Accelerometer Readings
  int x,y,z;
  if (adxl.getFIFOStatus() > 31) {
    for (int i=0; i<32; i++) {
      adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z
      readingEntry = micros();
      readingEntry.concat(", ");
      readingEntry.concat(x);
      readingEntry.concat(", ");
      readingEntry.concat(y);
      readingEntry.concat(", ");
      readingEntry.concat(z);
      readingEntry.concat("\n");
      
      /********** SD CARD **********/
      myFile = SD.open(fileNameChar, FILE_WRITE);
      if (myFile) {
        myFile.print(readingEntry);
        myFile.close();
      } else {
        // if the file didn't open, print an error:
        Serial.println("error opening file");
        abortLedWarning(500,250);
      }
    }
  }
}
