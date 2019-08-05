/*
 * BALLOON PROJECT - Pitot Calibration
 * 
 * Author :          Gerrit Motes 7/19
 * Special Contrib:  MakeFiles by Rodney Metoyer in The BeforeTime, in The LongLongAgo...
 * 
 * Equipment integrated into this build:
 *  - Adafruit microSD reader/writeer breakout
 *  - Sensirion SDP806-125Pa differential Pressure transducer
 *  - Hobbyking small pitotstatic tube with an Adafruit ADS115 ADC
 *  
 *  Previous Rev: N/A
 */


#include <Wire.h>             // wire library (needed for some sensors)
#include <SPI.h>              // SPI digital comms library (needed for SD card)
#include <pt.h>               // protothread library
//#include <avr/wdt.h>          // watchdog hardware reset library (not compatible with SAMD51)                   // watchdog
#include <SD.h>               // SD library
#include <Adafruit_Sensor.h>  // master sensor library
#include <Adafruit_ADS1015.h> // ADC library


#define usb Serial            // renames Serial as usb

//SPI communication pins, SCK, MISO, and MOSI are common to all SPI sensors, CS pins are unique to each 
// SPI sensor.
#define SD_CS 2
#define sdPower 3      // SD power transistor gate pin

// Pitot analog pin definition
//#define PITOT A15
Adafruit_ADS1115 ads;         // pitot ADC object
uint16_t pitotReadTotal = 0;        // global var for pitot reading

String OtherflushTotal = "";    // string object for sensor data (temporary storage)
File OtherData;                 // SD file for sensor data

bool isError = false;         // is there an error?
bool sdError = false;         // is there an SD-specific error?
bool makeFileError = false;   // did makeFiles() fail?
bool fatalError = false;      // is there a fatal error?

static struct pt getPitotPT;

#define pitotReadThresh 20               // time (ms) between each pitot reading
                            
long pitotStamp = 0;          // timestamp used to track last time the pitot was sampled

#define imuPower 12     // used on the imu power transistor gate for a hard reset
#define altPower 11           //altimeter power transistor pin


//*************************************************************************************************************
//*******                                   PROTOTHREAD getPitot_sense
//*************************************************************************************************************
static int getPitot_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, (pitotStamp - millis()) >= pitotReadThresh);
    pitotReadTotal = ads.readADC_SingleEnded(0);
    usb.println(pitotReadTotal);
    OtherflushTotal = OtherflushTotal + pitotReadTotal + F("\n");
    OtherData.print(OtherflushTotal);
    OtherData.flush();
    OtherflushTotal = "";
    pitotStamp = millis();
  }
  PT_END(pt);
}


//*************************************************************************************************************
//*******                                           MakeFiles
//*************************************************************************************************************
int makeFiles(){
  //usb.println(F("entering makeFiles()"));// 
  bool rtn = true;                          // set a boolean flag of rtn to be true

// commented out, filenames set to global vars  ***********************************
  char otherdatafile[15];                   // create a pointer for a character string of 15 for other data strings
  
  strcpy(otherdatafile, "AODATA05.TXT");    // set the string name to the pointer
  
  usb.println(F("made it through filename initialization stuffs"));                                                                                   
                                            //
  for(uint8_t i = 0; i < 100; i++)          // for the gps file 
  {                                         // for all numbers from 0 to 100
    otherdatafile[6] = '0' + i/10;          // replace the 6th character with a 0 plus the 1st digit of the division of i and 10
    otherdatafile[7] = '0' + i%10;          // replace the 7th character with a 0 plus the 2nd digit of the remainder of the division of i and 10
    if(!SD.exists(otherdatafile))           // if that new file name DOES NOT EXIST within the sd card
    {                                       //
      break;                                // then get out of the loop because you have found a new file name which doesn't exist
    }                                       //
  }                                         // otherwise, loop back again because you cannot overwrite files
  
  usb.println(F("made it through otherdatafile suffix search"));
                                            //
                                            //        
  OtherData = SD.open(otherdatafile, FILE_WRITE); // write a new file for the gps file on the SD card
  if(!OtherData)                                // if the file failed to be created
  {                                             // 
    usb.print(F("Failed to create "));             // display failed to create on the serial monitor
    usb.println(otherdatafile);                 // followed by the filename
    rtn = false;                                // set the boolean flag rtn to false
  }                                             //
  else{
    usb.println(F("made it through otherdata file creation")); 
  }
  usb.println(F("returning rtn"));
  usb.println(rtn); 
  usb.println(F("leaving makeFiles()"));                                        
  return (int)rtn; 
}


//*************************************************************************************************************
//*******                                           SETUP
//*************************************************************************************************************
void setup() {
  delay(5000);    // used to read setup debug code
  OtherflushTotal.reserve(200);   //
    
  usb.begin(115200);
  usb.println(F("INITIALIZING"));

  
  pinMode(sdPower, OUTPUT);     //
  pinMode(imuPower, OUTPUT);    // Sets all power transistor gate pins and reset pins to OUTPUT mode
  pinMode(altPower, OUTPUT);    //
  
  digitalWrite(sdPower, HIGH);  //
  digitalWrite(imuPower, HIGH); // closes path to ground on all sensors, and sets the imu rst pin to HIGH
  digitalWrite(altPower, HIGH); //
  
  delay(250);                   // allows time for the pin operations and sensors to come up 

  while(!SD.begin(SD_CS))
  {
    usb.println(F("startSD"));
    delay(10);
  }
//  if(!SD.begin(SD_CS)){         // initializes the SD card
//    usb.println(F("SD Card failed to initialize!")); // if the SD card fails, set sdError true and tell someone
//    sdError = true;
//    fatalError = true;
//  }                                 
//  else 
  if(!makeFiles()){        // initializes the SD files
    usb.println(F("File Initialization Failed!")); // if makeFiles fails, set makeFileError to true and tell someone
    makeFileError = true;
    fatalError = true;
  }
  else{
    usb.println(F("makefiles worked!"));
    ads.begin();                          // initializes the ADC for the pitot tube
  }                       
   
  PT_INIT(&getPitotPT);         // initialize protothreads
  
  usb.println(F("Setup complete, entering loop!"));
}

//*************************************************************************************************************
//*******                                           LOOP
//*************************************************************************************************************
void loop() {
  getPitot_sense(&getPitotPT);
}
