#include <Wire.h>             // wire library (needed for some sensors)
#include <Adafruit_Sensor.h>  // master sensor library
#include <Adafruit_ADS1015.h> // ADC library
#include <SD.h>               // SD library
#include <pt.h>               // protothread library

#define usb Serial

// Pitot analog pin definition
//#define PITOT A15
Adafruit_ADS1115 ads;         // pitot ADC object
int pitot = 0;                 // global var for averaged pitot value

//SPI communication pins, SCK, MISO, and MOSI are common to all SPI sensors, CS pins are unique to each 
// SPI sensor.
#define SD_CS 2
#define sdPower 3      // SD power transistor gate pin

File OtherData;                 // SD file for sensor data

#define pitotReadThresh 2               // time (ms) between each pitot reading
long pitotStamp = 0;          // timestamp used to track last time the pitot was sampled

static struct pt getPitotPT;

String OtherDataTotal = "";

long baudrate_array[15] = {300,1200,2400,4800,9600,19200,38400,57600,74880,115200,230400,250000,500000,1000000,2000000};

//*************************************************************************************************************
//*******                                   PROTOTHREAD getPitot_sense
//*************************************************************************************************************
static int getPitot_sense(struct pt *pt){
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, (millis()  - pitotStamp) >= pitotReadThresh);
    usb.println(F("derpderp"));
    pitot = ads.readADC_SingleEnded(0);
    usb.println(F("derpderpderp"));
    OtherDataTotal = OtherDataTotal + pitot + F("\n");
    
    OtherData.print(OtherDataTotal);
    
    OtherData.flush();
    
    OtherDataTotal = "";
    usb.println(pitot);
    pitotStamp = millis();
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
  
  //usb.println(F("made it through filename initialization stuffs"));                                          
                                            //                                       
                                            //
  for(uint8_t i = 0; i < 100; i++)          // for the file 
  {                                         // for all numbers from 0 to 100
    otherdatafile[6] = '0' + i/10;          // replace the 6th character with a 0 plus the 1st digit of the division of i and 10
    otherdatafile[7] = '0' + i%10;          // replace the 7th character with a 0 plus the 2nd digit of the remainder of the division of i and 10
    if(!SD.exists(otherdatafile))           // if that new file name DOES NOT EXIST within the sd card
    {                                       //
      break;                                // then get out of the loop because you have found a new file name which doesn't exist
    }                                       //
  }                                         // otherwise, loop back again because you cannot overwrite files
  
  //usb.println(F("made it through otherdatafile suffix search"));
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
    //usb.println(F("made it through otherdata file creation")); 
  }
  //usb.println(F("returning rtn"));
  //usb.println(rtn); 
  //usb.println(F("leaving makeFiles()"));                                        
  return (int)rtn; 
}


//*************************************************************************************************************
//*******                                           SETUP
//*************************************************************************************************************
void setup() {
  pinMode(sdPower, OUTPUT);     //    enable power to SD 
  digitalWrite(sdPower, HIGH);  //
  
  delay(5000);                  // delay long enough to open the serial monitor

  usb.begin(115200);
  usb.println(F("INITIALIZING"));

  OtherDataTotal.reserve(50);   // reserve to prevent heap foaming
  
  bool isError = false;

  if(!SD.begin(SD_CS)){         // initializes the SD card
    usb.println(F("SD Card failed to initialize!")); // if the SD card fails, set isError true and tell someone
    isError = true; 
  }                                 
  else if(!makeFiles()){        // initializes the SD files
    usb.println(F("File Initialization Failed!")); // if makeFiles fails, set isError to true and tell someone
    isError = true;
  }
  else{
    usb.println(F("makefiles worked!"));
  }
  
  ads.begin();                   // initializes the ADC for the pitot tube

  while(isError)                 // crash if errors occur
  {
    for(int i = 0; i < 15; i++)
    {
      usb.begin(baudrate_array[i]);
      usb.println(F("OOPS SOMETHING HAPPENED AND I POOPLED MY PANTALOONS"));
    }
  }

  PT_INIT(&getPitotPT);         // initialize the protothread stuff

  usb.println(F("Initialization complete!"));
}

//*************************************************************************************************************
//*******                                           LOOP
//*************************************************************************************************************
void loop() {
  usb.println(F("derp"));
  getPitot_sense(&getPitotPT);
}
