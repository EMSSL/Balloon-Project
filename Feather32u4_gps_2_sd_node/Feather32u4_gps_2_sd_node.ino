/*
 * Feather M4 - GPS node
 */

#include <pt.h>               // protothread library
#include <SD.h>               // SD library
#include <Adafruit_Sensor.h>  // master sensor library
#include <Adafruit_GPS.h>     // GPS library
#include <avr/wdt.h>          // watchdog hardware reset library

#define usb Serial            // renames Serial as usb
#define gps Serial1           // renames Serial2 as gps
#define SD_CS 4
#define gpsWriteThresh 5                // number of successful gps pulls before writing to SD
long gpsTimeStamp = 0;        // timestamp used to track how long its been since last GPS sentence (for error tracking)
String gpsStuff = "";           // string object for passing GPS data to various functions
String GPSflushTotal = "";      // string object for GPS data (temporary storage)
File GPSdata;                   // SD file for GPS data
uint8_t GPSwriteCount = 0;        // tracks the number of successful GPS pulls before writing to SD

// GPS stuff
Adafruit_GPS GPS(&gps); // GPS
#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean);

bool gpsError = false;        // is there a problem with the GPS?

static struct pt getGpsPT;
static struct pt GPSwritePT;

//*************************************************************************************************************
//*******                                   PROTOTHREAD getGPS_sense
//*************************************************************************************************************
static int getGPS_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, GPS.newNMEAreceived());
    getGPS();
    gpsTimeStamp = millis();
  }
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                           getGPS
//*************************************************************************************************************
void getGPS(){
  //usb.println(F("entering getGPS"));
  gpsStuff = GPS.lastNMEA();
  usb.println(gpsStuff);
  GPSdata.print(gpsStuff);
  GPSdata.flush();
  gpsStuff = "";
}

//*************************************************************************************************************
//*******                                           MakeFiles
//*************************************************************************************************************
int makeFiles(){
  //usb.println(F("entering makeFiles()"));// 
  bool rtn = true;                          // set a boolean flag of rtn to be true

// commented out, filenames set to global vars  ***********************************
  char gpsfile[15];                         // create a pointer for a character string of 15 for gps string
  
  strcpy(gpsfile, "GPSLOG05.TXT");          // set the string name to the pointer
  
  //usb.println(F("made it through filename initialization stuffs"));                                          
                                            //
  for(uint8_t i = 0; i < 100; i++)          // for the gps file 
  {                                         // for all numbers from 0 to 100
    gpsfile[6] = '0' + i/10;                // replace the 6th character with a 0 plus the 1st digit of the division of i and 10
    gpsfile[7] = '0' + i%10;                // replace the 7th character with a 0 plus the 2nd digit of the remainder of the division of i and 10
    if(!SD.exists(gpsfile))                 // if that new file name DOES NOT EXIST within the sd card
    {                                       //
      break;                                // then get out of the loop because you have found a new file name which doesn't exist
    }                                       //
  }                                         // otherwise, loop back again because you cannot overwrite files
  
  //usb.println(F("made it through gpsfile suffix search"));                                          
                                            //
  GPSdata = SD.open(gpsfile, FILE_WRITE);   // write a new file for the gps file on the SD card
  if(!GPSdata)                              // if the file failed to be created
  {                                         // 
    usb.print(F("Failed to create "));         // display failed to create on the serial monitor
    usb.println(gpsfile);                   // followed by the filename
    rtn = false;                            // set the boolean flag rtn to false
  }                                         // end
  else{
    //usb.println(F("made it through gps file creation")); 
  }
  
  //usb.println(F("returning rtn"));
  //usb.println(rtn); 
  //usb.println(F("leaving makeFiles()"));                                        
  return (int)rtn; 
}

//*************************************************************************************************************
//*******                                           CRITICAL GPS-RELATED FUNCTIONS
//*************************************************************************************************************
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  //if (GPSECHO)
    //if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than usb.print 
    // but only one character can be written at a time. 
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


//*************************************************************************************************************
//*******                                           SETUP
//*************************************************************************************************************
void setup() {
  wdt_disable(); // this prevents infinite loops from occuring with the watchdog reset                            // watchdog
  
    //calculates maximum string memory usage, and if it will fit on the arduino, reserves the memory 
   //this prevents "foaming" of the heap memory due to fragmentation, and helps to prevent stack overflow 
  unsigned int gpsTotalBytes = 350*gpsWriteThresh;                      //
  GPSflushTotal.reserve(gpsTotalBytes);       // Reserves a block of SRAM for the String variables (to prevent memory fragmentation)
  
  usb.begin(115200);
  usb.println(F("INITIALIZING"));
  
  if(!SD.begin(SD_CS)){         // initializes the SD card
    usb.println(F("SD Card failed to initialize!")); // if the SD card fails, set sdError true and tell someone
  }                                 
  else if(!makeFiles()){        // initializes the SD files
    usb.println(F("File Initialization Failed!")); // if makeFiles fails, set makeFileError to true and tell someone
  }
  else{
    usb.println(F("makefiles worked!"));
  }                       
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Getting RMA and GGA data (see http://www.gpsinformation.org/dale/nmea.htm)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);    //
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);    //GPS will refresh 5 times a second (doesn't mean you will get data that often)
  GPS.sendCommand(PMTK_ENABLE_WAAS);            // enables dgps reception (higher accuracy in North America)
  GPS.sendCommand(PGCMD_NOANTENNA);             // kills the antenna status update           
  useInterrupt(true);                           // uses interrupts to gather incoming data                          

  PT_INIT(&getGpsPT);           //
  PT_INIT(&GPSwritePT);         // Initilize all protothreads

  usb.println(F("Setup complete, entering loop!"));
  //wdt_enable(16384);
}

void loop() {
  wdt_enable(WDTO_8S);                  // enable watchdog timer
  getGPS_sense(&getGpsPT);
//  GPSwrite_sense(&GPSwritePT);
  wdt_reset();
}
