/****************************************************************************************************************
 * **************************************************************************************************************
 *                        READ THIS OR RISK AN ETERNITY OF HUMILIATION AND KIDNEY STONES
 * **************************************************************************************************************
 * IF YOU ATTEMPT TO MODIFY THIS CODE TO FIX ERRORS, ADD NEW SENSORS, OR ADD NEW MCUs, WHATEVER...YOU SHOULD READ
 * UP ON THE PROPER USE OF GIT AND GITHUB, SIGN UP AND INSTALL GIT, USE IT TO CONTROL REVISIONS TO THE CODE 
 * PROPERLY, AND MAKE SURE YOU DONT FUCK THIS CODE UP FOR EVERYONE. IF I CATCH YOU DOING THIS, YOUR P.I. WILL BE 
 * INSTRUCTED TO NEVER AWARD YOU A DEGREE, AND EVERYONE IN THE LAB WILL LOVE YOU LESS...BECAUSE YOU ARE THE 
 * TRASHIEST OF TRASH-PEOPLE.  -- Gerrit (^_^)
 * **************************************************************************************************************
 */
 /*!
  * \file FeatherM0_gps_2_sd_node.ino
  * 
  * \brief A barebones NMEA sentence datalogger build for the Adafruit Feather Adalogger Cortex-M0
  * \details
 *    Feather32u4_gps_2_sd_node by Gerrit Motes, EMSSL NCSU, 7/12/19
 * 
 *    Used to measure dGPS/GPS location data and write to a microSD card. 
 * 
 *    Uses the following hardware:
 *    - Adafruit Feather Cortex-M0 Adalogger
 *    - Adafruit Ultimate GPS FeatherWing
 */
//****************************************************************************************************************/

#include <pt.h>               // protothread library
#include <SD.h>               // SD library
#include <Adafruit_Sensor.h>  // master sensor library
#include <Adafruit_GPS.h>     // GPS library



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
//*************************************************************************************************************/
/*!
 * \brief The protothread that controls GPS sentence aquisition
 * 
 * \param[in,out] pt The pointer to the associated global protothread struct
 */
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
//*************************************************************************************************************/
/*!
 * \brief The function that grabs new GPS sentences
 * 
 * \details uses the .lastNMEA() method to grab an NMEA sentence in String form. It then prints the sentence to
 * the serial monitor, writes it to SD memory, and syncronizes the SD card. It then clears the memory.
 *
 */
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
//*************************************************************************************************************/
/*!
 * \brief Finds unique filenames on the SD card and opens/assigns them to global File declarations
 * 
 * \details 
 * Uses predefined const char filenames (e.g. GPSLOG05.TXT) and cycles through the last 2 digits from 00 to 99, 
 * checking for the existence of each file until a filename is found that doesn't alread exist. It then attempts
 * to open that file. If successful, it assigns it to its respective global File object (e.g. GPSdata). 
 *
 */
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
//*************************************************************************************************************/
/* ARM SPECIFIC STUFF, DO NOT INCLUDE UNLES YOU ARE USING AN MCU WITH A CORTEX-M PROCESSOR*/
void useInterrupt(boolean v) {    // call this with v as true in your setup() code
  if(v)
  {
    usingInterrupt = true;       // MAKE SURE TO DECLARE A "bool usingInterrupts;" at the top of the sketch,
  }                               // outside of any functions
  else
  {
    usingInterrupt = false;
  }
}

void serialEventRun()             // 
{
  while(gps.available()){
    char c = GPS.read();
    #ifdef GPSECHO
      Serial.print(c);
    #endif
  }
}
    

//*************************************************************************************************************
//*******                              Watchdog Timer Stuff (SAMD51)
//*************************************************************************************************************/
//
// * Used to configure and enable the watchdog timer for SAMD processors, meant to have a similar format to the
// * original avr/wdt.h library for AVR processors
// * 
// * \return boolean value that represents the success/failure of enabling of the timer
// * 
// * \param[in] periodCyc The number of millisecond periods counted until a watchdog reset occurs. Increments of 8*2^n up to 16384 should be used.
// */
//// periodCyc goes in increments of 8*2^n up to 16384 (integer n starts at zero)
//// fails to initialize if periodCyc doesn't match specific values
//bool wdt_enable(int periodCyc)
//{  
//  switch(periodCyc)   // operates in fall-through mode, returns false if periodCyc doesnt match
//  {
//    case 8:
//      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC8;
//    case 16:
//      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC16;
//    case 32:
//      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC32;
//    case 64:
//      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC64;
//    case 128:
//      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC128;
//    case 256:
//      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC256;
//    case 512:
//      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC512;
//    case 1024:
//      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC1024;
//    case 2048:
//      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC2048;
//    case 4096:
//      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC4096;
//    case 8192:
//      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC8192;
//    case 16384:
//      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC16384;
//    default:
//      return false;
//  }
//
//  REG_WDT_CTRLA = WDT_CTRLA_ENABLE;           // enable WDT
//  while(WDT->SYNCBUSY.bit.ENABLE)             // wait for bit to sync
//  {}  
//
//  return true;
//}
//
///*!
// * Used to disable the watchdog reset timer for SAMD processors, meant to have a similar format to the
// * original avr/wdt.h library for AVR processors
// */
//void wdt_disable()  // disables the WDT (does not work if WDT->CTRLA.bit.ALWAYSON = 1)
//{
//  WDT->CTRLA.bit.ENABLE = 0;          // disable the watchdog
//  while(WDT->SYNCBUSY.bit.ENABLE)     // wait for the ENABLE bit to syncronize
//  {}
//}
//
///*!
// * Used to reset the watchdog reset timer for SAMD processors, meant to have a similar format to the
// * original avr/wdt.h library for AVR processors
// */
//void wdt_reset()  // high performance WDT clear funtion
//{
//  if(!WDT->SYNCBUSY.bit.CLEAR)            // if not syncronizing from last CLEAR,
//  {                                       //
//    REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;  // write clear key to CLEAR register
//  }
//}

    

//*************************************************************************************************************
//*******                                           SETUP
//*************************************************************************************************************/
void setup() {
  //wdt_disable(); // this prevents infinite loops from occuring with the watchdog reset                            // watchdog
  
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

//*************************************************************************************************************
//*******                                           LOOP
//*************************************************************************************************************/
void loop() {
  //wdt_enable(WDTO_8S);                  // enable watchdog timer
  getGPS_sense(&getGpsPT);
//  GPSwrite_sense(&GPSwritePT);
 // wdt_reset();
}
