/****************************************************************************************************************
 * **************************************************************************************************************
 *                        READ THIS OR RISK AN ETERNITY OF HUMILIATION AND KIDNEY STONES
 * **************************************************************************************************************
 * IF YOU ATTEMPT TO MODIFY THIS CODE TO FIX ERRORS, ADD NEW SENSORS, OR ADD NEW MCUs...YOU SHOULD READ UP ON THE
 * PROPER USE OF GIT AND GITHUB, SIGN UP AND INSTALL GIT, USE IT TO CONTROL REVISIONS TO THE CODE PROPERLY, AND 
 * MAKE SURE YOU DONT FUCK THIS CODE UP FOR EVERYONE. IF I CATCH YOU DOING THIS, YOUR P.I. WILL BE INSTRUCTED TO 
 * NEVER AWARD YOU A DEGREE, AND EVERYONE IN THE LAB WILL LOVE YOU LESS...BECAUSE YOU ARE THE TRASHIEST OF 
 * TRASH-PEOPLE.  -- Gerrit (^_^)
 * **************************************************************************************************************
 * 
 * Sensor Tester by Gerrit Motes, EMSSL NCSU, 7/13/19
 * 
 * Used to validate basic operation (initilization and data collection) of any combination of the follow sensors:
 * 
 * - Adafruit Ultimate GPS Breakout
 * - Adafruit MicroSD reader/writer breakout
 * - Adafruit BME280 Altimeter Breakout
 * - Adafruit ADS1115 ADC Breakout
 * - Adafruit BNO055 9DOF IMU
 * 
 * The sketch is capable of running on any of the following MCUs:
 * 
 * - Adafruit Itsy Bitsy M4 Express
 * - Adafruit 32u4 Feather
 * - Arduino MEGA 2560 (offbrands too)
 * - Arduino UNO (offbrands too)
 * 
 * To configure:
 * 
 *  1) uncomment the MCU you are using (comment out the rest)
 *  2) uncomment which sensors you want to test (comment out the sensors you won't be using)
 *  
 * If you want to change the SD's chip select pin, find "#define SD_CS" below and change the default (5) to the
 *  pin of your choice. 
 *  
 * 
 ****************************************************************************************************************/

#include <Wire.h>             // wire library (needed for some sensors)
#include <SPI.h>              // SPI digital comms library (needed for SD card)
#include <pt.h>               // protothread library
#include <SD.h>               // SD library
#include <Adafruit_Sensor.h>  // master sensor library
#include <Adafruit_GPS.h>     // GPS library
#include <Adafruit_BNO055.h>  // IMU library
#include <Adafruit_BME280.h>  // Altimeter library
#include <Adafruit_ADS1015.h> // ADC library

//*************************************************************************************************************
//*******                                  CRITICAL COMPATIBILITY STUFF
//*************************************************************************************************************

// UNCOMMENT IF USING, COMMENT OUT THE REST IN THIS BOX

//#define USING_UNO               //
//#define USING_MEGA              //  WHICH BOARD IS BEING USED?
#define USING_M4_ITSY_BITSY       //
//#define USING_32U4_FEATHER      //
     
//*************************************************************************************************************

// UNCOMMENT IF USING, COMMENT OUT IF NOT USING IN THIS BOX

#define TESTING_IMU               //    
#define TESTING_ALT               //
#define TESTING_ADC               //  WHICH SENSORS ARE YOU TESTING
#define TESTING_GPS               //
#define TESTING_SD                //

//*************************************************************************************************************
//                                                  END
//*************************************************************************************************************

#if defined(USING_UNO) || defined(USING_MEGA) || defined(USING_32U4_FEATHER)
  #define USING_AVR
#elif defined(USING_M4_ITSY_BITSY)
  #define USING_ARM
#endif

#define usb Serial            // renames Serial as usb
#define gps Serial1           // renames Serial2 as gps

#define sensor_interval 50    // sample the sensors at 20Hz
#define alt_interval 1000     // sample the altimeter at 1Hz

//SPI communication pins, SCK, MISO, and MOSI are common to all SPI sensors, CS pins are unique to each 
// SPI sensor.
#define SD_CS 5

Adafruit_BNO055 IMU = Adafruit_BNO055(55,BNO055_ADDRESS_B); 
uint8_t system_status, self_test_results, system_error; // imu sensor error vars
long imuErrChk1 = 1;          // used for tracking non-obvious errors in IMU data
long imuErrChk2 = 1;          // used for tracking non-obvious errors in IMU data

// Pitot analog pin definition
//#define PITOT A15
Adafruit_ADS1115 ads;         // pitot ADC object
float pitot = 0;              // global var for pitot reading

// altimeter variables
float temp = 0;               //
float pressure = 0;           // global vars for altimeter data
float humid = 0;              //
float altAltitude = 0;        // holds the altitude data supplied by the altimeter
Adafruit_BME280 ALT;          //  ALT I2C

// GPS stuff
Adafruit_GPS GPS(&gps); // GPS
boolean usingInterrupt;
void useInterrupt(boolean);

String OtherflushTotal = "";    // string object for sensor data (temporary storage)
String gpsStuff = "";      // string object for GPS data (temporary storage)
File OtherData;                 // SD file for sensor data
File GPSdata;                   // SD file for GPS data

bool isError = false;         // is there an error?
bool sdError = false;         // is there an SD-specific error?
bool makeFileError = false;   // did makeFiles() fail?
bool gpsError = false;        // is there a problem with the GPS?
bool imuError = false;        // is there a problem with the IMU?
bool altError = false;        // is there a problem with the altimeter?

long sensor_stamp = 0;        // time-stamp for sensor_interval comparison
long alt_stamp = 0;           // time-stamp for alt_interval comparison

//*************************************************************************************************************
//*******                                           getGPS
//*************************************************************************************************************
void getGPS(){
  gpsStuff = GPS.lastNMEA();
  usb.println(gpsStuff);

  #ifdef TESTING_SD
    GPSdata.print(gpsStuff);
    GPSdata.flush();
  #endif
  
  gpsStuff = "";
}

//*************************************************************************************************************
//*******                                           getIMU
//*************************************************************************************************************
void getIMU(){
  #ifdef TESTING_IMU
    imu::Vector<3> BN_acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // creates vectors for each event
    imu::Vector<3> BN_gyro = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> BN_mag = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> BN_eul = IMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> BN_grav = IMU.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  #endif

  #ifdef TESTING_ADC
    pitot = (float)ads.readADC_SingleEnded(0);
  #endif

  OtherflushTotal = "";
  
  OtherflushTotal = OtherflushTotal + F(":");

  #ifdef TESTING_IMU
    OtherflushTotal = OtherflushTotal + BN_acc.x() + F(",") + BN_acc.y() + F(",") + BN_acc.z();
    OtherflushTotal = OtherflushTotal + F(",") + BN_gyro.x() + F(",") + BN_gyro.y() + F(",") + BN_gyro.z();
    OtherflushTotal = OtherflushTotal + F(",") + BN_mag.x() + F(",") + BN_mag.y() + F(",") + BN_mag.z();
    OtherflushTotal = OtherflushTotal + F(",") + BN_eul.x() + F(",") + BN_eul.y() + F(",") + BN_eul.z();
    OtherflushTotal = OtherflushTotal + F(",") + BN_grav.x() + F(",") + BN_grav.y() + F(",") + BN_grav.z();
  #endif
  
  #ifdef TESTING_ALT
    OtherflushTotal = OtherflushTotal + F(",") + temp + F(",") + pressure + F(",") + humid; 
  #endif
  
  #ifdef TESTING_ADC
    OtherflushTotal = OtherflushTotal + F(",") + pitot;
  #endif
  
  OtherflushTotal = OtherflushTotal + F("--");

//  #ifdef TESTING_IMU
//    imuErrChk1 = (long)(BN_acc.x()+BN_acc.y()+BN_acc.z()+BN_gyro.x()+BN_gyro.y()+BN_gyro.z()+BN_mag.x()+BN_mag.y()+BN_mag.z());
//    imuErrChk2 = (long)(BN_mag.x()-BN_mag.y()+BN_mag.z()+0.06);
//
//    if((((imuErrChk1 <= 0.01) && (imuErrChk1 >= -0.01)) || ((imuErrChk2 <= 0.01) && (imuErrChk2 >= -0.01))))
//    {
//      imuError = true;
//    }
//  #endif
  
  usb.println(OtherflushTotal);

  #ifdef TESTING_SD
    OtherData.print(OtherflushTotal);
    OtherData.flush();
  #endif
}

//*************************************************************************************************************
//*******                                           getIMUstatus
//*************************************************************************************************************
void getIMUstatus(){
  //usb.println(F("entering getIMUstatus"));
  IMU.getSystemStatus(&system_status, &self_test_results, &system_error); // gets system status
  if(system_error){
    imuError = true;
  } 
}

//*************************************************************************************************************
//*******                                           getALT
//*************************************************************************************************************
void getAltimeter(){
  temp = ALT.readTemperature();
  pressure = ALT.readPressure();
  humid = ALT.readHumidity();

  if((pressure > 108380) || !(pressure == pressure) || (pressure < 0)){ // checks for potential erros in 
    altError = true;
  }
}

//*************************************************************************************************************
//*******                                           MakeFiles
//*************************************************************************************************************
int makeFiles(){
  //usb.println(F("entering makeFiles()"));// 
  bool rtn = true;                          // set a boolean flag of rtn to be true

// commented out, filenames set to global vars  ***********************************
  char gpsfile[15];                         // create a pointer for a character string of 15 for gps string
  char otherdatafile[15];                   // create a pointer for a character string of 15 for other data strings
  
  strcpy(gpsfile, "GPSLOG05.TXT");          // set the string name to the pointer
  strcpy(otherdatafile, "AODATA05.TXT");    // set the string name to the pointer
  
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
  for(uint8_t i = 0; i < 100; i++)          // for the gps file 
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

void getErrors()
{
  getIMUstatus();
  
  isError = (sdError || makeFileError || gpsError || imuError || altError);
  
  #ifdef TESTING_IMU
    if(imuError)
    {
      usb.println(F("THERE WAS AN IMU ERROR!"));
      usb.print(F("imuErrChk1 : "));
      usb.println(imuErrChk1);
      usb.print(F("imuErrChk2 : "));
      usb.println(imuErrChk2);
      usb.print(F("system_error : "));
      usb.println(system_error);
      usb.print(F("self_test_results : "));
      usb.println(self_test_results);
      usb.print(F("system_status : "));
      usb.println(system_status);
    }
  #endif
  
  #ifdef TESTING_ALT
    if(altError)
    {
      
    }
  #endif
      
  #ifdef TESTING_GPS
    if(gpsError)
    {
      usb.println(F("THERE WAS A GPS ERROR!"));
    }
  #endif
    
  #ifdef TESTING_SD
    if(sdError)
    {
      usb.println(F("THERE WAS AN SD HARDWARE ERROR!"));
    }
  
    if(makeFileError)
    {
      usb.println(F("THERE WAS A MAKEFILE ERROR!"));
    }
  #endif

  if(!isError)
  {
    usb.println(F("NO ERRORS DETECTED!"));
  }
}

//*************************************************************************************************************
//*******                                           CRITICAL GPS-RELATED FUNCTIONS
//*************************************************************************************************************
/* ARM SPECIFIC STUFF*/
#ifdef USING_ARM
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
      #if GPSECHO
        Serial.print(c);
      #endif
    }
  }
#endif

/* AVR SPECIFIC STUFF.*/
#ifdef USING_AVR
  void SIGNAL(TIMER0_COMPA_vect) {
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
  #if defined(USING_UNO) || defined(USING_MEGA) 
    if (GPSECHO)
      if (c) UDR0 = c;  
      // writing direct to UDR0 is much much faster than usb.print 
      // but only one character can be written at a time. 
  #endif
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
#endif


//*************************************************************************************************************
//*******                                           SETUP
//*************************************************************************************************************
void setup() {
  delay(10000);

  usb.begin(115200);
  usb.println(F("INITIALIZING"));

  #if defined(TESTING_IMU) || defined(TESTING_ALT) || defined(TESTING_ADC)
    OtherflushTotal.reserve(300);
  #endif

  #ifdef TESTING_SD
    if(!SD.begin(SD_CS)){         // initializes the SD card
      usb.println(F("SD Card failed to initialize!")); // if the SD card fails, set sdError true and tell someone
      sdError = true;
    }                                 
    else if(!makeFiles()){        // initializes the SD files
      usb.println(F("File Initialization Failed!")); // if makeFiles fails, set makeFileError to true and tell someone
      makeFileError = true;
    }
    else{
      usb.println(F("makefiles worked!"));
    }
  #endif

  #ifdef TESTING_IMU
    if(!IMU.begin()){             // initializes IMU
      usb.println(F("BNO055 IMU Initialization Error")); // if IMU fails, set imuError to true and tell someone
      imuError = true;
    }
    else{
      usb.println(F("IMU works!"));
    }  
  #endif

  #ifdef TESTING_ALT
    if(!ALT.begin()){             // initializes the altimeter
      usb.println(F("BME280 Sensor Inilization Error"));
      altError = true;
    }
    else{
      usb.println(F("ALT works!"));
    }
  #endif

  #ifdef TESTING_ADC
    ads.begin();                          // initializes the ADC for the pitot tube
  #endif
      
  #ifdef TESTING_GPS
    gpsStuff.reserve(350);                       //
    GPS.begin(9600);
    GPS.sendCommand(PMTK_ENABLE_WAAS);            // enables dgps reception (higher accuracy in North America)
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Getting RMA and GGA data (see http://www.gpsinformation.org/dale/nmea.htm)
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);    //
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);    //GPS will refresh 5 times a second (doesn't mean you will get data that often)
    GPS.sendCommand(PGCMD_NOANTENNA);             // kills the antenna status update           
    useInterrupt(true);                           // uses interrupts to gather incoming data
  #endif

  getErrors();
 
  usb.println(F("INITIALIZATION COMPLETE! ENTERING LOOP"));              
}

void loop() {
  #ifdef TESTING_GPS
    if(GPS.newNMEAreceived())
    {
      getGPS();
    }
  #endif

  #if defined(TESTING_IMU) || defined(TESTING_ALT) || defined(TESTING_ADC)
    if(millis() - sensor_stamp >= sensor_interval)
    {
      getIMU();
      sensor_stamp = millis();
    }
  #endif

  #ifdef TESTING_ALT
    if(millis() - alt_stamp >= alt_interval)
    {
      getAltimeter();
      alt_stamp = millis();
    }
  #endif

  getErrors();
}
