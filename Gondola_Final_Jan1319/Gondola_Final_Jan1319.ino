/*
 * BALLOON PROJECT 
 * 
 * Author :          Gerrit Motes 6/19
 * Special Contrib:  MakeFiles by Rodney Metoyer in The BeforeTime, in The LongLongAgo...
 * 
 * Equipment integrated into this build:
 *  - Adafruit BNO055 9DOF IMU
 *  - Adafruit BME280 Altimeter
 *  - Adafruit Ultimate GPS breakout
 *  - Adafruit microSD reader/writeer breakout
 *  - Hobbyking small pitotstatic tube with an Adafruit ADS115 ADC
 *  - Digi Intl. Xbee 900HP S3B radio / sparkfun xplorer regulated
 *  
 *  Previous Rev: Gondola_Final_Jun19
 */

#include <Wire.h>             // wire library (needed for some sensors)
#include <SPI.h>              // SPI digital comms library (needed for SD card)
#include <pt.h>               // protothread library
//#include <avr/wdt.h>          // watchdog hardware reset library (not compatible with SAMD51)                   // watchdog
#include <SD.h>               // SD library
#include <Adafruit_Sensor.h>  // master sensor library
#include <Adafruit_GPS.h>     // GPS library
#include <Adafruit_BNO055.h>  // IMU library
#include <Adafruit_BME280.h>  // Altimeter library
#include <Adafruit_ADS1015.h> // ADC library
#include "xbeeAPI.h"          // xbee library (900HP S3B models in API SPI mode)
#include <Adafruit_DotStar.h> // controls LED on ItsyBitsyM4 MCU
//#include <Servo.h>            // controls rudder servo

#define usb Serial            // renames Serial as usb
#define gps Serial1           // renames Serial2 as gps

#define CUTDOWN 10             // pin to trigger cutdown

//Xbee OBJECT DEFINITION
#define XB1_SS (5)                          // M4 PIN FOR XBEE SPI SLAVE SELECT
#define XB1_ATTN (7)                        // M4 PIN FOR XBEE ATTN 
#define XB1_CMD (9)                         // M4 PIN NUMBER FOR COMMAND SIGNAL
#define XB1_LOC_COMMAND_DIO ('0')           // DIO NUMBER FOR LOCAL XBEE COMMAND PIN
#define XB1_LOC_CUTDOWN_DIO ('5')           // DIO NUMBER FOR LOCAL XBEE CUTDOWN PIN
#define XB1_DEST_COMMAND_DIO ('0')          // DIO NUMBER FOR DESTINATION COMMAND PIN
#define XB1_DEST_CUTDOWN_DIO ('5')          // DIO NUMBER FOR DESTINATION CUTDOWN
#define XB1_DEST_ADDR (0x0013A200417E38C1)  // DESTINATION XBEE 64-BIT ADDRESS 
#define XB1_SPI_CLK (3000000)               // SPI BUS FREQUENCY FOR XBEE DATA
#define XB1_PRIMARY_MODE (1)                // 0 = FAST, 1 = ACCURATE, 2 = RECEIVER
#define XB1_MAX_PAYLOAD_SIZE 256                 // cannot exceed NP (256)
Xbee xbee = Xbee(XB1_SS, XB1_ATTN);         // CONSTRUCTOR FOR XBEE OBJECT
uint16_t XB1_NP;                            // holds the value for AT property NP (max payload size)
bool stringGrab_init = false;
bool stringGrab_gps = false;
bool stringGrab_other = false;

// RUDDER CONTROL STUFFS
//#define rudderPot A0        // NOT USED UNTIL RUDDER CLASS FINISHED
//#define rudderServo A4

// IMU OBJECT DEFINITION
Adafruit_BNO055 IMU = Adafruit_BNO055(55,BNO055_ADDRESS_B); 
//#define imuReset 34     // used on the imu RST pin for a soft reset
#define imuPower 12     // used on the imu power transistor gate for a hard reset
uint8_t system_status, self_test_results, system_error; // imu sensor error vars

//SPI communication pins, SCK, MISO, and MOSI are common to all SPI sensors, CS pins are unique to each 
// SPI sensor.
#define SD_CS 2
#define sdPower 3      // SD power transistor gate pin

// Pitot analog pin definition
//#define PITOT A15
Adafruit_ADS1115 ads;         // pitot ADC object
float pitotReadTotal = 0;        // global var for pitot reading
float pitot = 0;                 // global var for averaged pitot value

// altimeter variables
float temp = 0;               //
float pressure = 0;           // global vars for altimeter data
float humid = 0;              //
float altAltitude = 0;        // holds the altitude data supplied by the altimeter
Adafruit_BME280 ALT;          //  ALT I2C
#define altPower 11           //altimeter power transistor pin
#define altitudeThresh (2000)   // threshold altitude for cutdown timer initiation
#define gndlvlPress 1023.4   // ground level pressure for calculating elevation  
                              //  (in mbar, even though the pressure output is in different units ¯\_(ツ)_/¯ )

uint8_t stringGrab_initial[500];    // char array for uncompressed stringGrab data
uint8_t stringGrab_compressed[500]; // char array for compressed stringGrabdata
uint8_t stringChop_payload[256];    // char array for xbee sendPayload

uint16_t compressed_size;           // how long the stringGrab_compressed array is

uint32_t packetNum = 0;         // holds the unique identifier of each xbee payload

String OtherflushTotal = "";    // string object for sensor data (temporary storage)
String GPSflushTotal = "";      // string object for GPS data (temporary storage)
String gpsStuff = "";           // string object for passing GPS data to various functions
String gpsTime = "";            // used for storage of the last GPS timestamp parsed from an NMEA sentence
String stringGrab_temp = "";    // used for storing the temp data prior to compressing and chopping
File OtherData;                 // SD file for sensor data
File GPSdata;                   // SD file for GPS data

// GPS stuff
Adafruit_GPS GPS(&gps); // GPS
//#define GPSECHO  false
boolean usingInterrupt;
void useInterrupt(boolean);
//#define gpsPower 26           // gps power transistor gate pin (used on the ENABLE pin to control GPS power)


bool isError = false;         // is there an error?
bool sdError = false;         // is there an SD-specific error?
bool makeFileError = false;   // did makeFiles() fail?
bool gpsError = false;        // is there a problem with the GPS?
bool imuError = false;        // is there a problem with the IMU?
long imuErrChk1 = 1;          // used for tracking non-obvious errors in IMU data
long imuErrChk2 = 1;          // used for tracking non-obvious errors in IMU data
bool altError = false;        // is there a problem with the altimeter?
bool fatalError = false;      // is there a fatal error?
bool cutdown = false;         // has the cutdown been fired?

static struct pt errorFixPT;
static struct pt fatalFixPT;
static struct pt getGpsPT;
static struct pt getImuPT;
static struct pt getAltPT;
static struct pt getPitotPT;
static struct pt altErrorPT;
static struct pt writePT;
static struct pt GPSwritePT;
static struct pt getIMUstatusPT;
static struct pt imuErrorPT;
static struct pt gpsErrorPT;
static struct pt cutdownStartPT;
static struct pt cutdownPT;
static struct pt cutdownResetPT;
static struct pt sdFixPT;
static struct pt fileFixPT;
static struct pt gpsFixPT;
static struct pt imuFixPT;
static struct pt altFixPT;

static struct pt stringGrabInitPT;
static struct pt stringGrabGpsPT;
static struct pt stringGrabOtherPT;
static struct pt stringChopPT;
static struct pt xbeeCommandPT;
static struct pt rudderAnglePT;

#define imuThresh 20                    // time (ms) between each imu reading
#define altThresh 1000                  // time (ms) between each altimeter reading
#define pitotReadThresh 2               // time (ms) between each pitot reading
#define writeThresh 2                   // number of successful data pulls before writing to SD
#define gpsWriteThresh 5                // number of successful gps pulls before writing to SD
#define cutdownThresh 120000            // waiting period (ms) after threshold altitude is reached before cutting down
#define getIMUstatusThresh 180000       // ms threshold for checking IMU system status (since it takes so long to run)
#define errorVal1 (long)0.00            //
#define errorVal2 (long)-0.01           // standard error vals for the IMU
#define errorVal3 (long)-0.06           //
#define gps_getTimeThresh 60000         // parses time data from NMEA sentences
#define stringGrab_thresh 1000          // ms threshold for grabbing a string of data for sending over xbee
#define rudderAngle_thresh 5000         // time (ms) between rudder angle checks

long imuStamp = 0;            // timestamp tracking last time IMU sensor data was gathered
long imuStatusStamp = 0;      // timestamp tracking last time IMU status info was gathered
long altStamp = 0;            // timestamp tracking last time data from altimeter was gathered
long pitotStamp = 0;          // timestamp used to track last time the pitot was sampled
long cutdownStamp = 0;        // timestamp used to track time above threshold altitude for cutdown
long errorFix_Stamp = 0;      // timestamp used to track when to call non-fatal error fix checks
long gpsTimeStamp = 0;        // timestamp used to track how long its been since last GPS sentence (for error tracking)
long gps_getTimeStamp = 0;    // timestamp used to track how long its been since the last time parse from a gps sentence
long stringGrab_stamp = 0;    // timestamp used to track how long its been since stringGrab
long rudderAngle_stamp = 0;   // timestamp used to check and apply changes to rudder position
uint8_t writeCount = 0;           // tracks the number of successful IMU pulls before writing to SD
uint8_t GPSwriteCount = 0;        // tracks the number of successful GPS pulls before writing to SD
uint8_t gpsCount = 0;             // tracks the number of successful NMEA sentence pulls before writing to SD
uint8_t fatalResetCount = 0;      // tracks the number of reset attempts, used to initiate watchdog reset
uint8_t imuResetCount = 0;        // tracks the number of IMU resets, used to tie recurring errors to watchdog
uint8_t altResetCount = 0;        // tracks the number of IMU resets, used to tie recurring errors to watchdog
uint8_t pitotCount = 0;           // tracks number of times the pitot was sampled before averaging 
float altitude = 0;           // holds the official altitude info (in case the GPS or parsing function breaks, use altimeter as backup)
float rudderAngle_actual;
float rudderAngle_setpoint = 0;


//char prev_gpsfile[15];        // previous file name for the GPS file in case of an sdError or makeFileError reset
//char prev_otherdatafile[15];  // previous file name for the sensor file in case of an sdError or makeFileError reset
//char gpsfile[15];             // current file name for the GPS file in case of an sdError or makeFileError reset
//char otherdatafile[15];       // current file name for the sensor file in case of an sdError or makeFileError rese

//*************************************************************************************************************
//*******                                   PROTOTHREAD getGPS_sense
//*************************************************************************************************************
static int getGPS_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, GPS.newNMEAreceived());
    getGPS();
    gpsTimeStamp = millis();
    gpsError = false;
  }
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD getIMU_sense
//*************************************************************************************************************
static int getIMU_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, (millis() - imuStamp) >= imuThresh);
    getIMU();
    imuStamp = millis();
  }
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD getALT_sense
//*************************************************************************************************************
static int getALT_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt,(millis() - altStamp) >= altThresh);
    getAltimeter();
    altStamp = millis();
  }
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD write_sense
//*************************************************************************************************************
static int write_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, writeCount >= writeThresh);
    write2SD();
    writeCount = 0;
    OtherflushTotal = "";
  }
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD GPSwrite_sense
//*************************************************************************************************************
static int GPSwrite_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, GPSwriteCount >= gpsWriteThresh);
    GPSwrite2SD();
    GPSwriteCount = 0;
    GPSflushTotal = "";
  }
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD getIMUstatus_sense
//*************************************************************************************************************
static int getIMUstatus_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt,(millis() - imuStatusStamp) >= getIMUstatusThresh);
    getIMUstatus();
    imuStatusStamp = millis();
  }
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD cutdownStart_sense
//*************************************************************************************************************
static int cutdownStart_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt,((cutdown == false) && ((float)altitude > (float)altitudeThresh) && (cutdownStamp < 1)));
    cutdownStamp = millis();
    //usb.println(F("cutdownStart"));
  }
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD cutdown_sense
//*************************************************************************************************************
static int cutdown_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt,((cutdownStamp > 0) && ((float)altitude > (float)altitudeThresh) && ((millis()-cutdownStamp) >= cutdownThresh)));
    cutdownFunc();
  }
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD cutdownReset_sense
//*************************************************************************************************************
static int cutdownReset_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt,((cutdownStamp > 0) && (altitude < altitudeThresh)));
    cutdownStamp = 0;
    //usb.println(F("cutdownReset"));
  }
  PT_END(pt);  
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD errorFix_sense
//*************************************************************************************************************
static int errorFix_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, (((millis()-errorFix_Stamp) >= gps_getTimeThresh)));
    fixErrors();
  }
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD fatalFix_sense
//*************************************************************************************************************
static int fatalFix_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt,(fatalError));
    fatalErrorFix();
  }
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD gpsError_sensese
//*************************************************************************************************************
static int gpsError_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, (millis() - gpsTimeStamp >= altThresh));
  gpsError = true;
  fatalError  = true;
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD imuError_sense
//*************************************************************************************************************
static int imuError_sense(struct pt *pt){
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, (((imuErrChk1 <= 0.01) && (imuErrChk1 >= -0.01)) || ((imuErrChk2 <= 0.01) && (imuErrChk2 >= -0.01))));
    imuError = true;
    isError = true; 
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD altError_sense
//*************************************************************************************************************
static int altError_sense(struct pt *pt){
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, (pressure > 108380) || !(pressure == pressure) || (pressure < 0));
    altError = true;
    isError = true;
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD getPitot_sense
//*************************************************************************************************************
static int getPitot_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, (pitotStamp - millis()) >= pitotReadThresh);
    pitotReadTotal = pitotReadTotal + (float)ads.readADC_SingleEnded(0);
    pitotCount++;
    pitotStamp = millis();
  }
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD sdFix_sense
//*************************************************************************************************************
static int sdFix_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, sdError);
  sdFix();
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD fileFix_sense
//*************************************************************************************************************
static int fileFix_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, makeFileError);
  fileFix();
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD gpsFix_sense
//*************************************************************************************************************
static int gpsFix_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, gpsError);
  gpsFix();
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD imuFix_sense
//*************************************************************************************************************
static int imuFix_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, imuError);
  imuFix();
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD altFix_sense
//*************************************************************************************************************
static int altFix_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, altError);
  altFix();
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD stringGrabInit_sense
//*************************************************************************************************************
static int stringGrabInit_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, !(stringGrab_init) && (millis() - stringGrab_stamp >= stringGrab_thresh));
    stringGrab_init = true;
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD stringGrabGPS_sense
//*************************************************************************************************************
static int stringGrabGPS_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, stringGrab_init && !(stringGrab_gps) && GPSwriteCount == 1);
    stringGrab_gps = true;
    stringGrab_init = false;
    stringGrab(GPSflushTotal);
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD stringGrabOther_sense
//*************************************************************************************************************
static int stringGrabOther_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, stringGrab_gps && !(stringGrab_other) && writeCount == 1);
    stringGrab_other = true;
    stringGrab_gps = false;
    compressed_size = stringGrab(OtherflushTotal);
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD stringChop_sense
//*************************************************************************************************************
static int stringChop_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, stringGrab_other && xbee.sendAvailable());
    stringGrab_stamp = millis();
    stringChop(stringGrab_compressed,compressed_size);
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD xbeeCommand_sense
//*************************************************************************************************************
static int xbeeCommand_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, xbee.command_receive_length);
    commandParse(xbee.command_receive_buffer,xbee.command_receive_length);
    xbee.command_receive_length = 0;
  PT_END(pt);
}

////*************************************************************************************************************
////*******                                   PROTOTHREAD rudderAngle_sense                                               // probably wrapped up into Rudder Class
////*************************************************************************************************************
//static int rudderAngle_sense(struct pt *pt){
//  PT_BEGIN(pt);
//  PT_WAIT_UNTIL(pt, (!rudderControl) && (getRudderAngle() != rudderAngle_set) && (millis() - rudderAngle_stamp > rudderAngle_thresh));
//    driveRudderManual(rudderAngle_set);
//    rudderAngle_stamp = millis();
//  PT_END(pt);
//}



//*************************************************************************************************************
//*******                                           getGPS
//*************************************************************************************************************
void getGPS(){
  //usb.println(F("entering getGPS"));
  gpsStuff = GPS.lastNMEA();
  //usb.println(gpsStuff);
  if(millis() - gps_getTimeStamp > gps_getTimeThresh){
    if(gps_GetTime(gpsStuff)){
      gps_GetAlt(gpsStuff);
      gps_getTimeStamp = millis();
    }
    GPSflushTotal = GPSflushTotal + gpsStuff;
    GPSwriteCount++;
    gpsStuff = "";
  }
  else{
    GPSflushTotal = GPSflushTotal + gpsStuff;
    GPSwriteCount++;
    gpsStuff = "";
  }
}

//*************************************************************************************************************
//*******                                           getIMU
//*************************************************************************************************************
void getIMU(){
  //usb.println(F("entering getIMU"));
  imu::Vector<3> BN_acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // creates vectors for each event
  imu::Vector<3> BN_gyro = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> BN_mag = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> BN_eul = IMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> BN_grav = IMU.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  pitot = (float)pitotReadTotal / (float)pitotCount;

  long gps_timeAddition =  millis() - gps_getTimeStamp; // calculates the duration since last GPS time fix (in ms)

  OtherflushTotal = OtherflushTotal + F("\n") + gpsTime + F(",") + gps_timeAddition + F("MS,") + altitude + F(":");
  OtherflushTotal = OtherflushTotal + BN_acc.x() + F(",") + BN_acc.y() + F(",") + BN_acc.z();
  OtherflushTotal = OtherflushTotal + F(",") + BN_gyro.x() + F(",") + BN_gyro.y() + F(",") + BN_gyro.z();
  OtherflushTotal = OtherflushTotal + F(",") + BN_mag.x() + F(",") + BN_mag.y() + F(",") + BN_mag.z();
  OtherflushTotal = OtherflushTotal + F(",") + BN_eul.x() + F(",") + BN_eul.y() + F(",") + BN_eul.z();
  OtherflushTotal = OtherflushTotal + F(",") + BN_grav.x() + F(",") + BN_grav.y() + F(",") + BN_grav.z();
  OtherflushTotal = OtherflushTotal + F(",") + temp + F(",") + pressure + F(",") + humid + F(",") + altAltitude + "," + pitot + F("--");
  
  
  imuErrChk1 = (long)(BN_acc.x()+BN_acc.y()+BN_acc.z()+BN_gyro.x()+BN_gyro.y()+BN_gyro.z()+BN_mag.x()+BN_mag.y()+BN_mag.z());
  imuErrChk2 = (long)(BN_mag.x()-BN_mag.y()+BN_mag.z()+0.06);

  writeCount++;
  pitotCount = 0;
  pitotReadTotal = 0;
  //usb.println(F("leaving getIMU"));
  //usb.println(altitude);
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
  //usb.println(F("entering getAltimeter"));
  temp = ALT.readTemperature();
  pressure = ALT.readPressure();
  humid = ALT.readHumidity();
  altAltitude = ALT.readAltitude(gndlvlPress);

  if((pressure > 108380) || !(pressure == pressure) || (pressure < 0)){ // checks for potential erros in 
    altError = true;
    isError = true;
  }
}

//*************************************************************************************************************
//*******                                           write2SD
//*************************************************************************************************************
void write2SD(){
  //usb.println(F("entering write2SD"));
  OtherData.print(OtherflushTotal);
  OtherData.flush();
  //usb.println(F("Otherflush"));
  writeCount = 0;
}

//*************************************************************************************************************
//*******                                           GPSwrite2SD
//*************************************************************************************************************
void GPSwrite2SD(){
  //usb.println(F("entering GPSwrite2SD"));
  GPSdata.print(GPSflushTotal);
  GPSdata.flush();
  //usb.println(F("GPSflush"));
  gpsCount = 0;
}

//*************************************************************************************************************
//*******                                           fixErrors
//*************************************************************************************************************
void fixErrors(){
  //usb.println(F("entering fixErrors"));
  imuFix_sense(&imuFixPT);
  altFix_sense(&altFixPT);
  if(!imuError && !altError){
    isError = false;
  }
  else{
    isError = true;
  }
  errorFix_Stamp = millis();
}

//*************************************************************************************************************
//*******                                           fatalErrorFix
//*************************************************************************************************************
void fatalErrorFix(){
  //usb.println(F("entering fatalErrorFix"));
  sdFix_sense(&sdFixPT);
  fileFix_sense(&fileFixPT);
  gpsFix_sense(&gpsFixPT);
  if(!sdError && !makeFileError && !gpsError){
    fatalError = false;
  }
  else{
    fatalError = true;
  }
  fatalResetCount++;
}

//*************************************************************************************************************
//*******                                           gps_GetTime
//*************************************************************************************************************
bool gps_GetTime(String NMEA_Sentence){
  //usb.println(F("entering gps_GetTime"));
  if(NMEA_Sentence.length() < 4){
    return false;
  }
  else{
    if((NMEA_Sentence.indexOf(F("GGA")) != -1) || (NMEA_Sentence.indexOf(F("RMC")) != -1)){
      int comma1 = NMEA_Sentence.indexOf(',');                        // determine index locations of 1st and
      int comma2 = NMEA_Sentence.indexOf(',', (comma1 + 1));          // 2nd comma, and build a substring of 
      String newtime = NMEA_Sentence.substring((comma1+1), comma2);
      usb.println(newtime);
      if(newtime.length() > 4){
        gpsTime = newtime;
        //gps_getTimeStamp = millis(); not needed, done in protothread
        return true;
      }
      else{
        return false;
      }
    }
    else{
      return false;
    }
  }
}

//*************************************************************************************************************
//*******                                           gps_GetAlt
//*************************************************************************************************************
void gps_GetAlt(String NMEA_Sentence){
  //usb.println(F("entering gps_GetAlt"));
  if(NMEA_Sentence.indexOf(F("GGA")) == -1){
    int countdown = 1000;
    while(!GPS.newNMEAreceived()){
      countdown = countdown - 1;
      if(countdown <=0){
        break;
      }
    }
    NMEA_Sentence = GPS.lastNMEA();
  } 
  if(NMEA_Sentence.indexOf(F("GGA")) != -1){
    int index1 = 0;
    int index2 = 0;
    index1 = NMEA_Sentence.indexOf(',');
    for(int i = 2; i <= 9; i++){
      index2 = NMEA_Sentence.indexOf(',', (index1+1));
      index1 = index2; 
    }
    int index3 = NMEA_Sentence.indexOf(',', (index2+1));
    int index4 = NMEA_Sentence.indexOf(',', (index3+1));
    String newElev = NMEA_Sentence.substring((index2+1), index3);
    String eleCheck = NMEA_Sentence.substring((index3+1), index4);
    if((newElev.length() > 3 || eleCheck == "M") && newElev.toFloat() > 10){
      altitude = newElev.toFloat();
    }
    else{
      altitude = altAltitude;
    }
  }
  else{
    altitude = altAltitude;
  }
}

//*************************************************************************************************************
//*******                                           sdFix
//*************************************************************************************************************
void sdFix(){
  //usb.println(F("entering sdFix"));
  digitalWrite(sdPower, LOW);
  delay(250);
  digitalWrite(sdPower, HIGH);
  delay(250);

  if(!SD.begin()){
    sdError = true;
  }
  else if(!makeFiles()){
    sdError = false;
    makeFileError = false;
  }

//  GPSdata = SD.open(gpsfile, FILE_WRITE);                 // attempts to re-open current files
//  OtherData = SD.open(otherdatafile, FILE_WRITE);
//                                                         // if this does not work...
//  if((!GPSdata) || (!OtherData) || (!SD.exists(otherdatafile))){                                       
//    sdError = true;                                             //reset the arduino             
//  }
//  else{
//    sdError = false;
//    makeFileError = false;                                        
//  }        
}

//*************************************************************************************************************
//*******                                           fileFix
//*************************************************************************************************************
void fileFix(){
  usb.println(F("entering fileFix"));
  digitalWrite(sdPower, LOW);                          // resets SD card reader
  delay(250);
  digitalWrite(sdPower, HIGH);
  delay(250);

//  strcpy(prev_gpsfile, gpsfile);                     // updates the previous gps and other file
//  strcpy(prev_otherdatafile, otherdatafile);         // names prior to calling makeFiles()
                                                         // attempts to creat new files
                                                         // if this does not work...
  if(!makeFiles()){                                         
    makeFileError = true;                                             //reset the arduino             
  }
  else{                                                   // if reset is successful, set error flag
                                                          // to false, and dump error message and 
    makeFileError = false;                                    // linking filenames to each file on SD
  }
}

//*************************************************************************************************************
//*******                                           gpsFix
//*************************************************************************************************************
void gpsFix(){
  //usb.println(F("entering gpsFix"));
  //usb.println(F("restarting gps"));
  //digitalWrite(gpsPower, HIGH);
  delay(250);
  //digitalWrite(gpsPower, LOW);
  delay(250);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Getting RMA and GGA data (see http://www.gpsinformation.org/dale/nmea.htm)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);    //
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);    //GPS will refresh 5 times a second (doesn't mean you will get data that often)
  GPS.sendCommand(PGCMD_NOANTENNA);             // kills the antenna status update           
  //useInterrupt(true);                                                                                                             //useinterrupt

  int gpsErrorTimer = millis();
  while((!GPS.newNMEAreceived()) && ((millis()-gpsErrorTimer) < 3000)){}
  String gpsErrTest = "";
  if(GPS.newNMEAreceived()){
    gpsErrTest = gpsErrTest + GPS.lastNMEA();
  }
  if(gpsErrTest.length() > 4){
    gpsError = false;
  }
  else{
    gpsError = true;
  }
}

//*************************************************************************************************************
//*******                                           imuFix
//*************************************************************************************************************
void imuFix(){
  //usb.println(F("entering imuFix"));
  digitalWrite(imuPower, LOW); // resets the imu by pulling the transistor gate pin low and then high
  delay(250);
  digitalWrite(imuPower, HIGH);
  delay(250);
  if(!IMU.begin()){
    imuError = true;
  }
  else{
    system_status = self_test_results = system_error = 0;
    IMU.getSystemStatus(&system_status, &self_test_results, &system_error);
    if(system_error){
      imuError = true;
    }
    else{
      imuError = false;
    }
  }
  if(!imuError){
    delay(250);
    imu::Vector<3> BN_acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // creates vectors for each event
    imu::Vector<3> BN_gyro = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> BN_mag = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    imuErrChk1 = (long)(BN_acc.x()+BN_acc.y()+BN_acc.z()+BN_gyro.x()+BN_gyro.y()+BN_gyro.z()+BN_mag.x()+BN_mag.y()+BN_mag.z());
    imuErrChk2 = (long)(BN_mag.x()-BN_mag.y()+BN_mag.z()+0.06);
    if(((imuErrChk1 <= 0.01) && (imuErrChk1 >= -0.01)) || ((imuErrChk2 <= 0.01) && (imuErrChk2 >= -0.01))){
      imuError = true;
    }
    else{
      imuError = false;
    }
  }
  imuResetCount++;
}

//*************************************************************************************************************
//*******                                           altFix
//*************************************************************************************************************
void altFix(){
  usb.println(F("entering altFix"));
  digitalWrite(altPower, LOW);  // resets the altimeter via the transistor gate
  delay(250);
  //digitalWrite(altPower, HIGH);
  delay(250);

  if(!ALT.begin()){            // re-checks the status of the altimeter
    altError = true;
  }
  else{
    float pressureTest = ALT.readPressure();
    if((pressure > 108380) || !(pressure == pressure) || (pressure < 0)){
      altError = true;
    }
    else{
      altError = false;
    }
  }
}

//*************************************************************************************************************
//*******                                           stringGrab
//*************************************************************************************************************
uint16_t stringGrab(String input_string){
  static uint16_t gps_size;
  static uint16_t other_size;
  bool concatted = false;
  
  if(stringGrab_other)
  {
    other_size = input_string.length();
    
    while(!concatted)
    {
      concatted = stringGrab_temp.concat(input_string);
    }
    
    uint16_t total_size = gps_size + other_size;
    for(int i = 0; i < total_size ; i++)
    {
      stringGrab_initial[i] = stringGrab_temp.charAt(i);    
    }
    //stringGrab_temp.toCharArray(stringGrab_initial,total_size);

    for(int i = (total_size - 1); i >= 0 ; i--)
    {
      stringGrab_initial[i+6] = stringGrab_initial[i];
    }

    uint8_t hundreds = 0;
    uint8_t tens = 0;
    uint16_t tempNum = gps_size;
    while(tempNum - 100 >= 0)
    {
      tempNum -= 100;
      hundreds++;
    }
    while(tempNum - 10 >= 0)
    {
      tempNum -= 10;
      tens++;
    }
    stringGrab_initial[0] = hundreds;
    stringGrab_initial[1] = tens;
    stringGrab_initial[2] = (uint8_t)tempNum;

    hundreds = 0;
    tens = 0;
    tempNum = other_size;
    while(tempNum - 100 >= 0)
    {
      tempNum -= 100;
      hundreds++;
    }
    while(tempNum - 10 >= 0)
    {
      tempNum -= 10;
      tens++;
    }
    stringGrab_initial[3] = hundreds;
    stringGrab_initial[4] = tens;
    stringGrab_initial[5] = (uint8_t)tempNum;
    
    gps_size = 0;
    other_size = 0;
    
    matchCaseCompress(stringGrab_initial,total_size);
    uint16_t compress_size = doubleStuff(stringGrab_initial,stringGrab_compressed,total_size);
    return compress_size;
  }
  else if(stringGrab_gps)
  {
    stringGrab_temp = "";
    gps_size = input_string.length();
    return 0;
  }
  else
  {
    return 0;
  }
}

//*************************************************************************************************************
//*******                                           stringChop
//*************************************************************************************************************
void stringChop(uint8_t payload[],uint16_t payload_size){
  packetNum++;                       // increment the packetNum

  for(int i = 0; i < 4; i++)        // turn it into the front 4 bytes of payload
  {
    stringChop_payload[i] = (uint8_t)(packetNum >> ((3-i)*8));
  }
  uint16_t realPayload_size = XB1_MAX_PAYLOAD_SIZE - 4;
  
  if(payload_size > realPayload_size)
  {                               // chop up a packet if the payload is too big
    uint16_t remainder = payload_size - realPayload_size;
    for(int i = 0; i < realPayload_size; i++)
    {
      stringChop_payload[i+4] = stringGrab_compressed[i];
    }
    for(int i = 0; i < remainder; i++)
    {
      payload[i] = payload[i + realPayload_size];
    }
    payload_size = remainder;
    xbee.sendPayload(stringChop_payload,XB1_MAX_PAYLOAD_SIZE);
  }
  else
  {
    for(int i = 0; i < payload_size; i++)
    {
      stringChop_payload[i+4] = stringGrab_compressed[i];
    }
    stringGrab_other = false;
    xbee.sendPayload(stringChop_payload,payload_size);
  }
}

//*************************************************************************************************************
//*******                                           commandParse
//*************************************************************************************************************
void commandParse(uint8_t cmd_buffer[], int buffer_size){
  uint8_t command_type;
  uint8_t command_val;
  
  if(buffer_size > 12)
  {
    command_type = cmd_buffer[12];
    command_val = cmd_buffer[13];
  }
  else
  {
    return;
  }

  switch(command_type)
  {
    case 0xFF:                        // initiates a cutdown
      cutdownFunc();
      break;
//    case 0xFE:                        // sets the rudder angle and kicks rudder control
//      rudderControl = false;          // into manual mode
//      rudderAngle_set = command_val;
//      break;
//    case 0xFD:                        // sets the auto-controller for rudderAngle
//      if(command_val == 0x01)
//      {
//        rudderControl = true;
//      }
//      break;
    case 0xFC:                        // prints out system stats (errors and stuff);
      getSystemStats();
      break;
    case 0xFB:                        // if receiver, used to receive and print out system stats
      parse_systemStats(command_val);
      break;
    default:
      break;
  };
}

//*************************************************************************************************************
//*******                                           getSystemStats
//*************************************************************************************************************
void getSystemStats(){
  uint8_t sys_stat = 0x00;
  
  sys_stat |= (uint8_t)(cutdown << 7);          //
  sys_stat |= (uint8_t)(isError << 6);          //
  sys_stat |= (uint8_t)(fatalError << 5);       //
  sys_stat |= (uint8_t)(sdError << 4);          //  build a single systemStatus byte
  sys_stat |= (uint8_t)(makeFileError << 3);    //
  sys_stat |= (uint8_t)(gpsError << 2);         //
  sys_stat |= (uint8_t)(imuError << 1);         //
  sys_stat |= (uint8_t)(altError);              //

  xbee.sendCommand((uint8_t)0xFB,sys_stat); // sends response as a command to ensure receipt by ground
}

//*************************************************************************************************************
//*******                                           getSystemStats
//*************************************************************************************************************
void parse_systemStats(uint8_t sys_stat){
  uint8_t masked_stat;
  String errorName;
  Serial.println(F("\n\n***System Status***\n"));
  for(int i = 7; i >= 8; i--)
  {
    masked_stat = sys_stat & (uint8_t)(1 << i);
    if(masked_stat)
    {
      switch(i)
      {
        case 7:
          errorName = "A CUTDOWN IN PROGRESS!";
          break;
        case 6:
          errorName = "an error present";
          break;
        case 5:
          errorName = "a fatal error present";
          break;
        case 4:
          errorName = "an SD card error";
          break;
        case 3:
          errorName = "an error involving makeFile function";
          break;
        case 2:
          errorName = "a GPS unit error";
          break;
        case 1:
          errorName = "an IMU error";
          break;
        case 0:
          errorName = "an altimeter error";
          break;
      };
      Serial.print(F("    - There is "));
      Serial.println(errorName);
    }
  }
}

//*************************************************************************************************************
//*******                                           cutdown
//*************************************************************************************************************
void cutdownFunc(){
  usb.println(F("CUTDOWN INITIATED! FUUUUUUUUU!"));
  digitalWrite(CUTDOWN, LOW);
  delay(10000);
  digitalWrite(CUTDOWN, HIGH);
  cutdown = true;
  cutdownStamp = 0;
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

//*************************************************************************************************************
//*******                                           CRITICAL GPS-RELATED FUNCTIONS
//*************************************************************************************************************
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

/* AVR SPECIFIC STUFF, DO NOT INCLUDE UNLESS YOU ARE USING AN UNO, MEGA, ETC.*/
//void SIGNAL(TIMER0_COMPA_vect) {
//  char c = GPS.read();
//  // if you want to debug, this is a good time to do it!
//  if (GPSECHO)
//    if (c) UDR0 = c;  
//    // writing direct to UDR0 is much much faster than usb.print 
//    // but only one character can be written at a time. 
//}

//void useInterrupt(boolean v) {
//  if (v) {
//    // Timer0 is already used for millis() - we'll just interrupt somewhere
//    // in the middle and call the "Compare A" function above
//    OCR0A = 0xAF;
//    TIMSK0 |= _BV(OCIE0A);
//    usingInterrupt = true;
//  } else {
//    // do not call the interrupt function COMPA anymore
//    TIMSK0 &= ~_BV(OCIE0A);
//    usingInterrupt = false;
//  }
//}

//*************************************************************************************************************
//*******                              DATA COMPRESSION CODE
//*************************************************************************************************************
uint16_t decompress(uint8_t payload_in[], uint8_t payload_out[], uint16_t payloadIn_size)
{
  uint16_t decompressInd = 0;
  for(int i = 0; i < payloadIn_size; i++)
  {
    uint8_t val = payload_in[i];
    if(val >= 0xF0)
    {
      payload_out[decompressInd] = val;
      decompressInd++;
    }
    else
    {
      payload_out[decompressInd] = (uint8_t)(val >> 4);
      decompressInd++;
      val = val & 0x0F;
      if(val < 0x0F)
      {
        payload_out[decompressInd] = val;
        decompressInd++;
      }
    }
  }

  return decompressInd;
}

uint16_t doubleStuff(uint8_t payload_in[], uint8_t payload_out[], uint16_t payloadIn_size)
{
  int compressedInd = 0;
  bool comByte_halfFull = false;
  
  for(int i = 0; i < payloadIn_size; i++)
  {
    uint8_t val = payload_in[i];
    if(!comByte_halfFull)
    {
      payload_out[compressedInd] = 0x00;
      if(val < 0x0F)
      {
        payload_out[compressedInd] = (uint8_t)(val << 4);
        comByte_halfFull = true;
      }
      else if (val >= 0xF0)
      {
        payload_out[compressedInd] = val;
        compressedInd++;
      }
      else
      {
        payload_out[compressedInd] = 0xFF;
        compressedInd++;
      }
    }
    else
    {
      if(val < 0x0F)
      {
        payload_out[compressedInd] = payload_out[compressedInd] | val;
        compressedInd++;
        comByte_halfFull = false;
      }
      else if (val >= 0xF0)
      {
        payload_out[compressedInd] = payload_out[compressedInd] | 0x0F;
        compressedInd++;
        payload_out[compressedInd] = val;
        compressedInd++;
        comByte_halfFull = false;
      }
      else
      {
        payload_out[compressedInd] = payload_out[compressedInd] | 0x0F;
        compressedInd++;
        payload_out[compressedInd] = 0xFF;
        compressedInd++;
        comByte_halfFull = false;
      }
    }
  }

  if(comByte_halfFull)
  {
    payload_out[compressedInd] = payload_out[compressedInd] | 0x0F;
    compressedInd++;
  }

  return compressedInd;
}

void matchCaseCompress(uint8_t payload[], uint16_t payloadsize)
{
  uint16_t count = 0;
  for(int i = 0; i < payloadsize; i++)
  {
    count++;
    if(payload[i] > 0x09)
    {
      switch((uint8_t)payload[i])
      {
        case 0x30:
          payload[i] = 0x00;
          break;
        case 0x31:
          payload[i] = 0x01;
          break;
        case 0x32:
          payload[i] = 0x02;
          break;
        case 0x33:
          payload[i] = 0x03;
          break;
        case 0x34:
          payload[i] = 0x04;
          break;
        case 0x35:
          payload[i] = 0x05;
          break;
        case 0x36:
          payload[i] = 0x06;
          break;
        case 0x37:
          payload[i] = 0x07;
          break;
        case 0x38:
          payload[i] = 0x08;
          break;
        case 0x39:                // 9
          payload[i] = 0x09;
          break;
        case 0x2D:                // '-'
          payload[i] = 0x0A;
          break;
        case 0x2E:                // '.'
          payload[i] = 0x0B;
          break;
        case 0x2C:                // ','
          payload[i] = 0x0C;
          break;
        case 0x24:                // '$'
          payload[i] = 0x0D;
          break;
        case 0x0A:                // '\n' (newline)
          payload[i] = 0x0E;
          break;
        case 0x2A:                 // '*'
          payload[i] = 0xF0;
          break;
        case 0x4D:                 // 'M'
          payload[i] = 0xF1;
          break;
        case 0x52:                 // 'R'
          payload[i] = 0xF2;
          break;
        case 0x4E:                 // 'N'
          payload[i] = 0xF3;
          break;
        case 0x47:                 // 'G'
          payload[i] = 0xF4;
          break;
        case 0x50:                 // 'P'
          payload[i] = 0xF5;
          break;
        case 0x53:                 // 'S'
          payload[i] = 0xF6;
          break;
        case 's':                  // 's'
          payload[i] = 0xF6;
          break;
        case 0x56:                 // 'V'
          payload[i] = 0xF7;
          break;
        case 0x57:                 // 'W'
          payload[i] = 0xF8;
          break;
        case 0x41:                 // 'A'
          payload[i] = 0xF9;
          break;
        case 0x42:                 // 'B'
          payload[i] = 0xFA;
          break;
        case 0x43       :          // 'M'
          payload[i] = 0xFB;
          break;
        case 'm'        :          // 'm'
          payload[i] = 0xFB;
          break;
        case 0x44:                 // 'D'
          payload[i] = 0xFC;
          break;
        case 0x45:                 // 'E'
          payload[i] = 0xFD;
          break;
        case 0x46:                 // 'F'
          payload[i] = 0xFE;
          break;
        default:
          payload[i] = 0xFF;
      };
    }
  }
}

void matchCaseDecompress(uint8_t payload[], uint16_t payloadsize)
{
  for(int i = 0; i< payloadsize; i++)
  {
    switch(payload[i])
    {
      case 0x00:
        payload[i] = '0';
        break;
      case 0x01:
        payload[i] = '1';
        break;
      case 0x02:
        payload[i] = '2';
        break;
      case 0x03:
        payload[i] = '3';
        break;
      case 0x04:
        payload[i] = '4';
        break;
      case 0x05:
        payload[i] = '5';
        break;
      case 0x06:
        payload[i] = '6';
        break;
      case 0x07:
        payload[i] = '7';
        break;
      case 0x08:
        payload[i] = '8';
        break;
      case 0x09:
        payload[i] = '9';
        break;
      case 0x0A:
        payload[i] = '-';
        break;
      case 0x0B:
        payload[i] = '.';
        break;
      case 0x0C:
        payload[i] = ',';
        break;
      case 0x0D:
        payload[i] = '$';
        break;
      case 0x0E:
        payload[i] = '\n';
        break;
      case 0xF0:
        payload[i] = '*';
        break;
      case 0xF1:
        payload[i] = 'M';
        break;
      case 0xF2:
        payload[i] = 'R';
        break;
      case 0xF3:
        payload[i] = 'N';
        break;
      case 0xF4:
        payload[i] = 'G';
        break;
      case 0xF5:
        payload[i] = 'P';
        break;
      case 0xF6:
        payload[i] = 'S';
        break;
      case 0xF7:
        payload[i] = 'V';
        break;
      case 0xF8:
        payload[i] = 'W';
        break;
      case 0xF9:
        payload[i] = 'A';
        break;
      case 0xFA:
        payload[i] = 'B';
        break;
      case 0xFB:
        payload[i] = 'C';
        break;
      case 0xFC:
        payload[i] = 'D';
        break;
      case 0xFD:
        payload[i] = 'E';
        break;
      case 0xFE:
        payload[i] = 'F';
        break;
      case 0xFF:
        payload[i] = ' ';
        break;
    };
  }
}



//*************************************************************************************************************
//*******                              SRAM Memory Measurement Function
//*************************************************************************************************************
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__
 
int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

//*************************************************************************************************************
//*******                              Watchdog Timer Stuff (SAMD51)
//*************************************************************************************************************
// periodCyc goes in increments of 8*2^n up to 16384 (integer n starts at zero)
// fails to initialize if periodCyc doesn't match specific values
bool wdt_enable(int periodCyc)
{  
  switch(periodCyc)   // operates in fall-through mode, returns false if periodCyc doesnt match
  {
    case 8:
      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC8;
    case 16:
      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC16;
    case 32:
      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC32;
    case 64:
      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC64;
    case 128:
      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC128;
    case 256:
      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC256;
    case 512:
      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC512;
    case 1024:
      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC1024;
    case 2048:
      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC2048;
    case 4096:
      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC4096;
    case 8192:
      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC8192;
    case 16384:
      REG_WDT_CONFIG = WDT_CONFIG_PER_CYC16384;
    default:
      return false;
  }

  REG_WDT_CTRLA = WDT_CTRLA_ENABLE;           // enable WDT
  while(WDT->SYNCBUSY.bit.ENABLE)             // wait for bit to sync
  {}  

  return true;
}

void wdt_disable()  // disables the WDT (does not work if WDT->CTRLA.bit.ALWAYSON = 1)
{
  WDT->CTRLA.bit.ENABLE = 0;          // disable the watchdog
  while(WDT->SYNCBUSY.bit.ENABLE)     // wait for the ENABLE bit to syncronize
  {}
}

void wdt_reset()  // high performance WDT clear funtion
{
  if(!WDT->SYNCBUSY.bit.CLEAR)            // if not syncronizing from last CLEAR,
  {                                       //
    REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;  // write clear key to CLEAR register
  }
}


//*************************************************************************************************************
//*******                                           setupXbee
//*************************************************************************************************************
void setupXbee()
{
  xbee.setSPI_clockFreq(XB1_SPI_CLK);
  xbee.setSPI_bitOrder(MSBFIRST);
  xbee.setSPI_mode(SPI_MODE0);
  xbee.setMode(XB1_PRIMARY_MODE);
  xbee.setDestinationAddress(XB1_DEST_ADDR);
  xbee.setCommandInterruptPin(XB1_CMD);
  xbee.setLocalCommand_DIO(XB1_LOC_COMMAND_DIO);
  //xbee.setCutdownInterruptPin();                      // use if needed
  xbee.setLocalCutdown_DIO(XB1_LOC_CUTDOWN_DIO);
  xbee.setDestinationCommand_DIO(XB1_DEST_COMMAND_DIO);
  xbee.setDestinationCutdown_DIO(XB1_DEST_CUTDOWN_DIO);
  xbee.setMaxPayloadSize(XB1_MAX_PAYLOAD_SIZE);
 
}

//*************************************************************************************************************
//*******                                           SETUP
//*************************************************************************************************************
void setup() {
  wdt_disable(); // this prevents infinite loops from occuring with the watchdog reset                            // watchdog
  
    //calculates maximum string memory usage, and if it will fit on the arduino, reserves the memory 
   //this prevents "foaming" of the heap memory due to fragmentation, and helps to prevent stack overflow 
  unsigned int gpsTotalBytes = 350*gpsWriteThresh;
  unsigned int OtherTotalBytes = 200*writeThresh;
  unsigned int totalStringBytes = (400+gpsTotalBytes+OtherTotalBytes);
  if(totalStringBytes > 5000){
    usb.println(F("Error: Strings occupy too much space, please lower your writeThresh and gpsWriteThresh global vars"));
    while(1){}
  }
  gpsStuff.reserve(350);                       //
  GPSflushTotal.reserve(gpsTotalBytes);       // Reserves a block of SRAM for the String variables (to prevent memory fragmentation)
  OtherflushTotal.reserve(OtherTotalBytes);   //
  //gpsTime.reserve(20);                        //
    
  usb.begin(115200);
  usb.println(F("INITIALIZING"));

  
  pinMode(sdPower, OUTPUT);     //
  //pinMode(gpsPower, OUTPUT);    //
  pinMode(imuPower, OUTPUT);    // Sets all power transistor gate pins and reset pins to OUTPUT mode
  pinMode(altPower, OUTPUT);    //
//  pinMode(imuReset, OUTPUT);    //
  pinMode(CUTDOWN, OUTPUT);     //
  
  digitalWrite(CUTDOWN, HIGH);   // ensures that the cutdown is not accidentally triggered during startup (HIGH ON THE CUTDOWN MEANS OFF ON THE CUTDOWN CIRCUIT)
  digitalWrite(sdPower, HIGH);  //
  //digitalWrite(gpsPower, LOW); //
  digitalWrite(imuPower, HIGH); // closes path to ground on all sensors, and sets the imu rst pin to HIGH
  digitalWrite(altPower, HIGH); //
//  digitalWrite(imuReset, HIGH); //

  delay(250);                   // allows time for the pin operations and sensors to come up 

  long xbeeTime = millis();
  while(XB1_NP != 256 && (millis() - xbeeTime <= 5000))
  {
    XB1_NP = xbee.begin();
  }
  // include any other xbee begin cycles here
  if(XB1_NP == 256 /*&& XB2_NP == 256, etc. */)
  {
    setupXbee(); // include all setup features in this fucntion (above)
  }
  if(!SD.begin(SD_CS)){         // initializes the SD card
    usb.println(F("SD Card failed to initialize!")); // if the SD card fails, set sdError true and tell someone
    sdError = true;
    fatalError = true;
  }                                 
  else if(!makeFiles()){        // initializes the SD files
    usb.println(F("File Initialization Failed!")); // if makeFiles fails, set makeFileError to true and tell someone
    makeFileError = true;
    fatalError = true;
  }
  else{
    usb.println(F("makefiles worked!"));
  }                       
  GPS.begin(9600);
  GPS.sendCommand(PMTK_ENABLE_WAAS);            // enables dgps reception (higher accuracy in North America)
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Getting RMA and GGA data (see http://www.gpsinformation.org/dale/nmea.htm)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);    //
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);    //GPS will refresh 5 times a second (doesn't mean you will get data that often)
  GPS.sendCommand(PGCMD_NOANTENNA);             // kills the antenna status update           
  useInterrupt(true);                           // uses interrupts to gather incoming data                          
  
  if(!IMU.begin()){             // initializes IMU
    usb.println(F("BNO055 IMU Initialization Error")); // if IMU fails, set imuError to true and tell someone
    imuError = true;
  }
  else{
    usb.println(F("IMU works!"));
  }
  if(!ALT.begin()){             // initializes the altimeter
    usb.println(F("BME280 Sensor Inilization Error"));
    altError = true;
  }
  else{
    usb.println(F("ALT works!"));
  }
  ads.begin();                          // initializes the ADC for the pitot tube

  PT_INIT(&errorFixPT);         //
  PT_INIT(&fatalFixPT);         //
  PT_INIT(&getGpsPT);           //
  PT_INIT(&getImuPT);           // 
  PT_INIT(&getAltPT);           //
  PT_INIT(&getPitotPT);         //
  PT_INIT(&altErrorPT);         //
  PT_INIT(&writePT);            //
  PT_INIT(&GPSwritePT);         // Initilize all protothreads
  PT_INIT(&getIMUstatusPT);     //
  PT_INIT(&imuErrorPT);         //
  PT_INIT(&gpsErrorPT);         //
  PT_INIT(&cutdownStartPT);     //
  PT_INIT(&cutdownResetPT);     //
  PT_INIT(&cutdownPT);          //
  PT_INIT(&sdFixPT);            //
  PT_INIT(&fileFixPT);          //
  PT_INIT(&gpsFixPT);           //
  PT_INIT(&imuFixPT);           //
  PT_INIT(&altFixPT);           //
  PT_INIT(&stringGrabInitPT);   //
  PT_INIT(&stringGrabGpsPT);    //
  PT_INIT(&stringGrabOtherPT);  //
  PT_INIT(&stringChopPT);       //
  PT_INIT(&xbeeCommandPT);      //
  //PT_INIT(&rudderAnglePT);      // not implemented yet, used with Rudder Class

  usb.println(F("Setup complete, entering loop!"));

  getAltimeter();               // grabs the first elevation value, otherwise there will be two minutes of zeros
  gps_GetAlt("");               // for the altitude, even though the altimeter data is ready immediately
  wdt_enable(16384); 
}

//*************************************************************************************************************
//*******                                           LOOP
//*************************************************************************************************************
void loop() {
  xbeeCommand_sense(&xbeeCommandPT);
  
  stringGrabInit_sense(&stringGrabInitPT);
  
  stringChop_sense(&stringChopPT);
  
  xbee.protothreadLoop();                                                                                            // enable watchdog timer
//  usb.print(F("1,"));
//  usb.println(freeMemory());
  getPitot_sense(&getPitotPT);
//  usb.print(F("2,"));
//  usb.println(freeMemory());
  getIMU_sense(&getImuPT);
//  usb.print(F("3,"));
//  usb.println(freeMemory());
  stringGrabOther_sense(&stringGrabOtherPT);

  
  imuError_sense(&imuErrorPT);
//  usb.print(F("4,"));
//  usb.println(freeMemory());
  getIMUstatus_sense(&getIMUstatusPT);
//  usb.print(F("5,"));
//  usb.println(freeMemory());
  getGPS_sense(&getGpsPT);
//  usb.print(F("6,"));
//  usb.println(freeMemory());
  stringGrabGPS_sense(&stringGrabGpsPT);
  //gpsError_sense(&gpsErrorPT);
//  usb.print(F("7,"));
//  usb.println(freeMemory());
  getALT_sense(&getAltPT);
//  usb.print(F("8,"));
//  usb.println(freeMemory());
  altError_sense(&altErrorPT);
//  usb.print(F("9,"));
//  usb.println(freeMemory());
  fatalFix_sense(&fatalFixPT);
//  usb.print(F("10,"));
//  usb.println(freeMemory());
  errorFix_sense(&errorFixPT);
//  usb.print(F("11,"));
//  usb.println(freeMemory());
  write_sense(&writePT);
//  usb.print(F("12,"));
//  usb.println(freeMemory());
  GPSwrite_sense(&GPSwritePT);
//  usb.print(F("13,"));
//  usb.println(freeMemory());
  cutdownStart_sense(&cutdownStartPT);
//  usb.print(F("14,"));
//  usb.println(freeMemory());
  cutdownReset_sense(&cutdownResetPT);
//  usb.println(F("15,"));
//  usb.println(freeMemory());
  cutdown_sense(&cutdownPT);
  
  wdt_reset();                                                                                                    // watchdog
}

 
