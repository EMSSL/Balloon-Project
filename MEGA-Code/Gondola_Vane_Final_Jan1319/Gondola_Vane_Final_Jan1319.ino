/*
 * BALLOON PROJECT 
 * 
 * Author :          Gerrit Motes 1/12/19
 * Special Contrib:  MakeFiles by Rodney Metoyer in The BeforeTime, in The LongLongAgo...
 * 
 * Equipment integrated into this build:
 *  - Adafruit BNO055 9DOF IMU
 *  - Adafruit BME280 Altimeter
 *  - Adafruit Ultimate GPS breakout
 *  - Adafruit microSD reader/writeer breakout
 *  - Hobbyking small pitotstatic tube with an Adafruit ADS115 ADC
 *  
 *  Previous Rev: Gondola_Vane_Final_Nov17
 *  Changes made in this rev.
 *
 *    - ADD ADS115 library, object, .begin() constructor in setup()  
 *    - ADD replaced pitot analogread with the ADS115 ADC on its AN0 pin
 */

#include <Wire.h>             // wire library (needed for some sensors)
#include <SPI.h>              // SPI digital comms library (needed for SD card)
#include <pt.h>               // protothread library
#include <avr/wdt.h>          // watchdog hardware reset library
#include <SD.h>               // SD library
#include <Adafruit_Sensor.h>  // master sensor library
#include <Adafruit_GPS.h>     // GPS library
#include <Adafruit_BNO055.h>  // IMU library
#include <Adafruit_BME280.h>  // Altimeter library
#include <Adafruit_ADS1015.h> // ADC library

#define usb Serial            // renames Serial as usb
#define gps Serial1           // renames Serial2 as gps

#define CUTDOWN 27            // pin to trigger cutdown

// IMU OBJECT DEFINITION
Adafruit_BNO055 IMU = Adafruit_BNO055(55, BNO055_ADDRESS_B); 
//#define imuReset 34     // used on the imu RST pin for a soft reset
#define imuPower 22     // used on the imu power transistor gate for a hard reset
uint8_t system_status, self_test_results, system_error; // imu sensor error vars

//SPI communication pins, SCK, MISO, and MOSI are common to all SPI sensors, CS pins are unique to each 
// SPI sensor.
#define SPI_SCK 52
#define SPI_MISO 50
#define SPI_MOSI 51
#define SD_CS 44
#define sdPower 24      // SD power transistor gate pin

// Pitot analog pin definition
//#define PITOT A15
Adafruit_ADS1115 ads;         // pitot ADC object
float pitotReadTotal = 0;        // global var for pitot reading
//#define pitotAddr (0x25)         // hex i2c address for pitot transducer
float pitot = 0;                 // global var for averaged pitot value

// altimeter variables
float temp = 0;               //
float pressure = 0;           // global vars for altimeter data
float humid = 0;              //
float altAltitude = 0;        // holds the altitude data supplied by the altimeter
Adafruit_BME280 ALT;          //  ALT I2C
#define altPower 25           //altimeter power transistor pin
#define altitudeThresh 1500   // threshold altitude for cutdown timer initiation
#define gndlvlPress 1023.9   // ground level pressure for calculating elevation  
                              //  (in mbar, even though the pressure output is in different units ¯\_(ツ)_/¯ )


String OtherflushTotal = "";  // string object for sensor data (temporary storage)
String GPSflushTotal = "";    // string object for GPS data (temporary storage)
String gpsStuff = "";         // string object for passing GPS data to various functions
String gpsTime = "";             // used for storage of the last GPS timestamp parsed from an NMEA sentence
File OtherData;               // SD file for sensor data
File GPSdata;                 // SD file for GPS data

// GPS stuff
Adafruit_GPS GPS(&gps); // GPS
#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean);
#define gpsPower 26           // gps power transistor gate pin (used on the ENABLE pin to control GPS power)


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
bool pitotError = false;      // is there a pitot error?

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
#define gps_getTimeThresh 60000         //parses time data from NMEA sentences

long imuStamp = 0;            // timestamp tracking last time IMU sensor data was gathered
long imuStatusStamp = 0;      // timestamp tracking last time IMU status info was gathered
long altStamp = 0;            // timestamp tracking last time data from altimeter was gathered
long pitotStamp = 0;          // timestamp used to track last time the pitot was sampled
long cutdownStamp = 0;        // timestamp used to track time above threshold altitude for cutdown
long errorFix_Stamp = 0;      // timestamp used to track when to call non-fatal error fix checks
long gpsTimeStamp = 0;        // timestamp used to track how long its been since last GPS sentence (for error tracking)
long gps_getTimeStamp = 0;    // timestamp used to track how long its been since the last time parse from a gps sentence
uint8_t writeCount = 0;           // tracks the number of successful IMU pulls before writing to SD
uint8_t GPSwriteCount = 0;        // tracks the number of successful GPS pulls before writing to SD
uint8_t gpsCount = 0;             // tracks the number of successful NMEA sentence pulls before writing to SD
uint8_t fatalResetCount = 0;      // tracks the number of reset attempts, used to initiate watchdog reset
uint8_t imuResetCount = 0;        // tracks the number of IMU resets, used to tie recurring errors to watchdog
uint8_t altResetCount = 0;        // tracks the number of IMU resets, used to tie recurring errors to watchdog
uint8_t pitotCount = 0;           // tracks number of times the pitot was sampled before averaging 
float altitude = 0;           // holds the official altitude info (in case the GPS or parsing function breaks, use altimeter as backup)
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
   // usb.println(F("cutdownStart"));
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
    usb.println(F("CUTDOWN INITIATED! FUUUUUUUUU!"));
    digitalWrite(CUTDOWN, LOW);
    delay(30000);
    digitalWrite(CUTDOWN, HIGH);
    cutdown = true;
    cutdownStamp = 0;
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

//************************************************************************************************************
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
//*******                                           getGPS
//*************************************************************************************************************
void getGPS(){
 // usb.println(F("entering getGPS"));
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

  pitot = (float)pitotReadTotal/(float)pitotCount;

  long gps_timeAddition =  millis() - gps_getTimeStamp; // calculates the duration since last GPS time fix (in ms)

  OtherflushTotal = OtherflushTotal + F("\n") + gpsTime + F(" + ") + gps_timeAddition + F(" ms, ") + altitude + F(" : ");
  OtherflushTotal = OtherflushTotal + BN_acc.x() + F(",") + BN_acc.y() + F(",") + BN_acc.z();
  OtherflushTotal = OtherflushTotal + F(",") + BN_gyro.x() + F(",") + BN_gyro.y() + F(",") + BN_gyro.z();
  OtherflushTotal = OtherflushTotal + F(",") + BN_mag.x() + F(",") + BN_mag.y() + F(",") + BN_mag.z();
  OtherflushTotal = OtherflushTotal + F(",") + BN_eul.x() + F(",") + BN_eul.y() + F(",") + BN_eul.z();
  OtherflushTotal = OtherflushTotal + F(",") + BN_grav.x() + F(",") + BN_grav.y() + F(",") + BN_grav.z();
  OtherflushTotal = OtherflushTotal + F(",") + temp + F(",") + pressure + F(",") + humid + F(",") + altAltitude + "," + pitot + F("%%");
  
  
  imuErrChk1 = (long)(BN_acc.x()+BN_acc.y()+BN_acc.z()+BN_gyro.x()+BN_gyro.y()+BN_gyro.z()+BN_mag.x()+BN_mag.y()+BN_mag.z());
  imuErrChk2 = (long)(BN_mag.x()-BN_mag.y()+BN_mag.z()+0.06);

  writeCount++;
  pitotCount = 0;
  pitotReadTotal = 0;
 // usb.println(F("leaving getIMU"));
}



//*************************************************************************************************************
//*******                                           getIMUstatus
//*************************************************************************************************************
void getIMUstatus(){
 // usb.println(F("entering getIMUstatus"));
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
 // usb.println(F("entering GPSwrite2SD"));
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
 // usb.println(F("entering gps_GetTime"));
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
 // usb.println(F("entering gps_GetAlt"));
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
  usb.println(altitude);
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
  //usb.println(F("entering fileFix"));
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
 // usb.println(F("entering gpsFix"));
 // usb.println(F("restarting gps"));
  digitalWrite(gpsPower, HIGH);
  delay(250);
  digitalWrite(gpsPower, LOW);
  delay(250);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Getting RMA and GGA data (see http://www.gpsinformation.org/dale/nmea.htm)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);    //
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);    //GPS will refresh 5 times a second (doesn't mean you will get data that often)
  GPS.sendCommand(PGCMD_NOANTENNA);             // kills the antenna status update           
  useInterrupt(true);

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
 // usb.println(F("entering altFix"));
  digitalWrite(altPower, LOW);  // resets the altimeter via the transistor gate
  delay(250);
  digitalWrite(altPower, HIGH);
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
  
  usb.println(F("made it through filename initialization stuffs"));                                          
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
  
  usb.println(F("made it through gpsfile suffix search"));                                          
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
  GPSdata = SD.open(gpsfile, FILE_WRITE);   // write a new file for the gps file on the SD card
  if(!GPSdata)                              // if the file failed to be created
  {                                         // 
    usb.print(F("Failed to create "));         // display failed to create on the serial monitor
    usb.println(gpsfile);                   // followed by the filename
    rtn = false;                            // set the boolean flag rtn to false
  }                                         // end
  else{
    usb.println(F("made it through gps file creation")); 
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
    usb.println(F("made it through otherdata file creation")); 
  }
  //usb.println(F("returning rtn"));
  usb.println(rtn); 
  usb.println(F("leaving makeFiles()"));                                        
  return (int)rtn; 
}

//*************************************************************************************************************
//*******                                           CRITICAL GPS-RELATED FUNCTIONS
//*************************************************************************************************************
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) UDR0 = c;  
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
//*******                                           SETUP
//*************************************************************************************************************
void setup() {
  wdt_disable(); // this prevents infinite loops from occuring with the watchdog reset
  
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
  pinMode(gpsPower, OUTPUT);    //
  pinMode(imuPower, OUTPUT);    // Sets all power transistor gate pins and reset pins to OUTPUT mode
  pinMode(altPower, OUTPUT);    //
  //pinMode(imuReset, OUTPUT);    //
  pinMode(CUTDOWN, OUTPUT);     //
  
  digitalWrite(CUTDOWN, HIGH);   // ensures that the cutdown is not accidentally triggered during startup
  digitalWrite(sdPower, HIGH);  //
  digitalWrite(gpsPower, LOW); //
  digitalWrite(imuPower, HIGH); // closes path to ground on all sensors, and sets the imu rst pin to HIGH
  digitalWrite(altPower, HIGH); //
  //digitalWrite(imuReset, HIGH); //

  delay(250);                   // allows time for the pin operations and sensors to come up 

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
  if(!ALT.begin()){             // initilizes the altimeter
    usb.println(F("BME280 Sensor Inilization Error"));
    altError = true;
  }
  else{
    usb.println(F("ALT works!"));
  }
  ads.begin();
  

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

  usb.println(F("Setup complete, entering loop!"));

  getAltimeter();               // grabs the first elevation value, otherwise there will be two minutes of zeros
  gps_GetAlt("");               // for the altitude, even though the altimeter data is ready immediately
}

//*************************************************************************************************************
//*******                                           LOOP
//*************************************************************************************************************
void loop() {
  wdt_enable(WDTO_8S);                  // enable watchdog timer
//  usb.print(F("1,"));
//  usb.println(freeMemory());
  getPitot_sense(&getPitotPT);
//  usb.print(F("2,"));
//  usb.println(freeMemory());
  getIMU_sense(&getImuPT);
//  usb.print(F("3,"));
//  usb.println(freeMemory());
  imuError_sense(&imuErrorPT);
//  usb.print(F("4,"));
//  usb.println(freeMemory());
  getIMUstatus_sense(&getIMUstatusPT);
//  usb.print(F("5,"));
//  usb.println(freeMemory());
  getGPS_sense(&getGpsPT);
//  usb.print(F("6,"));
//  usb.println(freeMemory());
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
//  cutdownStart_sense(&cutdownStartPT);
//  usb.print(F("14,"));
//  usb.println(freeMemory());
//  cutdownReset_sense(&cutdownResetPT);
//  usb.println(F("15,"));
//  usb.println(freeMemory());
//  cutdown_sense(&cutdownPT);
  wdt_reset();
  
 
}
