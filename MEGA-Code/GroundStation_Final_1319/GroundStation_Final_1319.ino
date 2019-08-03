/*
 * GROUND STATION SKETCH FOR WINTER 2018 BALLOON TEST (SAURABH, CHRIS, GERRIT)
 * 
 * ORIGINAL sdl_weatHer_80422_test CODE PROVIDED BY C. YODER, WITH MICRO-SD 
 *  FUNCTIONALITY ADDED BY G. MOTES. GroundStationNov18.ino INCORPORATES
 *  IMU AND GPS FUNCTIONALITY, UTILIZING PROTOTHREADS FOR WORKFLOW CONTROL
 *  
 *    REV 0 - ORIGINAL ATTEMPT BY G. MOTES, WEITER INS VERDERBEN
 * 
 * PINOUTS FOR ARDUINO MEGA 2560r3
 *    -GROUND STATION GREEN   ->
 *    -GROUND STATION GREEN   ->
 *    -GROUND STATION YELLOW  ->  gnd
 *    -GROUND STATION RED     ->
 *    -GROUND STATION BLACK   ->  gnd
 *    
 * **GENERAL NOTES**
 *    -"GROUND STATION" IN THE CONTEXT MEANS THE ANEMOMETER/WIND VANE COMBO
 *      IN THE CONTEXT OF THIS HEADER.
 *    -THE FULL GROUND STATION CONSISTS OF THE ANEMOMETER/WIND VANE, AN 
 *      IMU (BNO055 FROM ADAFRUIT), AN ADAFRUIT ULTIMATE GPS BREAKOUT, AND
 *      AN ADAFRUIT MICRO-SD BREAKOUT.
 *    -THE WIND VANE GIVES RELATIVE ANGLE TO A REFERENCE DIRECTION, RATHER 
 *      THAN ABSOLUTE DIRECTION. THE IMU PROVIDES THE ABOSOLUTE ORIENTATION, 
 *      THUS IT IS IMPERATIVE THAT THE ORIENTATION OF THE MAGNETOMETER AXES
 *      ON THE IMU BE KNOWN, AND WHICH AXES ARE ALIGNED WITH THE 0 DEG 
 *      DIRECTION ON THE WIND VANE.
 */

// ***********  PROGRAM OPTIONS  ***************
bool serial_output = true;    // set this to 'true' if you want output to serial monitor

// libraries required
#include <Wire.h>               // wire library to communicate
#include <SPI.h>              // SPI digital comms library (needed for SD card)
#include <pt.h>               // protothread library
#include <SD.h>               // SD library
#include <Adafruit_Sensor.h>  // master sensor library (just in case, not sure if needed)
#include <Adafruit_GPS.h>     // GPS library
#include <Adafruit_BNO055.h>  // IMU library
#include "SDL_Weather_80422.h"  // custom library from vendor
#include <Time.h>               // time library to get times

#define usb Serial  // So I avoid my crappy spelling mistakes
#define gps Serial2           // renames Serial2 as gps

//SPI communication pins, SCK, MISO, and MOSI are common to all SPI sensors, CS pins are unique to each 
// SPI sensor.
#define SPI_SCK 52
#define SPI_MISO 50
#define SPI_MOSI 51
#define SD_CS 44

#define SCREAMERPIN 36        //Pin used to signal the voltage screamer

// pin defines for anemometer/wind vane
#define pinLED     13   // LED connected to digital pin 13
#define pinAnem    18  // Anenometer connected to pin 18 - Int 5 - Mega   / Uno pin 2
#define pinRain    8  // Anenometer connected to pin 2 - Int 0 - Mega   / Uno Pin 3 
#define intAnem    5  // int 0 (check for Uno)
#define intRain    0  // int 1
// IN ADDITION, A0 (analog in pin 0) must be connected!!!

// IMU OBJECT DEFINITION
Adafruit_BNO055 IMU = Adafruit_BNO055(55, BNO055_ADDRESS_A); 
uint8_t system_status, self_test_results, system_error; // imu sensor error vars

// ITEMS USED FOR DATA CAPTURE AND MANIPULATION
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

// PROTOTHREAD OBJECTS, USED TO TRACK CONDITIONAL STATES OF PROTOTHREADS
static struct pt getGroundStationPT;
static struct pt getGpsPT;
static struct pt getImuPT;
static struct pt GPSwritePT;
static struct pt getIMUstatusPT;
static struct pt isErrorPT;
static struct pt writePT;
static struct pt imuErrorPT;

#define imuThresh 20                    // time (ms) between each imu reading
#define groundStationThresh 20        // time (ms) between each Ground Station read
#define writeThresh 2                   // number of successful data pulls before writing to SD
#define gpsWriteThresh 5                // number of successful gps pulls before writing to SD
#define getIMUstatusThresh 180000       // ms threshold for checking IMU system status (since it takes so long to run)
#define errorVal1 (long)0.00            //
#define errorVal2 (long)-0.01           // standard error vals for the IMU
#define errorVal3 (long)-0.06           //
#define gps_getTimeThresh 60000         //parses time data from NMEA sentences

long imuStamp = 0;            // timestamp tracking last time IMU sensor data was gathered
long imuStatusStamp = 0;      // timestamp tracking last time IMU status info was gathered
long groundStationStamp = 0;  // timestamp tracking last time ground station data was pulled
long gpsTimeStamp = 0;        // timestamp used to track how long its been since last GPS sentence (for error tracking)
long gps_getTimeStamp = 0;    // timestamp used to track how long its been since the last time parse from a gps sentence
uint8_t writeCount = 0;           // tracks the number of successful IMU pulls before writing to SD
uint8_t GPSwriteCount = 0;        // tracks the number of successful GPS pulls before writing to SD
uint8_t gpsCount = 0;             // tracks the number of successful NMEA sentence pulls before writing to SD
long imuErrChk1 = 0;
long imuErrChk2 = 0;
bool error = false;
float altitude = 0;
long temp = 0;

// initialize SDL_Weather_80422 library
SDL_Weather_80422 weatherStation(pinAnem, pinRain, intAnem, intRain, A0, SDL_MODE_INTERNAL_AD);

// DUNNO
uint8_t i;

// DEFINE variables as float for math
float currentWindSpeed;                               // wind speed
float currentWindGust;                                // wind gust
float currentWindDirection;                           // wind direction
int count = 0;  // not used...maybe used in future for driving down freq. of SD write


//*************************************************************************************************************
//*******                                   PROTOTHREAD getGroundStation_sense
//*************************************************************************************************************
static int getGroundStation_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, millis() - groundStationStamp >= groundStationThresh);
    getGroundStation();
    groundStationStamp = millis();
  }
  PT_END(pt);
}


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
//*******                                   PROTOTHREAD imuError_sense
//*************************************************************************************************************
static int imuError_sense(struct pt *pt){
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, (((imuErrChk1 <= 0.01) && (imuErrChk1 >= -0.01)) || ((imuErrChk2 <= 0.01) && (imuErrChk2 >= -0.01))));
    error = true;
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
//*******                                   PROTOTHREAD error_sense
//*************************************************************************************************************
static int error_sense(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, error); // if there is an error
    errorHandling();          // go to error handling functoin
    while(1){}                // then freeze and wait for watchdog
  }
  PT_END(pt);
}


//*************************************************************************************************************
//*******                                           getGroundStation
//*************************************************************************************************************
void getGroundStation(){
  currentWindSpeed = 0.44704*weatherStation.current_wind_speed()/1.6;     // get wind speed and convert to meters/second
  currentWindGust = 0.44704*weatherStation.get_wind_gust()/1.6;           // get wind gust and convert to meters/second
  currentWindDirection = weatherStation.current_wind_direction();
  usb.println(currentWindSpeed);
  usb.println(currentWindGust);
  usb.println(currentWindDirection);
}


//*************************************************************************************************************
//*******                                           getGPS
//*************************************************************************************************************
void getGPS(){
  usb.println(F("entering getGPS"));
  gpsStuff = GPS.lastNMEA();
  usb.println(gpsStuff);
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
  usb.println(F("entering getIMU"));
  imu::Vector<3> BN_acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // creates vectors for each event
  imu::Vector<3> BN_gyro = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> BN_mag = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> BN_eul = IMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> BN_grav = IMU.getVector(Adafruit_BNO055::VECTOR_GRAVITY);


  long gps_timeAddition =  millis() - gps_getTimeStamp; // calculates the duration since last GPS time fix (in ms)

  OtherflushTotal = OtherflushTotal + F("\n") + gpsTime + F(" + ") + gps_timeAddition + F(" ms, ") + altitude + F(" : ");
  OtherflushTotal = OtherflushTotal + BN_acc.x() + F(",") + BN_acc.y() + F(",") + BN_acc.z();
  OtherflushTotal = OtherflushTotal + F(",") + BN_gyro.x() + F(",") + BN_gyro.y() + F(",") + BN_gyro.z();
  OtherflushTotal = OtherflushTotal + F(",") + BN_mag.x() + F(",") + BN_mag.y() + F(",") + BN_mag.z();
  OtherflushTotal = OtherflushTotal + F(",") + BN_eul.x() + F(",") + BN_eul.y() + F(",") + BN_eul.z();
  OtherflushTotal = OtherflushTotal + F(",") + BN_grav.x() + F(",") + BN_grav.y() + F(",") + BN_grav.z();
  OtherflushTotal = OtherflushTotal + F(",") + IMU.getTemp() + F(",") + currentWindSpeed + F(",") + currentWindGust;
  OtherflushTotal = OtherflushTotal + F(",") + currentWindDirection + F("%%");
  
  imuErrChk1 = (long)(BN_acc.x()+BN_acc.y()+BN_acc.z()+BN_gyro.x()+BN_gyro.y()+BN_gyro.z()+BN_mag.x()+BN_mag.y()+BN_mag.z());
  imuErrChk2 = (long)(BN_mag.x()-BN_mag.y()+BN_mag.z()+0.06);

  writeCount++;
  usb.println(F("leaving getIMU"));
}


//*************************************************************************************************************
//*******                                           getIMUstatus
//*************************************************************************************************************
void getIMUstatus(){
  usb.println(F("entering getIMUstatus"));
  IMU.getSystemStatus(&system_status, &self_test_results, &system_error); // gets system status
  if(system_error){
    error = true;
  } 
}


//*************************************************************************************************************
//*******                                           write2SD
//*************************************************************************************************************
void write2SD(){
  usb.println(F("entering write2SD"));
  OtherData.print(OtherflushTotal);
  OtherData.flush();
  
  if(serial_output){
    usb.print(F("Otherflush : "));
    usb.println(OtherflushTotal);
  }
  writeCount = 0;
}


//*************************************************************************************************************
//*******                                           GPSwrite2SD
//*************************************************************************************************************
void GPSwrite2SD(){
  usb.println(F("entering GPSwrite2SD"));
  GPSdata.print(GPSflushTotal);
  GPSdata.flush();
  usb.println(F("GPSflush"));
  GPSwriteCount = 0;
}


//*************************************************************************************************************
//*******                                           gps_GetTime
//*************************************************************************************************************
bool gps_GetTime(String NMEA_Sentence){
  usb.println(F("entering gps_GetTime"));
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
  usb.println(F("entering gps_GetAlt"));
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
  }
}


//*************************************************************************************************************
//*******                                           errorHandling
//*************************************************************************************************************
void errorHandling(){
 usb.println(F("OH FUCK AH ERROR WE ALL GONNA DIE!"));
 digitalWrite(SCREAMERPIN,LOW);                         // set off screamer
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
//*******                                           MakeFiles
//*************************************************************************************************************
int makeFiles(){
  usb.println(F("entering makeFiles()"));// 
  bool rtn = true;                          // set a boolean flag of rtn to be true

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
    usb.print(F("Failed to create "));         // display failed to create on the usb monitor
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
    usb.print(F("Failed to create "));             // display failed to create on the usb monitor
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

  //calculates maximum string memory usage, and if it will fit on the arduino, reserves the memory 
  //this prevents "foaming" of the heap memory due to fragmentation, and helps to prevent stack overflow 
  unsigned int gpsTotalBytes = 350*gpsWriteThresh;
  unsigned int OtherTotalBytes = 350*writeThresh;
  unsigned int totalStringBytes = (400+gpsTotalBytes+OtherTotalBytes);
  if(totalStringBytes > 5000){
    usb.println(F("Error: Strings occupy too much space, please lower your writeThresh and gpsWriteThresh global vars"));
    while(1){}
  }
  gpsStuff.reserve(350);                       //
  GPSflushTotal.reserve(gpsTotalBytes);       // Reserves a block of SRAM for the String variables (to prevent memory fragmentation)
  OtherflushTotal.reserve(OtherTotalBytes);   //

  pinMode(SCREAMERPIN,OUTPUT);
  digitalWrite(SCREAMERPIN,HIGH);


  // INITIALIZE SERIAL OUTPUT STREAM AND OUTPUT EDUCATIONAL GIBBERISH  
  usb.begin(115200);
  usb.println(F("***GROUND STATION****"));
  usb.println(F("Featuring SDL Weather 80422 Anemometer/Wind Vane,"));
  usb.println(F("Adafruit BNO055 10-DOF IMU,"));
  usb.println(F("Adafruit Ultimate GPS Breakout,"));
  usb.println(F("Adafruit microSD breakout!"));
  usb.print(F("****INITIALIZING"));
  i = 0;
  while(i<5){
    usb.print(F(" . "));
    delay(1000);
    i++;
  }
  usb.println(F(" . "));

  // ANEMOMETER AND WIND VANE STURRRFFFF
  // wind speed is averaged with this time (moving average filter)
  weatherStation.setWindMode(SDL_MODE_SAMPLE, 0.5);   


  // INITIALIZE SENSORS/BREAKOUTS AND ENSURE THEY WORK
  if(!SD.begin(SD_CS)){         // initializes the SD card
    usb.println(F("SD Card failed to initialize!")); // if the SD card fails, set sdError true and tell someone
    error = true;
  }                                 
  else if(!makeFiles()){        // initializes the SD files
    usb.println(F("File Initialization Failed!")); // if makeFiles fails, set makeFileError to true and tell someone
    error = true;
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
    error = true;
  }
  else{
    usb.println(F("IMU works!"));
  }

  PT_INIT(&getGpsPT);           //
  PT_INIT(&getImuPT);           //
  PT_INIT(&GPSwritePT);         // Initilize all protothreads
  PT_INIT(&getIMUstatusPT);     //
  PT_INIT(&isErrorPT);          //
  PT_INIT(&getGroundStationPT); //
  PT_INIT(&writePT);            //
  PT_INIT(&imuErrorPT)          //

  if(error){
    usb.println(F("OH NO, SOMETHING BROKE :'("));
  }
  else{
    usb.println(F("HOORAY, EVERYTHING WORKS! MOVING TO LOOP()"));
  }
}





//*************************************************************************************************************
//*******                                           LOOP
//*************************************************************************************************************
void loop() {
  //error_sense(&isErrorPT);
  getGroundStation_sense(&getGroundStationPT);
  getGPS_sense(&getGpsPT);
  getIMU_sense(&getImuPT);
  getIMUstatus_sense(&getIMUstatusPT);
  imuError_sense(&imuErrorPT);
  write_sense(&writePT);
  GPSwrite_sense(&GPSwritePT);
}
