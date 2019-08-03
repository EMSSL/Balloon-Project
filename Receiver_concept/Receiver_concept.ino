//XBEE DESTINATION ADDRESS1: 
//  UNCOMMENT WHICH XBEE DESTINATION ADDRESS YOU WANT TO COMMUNICATE WITH, UNCOMMENT THE REMAINDER
// 
//#define XB_DEST_ADDR_1 (0x0013A200417E38C1)  // DESTINATION XBEE 64-BIT ADDRESS (A200K, DEFAULT GONDOLA)
#define XB_DEST_ADDR_1 (0x0013A200417E3816) // DESTINATION XBEE 64-BIT ADDRESS (B200K, DEFAULT SAIL)
//#define XB_DEST_ADDR_1 (0x0013A200417E3829)  // DESTINATION XBEE 64-BIT ADDRESS (C200K, DEFAULT GONDOLA RECEIVER)
//#define XB_DEST_ADDR_1 (0x0013A200417E381A) // DESTINATION XBEE 64-BIT ADDRESS (D200K, DEFAULT SAIL RECEIVER)
//#define XB_DEST_ADDR_1 (0x0013A200417E35F7) // DESTINATION XBEE 64-BIT ADDRESS (E200K, DEFAULT BACKUP UNIT)


//XBEE DESTINATION ADDRESS2: RECEIVER USE ONLY!
//  UNCOMMENT WHICH XBEE DESTINATION ADDRESS YOU WANT TO COMMUNICATE WITH, UNCOMMENT THE REMAINDER 
//
//#define XB_DEST_ADDR_2 (0x0013A200417E38C1)  // DESTINATION XBEE 64-BIT ADDRESS (A200K, DEFAULT GONDOLA)
//#define XB_DEST_ADDR_2 (0x0013A200417E3816) // DESTINATION XBEE 64-BIT ADDRESS (B200K, DEFAULT SAIL)
//#define XB_DEST_ADDR_2 (0x0013A200417E3829)  // DESTINATION XBEE 64-BIT ADDRESS (C200K, DEFAULT GONDOLA RECEIVER)
//#define XB_DEST_ADDR_2 (0x0013A200417E381A) // DESTINATION XBEE 64-BIT ADDRESS (D200K, DEFAULT SAIL RECEIVER)
//#define XB_DEST_ADDR_2 (0x0013A200417E35F7) // DESTINATION XBEE 64-BIT ADDRESS (E200K, DEFAULT BACKUP UNIT)


//XBEE UNIT NAMES : RECEIVER USE ONLY!
//  UNCOMMENT THE NAME OF EACH UNIT THE CORRESPONDING RECEIVE XBEE WILL BE RECEIVING FROM
//  COMMENT OUT THE REST, USED FOR FORMATTING THE HUD. 
//
//    XBEE1 TRANSMITTER NAME
//      #define XBEE1_NAME "GONDOLA"
      #define XBEE1_NAME "SAIL"
//      #define XBEE1_NAME "OTHER"    // CHANGE OTHER TO YOUR PREFERRED NAME (LIMIT 7 CHARACTERS)
//
//    XBEE2 TRANSMITTER NAME
//      #define XBEE2_NAME "GONDOLA"
//      #define XBEE2_NAME "SAIL"
//      #define XBEE2_NAME "OTHER"    // CHANGE OTHER TO YOUR PREFERRED NAME (LIMIT 7 CHARACTERS)
//
//
// FILE NAMES FOR THE SD CARD : UNCOMMENT EACH ONE AND TYPE IN A FILENAME
//
 #define GPS_FILENAME "GPS05.TXT"
 #define OTHER_FILENAME "ODAT05.TXT"
 #define LOG_FILENAME "LOG05.TXT"
//
//
// COLUMN WIDTH OF EACH RECEIVER'S PRINTOUTS
  #define COLUMN_WIDTH 42
//
//*************************************************************************************************************
//*******                                 END : CONFIGURATION SECTIONS
//*************************************************************************************************************

#include <Wire.h>             // wire library (needed for some sensors)
#include <SPI.h>              // SPI digital comms library (needed for SD card)
#include <pt.h>               // protothread library
#include <SD.h>               // SD library
#include <Adafruit_Sensor.h>  // master sensor library
#include <Adafruit_GPS.h>     // GPS library
#include <Adafruit_BNO055.h>  // IMU library
#include <Adafruit_BME280.h>  // Altimeter library
#include <Adafruit_ADS1015.h> // ADC library
#include "xbeeAPI.h"          // xbee library (900HP S3B models in API SPI mode)
#include <Adafruit_DotStar.h> // controls LED on ItsyBitsyM4 MCU

#define usb Serial            // renames Serial as usb
#define gps Serial1           // renames Serial2 as gps

// IMU POWER PIN ASSIGNMENT
#define imuPower 12

// ALTIMETER POWER PIN ASSIGNMENT
#define altPower 11 

//SPI communication pins, SCK, MISO, and MOSI are common to all SPI sensors, CS pins are unique to each 
// SPI sensor.
#define SD_CS 2
#define sdPower 3      // SD power transistor gate pin

// VARIOUS STRUCT DEFINITIONS
typedef struct{
  String x;
  String y;
  String z;
}ThreeAxis;

typedef struct{
  String x;
  String y;
  String z;
}ThreeSpace;

typedef struct{
  String Name;
  String TempflushTotal;
  String OtherflushTotal;
  String GPSflushTotal;
  String Time;
  String lat;
  String lon;
  String elev;
  String fixQual;
  ThreeAxis acc;
  ThreeAxis gyro;
  ThreeAxis mag;
  ThreeAxis eul;
  String pressure;
  String temperature;
  String humidity;
  String pitot;
}thing;

//Xbee OBJECT DEFINITION
#define XB_SS_1 (5)                          // M4 PIN FOR XBEE SPI SLAVE SELECT
#define XB_ATTN_1 (7)                        // M4 PIN FOR XBEE ATTN 
#define XB_CMD_1 (9)                         // M4 PIN NUMBER FOR COMMAND SIGNAL
#define XB_LOC_COMMAND_DIO_1 ('0')           // DIO NUMBER FOR LOCAL XBEE COMMAND PIN
#define XB_LOC_CUTDOWN_DIO_1 ('5')           // DIO NUMBER FOR LOCAL XBEE CUTDOWN PIN
#define XB_DEST_COMMAND_DIO_1 ('0')          // DIO NUMBER FOR DESTINATION COMMAND PIN
#define XB_DEST_CUTDOWN_DIO_1 ('5')          // DIO NUMBER FOR DESTINATION CUTDOWN 
#define XB_SPI_CLK_1 (3000000)               // SPI BUS FREQUENCY FOR XBEE DATA
#define XB_PRIMARY_MODE_1 (2)                // 0 = FAST, 1 = ACCURATE, 2 = RECEIVER
#define XB_MAX_PAYLOAD_SIZE_1 256                 // cannot exceed NP (256)
uint16_t XB_NP_1;                            // holds the value for AT property NP (max payload size)

// GPS stuff
Adafruit_GPS GPS(&gps); // GPS
//#define GPSECHO  false
boolean usingInterrupt;
void useInterrupt(boolean);

// LOGFILE GLOBALS
uint8_t LogPriority_sd;
uint8_t LogPriority_sm;
uint16_t LogbufferSize;
String LogString;
bool LogExclusive;

// INITIALIZE THE XBEE OBJECT AND ASSOCIATED STRUCTS
Xbee xbee_1 = Xbee(XB_SS_1, XB_ATTN_1);         // CONSTRUCTOR FOR XBEE OBJECT
thing receiver_1;                                // DECLARATION OF THE THING
thing System;                                    // DECLARATION OF A SYSTEM THING FOR LOGGING GENERAL SYSTEM STATS AND ERRORS

// XBEE GLOBALS
uint8_t xbeeTempbuff_1[300];
uint8_t xbeeDecomp_1[700];
uint16_t xbeeTempInd_1;
uint16_t xbeeDecompInd_1;
uint16_t xbeeDecodeInd_1; 

// INITIALIZE THE GLOBAL OBJECTS AND STRUCTS
File OtherData;                 // SD file for sensor data
File GPSdata;                   // SD file for GPS data
File logFile;                   // SD file for log data

// PROTOTHREAD OBJECTS
static struct pt xbeeReceivePT_1;
static struct pt xbeeDecompPT_1;
static struct pt xbeeDecodePT_1;
static struct pt classifyPT_1;
static struct pt parseGPSPT_1;
static struct pt parseOtherPT_1;
static struct pt updateHUDPT_1;
static struct pt printHUDPT;

//PROTOTHREAD-RELATED STUFF
bool classifyNow_1 = false;
bool nameSpaced_1 = false;
bool updateHUDOther_1 = false;
bool updateHUDGPS_1 = false;
bool rePrint = false;

//HUD LINE STRINGS
String hudLines_1[17];

//*************************************************************************************************************
//*******                               PROTOTHREAD - xbeeReceive_sense_1
//*************************************************************************************************************
static int xbeeReceive_sense_1(struct pt *pt)
{
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, xbee_1.data_received_length > 0);
    xbeeTempInd_1 = xbee_1.data_received_length;
    for(int i = 0; i < 300; i++)
    {
      xbeeTempbuff_1[i] = xbee_1.data_received[i]; 
    }
    xbee_1.clearBuffers();
    LogPrintln(&receiver_1,3,F("XBEERECEIVE_SENSE_1 : GOT SOMETHING"));

  PT_END(pt);
}

//*************************************************************************************************************
//*******                               PROTOTHREAD - xbeeDecomp_sense_1
//*************************************************************************************************************
static int xbeeDecomp_sense_1(struct pt *pt)
{
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, xbeeTempInd_1);
    xbeeDecompInd_1 = decompress(xbeeTempbuff_1, xbeeDecomp_1, xbeeTempInd_1);
    xbeeTempInd_1 = 0;
    for(int i = 0; i < xbeeDecompInd_1; i++)
    {
      Serial.print(xbeeDecomp_1[i]);
    }
    Serial.println(F(""));
    LogPrintln(&receiver_1,3,F("XBEEDECOMP_SENSE_1 : CALLED"));
  PT_END(pt);
}

//*************************************************************************************************************
//*******                               PROTOTHREAD - xbeeDecode_sense_1
//*************************************************************************************************************
static int xbeeDecode_sense_1(struct pt *pt)
{
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, xbeeDecompInd_1);
    matchCaseDecompress(xbeeDecomp_1, xbeeDecompInd_1);
    receiver_1.TempflushTotal = "";
    for(int i = 0; i < xbeeDecompInd_1; i++)
    {
      receiver_1.TempflushTotal += (char)xbeeDecomp_1[i];
    }
    Serial.println(receiver_1.TempflushTotal);
    classifyNow_1 = true;
    xbeeDecompInd_1 = 0;
    GPSdata.print(receiver_1.Name);
    GPSdata.print(F(" : "));
    GPSdata.print(receiver_1.TempflushTotal);
    GPSdata.flush();
    LogPrintln(&receiver_1,3,F("XBEEDECODE_SENSE_1 : CALLED"));
  PT_END(pt);
}

//*************************************************************************************************************
//*******                               PROTOTHREAD - classifyStuff_sense_1
//*************************************************************************************************************
static int classifyStuff_sense_1(struct pt *pt)
{
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, classifyNow_1);
    LogPrintln(&receiver_1,3,F("CLASSIFYSTUFF_SENSE_1 : CALLED"));
    
    if((receiver_1.TempflushTotal.indexOf("GPRMC") >= 0) || (receiver_1.TempflushTotal.indexOf("GPGGA") >= 0)){
      int gpsStart = receiver_1.TempflushTotal.indexOf("$G");
      int otherStart = receiver_1.TempflushTotal.indexOf("MS,");
      if(otherStart < 0){
        receiver_1.GPSflushTotal = receiver_1.TempflushTotal.substring(gpsStart);
        GPSdata.print(receiver_1.Name);
        GPSdata.print(F(" : "));
        GPSdata.print(receiver_1.GPSflushTotal);
        GPSdata.flush();
        LogPrintln(&receiver_1,3,F("CLASSIFYSTUFF_SENSE_1 : CLASSIFIED GPS ONLY"));
        Serial.println(receiver_1.GPSflushTotal);
        Serial.println(receiver_1.OtherflushTotal);
      }
      else if(otherStart < gpsStart){
        receiver_1.GPSflushTotal = receiver_1.TempflushTotal.substring(gpsStart);
        receiver_1.OtherflushTotal = receiver_1.TempflushTotal.substring(otherStart-16,gpsStart);
      
        GPSdata.print(receiver_1.Name);
        GPSdata.print(F(" : "));
        GPSdata.print(receiver_1.GPSflushTotal);
        GPSdata.flush();
  
        OtherData.print(receiver_1.Name);
        OtherData.print(F(" : "));
        OtherData.print(receiver_1.OtherflushTotal);
        OtherData.flush();

        LogPrintln(&receiver_1,3,F("CLASSIFYSTUFF_SENSE_1 : CLASSIFIED GPS OTHER OTHER"));
        Serial.println(receiver_1.GPSflushTotal);
        Serial.println(receiver_1.OtherflushTotal);
      }
      else{
        receiver_1.GPSflushTotal = receiver_1.TempflushTotal.substring(gpsStart,otherStart-16);
        receiver_1.OtherflushTotal = receiver_1.TempflushTotal.substring(otherStart-16);
      
        GPSdata.print(receiver_1.Name);
        GPSdata.print(F(" : "));
        GPSdata.print(receiver_1.GPSflushTotal);
        GPSdata.flush();
  
        OtherData.print(receiver_1.Name);
        OtherData.print(F(" : "));
        OtherData.print(receiver_1.OtherflushTotal);
        OtherData.flush();

        LogPrintln(&receiver_1,3,F("CLASSIFYSTUFF_SENSE_1 : CLASSIFIED GPS OTHER OTHER"));
        Serial.println(receiver_1.GPSflushTotal);
        Serial.println(receiver_1.OtherflushTotal);
      }
      
      LogPrintln(&receiver_1,3,F("CLASSIFYSTUFF_SENSE_1 : CLASSIFIED GPS OTHER OTHER"));
      Serial.println(receiver_1.GPSflushTotal);
      Serial.println(receiver_1.OtherflushTotal);
    }
    else{
      int otherStart = receiver_1.TempflushTotal.indexOf("MS,");
      if(otherStart >= 0){
        receiver_1.OtherflushTotal = receiver_1.TempflushTotal.substring(otherStart-16);
        OtherData.print(receiver_1.Name);
        OtherData.print(F(" : "));
        OtherData.print(receiver_1.OtherflushTotal);
        OtherData.flush();
        LogPrintln(&receiver_1,3,F("CLASSIFYSTUFF_SENSE_1 : CLASSIFIED JUST OTHERSTUFF"));
        Serial.println(receiver_1.OtherflushTotal);
      }
    }
    classifyNow_1 = false;
  PT_END(pt);
}

//*************************************************************************************************************
//*******                               PROTOTHREAD - parseGPS_sense_1
//*************************************************************************************************************
static int parseGPS_sense_1(struct pt *pt)
{
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, receiver_1.GPSflushTotal.length());
    LogPrintln(&receiver_1,3,F("PARSEGPS_SENSE_1 : CALLED"));
    parseGPS_1();
  PT_END(pt);
}

//*************************************************************************************************************
//*******                               PROTOTHREAD - parseOther_sense_1
//*************************************************************************************************************
static int parseOther_sense_1(struct pt *pt)
{
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, receiver_1.OtherflushTotal.length());
    LogPrintln(&receiver_1,3,F("PARSEOTHER_SENSE_1 : CALLED"));
    parseOther_1(&receiver_1);
    updateHUDOther_1 = true;
  PT_END(pt);
}


//*************************************************************************************************************
//*******                               PROTOTHREAD - updateHUD_sense_1
//*************************************************************************************************************
static int updateHUD_sense_1(struct pt *pt)
{
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, (updateHUDGPS_1 || updateHUDOther_1));
    LogPrintln(&receiver_1,3,F("UPDATEHUD_SENSE_1 : CALLED"));
    printPrep_1();
    printHUD();  
  PT_END(pt);
}

//*************************************************************************************************************
//*******                               PROTOTHREAD - printHUD_sense
//*************************************************************************************************************
static int printHUD_sense(struct pt *pt)
{
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, rePrint);
    LogPrintln(&receiver_1,3,F("PRINTHUD_SENSE : CALLED"));
    printHUD(); 
    rePrint = false; 
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                 parseOther_1
//*************************************************************************************************************
void parseOther_1(thing* receiver){
  LogPrintln(&receiver_1,3,F("PARSEOTHER_1 : CALLED"));
  
  int pretimeadd = receiver->OtherflushTotal.indexOf(",");             // finds the locations on the 
  int preElev = receiver->OtherflushTotal.indexOf("," , pretimeadd+1); // string for each datafield
  int preAccX = receiver->OtherflushTotal.indexOf(",", preElev+1);                //
  int preAccY = receiver->OtherflushTotal.indexOf("," , preAccX+1);    //
  int preAccZ = receiver->OtherflushTotal.indexOf("," , preAccY+1);    //
                                                                            //
  int preGyroX = receiver->OtherflushTotal.indexOf("," , preAccZ+1);   //
  int preGyroY = receiver->OtherflushTotal.indexOf("," , preGyroX+1);  //
  int preGyroZ = receiver->OtherflushTotal.indexOf("," , preGyroY+1);  //
                                                                            //
  int preMagX = receiver->OtherflushTotal.indexOf("," , preGyroZ+1);   //
  int preMagY = receiver->OtherflushTotal.indexOf("," , preMagX+1);    //
  int preMagZ = receiver->OtherflushTotal.indexOf("," , preMagY+1);    //
                                                                            //
  int preEulX = receiver->OtherflushTotal.indexOf("," , preMagZ+1);    //
  int preEulY = receiver->OtherflushTotal.indexOf("," , preEulX+1);    //
  int preEulZ = receiver->OtherflushTotal.indexOf("," , preEulY+1);    //
                                                                            //
  int preGravX = receiver->OtherflushTotal.indexOf("," , preEulZ+1);   //
  int preGravY = receiver->OtherflushTotal.indexOf("," , preGravX+1);  //
  int preGravZ = receiver->OtherflushTotal.indexOf("," , preGravY+1);  //
                                                                            //
  int preTemp = receiver->OtherflushTotal.indexOf("," , preGravZ+1);   //
  int prePressure = receiver->OtherflushTotal.indexOf("," , preTemp+1);  //
  int preHumidity = receiver->OtherflushTotal.indexOf("," , prePressure+1);  //
  int preAltAltitude = receiver->OtherflushTotal.indexOf("," , preHumidity+1); //
  int prePitot = receiver->OtherflushTotal.indexOf("," , preAltAltitude+1);  //
  int postPitot = receiver->OtherflushTotal.indexOf("--");                   //

  Serial.println(pretimeadd);
  Serial.println(preElev);
  Serial.println(preAccX);
  Serial.println(preAccY);
  Serial.println(preAccZ);
  Serial.println(preGyroX);
  Serial.println(preGyroY);
  Serial.println(preGyroZ);
  Serial.println(preMagX);
  Serial.println(preMagY);
  Serial.println(preMagZ);
  Serial.println(preGravX);
  Serial.println(preGravY);
  Serial.println(preGravZ);
  Serial.println(preTemp);
  Serial.println(prePressure);
  Serial.println(preHumidity);
  Serial.println(preAltAltitude);
  Serial.println(prePitot);
  Serial.println(postPitot);

  
  

  if(pretimeadd + preElev + preAccX + preAccY + preAccZ + preGyroX + preGyroY + preGyroZ + preMagX + preMagY + preMagZ + preEulX + preEulY + preEulZ + preGravX + preGravY + preGravZ + preTemp + prePressure + preHumidity + preAltAltitude + prePitot + postPitot < 22)
  {
    LogPrintln(&receiver_1,1,F("PARSEOTHER_1 : MISSING FIELDS IN NMEA SENTENCE, BUFFER CLEARED"));
    receiver_1.OtherflushTotal = "";
    return;
  }

  // parses up the datafields and assigns them to the corresponding struct members
  receiver->acc.x = receiver->OtherflushTotal.substring(preAccX+1,preAccY);
  receiver->acc.y = receiver->OtherflushTotal.substring(preAccY+1,preAccZ);
  receiver->acc.z = receiver->OtherflushTotal.substring(preAccZ+1,preGyroX);

  receiver->gyro.x = receiver->OtherflushTotal.substring(preGyroX+1,preGyroY);
  receiver->gyro.y = receiver->OtherflushTotal.substring(preGyroY+1,preGyroZ);
  receiver->gyro.z = receiver->OtherflushTotal.substring(preGyroZ+1,preMagX);

  receiver->mag.x = receiver->OtherflushTotal.substring(preMagX+1,preMagY);
  receiver->mag.y = receiver->OtherflushTotal.substring(preMagY+1,preMagZ);
  receiver->mag.z = receiver->OtherflushTotal.substring(preMagZ+1,preEulX);

  receiver->eul.x = receiver->OtherflushTotal.substring(preEulX+1,preEulY);
  receiver->eul.y = receiver->OtherflushTotal.substring(preEulY+1,preEulZ);
  receiver->eul.z = receiver->OtherflushTotal.substring(preEulZ+1,preTemp);

  receiver->pressure = receiver->OtherflushTotal.substring(prePressure+1,preHumidity);
  receiver->temperature = receiver->OtherflushTotal.substring(preTemp+1,prePressure);
  receiver->humidity = receiver->OtherflushTotal.substring(preHumidity+1,preAltAltitude);
  receiver->pitot = receiver->OtherflushTotal.substring(prePitot+1,postPitot);
    
  receiver->OtherflushTotal = "";
  LogPrintln(&receiver_1,3,F("PARSEOTHER_1 : OTHER SENSOR SENTENCE DETECTED/PARSED, BUFFER CLEARED"));
  
}

//*************************************************************************************************************
//*******                                 parseGPS_1
//*************************************************************************************************************
void parseGPS_1(){
  if(receiver_1.GPSflushTotal.indexOf("GPRMC") >= 0){
    uint8_t preTime = receiver_1.GPSflushTotal.indexOf(",");
    uint8_t preStatus = receiver_1.GPSflushTotal.indexOf(",", preTime + 1);
    uint8_t preLat = receiver_1.GPSflushTotal.indexOf(",", preStatus + 1);
    uint8_t preLatSuffix = receiver_1.GPSflushTotal.indexOf(",", preLat + 1);
    uint8_t preLon = receiver_1.GPSflushTotal.indexOf(",", preLatSuffix + 1);
    uint8_t preLonSuffix = receiver_1.GPSflushTotal.indexOf(",", preLon + 1);
    uint8_t postLonSuffix = receiver_1.GPSflushTotal.indexOf(",", preLonSuffix + 1);

    if((preTime + preStatus + preLat + preLatSuffix + preLon + preLonSuffix + postLonSuffix) < 6)
    {
      LogPrintln(&receiver_1,1,F("PARSEGPS_1 : MISSING FIELDS IN NMEA SENTENCE, BUFFER CLEARED"));
      receiver_1.GPSflushTotal = "";
      return;
    }

    receiver_1.Time = receiver_1.GPSflushTotal.substring(preTime + 1,preStatus);
    receiver_1.lat = receiver_1.GPSflushTotal.substring(preLat + 1,preLon);
    receiver_1.lon = receiver_1.GPSflushTotal.substring(preLon + 1,postLonSuffix);
    
    receiver_1.GPSflushTotal = "";
    LogPrintln(&receiver_1,3,F("PARSEGPS_1 : RMC SENTENCE DETECTED/PARSED, BUFFER CLEARED"));
    updateHUDGPS_1 = true;
  }
  else if(receiver_1.GPSflushTotal.indexOf("GPGGA") >= 0){
    uint8_t preTime = receiver_1.GPSflushTotal.indexOf(",");
    uint8_t preLat = receiver_1.GPSflushTotal.indexOf(",", preTime + 1);
    uint8_t preLatSuffix = receiver_1.GPSflushTotal.indexOf(",", preLat + 1);
    uint8_t preLon = receiver_1.GPSflushTotal.indexOf(",", preLatSuffix + 1);
    uint8_t preLonSuffix = receiver_1.GPSflushTotal.indexOf(",", preLon + 1);
    uint8_t preFixQuality = receiver_1.GPSflushTotal.indexOf(",", preLonSuffix + 1);
    uint8_t preSatTracked = receiver_1.GPSflushTotal.indexOf(",", preFixQuality + 1);
    uint8_t preHDOP = receiver_1.GPSflushTotal.indexOf(",", preSatTracked + 1);
    uint8_t preElev = receiver_1.GPSflushTotal.indexOf(",", preHDOP + 1);
    uint8_t postElev = receiver_1.GPSflushTotal.indexOf(",", preElev + 1);

    if((preTime + preLat + preLatSuffix + preLon + preLonSuffix + preFixQuality + preSatTracked + preHDOP + preElev + postElev) < 9)
    {
      LogPrintln(&receiver_1,1,F("PARSEGPS_1 : MISSING FIELDS IN NMEA SENTENCE, BUFFER CLEARED"));
      receiver_1.GPSflushTotal = "";
      return;
    }

    receiver_1.Time = receiver_1.GPSflushTotal.substring(preTime + 1,preLat);
    receiver_1.lat = receiver_1.GPSflushTotal.substring(preLat + 1,preLon);
    receiver_1.lon = receiver_1.GPSflushTotal.substring(preLon + 1,preFixQuality);
    receiver_1.fixQual = receiver_1.GPSflushTotal.substring(preFixQuality + 1,preSatTracked);
    receiver_1.elev = receiver_1.GPSflushTotal.substring(preElev + 1,postElev);
    
    receiver_1.GPSflushTotal = "";
    LogPrintln(&receiver_1,3,F("PARSEGPS_1 : GGA SENTENCE DETECTED/PARSED, BUFFER CLEARED"));
    updateHUDGPS_1 = true;
  }
  else{
    LogPrintln(&receiver_1,1,F("PARSEGPS_1 : UNIDENTIFIED NMEA SENTENCE, BUFFER CLEARED"));
    receiver_1.GPSflushTotal = "";
  }
}

//*************************************************************************************************************
//*******                                 printPrep_1
//*************************************************************************************************************
void printPrep_1(){
  LogPrintln(&receiver_1,3,F("PRINTPREP_1 : CALLED"));
  if(!nameSpaced_1){
    LogPrintln(&receiver_1,3,F("PRINTPREP_1 : UPDATING THE NAME FIELD"));
    uint8_t preName_space = (COLUMN_WIDTH - receiver_1.Name.length())/2;
    uint8_t postName_space = COLUMN_WIDTH - (preName_space + receiver_1.Name.length());
    for(int i = 0; i < preName_space; i++){
      receiver_1.Name = " " + receiver_1.Name;
    }
    for(int i = 0; i < postName_space; i++){
      receiver_1.Name = receiver_1.Name + F(" ");
    }
    nameSpaced_1 = true; 
  }
  
  if(updateHUDOther_1){
    LogPrintln(&receiver_1,3,F("PRINTPREP_1 : UPDATEHUDOTHER_1"));
 
    receiver_1.acc.x = "ACC X : " + receiver_1.acc.x;
    receiver_1.acc.y = "ACC Y : " + receiver_1.acc.y;
    receiver_1.acc.z = "ACC Z : " + receiver_1.acc.z;
  
    receiver_1.gyro.x = "GYRO X : " + receiver_1.gyro.x;
    receiver_1.gyro.y = "GYRO Y : " + receiver_1.gyro.y;
    receiver_1.gyro.z = "GYRO Z : " + receiver_1.gyro.z;
  
    receiver_1.mag.x = "MAG X : " + receiver_1.mag.x;
    receiver_1.mag.y = "MAG Y : " + receiver_1.mag.y;
    receiver_1.mag.z = "MAG Z : " + receiver_1.mag.z;
  
    receiver_1.eul.x = "EUL X : " + receiver_1.eul.x;
    receiver_1.eul.y = "EUL Y : " + receiver_1.eul.y;
    receiver_1.eul.z = "EUL Z : " + receiver_1.eul.z;
  
    receiver_1.pressure = "PRESS : " + receiver_1.pressure;
    receiver_1.temperature = "TEMP : " + receiver_1.temperature;
    receiver_1.humidity = "HUMID : " + receiver_1.humidity;
    receiver_1.pitot = "PITOT : " + receiver_1.pitot;

  
    uint8_t accSpace_x_1 = (15 - receiver_1.acc.x.length());
    for(int i = 0; i < accSpace_x_1; i++){
      receiver_1.acc.x += F(" ");
    }
    uint8_t accSpace_y_1 = (15 - receiver_1.acc.y.length());
    for(int i = 0; i < accSpace_y_1; i++){
      receiver_1.acc.y += F(" ");
    }
    uint8_t accSpace_z_1 = (15 - receiver_1.acc.z.length());
    for(int i = 0; i < accSpace_z_1; i++){
      receiver_1.acc.z += F(" ");
    }

    uint8_t gyroSpace_x_1 = (15 - receiver_1.gyro.x.length());
    for(int i = 0; i < gyroSpace_x_1; i++){
      receiver_1.gyro.x += F(" ");
    }
    uint8_t gyroSpace_y_1 = (15 - receiver_1.gyro.y.length());
    for(int i = 0; i < gyroSpace_y_1; i++){
      receiver_1.gyro.y += F(" ");
    }
    uint8_t gyroSpace_z_1 = (15 - receiver_1.gyro.z.length());
    for(int i = 0; i < gyroSpace_z_1; i++){
      receiver_1.gyro.z += F(" ");
    }

    uint8_t magSpace_x_1 = (15 - receiver_1.mag.x.length());
    for(int i = 0; i < magSpace_x_1; i++){
      receiver_1.mag.x += F(" ");
    }
    uint8_t magSpace_y_1 = (15 - receiver_1.mag.y.length());
    for(int i = 0; i < magSpace_y_1; i++){
      receiver_1.mag.y += F(" ");
    }
    uint8_t magSpace_z_1 = (15 - receiver_1.mag.z.length());
    for(int i = 0; i < magSpace_z_1; i++){
      receiver_1.mag.z += F(" ");
    }

    uint8_t eulSpace_x_1 = (15 - receiver_1.eul.x.length());
    for(int i = 0; i < eulSpace_x_1; i++){
      receiver_1.eul.x += F(" ");
    }
    uint8_t eulSpace_y_1 = (15 - receiver_1.eul.y.length());
    for(int i = 0; i < eulSpace_y_1; i++){
      receiver_1.eul.y += F(" ");
    }
    uint8_t eulSpace_z_1 = (15 - receiver_1.eul.z.length());
    for(int i = 0; i < eulSpace_z_1; i++){
      receiver_1.eul.z += F(" ");
    }

    uint8_t pressSpace_1 = (22 - receiver_1.pressure.length());
    for(int i = 0; i < pressSpace_1; i++){
      receiver_1.pressure += F(" ");
    }
    uint8_t tempSpace_1 = (22 - receiver_1.temperature.length());
    for(int i = 0; i < tempSpace_1; i++){
      receiver_1.temperature += F(" ");
    }
    uint8_t humidSpace_1 = (22 - receiver_1.humidity.length());
    for(int i = 0; i < humidSpace_1; i++){
      receiver_1.humidity += F(" ");
    }
    uint8_t pitotSpace_1 = (22 - receiver_1.pitot.length());
    for(int i = 0; i < pitotSpace_1; i++){
      receiver_1.pitot += F(" ");
    }

    if(!nameSpaced_1){
      hudLines_1[0] = receiver_1.Name + F("|| ");
      for(int i = 0; i < COLUMN_WIDTH; i++)
      {
        hudLines_1[1] += F(" ");
        hudLines_1[5] += F(" ");
        hudLines_1[6] += F(" ");
        hudLines_1[11] += F(" ");
        hudLines_1[12] += F(" ");
      }
      hudLines_1[1] += F("|| ");
      hudLines_1[5] += F("|| ");
      hudLines_1[6] += F("|| ");
      hudLines_1[11] += F("|| ");
      hudLines_1[12] += F("|| ");
      hudLines_1[7] = "ACC        GYRO       MAG        || ";
    }
  
    hudLines_1[8] = receiver_1.acc.x + receiver_1.gyro.x + receiver_1.mag.x + F("|| ");
    hudLines_1[9] = receiver_1.acc.y + receiver_1.gyro.y + receiver_1.mag.y + F("|| ");
    hudLines_1[10] = receiver_1.acc.z + receiver_1.gyro.z + receiver_1.mag.z + F("|| ");
    
    hudLines_1[13] = "EUL        " + receiver_1.pressure + F("|| ");
    hudLines_1[14] = receiver_1.eul.x + receiver_1.temperature + F("|| ");
    hudLines_1[15] = receiver_1.eul.y + receiver_1.humidity + F("|| ");
    hudLines_1[16] = receiver_1.eul.z + receiver_1.pitot + F("|| ");

    updateHUDOther_1 = false;
  }

  if(updateHUDGPS_1 == true){
    LogPrintln(&receiver_1,3,F("PRINTPREP_1 : updateHUDGPS_1"));
    receiver_1.lat = "LAT : " + receiver_1.lat;
    receiver_1.lon = "LON : " + receiver_1.lon;
    receiver_1.elev ="ELEV : " + receiver_1.elev;

    uint8_t latSpace_1 = COLUMN_WIDTH - receiver_1.lat.length();
    uint8_t lonSpace_1 = COLUMN_WIDTH - receiver_1.lon.length();
    uint8_t elevSpace_1 = COLUMN_WIDTH - receiver_1.elev.length();

    for(int i = 0; i < latSpace_1; i++){
      receiver_1.lat += F(" ");
    }
    for(int i = 0; i < lonSpace_1; i++){
      receiver_1.lon += F(" ");
    }
    for(int i = 0; i < elevSpace_1; i++){
      receiver_1.elev += F(" ");
    }

    hudLines_1[2] = receiver_1.lat + F("|| ");
    hudLines_1[3] = receiver_1.lon + F("|| ");
    hudLines_1[4] = receiver_1.elev + F("|| ");

    updateHUDGPS_1 = false;
  }
  LogPrintln(&receiver_1,3,F("PRINTPREP_1 : UPDATED"));
  rePrint = true;
}

//*************************************************************************************************************
//*******                                 printHUD
//*************************************************************************************************************
void printHUD(){
  LogPrintln(&receiver_1,3,F("PRINTHUD : CALLED"));

  #ifndef XB_DEST_ADDR_2
    for(int i = 0; i < 17; i++){
      Serial.println(hudLines_1[i]);
    }
  #else
    for(int i = 0; i < 17; i++){
      Serial.print(hudLines_1[i]);
      Serial.println(hudLines_2[i]);
    }
  #endif
 
  LogPrintln(&receiver_1,3,F("PRINTHUD : COMPLETED"));
}

//*************************************************************************************************************
//*******                                 stringReserveMem_1
//*************************************************************************************************************
void stringReserveMem_1(){
  LogPrintln(&receiver_1,3,F("STRINGRESERVEMEM_1 : CALLED"));
  receiver_1.Name.reserve(COLUMN_WIDTH);
  receiver_1.TempflushTotal.reserve(300);
  receiver_1.OtherflushTotal.reserve(700);
  receiver_1.GPSflushTotal.reserve(700);
//  receiver_1.Time.reserve(COLUMN_WIDTH);
//  receiver_1.lat.reserve(COLUMN_WIDTH);
//  receiver_1.lon.reserve(COLUMN_WIDTH);
//  receiver_1.elev.reserve(COLUMN_WIDTH);
//  receiver_1.fixQual.reserve(COLUMN_WIDTH);
//  receiver_1.pressure.reserve(COLUMN_WIDTH);
//  receiver_1.temperature.reserve(COLUMN_WIDTH);
//  receiver_1.humidity.reserve(COLUMN_WIDTH);
//  receiver_1.pitot.reserve(COLUMN_WIDTH);
//
//  for(int i = 0; i < 17; i++){
//     hudLines_1[i].reserve(COLUMN_WIDTH);
//  }
}

//*************************************************************************************************************
//*******                                 setupXbee_1
//*************************************************************************************************************
void setupXbee_1()
    {
      LogPrintln(&receiver_1,3,F("SETUPXBEE_1 : CALLED"));
      xbee_1.setSPI_clockFreq(XB_SPI_CLK_1);
      xbee_1.setSPI_bitOrder(MSBFIRST);
      xbee_1.setSPI_mode(SPI_MODE0);
      xbee_1.setMode(XB_PRIMARY_MODE_1);
      xbee_1.setDestinationAddress(XB_DEST_ADDR_1);
      xbee_1.setCommandInterruptPin(XB_CMD_1);
      xbee_1.setLocalCommand_DIO(XB_LOC_COMMAND_DIO_1);
      //xbee_1.setCutdownInterruptPin();                      // use if needed
      xbee_1.setLocalCutdown_DIO(XB_LOC_CUTDOWN_DIO_1);
      xbee_1.setDestinationCommand_DIO(XB_DEST_COMMAND_DIO_1);
      xbee_1.setDestinationCutdown_DIO(XB_DEST_CUTDOWN_DIO_1);
      xbee_1.setMaxPayloadSize(XB_MAX_PAYLOAD_SIZE_1); 
    }


//*************************************************************************************************************
//*******                                 LOGFILE STUFF
//*************************************************************************************************************
void Log_reserveMem(uint16_t buffer_size){
  LogbufferSize = buffer_size;
  LogString.reserve(buffer_size);
}

void Log_setPriority(uint8_t priority_sd, uint8_t priority_sm){
  LogPriority_sd = priority_sd;
  LogPriority_sm = priority_sm;
}

void Log_exclusiveMode(bool mode){
  LogExclusive = mode;
}

void Log_autoFlush(uint16_t sizeToAdd){
  if((LogString.length() + sizeToAdd) > LogbufferSize)
  {
    logFile.print(LogString);
    logFile.flush();
    LogString = "";
  }
}

char Log_classify(uint8_t priority){
  switch(priority){
    case 1:
      return 'E';
      break;
    case 2:
      return 'W';
      break;
    case 3:
      return 'S';
      break;
    default:
      return '?';
      break;
  };
}

uint16_t getLength(const __FlashStringHelper *message)
{
  PGM_P p = reinterpret_cast<PGM_P>(message);
  uint16_t n = 0;
  while (1) {
    unsigned char c = pgm_read_byte(p++);
    if (c == 0) break;
    n++;
  }
  return n;
}

void LogPrintln(thing* thisThing, uint8_t priority, const char* message){
  if(((LogPriority_sd == priority)) || ((!LogExclusive) && (priority <= LogPriority_sd))){
    Log_autoFlush((uint16_t)(strlen(message) + 7 + thisThing->Name.length()));
    LogString = LogString + Log_classify(priority) + F(" : ") + thisThing->Name + F(" : ");
    LogString.concat(message);
    LogString += F("\n");
  }

  if(((LogPriority_sm == priority)) || ((!LogExclusive) && (priority <= LogPriority_sm))){  
    Serial.print(Log_classify(priority));
    Serial.print(F(" : "));
    Serial.print(thisThing->Name);
    Serial.print(F(" : "));
    Serial.println(message);
  }
} 

void LogPrintln(thing* thisThing, uint8_t priority, char message[], uint16_t message_length){
  if(((LogPriority_sd == priority)) || ((!LogExclusive) && (priority <= LogPriority_sd))){
    Log_autoFlush(message_length + 7 + thisThing->Name.length());
    LogString = LogString + Log_classify(priority) + F(" : ") + thisThing->Name + F(" : ");
    for(int i = 0; i < message_length; i++){
      LogString += message[i];
    }
    LogString += F("\n"); 
  }
  
  if(((LogPriority_sm == priority)) || ((!LogExclusive) && (priority <= LogPriority_sm))){  
    Serial.print(Log_classify(priority));
    Serial.print(F(" : "));
    Serial.print(thisThing->Name);
    Serial.print(F(" : "));
    for(int i = 0; i < message_length; i++){
      Serial.print(message[i]);
    }
    Serial.println(F(""));
  }
}

void LogPrintln(thing* thisThing, uint8_t priority, String message){
  if(((LogPriority_sd == priority)) || ((!LogExclusive) && (priority <= LogPriority_sd))){
    Log_autoFlush(message.length() + 7 + thisThing->Name.length());
    LogString = LogString + Log_classify(priority) + " : " + thisThing->Name + F(" : ");
    LogString = LogString +  message + "\n" ;
  }
  if(((LogPriority_sm == priority)) || ((!LogExclusive) && (priority <= LogPriority_sm))){  
    Serial.print(Log_classify(priority));
    Serial.print(F(" : "));
    Serial.print(thisThing->Name);
    Serial.print(F(" : "));
    Serial.println(message);
  }
}

void LogPrintln(thing* thisThing, uint8_t priority, const __FlashStringHelper* message){
  if(((LogPriority_sd == priority)) || ((!LogExclusive) && (priority <= LogPriority_sd))){
    Log_autoFlush(getLength(message) + 7 + thisThing->Name.length());
    LogString = LogString + Log_classify(priority) + " : " + thisThing->Name + F(" : ");
    LogString.concat(message);
    LogString +=  "\n";
  }
  if(((LogPriority_sm == priority)) || ((!LogExclusive) && (priority <= LogPriority_sm))){  
    Serial.print(Log_classify(priority));
    Serial.print(F(" : "));
    Serial.print(thisThing->Name);
    Serial.print(F(" : "));
    Serial.println(message);
  }
}

void LogPrint(thing* thisThing, uint8_t priority, char message[], uint16_t message_length){
  if(((LogPriority_sd == priority)) || ((!LogExclusive) && (priority <= LogPriority_sd))){
    Log_autoFlush(message_length + 7 + thisThing->Name.length());
    LogString = LogString + Log_classify(priority) + F(" : ") + thisThing->Name + F(" : ");
    for(int i = 0; i < message_length; i++){
      LogString += message[i];
    }
  }
  
  if(((LogPriority_sm == priority)) || ((!LogExclusive) && (priority <= LogPriority_sm))){  
    Serial.print(Log_classify(priority));
    Serial.print(F(" : "));
    Serial.print(thisThing->Name);
    Serial.print(F(" : "));
    for(int i = 0; i < message_length; i++){
      Serial.print(message[i]);
    }
  }
}

void LogPrint(thing* thisThing, uint8_t priority, String message){
  if(((LogPriority_sd == priority)) || ((!LogExclusive) && (priority <= LogPriority_sd))){
    Log_autoFlush(message.length() + 7 + thisThing->Name.length());
    LogString = LogString + Log_classify(priority) + " : " + thisThing->Name + F(" : ");
    LogString = LogString +  message;
  }
  if(((LogPriority_sm == priority)) || ((!LogExclusive) && (priority <= LogPriority_sm))){  
    Serial.print(Log_classify(priority));
    Serial.print(F(" : "));
    Serial.print(thisThing->Name);
    Serial.print(F(" : "));
    Serial.print(message);
  }
}

void LogPrint(thing* thisThing, uint8_t priority, const __FlashStringHelper* message){
  if(((LogPriority_sd == priority)) || ((!LogExclusive) && (priority <= LogPriority_sd))){
    Log_autoFlush(getLength(message) + 7 + thisThing->Name.length());
    LogString = LogString + Log_classify(priority) + " : " + thisThing->Name + F(" : ");
    LogString.concat(message);
  }
  if(((LogPriority_sm == priority)) || ((!LogExclusive) && (priority <= LogPriority_sm))){  
    Serial.print(Log_classify(priority));
    Serial.print(F(" : "));
    Serial.print(thisThing->Name);
    Serial.print(F(" : "));
    Serial.print(message);
  }
}


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
//*******                                           MakeFiles
//*************************************************************************************************************

bool makeFiles(){  
  //LogPrintln(&System,3,F("entering makeFiles()")); // 
  bool rtn = true;                          // set a boolean flag of rtn to be true
  bool gpsFunctionState = true;
  bool otherFunctionState = true;
  bool logFunctionState = true;
  
  #ifdef GPS_FILENAME
  
    uint8_t gpsfile_size = strlen(GPS_FILENAME);
    uint8_t gpssuffix_ind = gpsfile_size - 6;
    char gpsfile[gpsfile_size];
    strcpy(gpsfile, GPS_FILENAME);
    bool gpsfind_state = false;
    for(uint8_t i = 0; i < 100; i++)          // for the gps file 
    {                                         // for all numbers from 0 to 100
      gpsfile[gpssuffix_ind] = '0' + i/10;                // replace the 6th character with a 0 plus the 1st digit of the division of i and 10
      gpsfile[gpssuffix_ind+1] = '0' + i%10;                // replace the 7th character with a 0 plus the 2nd digit of the remainder of the division of i and 10
      if(!SD.exists(gpsfile))                 // if that new file name DOES NOT EXIST within the sd card
      {                                       //
        gpsfind_state = true;
        break;                                // then get out of the loop because you have found a new file name which doesn't exist
      }                                       //
    }
  
    if(gpsfind_state){
      LogPrintln(&System,3,F("filename find successful!"));
      LogPrintln(&System,3,gpsfile);
      //Serial.print(F("MAKEFILES: filename find successful! : "));
      //Serial.println(gpsfile);
      
      GPSdata = SD.open(gpsfile, FILE_WRITE);   // write a new file for the gps file on the SD card
      if(!GPSdata)                              // if the file failed to be created
      {                                         // 
        LogPrint(&System,1,F("MAKEFILES:Failed to create : "));         // display failed to create on the serial monitor
        LogPrintln(&System,1,gpsfile,gpsfile_size);                   // followed by the filename
        
        //Serial.print(F("MAKEFILES: FAILED TO CREATE:"));
        //Serial.println(gpsfile);
        gpsFunctionState = false;                            // set the boolean flag rtn to false
      }                                         // end
      else{
        LogPrint(&System,3,F("MAKEFILES:SUCCESSFULLY OPENED"));
        LogPrintln(&System,3,gpsfile,gpsfile_size);

        //Serial.print(F("MAKEFILES: SUCCESSFULLY OPENED : "));
        //Serial.println(gpsfile);
      }
    }
    else{
      LogPrint(&System,1,F("MAKEFILES:Failed to find a good filename"));
      //Serial.println(F("MAKEFILES:Failed to find a good filename, gps"));
      
      gpsFunctionState = false;
    }
    
  #endif


  #ifdef OTHER_FILENAME
   
    //delay(100);
    uint8_t other_size = strlen(OTHER_FILENAME);
    uint8_t othersuffix_ind = other_size - 6;
    char otherfile[other_size];
    strcpy(otherfile, OTHER_FILENAME);
    bool otherfind_state = false;
    for(uint8_t i = 0; i < 100; i++)          // for the gps file 
    {                                         // for all numbers from 0 to 100
      otherfile[othersuffix_ind] = '0' + i/10;                // replace the 6th character with a 0 plus the 1st digit of the division of i and 10
      otherfile[othersuffix_ind+1] = '0' + i%10;                // replace the 7th character with a 0 plus the 2nd digit of the remainder of the division of i and 10
      if(!SD.exists(otherfile))                 // if that new file name DOES NOT EXIST within the sd card
      {                                       //
        otherfind_state = true;
        break;                                // then get out of the loop because you have found a new file name which doesn't exist
      }                                       //
    }
  
    if(otherfind_state){
      LogPrint(&System,3,F("MAKEFILES: filename find successful! : "));       
      LogPrintln(&System,3,otherfile,other_size);   
      
      //Serial.print(F("MAKEFILES: filename find successful! : "));
      //Serial.println(otherfile);
      
      OtherData = SD.open(otherfile, FILE_WRITE);   // write a new file for the gps file on the SD card
      if(!OtherData)                              // if the file failed to be created
      {                                         // 
        LogPrint(&System,1,F("MAKEFILES:Failed to create : "));         // display failed to create on the serial monitor
        LogPrintln(&System,1,otherfile,other_size);                   // followed by the filename
        
        //Serial.print(F("MAKEFILES: FAILED TO CREATE:"));
        //Serial.println(otherfile);
        otherFunctionState = false;                            // set the boolean flag rtn to false
      }                                         // end
      else{
        LogPrint(&System,3,F("MAKEFILES:SUCCESSFULLY OPENED"));
        LogPrintln(&System,3,otherfile,other_size); 
        //Serial.print(F("MAKEFILES:SUCCESSFULLY OPENED : "));
        //Serial.println(otherfile);
      }
    }
    else{
      LogPrint(&System,1,F("MAKEFILES:Failed to find a good filename"));
      //Serial.println(F("MAKEFILES: Failed to find a good filename"));
      otherFunctionState = false;
    }
    
  #endif

  #ifdef LOG_FILENAME
  
    //delay(100);
    uint8_t log_size = strlen(LOG_FILENAME);
    uint8_t logsuffix_ind = log_size - 6;
    char logfile[log_size];
    strcpy(logfile, LOG_FILENAME);
    bool logfind_state = false;
    for(uint8_t i = 0; i < 100; i++)          // for the gps file 
    {                                         // for all numbers from 0 to 100
      logfile[logsuffix_ind] = '0' + i/10;                // replace the 6th character with a 0 plus the 1st digit of the division of i and 10
      logfile[logsuffix_ind+1] = '0' + i%10;                // replace the 7th character with a 0 plus the 2nd digit of the remainder of the division of i and 10
      if(!SD.exists(logfile))                 // if that new file name DOES NOT EXIST within the sd card
      {                                       //
        logfind_state = true;
        break;                                // then get out of the loop because you have found a new file name which doesn't exist
      }                                       //
    }
  
    if(logfind_state){
      LogPrint(&System,3,F("MAKEFILES: filename find successful! : "));       
      LogPrintln(&System,3,logfile,log_size);   
      
      //Serial.print(F("MAKEFILES: filename find successful! : "));
      //Serial.println(logfile);
      
      logFile = SD.open(logfile, FILE_WRITE);   // write a new file for the gps file on the SD card
      if(!logFile)                              // if the file failed to be created
      {                                         // 
        LogPrint(&System,1,F("MAKEFILES:Failed to create : "));         // display failed to create on the serial monitor
        LogPrintln(&System,1,logfile,log_size);                   // followed by the filename

        //Serial.print(F("MAKEFILES: FAILED TO CREATE:"));
        //Serial.println(logfile);
        logFunctionState = false;                            // set the boolean flag rtn to false
      }                                         // end
      else{
        LogPrint(&System,3,F("MAKEFILES:SUCCESSFULLY OPENED"));
        LogPrintln(&System,3,logfile,log_size);

        //Serial.print(F("MAKEFILES: MAKEFILES:SUCCESSFULLY OPENED : "));
        //Serial.println(logfile);
      }
    }
    else{
      LogPrint(&System,1,F("MAKEFILES:Failed to find a good filename"));

      //Serial.println(F("MAKEFILES:Failed to find a good filename"));
      logFunctionState = false;
    }
    
  #endif

  rtn = (gpsFunctionState && otherFunctionState && logFunctionState);

  if(!rtn){
    
    #ifdef GPS_FILENAME
    
    if(gpsFunctionState){
      GPSdata.close();
      SD.remove(gpsfile);
    }
      
    #endif
  
    #ifdef OTHER_FILENAME
    
    if(otherFunctionState){
      OtherData.close();
      SD.remove(otherfile);
    }
      
    #endif
  
    #ifdef LOG_FILENAME
    
    if(logFunctionState){
      logFile.close();
      SD.remove(logfile);
    }
      
    #endif
  }
  

  LogPrint(&System,3,F("MAKEFILE: LEAVING"));
  
  return rtn; 
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

//*************************************************************************************************************
//*******                                 SETUP
//*************************************************************************************************************
void setup() {
  delay(5000);
  Serial.begin(115200);

  Log_reserveMem(1000);
  Log_setPriority((uint8_t)3,(uint8_t)3);
  LogPrintln(&System,3,F("SETUP : BEGIN INITIALIZING"));
  
  System.Name = "System";
  receiver_1.Name = XBEE1_NAME;
  LogPrintln(&System,3,F("SETUP : ASSIGNED THING NAMES"));

  stringReserveMem_1;
  LogPrintln(&System,3,F("SETUP : RESERVED SYSTEM STRING MEMORY"));
  
  pinMode(sdPower, OUTPUT);
  digitalWrite(sdPower, HIGH);
  LogPrintln(&System,3,F("SETUP : POWERED UP SENSORS"));
  
  long xbeeTime = millis();
  while(XB_NP_1 != 256 && (millis() - xbeeTime <= 5000))
  {
    XB_NP_1 = xbee_1.begin();
  }
  // include any other xbee begin cycles here
  if(XB_NP_1 == 256 /*&& XB2_NP == 256, etc. */)
  {
    setupXbee_1(); // include all setup features in this fucntion (above) 
  }
  LogPrintln(&System,3,F("SETUP : XBEE_1 HAS BEEN SET UP"));

  if(!SD.begin(SD_CS)){         // initializes the SD card
    LogPrintln(&System,1,F("SETUP : SD CARD FAILED TO INITIALIZE")); // if the SD card fails, set sdError true and tell someone
  }                                 
  else{
    LogPrintln(&System,3,F("SETUP : SD CARD CONNECTED"));
    uint8_t makefile_attempts = 0;
    while(!makeFiles() && (makefile_attempts < 99)){ 
        makefile_attempts++;        
    }
    if(makefile_attempts >= 99){
      LogPrintln(&System,1,F("SETUP : MAKEFILES FAILED TO INITIALIZE")); // if makeFiles fails, set makeFileError to true and tell someone
    }
    else{
      LogPrintln(&System,3,F("SETUP : MAKEFILES WORKED!"));
    }
  }

  PT_INIT(&xbeeReceivePT_1);
  PT_INIT(&xbeeDecompPT_1);
  PT_INIT(&xbeeDecodePT_1);
  PT_INIT(&classifyPT_1);
  PT_INIT(&parseGPSPT_1);
  PT_INIT(&parseOtherPT_1);
  PT_INIT(&updateHUDPT_1);
  PT_INIT(&printHUDPT);
  LogPrintln(&System,3,F("SETUP : PROTOTHREADS INITIALIZED"));

  
  
  LogPrintln(&System,3,F("SETUP : COMPLETED, MOVING TO LOOP"));
}

//*************************************************************************************************************
//*******                                 LOOP
//*************************************************************************************************************
void loop() {
  xbee_1.protothreadLoop();
  xbeeReceive_sense_1(&xbeeReceivePT_1);
  xbeeDecomp_sense_1(&xbeeDecompPT_1);
  xbeeDecode_sense_1(&xbeeDecodePT_1);
  //classifyStuff_sense_1(&classifyPT_1);
  //parseGPS_sense_1(&parseGPSPT_1);
  //parseOther_sense_1(&parseOtherPT_1);
  //updateHUD_sense_1(&updateHUDPT_1);
  //printHUD_sense(&printHUDPT);

}
