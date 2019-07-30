//XBEE DESTINATION ADDRESS1: 
//  UNCOMMENT WHICH XBEE DESTINATION ADDRESS YOU WANT TO COMMUNICATE WITH, UNCOMMENT THE REMAINDER
// 
//#define XB_DEST_ADDR_1 (0x0013A200417E38C1)  // DESTINATION XBEE 64-BIT ADDRESS (A200K, DEFAULT GONDOLA)
//#define XB_DEST_ADDR_1 (0x0013A200417E3816) // DESTINATION XBEE 64-BIT ADDRESS (B200K, DEFAULT SAIL)
//#define XB_DEST_ADDR_1 (0x0013A200417E3829)  // DESTINATION XBEE 64-BIT ADDRESS (C200K, DEFAULT GONDOLA RECEIVER)
#define XB_DEST_ADDR_1 (0x0013A200417E381A) // DESTINATION XBEE 64-BIT ADDRESS (D200K, DEFAULT SAIL RECEIVER)
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
      #define XBEE1_NAME "GONDOLA"
//      #define XBEE1_NAME "SAIL"
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
 #define GPS_FILENAME "GPSLOG05.TXT"
 #define OTHER_FILENAME "AODATA05.TXT"
 #define LOG_FILENAME "LOG05.TXT"
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
  ThreeAxis acc;
  ThreeAxis gyro;
  ThreeAxis mag;
  ThreeAxis eul;
  String pressure;
  String temperature;
  String humidity;
  String pitot;
  
  String Name_space;
  ThreeSpace acc_space;
  ThreeSpace gyro_space;
  ThreeSpace mag_space;
  ThreeSpace eul_space;
  String pressure_space;
  String temperature_space;
  String humidity_space;
  String pitot_space;
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
#define XB_PRIMARY_MODE_1 (1)                // 0 = FAST, 1 = ACCURATE, 2 = RECEIVER
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
uint8_t xbeeTempbuff_1[256];
uint8_t xbeeDecomp_1[600];
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
static struct pt parseGPS_1;
static struct pt parseOther_1;

//PROTOTHREAD-RELATED STUFF
bool classifyNow_1;

//*************************************************************************************************************
//*******                               PROTOTHREAD - xbeeReceive_sense_1
//*************************************************************************************************************
static int xbeeReceive_sense_1(struct pt *pt)
{
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, xbee_1.data_received_length);
    xbeeTempInd_1 = xbee_1.data_received_length;
    for(int i = 0; i < xbee_1.data_received_length; i++)
    {
      xbeeTempbuff_1[i] = xbee.data_received;
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
    xbeeTempInde_1 = 0;
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
    matchCaseDecompress(xbeeDecomp_1, xbeeDecompInd_1)
    TempflushTotal = "";
    for(int i = 0; i < xbeeDecompInd_1; i++)
    {
      TempflushTotal += (char)xbeeDecomp_1[i];
    }
    classifyNow_1 = true;
    xbeeDecompInd_1 = 0;
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
      receiver_1.GPSflushTotal = receiver_1.TempflushTotal;
      GPSdata.print(receiver_1.Name);
      GPSdata.print(F(" : "));
      GPSdata.print(receiver_1.GPSflushTotal);
      GPSdata.flush();
      LogPrintln(&receiver_1,3,F("CLASSIFYSTUFF_SENSE_1 : CLASSIFIED GPS"));
    }
    else{
      receiver_1.OtherflushTotal = receiver_1.TempflushTotal;
      OtherData.print(receiver_1.Name);
      OtherData.print(F(" : "));
      OtherData.print(receiver_1.GPSflushTotal);
      GPSdata.flush();
      LogPrintln(&receiver_1,3,F("CLASSIFYSTUFF_SENSE_1 : CLASSIFIED OTHERSTUFF"));
    }
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
    if(receiver_1.GPSflushTotal.indexOf("GPRMC") >= 0){
      
    }
    else if(receiver_1.GPSflushTotal.indexOf("GPGGA") >= 0){
      
    }
    else{
      
    }
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                 setupXbee_1
//*************************************************************************************************************
void setupXbee_1()
    {
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
  if((!(LogPriority_sd == priority)) || (!LogExclusive && (priority <= LogPriority_sd))){
    Log_autoFlush((uint16_t)(strlen(message) + 7 + thisThing->Name.length()));
    LogString = LogString + Log_classify(priority) + F(" : ") + thisThing->Name + F(" : ");
    LogString.concat(message);
    LogString += F("\n");
  }

  if((!(LogPriority_sm == priority)) || (!LogExclusive && (priority <= LogPriority_sm))){  
    Serial.print(Log_classify(priority));
    Serial.print(F(" : "));
    Serial.print(thisThing->Name);
    Serial.print(F(" : "));
    Serial.println(message);
  }
} 

void LogPrintln(thing* thisThing, uint8_t priority, char message[], uint16_t message_length){
  if((!(LogPriority_sd == priority)) || (!LogExclusive && (priority <= LogPriority_sd))){
    Log_autoFlush(message_length + 7 + thisThing->Name.length());
    LogString = LogString + Log_classify(priority) + F(" : ") + thisThing->Name + F(" : ");
    for(int i = 0; i < message_length; i++){
      LogString += message[i];
    }
    LogString += F("\n"); 
  }
  
  if((!(LogPriority_sm == priority)) || (!LogExclusive && (priority <= LogPriority_sm))){  
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
  if((!(LogPriority_sd == priority)) || (!LogExclusive && (priority <= LogPriority_sd))){
    Log_autoFlush(message.length() + 7 + thisThing->Name.length());
    LogString = LogString + Log_classify(priority) + " : " + thisThing->Name + F(" : ");
    LogString = LogString +  message + "\n" ;
  }
  if((!(LogPriority_sm == priority)) || (!LogExclusive && (priority <= LogPriority_sm))){  
    Serial.print(Log_classify(priority));
    Serial.print(F(" : "));
    Serial.print(thisThing->Name);
    Serial.print(F(" : "));
    Serial.println(message);
  }
}

void LogPrintln(thing* thisThing, uint8_t priority, const __FlashStringHelper* message){
  if(((LogPriority_sd == priority)) || (!LogExclusive && (priority <= LogPriority_sd))){
    Log_autoFlush(getLength(message) + 7 + thisThing->Name.length());
    LogString = LogString + Log_classify(priority) + " : " + thisThing->Name + F(" : ");
    LogString.concat(message);
    LogString +=  "\n";
  }
  if((!(LogPriority_sm == priority)) || (!LogExclusive && (priority <= LogPriority_sm))){  
    Serial.print(Log_classify(priority));
    Serial.print(F(" : "));
    Serial.print(thisThing->Name);
    Serial.print(F(" : "));
    Serial.println(message);
  }
}

void LogPrint(thing* thisThing, uint8_t priority, char message[], uint16_t message_length){
  if((!(LogPriority_sd == priority)) || (!LogExclusive && (priority <= LogPriority_sd))){
    Log_autoFlush(message_length + 7 + thisThing->Name.length());
    LogString = LogString + Log_classify(priority) + F(" : ") + thisThing->Name + F(" : ");
    for(int i = 0; i < message_length; i++){
      LogString += message[i];
    }
  }
  
  if((!(LogPriority_sm == priority)) || (!LogExclusive && (priority <= LogPriority_sm))){  
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
  if((!(LogPriority_sd == priority)) || (!LogExclusive && (priority <= LogPriority_sd))){
    Log_autoFlush(message.length() + 7 + thisThing->Name.length());
    LogString = LogString + Log_classify(priority) + " : " + thisThing->Name + F(" : ");
    LogString = LogString +  message;
  }
  if((!(LogPriority_sm == priority)) || (!LogExclusive && (priority <= LogPriority_sm))){  
    Serial.print(Log_classify(priority));
    Serial.print(F(" : "));
    Serial.print(thisThing->Name);
    Serial.print(F(" : "));
    Serial.print(message);
  }
}

void LogPrint(thing* thisThing, uint8_t priority, const __FlashStringHelper* message){
  if((!(LogPriority_sd == priority)) || (!LogExclusive && (priority <= LogPriority_sd))){
    Log_autoFlush(getLength(message) + 7 + thisThing->Name.length());
    LogString = LogString + Log_classify(priority) + " : " + thisThing->Name + F(" : ");
    LogString.concat(message);
  }
  if((!(LogPriority_sm == priority)) || (!LogExclusive && (priority <= LogPriority_sm))){  
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

  if(!rtn)
  {
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

  System.Name = "System";
  receiver_1.Name = XBEE1_NAME;

  Log_reserveMem(100);
  Log_setPriority(3,3);
  
  pinMode(sdPower, OUTPUT);
  digitalWrite(sdPower, HIGH);
  
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
  if(!SD.begin(SD_CS)){         // initializes the SD card
    Serial.println(F("SD Card failed to initialize!")); // if the SD card fails, set sdError true and tell someone
  }                                 
  else{
    uint8_t makefile_attempts = 0;
    while(!makeFiles() && (makefile_attempts < 99)){ 
        makefile_attempts++;        
    }
    if(makefile_attempts >= 99){
      Serial.println(F("File Initialization Failed!")); // if makeFiles fails, set makeFileError to true and tell someone
    }
    else{
      Serial.println(F("makefiles worked!"));
    }
  }

  OtherData.print(F("w00t!"));                 
  GPSdata.print(F("w00t!"));                  

  OtherData.flush();
  GPSdata.flush();
  

}

//*************************************************************************************************************
//*******                                 LOOP
//*************************************************************************************************************
void loop() {

  LogPrintln(&System,3,F("w00t!"));
  LogPrintln(&receiver_1,3,F("w00t!"));
  // put your main code here, to run repeatedly:

}
