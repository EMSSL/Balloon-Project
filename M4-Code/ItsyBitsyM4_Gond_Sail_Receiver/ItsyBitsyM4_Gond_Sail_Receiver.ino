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



//*************************************************************************************************************
//*******                            CRITICAL STUFF TO CONFIGURE BEFORE UPLOAD
//*************************************************************************************************************
//*************************************************************************************************************
// SENSOR PACK TYPE : UNCOMMENT WHICH HARDWARE APPLICATION YOU ARE USING THIS FOR, COMMENT OUT THE OTHERS
//
#define SENSORMODE_GONDOLA
//#define SENSORMODE_SAIL
//#define SENSORMODE_RECEIVER


//XBEE DESTINATION ADDRESS1: 
//  UNCOMMENT WHICH XBEE DESTINATION ADDRESS YOU WANT TO COMMUNICATE WITH, UNCOMMENT THE REMAINDER
// 
//#define XB1_DEST_ADDR (0x0013A200417E38C1)  // DESTINATION XBEE 64-BIT ADDRESS (A200K, DEFAULT GONDOLA)
//#define XB1_DEST_ADDR (0x0013A200417E3816) // DESTINATION XBEE 64-BIT ADDRESS (B200K, DEFAULT SAIL)
#define XB1_DEST_ADDR (0x0013A200417E3829)  // DESTINATION XBEE 64-BIT ADDRESS (C200K, DEFAULT GONDOLA RECEIVER)
//#define XB1_DEST_ADDR (0x0013A200417E381A) // DESTINATION XBEE 64-BIT ADDRESS (D200K, DEFAULT SAIL RECEIVER)
//#define XB1_DEST_ADDR (0x0013A200417E35F7) // DESTINATION XBEE 64-BIT ADDRESS (E200K, DEFAULT BACKUP UNIT)


//XBEE DESTINATION ADDRESS2: RECEIVER USE ONLY!
//  UNCOMMENT WHICH XBEE DESTINATION ADDRESS YOU WANT TO COMMUNICATE WITH, UNCOMMENT THE REMAINDER 
//
//#define XB2_DEST_ADDR (0x0013A200417E38C1)  // DESTINATION XBEE 64-BIT ADDRESS (A200K, DEFAULT GONDOLA)
//#define XB2_DEST_ADDR (0x0013A200417E3816) // DESTINATION XBEE 64-BIT ADDRESS (B200K, DEFAULT SAIL)
//#define XB2_DEST_ADDR (0x0013A200417E3829)  // DESTINATION XBEE 64-BIT ADDRESS (C200K, DEFAULT GONDOLA RECEIVER)
//#define XB2_DEST_ADDR (0x0013A200417E381A) // DESTINATION XBEE 64-BIT ADDRESS (D200K, DEFAULT SAIL RECEIVER)
//#define XB2_DEST_ADDR (0x0013A200417E35F7) // DESTINATION XBEE 64-BIT ADDRESS (E200K, DEFAULT BACKUP UNIT)


//XBEE UNIT NAMES : RECEIVER USE ONLY!
//  UNCOMMENT THE NAME OF EACH UNIT THE CORRESPONDING RECEIVE XBEE WILL BE RECEIVING FROM
//  COMMENT OUT THE REST, USED FOR FORMATTING THE HUD. 
//
//    XBEE1 TRANSMITTER NAME
//      #define XBEE1_NAME "GONDOLA"
//      #define XBEE1_NAME "SAIL"
//      #define XBEE1_NAME "OTHER"    // CHANGE OTHER TO YOUR PREFERRED NAME (LIMIT 7 CHARACTERS)
//
//    XBEE2 TRANSMITTER NAME
//      #define XBEE2_NAME "GONDOLA"
//      #define XBEE2_NAME "SAIL"
//      #define XBEE2_NAME "OTHER"    // CHANGE OTHER TO YOUR PREFERRED NAME (LIMIT 7 CHARACTERS)
//*************************************************************************************************************
//*******                                 END : CONFIGURATION SECTIONS
//*************************************************************************************************************

// error checking the macro logic for xbee errors
#if (defined(SENSORMODE_GONDOLA) || defined(SENSORMODE_SAIL)) && defined(XB2_DEST_ADDR)
  #define SENSORMODE_RECEIVER
#endif

// error checking the configuration macro logic to make the code more error-resistant
#if ((defined(SENSORMODE_GONDOLA) && !defined(SENSORMODE_SAIL) && !defined(SENSORMODE_RECEIVER))\
 || (!defined(SENSORMODE_GONDOLA) && defined(SENSORMODE_SAIL) && !defined(SENSORMODE_RECEIVER))\
 || (!defined(SENSORMODE_GONDOLA) && !defined(SENSORMODE_SAIL) && defined(SENSORMODE_RECEIVER)))

  #if (defined(SENSORMODE_GONDOLA) || defined(SENSORMODE_SAIL))

/**************************************************************************************************************
 * *************************************************************************************************************
 * *************************************************************************************************************
 *                                        
 *                                   GENERAL CONFIGURATION CODE ABOVE HERE   
 *                                     GONDOLA / SAIL CODE BELOW HERE
 *                                        
 * *************************************************************************************************************                                       
 * *************************************************************************************************************
 * **************************************************************************************************************/
  
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
         


    // SETUP DOTSTAR STUFF, INCLUDING SPECIFIC COLORS FOR EACH CONFIG
    #define NUMPIXELS 1             // There is only one pixel on the board
    #define DATAPIN    8            //Use these pin definitions for the ItsyBitsy M4
    #define CLOCKPIN   6            //
    Adafruit_DotStar px(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
    
    #if defined(SENSORMODE_GONDOLA)
      #define DOTSTAR_MODECOLOR_RED 0
      #define DOTSTAR_MODECOLOR_GREEN 0
      #define DOTSTAR_MODECOLOR_BLUE 100
    #elif defined(SENSORMODE_SAIL)
      #define DOTSTAR_MODECOLOR_RED 0
      #define DOTSTAR_MODECOLOR_GREEN 100
      #define DOTSTAR_MODECOLOR_BLUE 0
    #endif
    
    #define dotStarUpdate_thresh 20 // update the color combo of the dotstar every 20 milliseconds
    #define dotStarPostSetup_thresh 60000      // after 2 minutes of imu setup colors, revert to config-specific color
    long dotStarUpdate_stamp;       // used to track updates
    long dotStarSetup_stamp;               // used to track the period of setup
    bool dotStarPostSetup = false;       // used to kill the post-setup protothread

    
    
    
    
    // sets up IMU addresses based on the sensormode defined (remote IMU is Add.A, on-board IMU is Add.B)
    #if defined(SENSORMODE_GONDOLA) || defined(SENSORMODE_RECEIVER)
      Adafruit_BNO055 IMU = Adafruit_BNO055(55,BNO055_ADDRESS_B);
      #define imuPower 12     // used on the imu power transistor gate for a hard reset
    #elif defined(SENSORMODE_SAIL)
      Adafruit_BNO055 IMU = Adafruit_BNO055(55,BNO055_ADDRESS_B);
      #define imuPower 12     // used on the imu power transistor gate for a hard reset
    #endif
    
    #define usb Serial            // renames Serial as usb
    #define gps Serial1           // renames Serial2 as gps
    
    //Xbee OBJECT DEFINITION
    #define XB1_SS (5)                          // M4 PIN FOR XBEE SPI SLAVE SELECT
    #define XB1_ATTN (7)                        // M4 PIN FOR XBEE ATTN 
    #define XB1_CMD (9)                         // M4 PIN NUMBER FOR COMMAND SIGNAL
    #define XB1_LOC_COMMAND_DIO ('0')           // DIO NUMBER FOR LOCAL XBEE COMMAND PIN
    #define XB1_LOC_CUTDOWN_DIO ('5')           // DIO NUMBER FOR LOCAL XBEE CUTDOWN PIN
    #define XB1_DEST_COMMAND_DIO ('0')          // DIO NUMBER FOR DESTINATION COMMAND PIN
    #define XB1_DEST_CUTDOWN_DIO ('5')          // DIO NUMBER FOR DESTINATION CUTDOWN 
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
     
    //#define imuReset 34     // used on the imu RST pin for a soft reset
    
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
    
    uint8_t stringGrab_initial[700];    // char array for uncompressed stringGrab data
    uint8_t stringGrab_compressed[700]; // char array for compressed stringGrabdata
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

    static struct pt dotStarIMUsetupPT;
    static struct pt dotStarPostSetupPT;
    
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
    float altitude = 0;           // holds the official altitude info (in //the GPS or parsing function breaks, use altimeter as backup)
    float rudderAngle_actual;
    float rudderAngle_setpoint = 0;
    
    
    //char prev_gpsfile[15];        // previous file name for the GPS file in //of an sdError or makeFileError reset
    //char prev_otherdatafile[15];  // previous file name for the sensor file in //of an sdError or makeFileError reset
    //char gpsfile[15];             // current file name for the GPS file in //of an sdError or makeFileError reset
    //char otherdatafile[15];       // current file name for the sensor file in //of an sdError or makeFileError rese
    
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
      PT_WAIT_UNTIL(pt, (stringGrab_init) && (!stringGrab_gps) && (GPSwriteCount == 1));
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
      PT_WAIT_UNTIL(pt, (stringGrab_gps) && (!stringGrab_other) && (writeCount == 1));
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

    //*************************************************************************************************************
    //*******                                   PROTOTHREAD dotStarIMUsetup_sense
    //*************************************************************************************************************
    static int dotStarIMUsetup_sense(struct pt *pt){
      PT_BEGIN(pt);
      PT_WAIT_UNTIL(pt, ((!dotStarPostSetup) && ((millis() - dotStarSetup_stamp) <= dotStarPostSetup_thresh) && (millis() - dotStarUpdate_stamp >= dotStarUpdate_thresh)));
        
        imu::Vector<3> BN_eul = IMU.getVector(Adafruit_BNO055::VECTOR_EULER); // get IMU euler angles
        
        uint32_t eulx =  100 * BN_eul.x();    //
        uint32_t euly =  100 * BN_eul.y();    // turn the 0.00 - 360.00 readings into 0 to 36000 integers
        uint32_t eulz =  100 * BN_eul.z();    //

        eulx = map(eulx,0,36000,0,255);       // 
        euly = map(euly,0,36000,0,255);       // map the integers to a 0 to 255 (uint8_t) value that can be accepted by the dotstar
        eulz = map(eulz,0,36000,0,255);       //

        px.setPixelColor(0, eulx, euly, eulz);  // Set the pixel colors
        px.show();                              // Refresh strip

        dotStarUpdate_stamp = millis();       // update the refresh timestamp so it'll re-trigger the protothread properly
        
      PT_END(pt);
    }

    //*************************************************************************************************************
    //*******                                   PROTOTHREAD dotStarPostSetupPT_sense
    //*************************************************************************************************************
    static int dotStarPostSetupPT_sense(struct pt *pt){
      PT_BEGIN(pt);
      PT_WAIT_UNTIL(pt, (!dotStarPostSetup) && (millis() - dotStarSetup_stamp >= dotStarPostSetup_thresh));
        
        px.setPixelColor(0, DOTSTAR_MODECOLOR_GREEN, DOTSTAR_MODECOLOR_RED, DOTSTAR_MODECOLOR_BLUE); // Set the config-specific pixel colors
        px.show();                              // Refresh strip

        dotStarPostSetup = true;    // make sure the protothread never triggers again so its not eating up resources
        
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
        
        
        
        stringGrab_temp += input_string;
        
        
        uint16_t total_size = gps_size + other_size;
        for(int i = 0; i < total_size ; i++)
        {
          stringGrab_initial[i] = stringGrab_temp.charAt(i);    
        }
        //stringGrab_temp.toCharArray(stringGrab_initial,total_size);
    
//        for(int i = (total_size - 1); i >= 0 ; i--)
//        {
//          stringGrab_initial[i+6] = stringGrab_initial[i];
//        }
//    
//        uint8_t hundreds = 0;
//        uint8_t tens = 0;
//        uint16_t tempNum = gps_size;
//        while(tempNum - 100 >= 0)
//        {
//          tempNum -= 100;
//          hundreds++;
//        }
//        while(tempNum - 10 >= 0)
//        {
//          tempNum -= 10;
//          tens++;
//        }
//        stringGrab_initial[0] = hundreds;
//        stringGrab_initial[1] = tens;
//        stringGrab_initial[2] = (uint8_t)tempNum;
//    
//        hundreds = 0;
//        tens = 0;
//        tempNum = other_size;
//        while(tempNum - 100 >= 0)
//        {
//          tempNum -= 100;
//          hundreds++;
//        }
//        while(tempNum - 10 >= 0)
//        {
//          tempNum -= 10;
//          tens++;
//        }
//        stringGrab_initial[3] = hundreds;
//        stringGrab_initial[4] = tens;
//        stringGrab_initial[5] = (uint8_t)tempNum;
        Serial.print(F("gps_size = "));
        Serial.println(gps_size);
        Serial.print(F("other_size = "));
        Serial.println(other_size);
        Serial.print(F("total_size = "));
        Serial.println(total_size);
        Serial.println(F("stringGrab_temp : "));
        Serial.println(stringGrab_temp);
        Serial.println(F("stringGrab_initial : "));
        for(int i = 0; i < total_size; i++){
          Serial.print((char)stringGrab_initial[i]);
        }
        Serial.println(F(""));

        
        gps_size = 0;
        other_size = 0;
        
        matchCaseCompress(stringGrab_initial,total_size);
        uint16_t compress_size = doubleStuff(stringGrab_initial,stringGrab_compressed,total_size);
        usb.print(F("compressed size of doublestuffed gps + other : "));
        usb.println(compress_size);
        return compress_size;
      }
      else if(stringGrab_gps)
      {
        stringGrab_temp = input_string;
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
        #ifdef SENSORMODE_GONDOLA
        
          case 0xFF:                        // initiates a cutdown
            cutdownFunc();
            break;
        #endif
        
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
    //*******                                           parse_systemStats
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
      //digitalWrite(CUTDOWN, LOW);
      delay(10000);
      //digitalWrite(CUTDOWN, HIGH);
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
      
      px.begin(); // Initialize pins for output
      px.setPixelColor(0, DOTSTAR_MODECOLOR_GREEN, DOTSTAR_MODECOLOR_RED, DOTSTAR_MODECOLOR_BLUE); // Set the config-specific pixel colors
      px.show();  // apply to dotstar
      
      delay(5000);    // used to read setup debug code
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
      stringGrab_temp.reserve(700);
        
      usb.begin(115200);
      usb.println(F("INITIALIZING"));
    
      
      pinMode(sdPower, OUTPUT);     //
      //pinMode(gpsPower, OUTPUT);    //
      pinMode(imuPower, OUTPUT);    // Sets all power transistor gate pins and reset pins to OUTPUT mode
      pinMode(altPower, OUTPUT);    //
    //  pinMode(imuReset, OUTPUT);    //
    //  pinMode(CUTDOWN, OUTPUT);     // MCU cutdown no longer used
      
    //  digitalWrite(CUTDOWN, HIGH);   // ensures that the cutdown is not accidentally triggered during startup (HIGH ON THE CUTDOWN MEANS OFF ON THE CUTDOWN CIRCUIT)
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

      usb.println(F("begin imu stuff"));
      
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
      PT_INIT(&dotStarIMUsetupPT);  //
      PT_INIT(&dotStarPostSetupPT); //
      
      //PT_INIT(&rudderAnglePT);      // not implemented yet, used with Rudder Class
    
      usb.println(F("Setup complete, entering loop!"));
    
      getAltimeter();               // grabs the first elevation value, otherwise there will be two minutes of zeros
      gps_GetAlt("");               // for the altitude, even though the altimeter data is ready immediately
      dotStarSetup_stamp = millis();
      dotStarUpdate_stamp = millis();
      
      wdt_enable(16384);            // enable the watchdog timer
    }
    
    //*************************************************************************************************************
    //*******                                           LOOP
    //*************************************************************************************************************
    void loop() {
      xbeeCommand_sense(&xbeeCommandPT);
      
      stringGrabInit_sense(&stringGrabInitPT);
      
      stringChop_sense(&stringChopPT);
      
      xbee.protothreadLoop(); // enable watchdog timer
    //  usb.print(F("1,"));
    //  usb.println(freeMemory());

      getGPS_sense(&getGpsPT);
    //  usb.print(F("6,"));
    //  usb.println(freeMemory());
      stringGrabGPS_sense(&stringGrabGpsPT);
    
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
      dotStarIMUsetup_sense(&dotStarIMUsetupPT);

      dotStarPostSetupPT_sense(&dotStarPostSetupPT);
    
      #ifdef SENSORMODE_GONDOLA
      
        cutdownStart_sense(&cutdownStartPT);
    //  usb.print(F("14,"));
    //  usb.println(freeMemory());
        cutdownReset_sense(&cutdownResetPT);
    //  usb.println(F("15,"));
    //  usb.println(freeMemory());
        cutdown_sense(&cutdownPT);
        
      #endif
      
      wdt_reset();    // watchdog
    }




   #elif defined(SENSORMODE_RECEIVER)


  
/**************************************************************************************************************
 * *************************************************************************************************************
 * *************************************************************************************************************
 *                                        
 *                                     GONDOLA / SAIL CODE ABOVE HERE
 *                                        RECEIVER CODE BELOW HERE
 *                                        
 * *************************************************************************************************************                                       
 * *************************************************************************************************************
 * *************************************************************************************************************
 */
    #define debug_general
    
    
    #include <Wire.h>             // wire library (needed for some sensors)
    #include <SPI.h>              // SPI digital comms library (needed for SD card)
    #include <pt.h>               // protothread library
    #include <SD.h>               // SD library
    #include <Adafruit_Sensor.h>  // master sensor library
    #include "xbeeAPI.h"          // xbee library (900HP S3B models in API SPI mode)
    #include <Adafruit_DotStar.h> // controls LED on ItsyBitsyM4 MCU
    
    #define usb Serial            // renames Serial as usb
    
//    //Xbee1 OBJECT DEFINITION
//    #define XB1_SS (5)                          // M4 PIN FOR XBEE SPI SLAVE SELECT
//    #define XB1_ATTN (7)                        // M4 PIN FOR XBEE ATTN 
//    #define XB1_CMD (9)                         // M4 PIN NUMBER FOR COMMAND SIGNAL
//    #define XB1_LOC_COMMAND_DIO ('0')           // DIO NUMBER FOR LOCAL XBEE COMMAND PIN
//    #define XB1_LOC_CUTDOWN_DIO ('5')           // DIO NUMBER FOR LOCAL XBEE CUTDOWN PIN
//    #define XB1_DEST_COMMAND_DIO ('0')          // DIO NUMBER FOR DESTINATION COMMAND PIN
//    #define XB1_DEST_CUTDOWN_DIO ('5')          // DIO NUMBER FOR DESTINATION CUTDOWN 
//    #define XB1_SPI_CLK (3000000)               // SPI BUS FREQUENCY FOR XBEE DATA
//    #define XB1_PRIMARY_MODE (2)                // 0 = FAST, 1 = ACCURATE, 2 = RECEIVER
//    #define XB1_MAX_PAYLOAD_SIZE 256                 // cannot exceed NP (256)
//    Xbee xbee1 = Xbee(XB1_SS, XB1_ATTN);         // CONSTRUCTOR FOR XBEE OBJECT
//    uint16_t XB1_NP;                            // holds the value for AT property NP (max payload size)
//    bool packetGrab_init_1 = false;
//    bool packetGrab_decomp_1 = false;
//    bool packetGrab_decode_1 = false;
//
//    //Xbee2 OBJECT DEFINITION
//    #ifdef XB2_DEST_ADDR
//      #define XB2_SS (10)                          // M4 PIN FOR XBEE SPI SLAVE SELECT
//      #define XB2_ATTN (A3)                        // M4 PIN FOR XBEE ATTN 
//      #define XB2_CMD (A2)                         // M4 PIN NUMBER FOR COMMAND SIGNAL
//      #define XB2_LOC_COMMAND_DIO ('0')           // DIO NUMBER FOR LOCAL XBEE COMMAND PIN
//      #define XB2_LOC_CUTDOWN_DIO ('5')           // DIO NUMBER FOR LOCAL XBEE CUTDOWN PIN
//      #define XB2_DEST_COMMAND_DIO ('0')          // DIO NUMBER FOR DESTINATION COMMAND PIN
//      #define XB2_DEST_CUTDOWN_DIO ('5')          // DIO NUMBER FOR DESTINATION CUTDOWN 
//      #define XB2_SPI_CLK (3000000)               // SPI BUS FREQUENCY FOR XBEE DATA
//      #define XB2_PRIMARY_MODE (2)                // 0 = FAST, 1 = ACCURATE, 2 = RECEIVER
//      #define XB2_MAX_PAYLOAD_SIZE 256                 // cannot exceed NP (256)
//      Xbee xbee2 = Xbee(XB2_SS, XB2_ATTN);         // CONSTRUCTOR FOR XBEE OBJECT
//      uint16_t XB2_NP;                            // holds the value for AT property NP (max payload size)
//      bool packetGrab_init_2 = false;
//      bool packetGrab_decomp_2 = false;
//      bool packetGrab_decode_2 = false;
//    #endif
    
    //SPI communication pins, SCK, MISO, and MOSI are common to all SPI sensors, CS pins are unique to each 
    // SPI sensor.
    #define SD_CS 2
    #define sdPower 3      // SD power transistor gate pin

    // stuff that needs to be on in order for all of the i2c stuff to work.
    #define imuPower 12     // used on the imu power transistor gate for a hard reset
    #define altPower 11           //altimeter power transistor pin

    //PacketGrab required vars
    uint8_t packetGrab_decompressed[600]; // char array for compressed stringGrabdata
    uint8_t packetGrab_xbee;            // designates the xbee number where the packetGrab data came from 
    uint16_t decompressed_size;           // how long the packetGrab_decompressed array is

    // STRUCT DEFINITIONS USED TO STORE, ALIGN, AND PRINT DATA RECEIVED TO A HEADS UP DISPLAY IN THE 
    // SERIAL MONITOR
    #define NAME_SIZE 7            //
    #define AccGyroMagEul_SIZE 7   //
    #define TempHumid_SIZE 5       //
    #define PRESSURE_SIZE 9        //  Constants denoting the max byte size of each datafield
    #define PITOT_SIZE 8           //
    #define LAT_SIZE 11            //
    #define LON_SIZE 12            //
    #define ELEV_SIZE 8            //
    #define FLUSHTOTAL_SIZE 600    //
    #define FLUSHTOTAL_SIZE 600    //

    typedef struct    // struct XbeeSettings : used to set up each xbee
    {
      uint64_t destination_address;        // 64 BIT UNIIQUE DESTINATION XBEE ADDRESS
      uint8_t ss_pin;                 // PIN FOR XBEE SPI SLAVE SELECT
      uint8_t attn_pin;               // PIN FOR XBEE ATTN 
      uint8_t cmd_pin;                // PIN NUMBER FOR COMMAND SIGNAL
      //uint8_t cutdown_interrupt;      // PIN NUMBER FOR CUTDOWN INTERRUPT (NOT USED HERE)
      char local_cmd_dio;             // DIO NUMBER FOR LOCAL XBEE COMMAND PIN
      char local_cutdown_dio;         // DIO NUMBER FOR LOCAL XBEE CUTDOWN PIN
      char destination_cmd_dio;       // DIO NUMBER FOR DESTINATION COMMAND PIN
      char destination_cutdown_dio;   // DIO NUMBER FOR DESTINATION CUTDOWN 
      long spi_clk_freq_hz;           // SPI BUS FREQUENCY FOR XBEE DATA
      uint8_t primary_mode;           // 0 = FAST, 1 = ACCURATE, 2 = RECEIVER
      int max_payload_size_bytes;     // cannot exceed NP (256)
    }XbeeSettings;
    
    typedef struct  // struct ThreeAxis : used to store the x,y,z components of data
    {
      String x;   // string member used to store the x values of the struct
      String y;   // string member used to store the y values of the struct 
      String z;   // string member used to store the z values of the struct
    }ThreeAxis;
    
    typedef struct  // struct ThreeSpace : used to store the x,y,z components of alingment space
    {               //                      usually corresponds to a ThreeAxis struct
      uint8_t x;    // uint8_t member used to store the x values of the struct
      uint8_t y;    // uint8_t member used to store the x values of the struct
      uint8_t z;    // uint8_t member used to store the x values of the struct
    }ThreeSpace;

    typedef struct  // struct status buffer : used to display the latest status messages
    {
      String line1;
      String line2;
      String line3;
      String line4;
      String line5;
      String line6;
      String line7;
      String line8;
    }StatusBuffer;

    typedef struct  // struct status buffer : used to display the latest status messages
    {
      uint8_t line1;
      uint8_t line2;
      uint8_t line3;
      uint8_t line4;
      uint8_t line5;
      uint8_t line6;
      uint8_t line7;
      uint8_t line8;
    }StatBuffSpace;
    
    typedef struct  // struct thing : used to store all relevant info about the received data and how to 
                    //                print it in an aligned manner
    {
      String Name;                    // string member used to store the name of the struct
      String lat;                     // string member used to store the latitude data of the struct
      String lon;                     // string member used to store the longitude data of the struct
      String elev;                    // string member used to store the elevation data of the struct
      ThreeAxis acc;                  // ThreeAxix member used to store the acceleration data of the struct
      ThreeAxis gyro;                 // ThreeAxix member used to store the gyroscope data of the struct
      ThreeAxis mag;                  // ThreeAxix member used to store the magnetometer data of the struct
      ThreeAxis eul;                  // ThreeAxix member used to store the euler angle data of the struct
      String pressure;                // string member used to store the pressure data of the struct
      String temperature;             // string member used to store the temperature data of the struct
      String humidity;                // string member used to store the humidity data of the struct   
      String pitot;                   // string member used to store the pitot data of the struct
      StatusBuffer statBuff;           // used to hold the latest status messages
      String OtherflushTotal = "";    // string member for sensor data (temporary storage)
      String GPSflushTotal = "";      // string member for GPS data (temporary storage)
      String packetGrab_temp = "";    // used for storing the temp data prior to compressing and chopping
      int gga;                        // used for storing gps sentence existence
      int rmc;                        // used for storing gps sentence existence
      bool newGps;                    // used for storing a control bool for new gps sentences
      bool newOther;                  // used for storing a control bool for new othersensor strings
      bool cutdown_triggered;         // used for storing data about the state of the cutdown
      File OtherData;                 // SD file for sensor data
      File GPSdata;                   // SD file for GPS data
      uint8_t Name_space;             // space used to align name string in printThings()
      uint8_t lat_space;              // space used to align lat string in printThings()
      uint8_t lon_space;              // space used to align name string in printThings()
      uint8_t elev_space;             // space used to align name string in printThings()
      long cutdown_stamp;             // used to store millis() timestamp of last cutdown
      ThreeSpace acc_space;           // space used to align acc members in printThings()
      ThreeSpace gyro_space;          // space used to align gyro members in printThings()
      ThreeSpace mag_space;           // space used to align mag members in printThings()
      ThreeSpace eul_space;           // space used to align eul members in printThings()
      uint8_t pressure_space;         // space used to align pressure string in printThings()
      uint8_t temperature_space;      // space used to align temperature string in printThings()
      uint8_t humidity_space;         // space used to align humidity string in printThings()
      uint8_t pitot_space;            // space used to align pitot string in printThings()
      StatBuffSpace statSpace;        // space used to align all of the messages in the status buffer
      struct pt packetGrab_PT;        // protothread for the initial packetGrab
      struct pt packetParse_PT;       // protothread for the packetparsing function
      struct pt gpsParse_PT;          // protothread for parsing the GPS sentence and modifying struct members
      struct pt otherParse_PT;        // protothread for parsing the othersensor sentence and modifying struct members
      struct pt gpsWrite_PT;          // protothread for writing the GPS sentence to the appropriate SD file
      struct pt otherWrite_PT;        // protothread for writing the othersensor sentence to the appropriate SD file
      struct pt parseInput_PT;          // protothread for figuring out if new user input is for the struct, and how to handle it
    }thing;

    thing one;                // initialize the first struct
    XbeeSettings xbee_set1 =  // initialize the first xbeesettings struct for xbee1
    {
      XB1_DEST_ADDR,
      5,
      7,
      9,
      //0, CUTDOWN INTERRUPT
      '0',
      '5',
      '0',
      '5',
      3000000,
      2,
      256
    };
    uint16_t XB1_NP;
    Xbee xbee1 = Xbee(xbee_set1.ss_pin, xbee_set1.attn_pin);  // initialize the first xbee

    #ifdef XB2_DEST_ADDR            // if the 2nd xbee addr is fully described
      thing two;                    // initialize the second struct
      XbeeSettings xbee_set2 =      // initialize the second xbee settings struct
      {
        XB2_DEST_ADDR,
        10,
        A2,
        A3,
        //0, CUTDOWN INTERRUPT
        '0',
        '5',
        '0',
        '5',
        3000000,
        2,
        256
      };
      uint16_t XB2_NP;
      Xbee xbee2 = Xbee(xbee_set2.ss_pin, xbee_set2.attn_pin);  // initialize the second xbee
    #endif

    // ERROR DETECTION GLOBALS
    bool isError = false;         // is there an error?
    bool sdError = false;         // is there an SD-specific error?
    bool makeFileError = false;   // did makeFiles() fail?
    bool fatalError = false;      // is there a fatal error?
    bool printStarted = false;  

    String user_input;

    static struct pt printHud_PT;          // protothread for printing the HUD
    static struct pt userInput_PT;         // protothread for handling user input commands
    
    // TIMING THRESHOLD CONSTANTS FOR TIME-BASED EVENT HANDLING
    #define printHud_Thresh 1000
    #define cutdown_Thresh 300000
    
    // TIME_STAMPS FOR TIME-BASED EVENT HANDLING
    long errorFix_Stamp = 0;      // timestamp used to track when to call non-fatal error fix checks
    long printHud_Stamp = 0;

    // OTHERSTUFF
    int print_counter = 0;

    //*************************************************************************************************************
    //*******                                   PROTOTHREAD packetGrab_sense
    //*************************************************************************************************************
    static int packetGrab_sense(Xbee *xbee, thing *thisThing){
      PT_BEGIN(&(thisThing->packetGrab_PT));
      PT_WAIT_UNTIL(&(thisThing->packetGrab_PT), xbee->data_received_length);
        bool packetGrabbed;
        packetGrabbed = packetGrab(thisThing,xbee);
        if(packetGrabbed)
        {
          xbee->clearBuffers();
        }
        else
        {
          thisThing->packetGrab_temp = "";
        } 
      PT_END(&(thisThing->packetGrab_PT));
    }

    //*************************************************************************************************************
    //*******                                   PROTOTHREAD packetParse_sense
    //*************************************************************************************************************
    static int packetParse_sense(thing *thisThing){
      PT_BEGIN(&(thisThing->packetParse_PT));
      PT_WAIT_UNTIL(&(thisThing->packetParse_PT), thisThing->packetGrab_temp.length());
          thisThing->gga = thisThing->packetGrab_temp.indexOf("$GPGGA");  // search for the index of the argument
          thisThing->rmc = thisThing->packetGrab_temp.indexOf("$GPRMC");  // returns -1 if the argument can't be found
          if((thisThing->gga > -1) || (thisThing->rmc > -1))  // if either gga or rmc exists in the temp string,
          {
            thisThing->GPSflushTotal = thisThing->packetGrab_temp;  // store the sentence in the gpsflushtotal string
            thisThing->packetGrab_temp = "";      // empty the temp string to prevent the protothread from activating
            thisThing->newGps = true;
          }
          else
          {
            thisThing->OtherflushTotal = thisThing->packetGrab_temp;  // store the sentence in the otherflushtotal string
            thisThing->packetGrab_temp = "";      // empty the temp string to prevent the protothread from activating
            thisThing->newOther = true;
          } 
      PT_END(&(thisThing->packetParse_PT));
    }

    //*************************************************************************************************************
    //*******                                   PROTOTHREAD gpsParse_sense
    //*************************************************************************************************************
    static int gpsParse_sense(thing *thisThing){
      PT_BEGIN(&(thisThing->gpsParse_PT));
      PT_WAIT_UNTIL(&(thisThing->gpsParse_PT), thisThing->newGps);
        if(thisThing->rmc > -1)
        {
          uint8_t preTime = thisThing->GPSflushTotal.indexOf(",");
          uint8_t prethrowaway1 = thisThing->GPSflushTotal.indexOf("," , preTime+1);
          uint8_t prelat = thisThing->GPSflushTotal.indexOf("," , prethrowaway1+1);
          uint8_t prelatSuffix = thisThing->GPSflushTotal.indexOf("," , prelat+1);
          uint8_t prelon = thisThing->GPSflushTotal.indexOf("," , prelatSuffix+1);
          uint8_t prelonSuffix = thisThing->GPSflushTotal.indexOf("," , prelon+1);
          uint8_t postlonSuffix = thisThing->GPSflushTotal.indexOf("," , prelonSuffix+1);
        
          thisThing->lat = thisThing->GPSflushTotal.substring(prelat+1,prelon);
          thisThing->lon = thisThing->GPSflushTotal.substring(prelon+1,postlonSuffix);
        }
        else
        {
          uint8_t preTime = thisThing->GPSflushTotal.indexOf(",");
          uint8_t prelat = thisThing->GPSflushTotal.indexOf("," , preTime+1);
          uint8_t prelatSuffix = thisThing->GPSflushTotal.indexOf("," , prelat+1);
          uint8_t prelon = thisThing->GPSflushTotal.indexOf("," , prelatSuffix+1);
          uint8_t prelonSuffix = thisThing->GPSflushTotal.indexOf("," , prelon+1);
          uint8_t preFixData = thisThing->GPSflushTotal.indexOf("," , prelonSuffix+1);
          uint8_t prethrowaway1 = thisThing->GPSflushTotal.indexOf("," , preFixData+1);
          uint8_t prethrowaway2 = thisThing->GPSflushTotal.indexOf("," , prethrowaway1+1);
          uint8_t preElev = thisThing->GPSflushTotal.indexOf("," , prethrowaway2+1);
          uint8_t preElevSuffix = thisThing->GPSflushTotal.indexOf("," , preElev+1);
          uint8_t postElevSuffix = thisThing->GPSflushTotal.indexOf("," , preElevSuffix+1);
        
          thisThing->lat = thisThing->GPSflushTotal.substring(prelat+1,prelon);
          thisThing->lon = thisThing->GPSflushTotal.substring(prelon+1,preFixData);
          thisThing->elev = thisThing->GPSflushTotal.substring(preElev+1,postElevSuffix);
        }
        thisThing->newGps = false;
      PT_END(&(thisThing->gpsParse_PT));
    }

    //*************************************************************************************************************
    //*******                                   PROTOTHREAD otherParse_sense
    //*************************************************************************************************************
    static int otherParse_sense(thing *thisThing){
      PT_BEGIN(&(thisThing->otherParse_PT));
      PT_WAIT_UNTIL(&(thisThing->otherParse_PT), thisThing->newOther);
        uint8_t pretimeadd = thisThing->OtherflushTotal.indexOf(",");             // finds the locations on the 
        uint8_t preElev = thisThing->OtherflushTotal.indexOf("," , pretimeadd+1); // string for each datafield
        uint8_t preAccX = thisThing->OtherflushTotal.indexOf(":");                //
        uint8_t preAccY = thisThing->OtherflushTotal.indexOf("," , preAccX+1);    //
        uint8_t preAccZ = thisThing->OtherflushTotal.indexOf("," , preAccY+1);    //
                                                                                  //
        uint8_t preGyroX = thisThing->OtherflushTotal.indexOf("," , preAccZ+1);   //
        uint8_t preGyroY = thisThing->OtherflushTotal.indexOf("," , preGyroX+1);  //
        uint8_t preGyroZ = thisThing->OtherflushTotal.indexOf("," , preGyroY+1);  //
                                                                                  //
        uint8_t preMagX = thisThing->OtherflushTotal.indexOf("," , preGyroZ+1);   //
        uint8_t preMagY = thisThing->OtherflushTotal.indexOf("," , preMagX+1);    //
        uint8_t preMagZ = thisThing->OtherflushTotal.indexOf("," , preMagY+1);    //
                                                                                  //
        uint8_t preEulX = thisThing->OtherflushTotal.indexOf("," , preMagZ+1);    //
        uint8_t preEulY = thisThing->OtherflushTotal.indexOf("," , preEulX+1);    //
        uint8_t preEulZ = thisThing->OtherflushTotal.indexOf("," , preEulY+1);    //
                                                                                  //
        uint8_t preGravX = thisThing->OtherflushTotal.indexOf("," , preEulZ+1);   //
        uint8_t preGravY = thisThing->OtherflushTotal.indexOf("," , preGravX+1);  //
        uint8_t preGravZ = thisThing->OtherflushTotal.indexOf("," , preGravY+1);  //
                                                                                  //
        uint8_t preTemp = thisThing->OtherflushTotal.indexOf("," , preGravZ+1);   //
        uint8_t prePressure = thisThing->OtherflushTotal.indexOf("," , preTemp+1);  //
        uint8_t preHumidity = thisThing->OtherflushTotal.indexOf("," , prePressure+1);  //
        uint8_t preAltAltitude = thisThing->OtherflushTotal.indexOf("," , preHumidity+1); //
        uint8_t prePitot = thisThing->OtherflushTotal.indexOf("," , preAltAltitude+1);  //
        uint8_t postPitot = thisThing->OtherflushTotal.indexOf("--");                   //
      
          // parses up the datafields and assigns them to the corresponding struct members
        thisThing->acc.x = thisThing->OtherflushTotal.substring(preAccX+1,preAccY);
        thisThing->acc.y = thisThing->OtherflushTotal.substring(preAccY+1,preAccZ);
        thisThing->acc.z = thisThing->OtherflushTotal.substring(preAccZ+1,preGyroX);
      
        thisThing->gyro.x = thisThing->OtherflushTotal.substring(preGyroX+1,preGyroY);
        thisThing->gyro.y = thisThing->OtherflushTotal.substring(preGyroY+1,preGyroZ);
        thisThing->gyro.z = thisThing->OtherflushTotal.substring(preGyroZ+1,preMagX);
      
        thisThing->mag.x = thisThing->OtherflushTotal.substring(preMagX+1,preMagY);
        thisThing->mag.y = thisThing->OtherflushTotal.substring(preMagY+1,preMagZ);
        thisThing->mag.z = thisThing->OtherflushTotal.substring(preMagZ+1,preEulX);
      
        thisThing->eul.x = thisThing->OtherflushTotal.substring(preEulX+1,preEulY);
        thisThing->eul.y = thisThing->OtherflushTotal.substring(preEulY+1,preEulZ);
        thisThing->eul.z = thisThing->OtherflushTotal.substring(preEulZ+1,preTemp);
      
        thisThing->pressure = thisThing->OtherflushTotal.substring(prePressure+1,preHumidity);
        thisThing->temperature = thisThing->OtherflushTotal.substring(preTemp+1,prePressure);
        thisThing->humidity = thisThing->OtherflushTotal.substring(preHumidity+1,preAltAltitude);
        thisThing->pitot = thisThing->OtherflushTotal.substring(prePitot+1,postPitot);
        
        thisThing->newOther = false;  // resets the newOther flag to prevent retriggering the protothread
      PT_END(&(thisThing->otherParse_PT));
    }

    //*************************************************************************************************************
    //*******                                   PROTOTHREAD gpsWrite_sense
    //*************************************************************************************************************
    static int gpsWrite_sense(thing *thisThing){
      PT_BEGIN(&(thisThing->gpsWrite_PT));
      PT_WAIT_UNTIL(&(thisThing->gpsWrite_PT), thisThing->newGps);
        thisThing->GPSdata.print(thisThing->GPSflushTotal);
        thisThing->GPSdata.flush();   
      PT_END(&(thisThing->gpsWrite_PT));
    }

    //*************************************************************************************************************
    //*******                                   PROTOTHREAD otherWrite_sense
    //*************************************************************************************************************
    static int otherWrite_sense(thing *thisThing){
      PT_BEGIN(&(thisThing->otherWrite_PT));
      PT_WAIT_UNTIL(&(thisThing->otherWrite_PT), thisThing->newOther);
        thisThing->OtherData.print(thisThing->OtherflushTotal);
        thisThing->OtherData.flush();   
      PT_END(&(thisThing->otherWrite_PT));
    }

    //*************************************************************************************************************
    //*******                                   PROTOTHREAD HUD_sense
    //*************************************************************************************************************
    static int HUD_sense(struct pt *pt){ // prints the heads up display
      PT_BEGIN(pt);
      PT_WAIT_UNTIL(pt, printStarted || (millis() - printHud_Stamp >= printHud_Thresh));
        #ifndef XB2_DEST_ADDR
          calcSpace(&one);
          printThings(&one);
        #else
          calcSpace(&one);
          calcSpace(&two);
          printThings(&one,&two);
        #endif
        printHud_Stamp = millis();
      PT_END(pt);
    }

    //*************************************************************************************************************
    //*******                                   PROTOTHREAD userInput_sense
    //*************************************************************************************************************
    static int userInput_sense(struct pt *pt){ // prints the heads up display
      PT_BEGIN(pt);
      PT_WAIT_UNTIL(pt,usb.available());
        user_input = "";
        while(usb.available())
        {
          user_input += (char)usb.read();
        }
      PT_END(pt);
    }

    //*************************************************************************************************************
    //*******                                   PROTOTHREAD parseInput_sense
    //*************************************************************************************************************
    static int parseInput_sense(Xbee *xbee, thing *thisThing, XbeeSettings *theseSettings){
      PT_BEGIN(&(thisThing->parseInput_PT));
      PT_WAIT_UNTIL(&(thisThing->parseInput_PT), user_input.length() || (thisThing->cutdown_triggered && ((millis() - thisThing->cutdown_stamp) >= cutdown_Thresh)));
      
      if(thisThing -> cutdown_triggered)
      {
        uint8_t cutdown_cmd[20] = {0x7E,0X00,0X10,0X17,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0XFE,0X02,0X44,0X00,0X00,0X00};
        
        uint8_t address[8];   // convert the 64-bit address field into an 8-bit array
        uint32_t upperAddy = (uint32_t)((theseSettings->destination_address) >> 32);
        uint32_t lowerAddy = (uint32_t)(theseSettings->destination_address);
        for(int i = 3; i >=0; i--)          
        {
          uint32_t bitmask = 0b11111111 << ((3-i)*8);
          address[i+4] = (uint8_t)(lowerAddy & bitmask)>>((3-i)*8);
          address[i] = (uint8_t)((upperAddy & bitmask)>>((3-i)*8));
        }
        
        for(int i = 0; i < 8; i++)    // apply the 8-bit array of the address to cutdown_cmd
        {
          cutdown_cmd[i+5] = address[i];
        }
  
        cutdown_cmd[17] = theseSettings->destination_cutdown_dio; // this sets the dio pin to be manipulated
        cutdown_cmd[18] = 0x05;                                   // 0x05 means pin set HIGH, CUTDOWN = OFF
        
        uint8_t chksum = 0; // calculate the checksum 
        for(int i = 3; i < 19; i++) // for the subframe starting after {0x7E,0x00,0x10} and ending before the final element,
        {
          chksum += cutdown_cmd[i];
        }
        cutdown_cmd[19] = 0xFF - chksum;  // assign the checksum value to the final byte of the cutdown_cmd array
  
        xbee->sendFrame(cutdown_cmd,20);  // send the frame to the other end
        thisThing->cutdown_triggered = false;
        user_input = "";
        addStat((String)("ctdwn"),thisThing,thisThing->cutdown_triggered);
      }
      else
      {
        if((user_input.indexOf(thisThing->Name))>-1)
        {
          String command = user_input.substring(user_input.indexOf(":")+1,user_input.length()-1);
          if(command.equalsIgnoreCase("CUTDOWN"))
          {
            uint8_t cutdown_cmd[20] = {0x7E,0X00,0X10,0X17,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0XFE,0X02,0X44,0X00,0X00,0X00};
            
            uint8_t address[8];   // convert the 64-bit address field into an 8-bit array
            uint32_t upperAddy = (uint32_t)((theseSettings->destination_address) >> 32);
            uint32_t lowerAddy = (uint32_t)(theseSettings->destination_address);
            for(int i = 3; i >=0; i--)          
            {
              uint32_t bitmask = 0b11111111 << ((3-i)*8);
              address[i+4] = (uint8_t)(lowerAddy & bitmask)>>((3-i)*8);
              address[i] = (uint8_t)((upperAddy & bitmask)>>((3-i)*8));
            }
            
            for(int i = 0; i < 8; i++)    // apply the 8-bit array of the address to cutdown_cmd
            {
              cutdown_cmd[i+5] = address[i];
            }
  
            cutdown_cmd[17] = theseSettings->destination_cutdown_dio; // this sets the dio pin to be manipulated
            cutdown_cmd[18] = 0x04;
            
            uint8_t chksum = 0; // calculate the checksum 
            for(int i = 3; i < 19; i++) // for the subframe starting after {0x7E,0x00,0x10} and ending before the final element,
            {
              chksum += cutdown_cmd[i];
            }
            cutdown_cmd[19] = 0xFF - chksum;  // assign the checksum value to the final byte of the cutdown_cmd array

            xbee->sendFrame(cutdown_cmd,20);  // send the frame to the other end
            thisThing->cutdown_triggered = true;
            thisThing->cutdown_stamp = millis();
            user_input = "";
            addStat((String)("ctdwn"),thisThing,thisThing->cutdown_triggered);
          }
          else
          {
            String command = user_input.substring(user_input.indexOf(":")+1,user_input.length()-1);
            if(command.length() == 2)
            {
              uint8_t type = command.substring(0,1).toInt();
              uint8_t val = command.substring(1).toInt();
          
              long command_stamp = millis();
              bool command_state = false;
              while((!command_state) && (millis() - command_stamp < 2000))
              {
                xbee->sendCommand(type,val);
              }
              addStat(thisThing,type,val,command_state);
              user_input = "";  
            }
            else
            {
              addStat(thisThing,command,(String)("not cmd"));
              user_input = "";
            }
          }
        }
      }
      PT_END(&(thisThing->parseInput_PT));
    }


    //*************************************************************************************************************
    //*******                                           packetGrab
    //*************************************************************************************************************
    bool packetGrab(thing *theThing, Xbee *xbee)
    {
      usb.println(F("packetGrab:called"));
      bool concat_bool;           // used to see if the concat() function worked
      bool rtn = true;            // used to tell if the function worked
      uint8_t restart_count;      // used to track the number of attempted concat() attempts 
                                                                  // pass the respective xbee buffer and bufferlength
                                                                 // to the decompression function
      decompressed_size = decompress(xbee->data_received, packetGrab_decompressed, xbee->data_received_length);
      matchCaseDecompress(packetGrab_decompressed, decompressed_size);  // then decode to relevant data streams
      
      for(int i = 0; i < decompressed_size; i++)
      {
        restart_count = 0;        // reset the restart counter
        concat_bool = false;      // reset the concat_bool to ensure the while loop works
        while((!concat_bool) && (restart_count < 251))    // kicks out if the concat worked or the max concat
        {                                                 // attempts is reached
          concat_bool = theThing->packetGrab_temp.concat(packetGrab_decompressed[i]);
          restart_count++;
        }
        rtn &= concat_bool; // bitwise AND rtn and concat_bool to accumulate any error into a function error
      }
      return rtn;
    }
    
    //*************************************************************************************************************
    //*******                                           MakeFiles
    //*************************************************************************************************************
    int makeFiles(thing *thisThing){
      //usb.println(F("entering makeFiles()"));// 
      bool rtn = true;                          // set a boolean flag of rtn to be true
    
    // commented out, filenames set to global vars  ***********************************
      char gpsfile[20];                         // create a pointer for a character string of 15 for gps string
      char otherdatafile[20];                   // create a pointer for a character string of 15 for other data strings

      String gpsFilename = thisThing->Name;
      String otherFilename = thisThing->Name;

      uint8_t countdown = 255;
      bool concat_worked = false;
      while((!concat_worked) && countdown)
      {
        concat_worked = gpsFilename.concat("_GPSLOG05.TXT");
      }
      if(concat_worked){
        countdown = 255;
        concat_worked = false;
        while((!concat_worked) && countdown)
        {
          concat_worked = otherFilename.concat("_AODATA05.TXT");
        }
        if(!concat_worked)
        {
          return false;
        }
      }
      else
      {
        return false;
      }

      uint8_t gpsIndex = thisThing->Name.length() + 7;
      uint8_t otherIndex = thisThing->Name.length() + 7;

      for(int i = 0; i < 20; i++)
      {
        gpsfile[i] = gpsFilename.charAt(i);
        otherdatafile[i] = otherFilename.charAt(i);
      }
      
//      strcpy(gpsfile, gpsFilename.c_str());          // set the string name to the pointer
//      strcpy(otherdatafile, otherFilename.c_str());    // set the string name to the pointer
      
      //usb.println(F("made it through filename initialization stuffs"));                                          
                                                //
      for(uint8_t i = 0; i < 100; i++)          // for the gps file 
      {                                         // for all numbers from 0 to 100
        gpsfile[gpsIndex] = '0' + i/10;                // replace the 6th character with a 0 plus the 1st digit of the division of i and 10
        gpsfile[gpsIndex+1] = '0' + i%10;                // replace the 7th character with a 0 plus the 2nd digit of the remainder of the division of i and 10
        if(!SD.exists(gpsfile))                 // if that new file name DOES NOT EXIST within the sd card
        {                                       //
          break;                                // then get out of the loop because you have found a new file name which doesn't exist
        }                                       //
      }                                         // otherwise, loop back again because you cannot overwrite files
      
      //usb.println(F("made it through gpsfile suffix search"));                                          
                                                //
      for(uint8_t i = 0; i < 100; i++)          // for the gps file 
      {                                         // for all numbers from 0 to 100
        otherdatafile[otherIndex] = '0' + i/10;          // replace the 6th character with a 0 plus the 1st digit of the division of i and 10
        otherdatafile[otherIndex+1] = '0' + i%10;          // replace the 7th character with a 0 plus the 2nd digit of the remainder of the division of i and 10
        if(!SD.exists(otherdatafile))           // if that new file name DOES NOT EXIST within the sd card
        {                                       //
          break;                                // then get out of the loop because you have found a new file name which doesn't exist
        }                                       //
      }                                         // otherwise, loop back again because you cannot overwrite files
      
      //usb.println(F("made it through otherdatafile suffix search"));
                                                //
      thisThing->GPSdata = SD.open(gpsfile, FILE_WRITE);   // write a new file for the gps file on the SD card
      if(!(thisThing->GPSdata))                              // if the file failed to be created
      {                                         // 
        usb.print(F("Failed to create "));         // display failed to create on the serial monitor
        usb.println(gpsfile);                   // followed by the filename
        rtn = false;                            // set the boolean flag rtn to false
      }                                         // end
      else{
        //usb.println(F("made it through gps file creation")); 
      }
                                                //        
      thisThing->OtherData = SD.open(otherdatafile, FILE_WRITE); // write a new file for the gps file on the SD card
      if(!(thisThing->OtherData))                                // if the file failed to be created
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
    //*******                              DATA COMPRESSION CODE
    //*************************************************************************************************************
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
    //*******                                           setupXbee
    //*************************************************************************************************************
    void setupXbee(Xbee *this_xbee, XbeeSettings *these_settings)
    {
      this_xbee->setSPI_clockFreq(these_settings->spi_clk_freq_hz);
      this_xbee->setSPI_bitOrder(MSBFIRST);
      this_xbee->setSPI_mode(SPI_MODE0);
      this_xbee->setMode(these_settings->primary_mode);
      this_xbee->setDestinationAddress(these_settings->destination_address);
      this_xbee->setCommandInterruptPin(these_settings->cmd_pin);
      this_xbee->setLocalCommand_DIO(these_settings->local_cmd_dio);
      //this_xbee.setCutdownInterruptPin(these_settings->cutdown_interrupt);      // use if needed
      this_xbee->setLocalCutdown_DIO(these_settings->local_cutdown_dio);
      this_xbee->setDestinationCommand_DIO(these_settings->destination_cmd_dio);
      this_xbee->setDestinationCutdown_DIO(these_settings->destination_cutdown_dio);
      this_xbee->setMaxPayloadSize(these_settings->max_payload_size_bytes);
      
//      xbee1.setSPI_clockFreq(XB1_SPI_CLK);
//      xbee1.setSPI_bitOrder(MSBFIRST);
//      xbee1.setSPI_mode(SPI_MODE0);
//      xbee1.setMode(XB1_PRIMARY_MODE);
//      xbee1.setDestinationAddress(XB1_DEST_ADDR);
//      xbee1.setCommandInterruptPin(XB1_CMD);
//      xbee1.setLocalCommand_DIO(XB1_LOC_COMMAND_DIO);
//      //xbee.setCutdownInterruptPin();                      // use if needed
//      xbee1.setLocalCutdown_DIO(XB1_LOC_CUTDOWN_DIO);
//      xbee1.setDestinationCommand_DIO(XB1_DEST_COMMAND_DIO);
//      xbee1.setDestinationCutdown_DIO(XB1_DEST_CUTDOWN_DIO);
//      xbee1.setMaxPayloadSize(XB1_MAX_PAYLOAD_SIZE);
//
//      #ifdef XB2_DEST_ADDR
//        xbee2.setSPI_clockFreq(XB2_SPI_CLK);
//        xbee2.setSPI_bitOrder(MSBFIRST);
//        xbee2.setSPI_mode(SPI_MODE0);
//        xbee2.setMode(XB2_PRIMARY_MODE);
//        xbee2.setDestinationAddress(XB2_DEST_ADDR);
//        xbee2.setCommandInterruptPin(XB2_CMD);
//        xbee2.setLocalCommand_DIO(XB2_LOC_COMMAND_DIO);
//        //xbee.setCutdownInterruptPin();                      // use if needed
//        xbee2.setLocalCutdown_DIO(XB2_LOC_CUTDOWN_DIO);
//        xbee2.setDestinationCommand_DIO(XB2_DEST_COMMAND_DIO);
//        xbee2.setDestinationCutdown_DIO(XB2_DEST_CUTDOWN_DIO);
//        xbee2.setMaxPayloadSize(XB2_MAX_PAYLOAD_SIZE);
//      #endif
    }

    //*************************************************************************************************************
    //*******                                    struct Thing reserveMemory
    //*************************************************************************************************************
    void reserveMemory(thing *theThing)
    {                                     // RESERVES SPACE FOR ALL THE STRINGS SO THEY DONT DYNAMICALLY BLOW THINGS UP
      theThing->Name.reserve(NAME_SIZE);
      theThing->lat.reserve(LAT_SIZE);
      theThing->lon.reserve(LON_SIZE);
      theThing->elev.reserve(ELEV_SIZE);
      
      theThing->acc.x.reserve(AccGyroMagEul_SIZE);
      theThing->acc.y.reserve(AccGyroMagEul_SIZE);
      theThing->acc.z.reserve(AccGyroMagEul_SIZE);
    
      theThing->gyro.x.reserve(AccGyroMagEul_SIZE);
      theThing->gyro.y.reserve(AccGyroMagEul_SIZE);
      theThing->gyro.z.reserve(AccGyroMagEul_SIZE);
    
      theThing->mag.x.reserve(AccGyroMagEul_SIZE);
      theThing->mag.y.reserve(AccGyroMagEul_SIZE);
      theThing->mag.z.reserve(AccGyroMagEul_SIZE);
    
      theThing->eul.x.reserve(AccGyroMagEul_SIZE);
      theThing->eul.y.reserve(AccGyroMagEul_SIZE);
      theThing->eul.z.reserve(AccGyroMagEul_SIZE);
    
      theThing->pressure.reserve(PRESSURE_SIZE);
      theThing->temperature.reserve(TempHumid_SIZE);
      theThing->humidity.reserve(TempHumid_SIZE);
      theThing->pitot.reserve(PITOT_SIZE);

      theThing->statBuff.line1.reserve(39);
      theThing->statBuff.line2.reserve(39);
      theThing->statBuff.line3.reserve(39);
      theThing->statBuff.line4.reserve(39);
      theThing->statBuff.line5.reserve(39);
      theThing->statBuff.line6.reserve(39);
      theThing->statBuff.line7.reserve(39);
      theThing->statBuff.line8.reserve(39);

      theThing->OtherflushTotal.reserve(FLUSHTOTAL_SIZE);
      theThing->GPSflushTotal.reserve(FLUSHTOTAL_SIZE);
      theThing->packetGrab_temp.reserve(FLUSHTOTAL_SIZE);  
    }

    //*************************************************************************************************************
    //*******                                    struct Thing calcspace
    //*************************************************************************************************************
    void calcSpace(thing *theThing)
    {
      //print_counter++;
      //if(print_counter > 30)
      //{
       // print_counter = 1;
      //}
      
        
        //2:
          theThing->Name_space = (39 - theThing->Name.length()) / 2;      // calculates the additional space needed
          //break;

        //5:
          theThing->lat_space = LAT_SIZE - theThing->lat.length();        // in order for strings of varying sizes to 
          //break;

        //6:      
          theThing->lon_space = LON_SIZE - theThing->lon.length();        // be printed in an aligned format in
          //break;

        //7:          
          theThing->elev_space = ELEV_SIZE - theThing->elev.length();     // printThings()
          //break;

        //10:          
          theThing->acc_space.x = AccGyroMagEul_SIZE - theThing->acc.x.length();
          theThing->gyro_space.x = AccGyroMagEul_SIZE - theThing->gyro.x.length();
          theThing->mag_space.x = AccGyroMagEul_SIZE - theThing->mag.x.length();
          //break;

        //11:          
          theThing->acc_space.y = AccGyroMagEul_SIZE - theThing->acc.y.length();
          theThing->gyro_space.y = AccGyroMagEul_SIZE - theThing->gyro.y.length();
          theThing->mag_space.y = AccGyroMagEul_SIZE - theThing->mag.y.length();
         // break;

        //12:          
          theThing->acc_space.z = AccGyroMagEul_SIZE - theThing->acc.z.length();
          theThing->gyro_space.z = AccGyroMagEul_SIZE - theThing->gyro.z.length();
          theThing->mag_space.z = AccGyroMagEul_SIZE - theThing->mag.z.length();
          //break;

        //14:
          theThing->pressure_space = PRESSURE_SIZE - theThing->pressure.length();
          //break;

        //15:
          theThing->eul_space.x = AccGyroMagEul_SIZE - theThing->eul.x.length();
          theThing->temperature_space = TempHumid_SIZE - theThing->temperature.length();
         //break;

        //16:          
          theThing->eul_space.y = AccGyroMagEul_SIZE - theThing->eul.y.length();
          theThing->humidity_space = TempHumid_SIZE - theThing->humidity.length();
         // break;

        //17:          
          theThing->eul_space.z = AccGyroMagEul_SIZE - theThing->eul.z.length();
          theThing->pitot_space = PITOT_SIZE - theThing->pitot.length();
          //break;

        //21:          
          theThing->statSpace.line1 = 39 - theThing->statBuff.line1.length();
          //break;

        //22:          
          theThing->statSpace.line2 = 39 - theThing->statBuff.line2.length();
          //break;

        //23:          
          theThing->statSpace.line3 = 39 - theThing->statBuff.line3.length();
          //break;

        //24:          
          theThing->statSpace.line4 = 39 - theThing->statBuff.line4.length();
          //break;

        //25:          
          theThing->statSpace.line5 = 39 - theThing->statBuff.line5.length();
          //break;

        //26:          
          theThing->statSpace.line6 = 39 - theThing->statBuff.line6.length();
          //break;

        //27:          
          theThing->statSpace.line7 = 39 - theThing->statBuff.line7.length();
          //break;

        //28:          
          theThing->statSpace.line8 = 39 - theThing->statBuff.line8.length();
          //break;

        //default:
          //break;
      //};
    }

    //*************************************************************************************************************
    //*******                                 struct Thing printThings - 1 thing
    //*************************************************************************************************************
    void printThings(thing *theThing)
    {
        for(int i = 0; i < 50; i++)   //
        {                             //
          usb.println(F(""));         // clear the print area
        }                             //
    
        // HEADER SETUP WTIH NAMES OF THINGS
        usb.println(F("|||||||||||||||||||||||||||||||||||||||||"));
        uint8_t Name_spaceUsed = theThing->Name_space + theThing->Name.length();
        while(theThing->Name_space > 0)
        {
          usb.print(F(" "));
          theThing->Name_space--;
        }
        usb.print(theThing->Name);
        Name_spaceUsed = 39 - Name_spaceUsed;
        while(Name_spaceUsed > 0)
        {
          usb.print(F(" "));
        }
        usb.println(F("|| "));
        usb.println(F("***************************************||"));
        usb.println(F("                                       ||"));
        
      
        // LINE ONE : LATITUDE
        usb.print(F("Lat:  "));       // print the label
        while(theThing->lat_space > 0)      // print the required space to stay aligned
        {                             //
          usb.print(F(" "));          //
          theThing->lat_space--;            //
        }                             //
        usb.print(theThing->lat); usb.println(F("                      || "));  // print the rest of the line
      
        
      
        // LINE TWO : LONGITUDE
        usb.print(F("Lon: "));        // print the label
        while(theThing->lon_space > 0)      // print the required space to stay aligned
        {                             //
          usb.print(F(" "));          //
          theThing->lon_space--;            //
        }                             //
        usb.print(theThing->lon); usb.println(F("                      || "));  // print the rest of the line
      
        // LINE THREE : ELEVATION
        usb.print(F("Elev:    "));    // print the label
        while(theThing->elev_space > 0)     // print the required space to stay aligned
        {                             //
          usb.print(F(" "));          //
          theThing->elev_space--;           //
        }                             //
        usb.print(theThing->elev); usb.println(F("                      || "));  // print the rest of the line
      
        // LINE FOUR : BLANK
        usb.println(F("                                       ||"));
      
        // LINE FIVE : ACC/GYRO/MAG LABELS
        usb.println(F("ACC           GYRO          MAG        ||"));
      
        // LINE SIX : ACC/GYRO/MAG X VALUES
        usb.print(F("X: "));          // print the label                          //
        while(theThing->acc_space.x > 0)    // print the required space to stay aligned //
        {                             //                                          // ACC.X
          usb.print(F(" "));          //                                          //
          theThing->acc_space.x--;          //                                          //
        }                             //                                          //
        usb.print(theThing->acc.x);         // print the value                          //
      
        usb.print(F("    X: "));         // print the label                          //
        while(theThing->gyro_space.x > 0)   // print the required space to stay aligned //
        {                             //                                          // GYRO.X
          usb.print(F(" "));          //                                          //
          theThing->gyro_space.x--;         //                                          //
        }                             //                                          //
        usb.print(theThing->gyro.x);        // print the value                          //
      
        usb.print(F("    X: "));         // print the label                          //
        while(theThing->mag_space.x > 0)    // print the required space to stay aligned //
        {                             //                                          // MAG.X
          usb.print(F(" "));          //                                          //
          theThing->mag_space.x--;          //                                          //
        }                             //                                          //
        usb.print(theThing->mag.x);         // print the value                          //
        usb.println(F(" || "));       // print the rest of the line               //
      
      
        // LINE SEVEN : ACC/GYRO/MAG Y VALUES
        usb.print(F("Y: "));          // print the label                          //
        while(theThing->acc_space.y > 0)    // print the required space to stay aligned //
        {                             //                                          // ACC.Y
          usb.print(F(" "));          //                                          //
          theThing->acc_space.y--;          //                                          //
        }                             //                                          //
        usb.print(theThing->acc.y);         // print the value                          //
      
        usb.print(F("    Y: "));         // print the label                          //
        while(theThing->gyro_space.y > 0)   // print the required space to stay aligned //
        {                             //                                          // GYRO.Y
          usb.print(F(" "));          //                                          //
          theThing->gyro_space.y--;         //                                          //
        }                             //                                          //
        usb.print(theThing->gyro.y);        // print the value                          //
      
        usb.print(F("    Y: "));         // print the label                          //
        while(theThing->mag_space.y > 0)    // print the required space to stay aligned //
        {                             //                                          // MAG.Y
          usb.print(F(" "));          //                                          //
          theThing->mag_space.y--;          //                                          //
        }                             //                                          //
        usb.print(theThing->mag.y);         // print the value                          //
        usb.println(F(" || "));       // print the rest of the line               //
      
      
        // LINE EIGHT : ACC/GYRO/MAG Z VALUES
        usb.print(F("Z: "));          // print the label                          //
        while(theThing->acc_space.z > 0)    // print the required space to stay aligned //
        {                             //                                          // ACC.Z
          usb.print(F(" "));          //                                          //
          theThing->acc_space.z--;          //                                          //
        }                             //                                          //
        usb.print(theThing->acc.z);         // print the value                          //
      
        usb.print(F("    Z: "));         // print the label                          //
        while(theThing->gyro_space.z > 0)   // print the required space to stay aligned //
        {                             //                                          // GYRO.Z
          usb.print(F(" "));          //                                          //
          theThing->gyro_space.z--;         //                                          //
        }                             //                                          //
        usb.print(theThing->gyro.z);        // print the value                          //
      
        usb.print(F("    Z: "));         // print the label                          //
        while(theThing->mag_space.z > 0)    // print the required space to stay aligned //
        {                             //                                          // MAG.Z
          usb.print(F(" "));          //                                          //
          theThing->mag_space.z--;          //                                          //
        }                             //                                          //
        usb.print(theThing->mag.z);         // print the value                          //
        usb.println(F(" || "));       // print the rest of the line               //
      
      
        // LINE NINE : BLANK
        usb.println(F("                                       ||"));
      
      
        // LINE TEN : EULER ANGLE LABEL AND PRESSURE
        usb.print(F("EUL              PRESS: "));   // print the labels
        while(theThing->pressure_space > 0)         // print the required space for alignment
        {                                     //
          usb.print(F(" "));                  //
          theThing->pressure_space--;               //
        }                                     //
        usb.print(theThing->pressure);              // print the value
        usb.println(F("      || "));            // print the rest of the line
      
        // LINE ELEVEN : EULER ANGLE X AND TEMP
        usb.print(F("X: "));   // print the labels
        while(theThing->eul_space.x > 0)            // print the required space for alignment
        {                                     //
          usb.print(F(" "));                  //
          theThing->eul_space.x--;                  //
        }                                     //
        usb.print(theThing->eul.x);                 // print the value
        usb.print(F("        TEMP:     "));        // print the next label
        while(theThing->temperature_space > 0)      // print the required space for alignment
        {                                     //
          usb.print(F(" "));                  //
          theThing->temperature_space--;            //
        }                                     //
        usb.print(theThing->temperature);           // print the value
        usb.println(F("      || "));            // print the rest of the line
      
      
        // LINE TWELVE : EULER ANGLE Y AND HUMIDITY
        usb.print(F("Y: "));   // print the labels
        while(theThing->eul_space.y > 0)            // print the required space for alignment
        {                                     //
          usb.print(F(" "));                  //
          theThing->eul_space.y--;                  //
        }                                     //
        usb.print(theThing->eul.y);                 // print the value
        usb.print(F("       HUMID:     "));        // print the next label
        while(theThing->humidity_space > 0)         // print the required space for alignment
        {                                     //
          usb.print(F(" "));                  //
          theThing->humidity_space--;
        }                                     //
        usb.print(theThing->humidity);              // print the value
        usb.println(F("      || "));            // print the rest of the line
      
        // LINE THIRTEEN : EULER ANGLE Z AND PITOT READING
        usb.print(F("Z: "));   // print the labels
        while(theThing->eul_space.z > 0)            // print the required space for alignment
        {                                     //
          usb.print(F(" "));                  //
          theThing->eul_space.z--;                  //
        }                                     //
        usb.print(theThing->eul.z);                 // print the value
        usb.print(F("       PITOT:  "));           // print the next label
        while(theThing->pitot_space > 0)            // print the required space for alignment
        {                                     //
          usb.print(F(" "));                  //
          theThing->pitot_space--;                  //
        }                                     //
        usb.print(theThing->pitot);                 // print the value
        usb.println(F("      || "));            // print the rest of the line


        // LINE FOURTEEN : DIVIDER BETWEEEN HUD AND THE STATUS BOX
        usb.println(F("                                       ||"));
        usb.println(F("***************************************||"));
        usb.println(F("                                       ||"));

        // LINE FIFTHTEEN : LINE 1 OF THE STATUS BOX
        usb.print(theThing->statBuff.line1);          // print the line
        for(int i = 0; i < theThing->statSpace.line1; i++)  // print the alignment spaces
        {
          usb.print(F(" "));
        }
        usb.println(F("|| "));                        // print the end frame

        // LINE SIXTEEN : LINE 2 OF THE STATUS BOX
        usb.print(theThing->statBuff.line2);          // print the line
        for(int i = 0; i < theThing->statSpace.line2; i++)  // print the alignment spaces
        {
          usb.print(F(" "));
        }
        usb.println(F("|| "));                        // print the end frame

        // LINE SEVENTEEN : LINE 3 OF THE STATUS BOX
        usb.print(theThing->statBuff.line3);          // print the line
        for(int i = 0; i < theThing->statSpace.line3; i++)  // print the alignment spaces
        {
          usb.print(F(" "));
        }
        usb.println(F("|| "));                        // print the end frame

        // LINE EIGHTEEN : LINE 4 OF THE STATUS BOX
        usb.print(theThing->statBuff.line4);          // print the line
        for(int i = 0; i < theThing->statSpace.line4; i++)  // print the alignment spaces
        {
          usb.print(F(" "));
        }
        usb.println(F("|| "));                        // print the end frame

        // LINE NINETEEN : LINE 5 OF THE STATUS BOX
        usb.print(theThing->statBuff.line5);          // print the line
        for(int i = 0; i < theThing->statSpace.line5; i++)  // print the alignment spaces
        {
          usb.print(F(" "));
        }
        usb.println(F("|| "));                        // print the end frame

        // LINE TWENTY : LINE 6 OF THE STATUS BOX
        usb.print(theThing->statBuff.line6);          // print the line
        for(int i = 0; i < theThing->statSpace.line6; i++)  // print the alignment spaces
        {
          usb.print(F(" "));
        }
        usb.println(F("|| "));                        // print the end frame

        // LINE TWENTY-ONE : LINE 7 OF THE STATUS BOX
        usb.print(theThing->statBuff.line7);          // print the line
        for(int i = 0; i < theThing->statSpace.line7; i++)  // print the alignment spaces
        {
          usb.print(F(" "));
        }
        usb.println(F("|| "));                        // print the end frame

        // LINE TWENTY-TWO : LINE 8 OF THE STATUS BOX
        usb.print(theThing->statBuff.line8);          // print the line
        for(int i = 0; i < theThing->statSpace.line8; i++)  // print the alignment spaces
        {
          usb.print(F(" "));
        }
        usb.println(F("|| "));                        // print the end frame
        usb.println(F("                                       ||"));

        // LINE TWENTY-THREE : LINE BOTTOM BORDER OF THE STATUS BOX
        usb.println(F("|||||||||||||||||||||||||||||||||||||||||"));
    }

    //*************************************************************************************************************
    //*******                                 struct Thing printThings - 2 things
    //*************************************************************************************************************
    void printThings(thing *thing1, thing *thing2)
    {
      uint8_t Name_spaceUsed;
      //switch(print_counter)
      //{
        //1:
          for(int i = 0; i < 50; i++)   //
          {                             //
            usb.println(F(""));         // clear the print area
          }                             //
      
      
          // HEADER SETUP WTIH NAMES OF THINGS
          usb.println(F("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"));
         // break;
          
        //2:
          Name_spaceUsed = thing1->Name_space + thing1->Name.length();
          for(int i = 0 ; i < thing1->Name_space; i++)
          {
            usb.print(F(" "));
          }
          usb.print(thing1->Name);
          Name_spaceUsed = 39 - Name_spaceUsed;
          for(int i = 0 ; i < Name_spaceUsed; i++)
          {
            usb.print(F(" "));
          }
          usb.print(F("|| "));
      
          Name_spaceUsed = thing2->Name_space + thing2->Name.length();
          for(int i = 0 ; i < thing2->Name_space; i++)
          {
            usb.print(F(" "));
          }
          usb.print(thing2->Name);
          Name_spaceUsed = 39 - Name_spaceUsed;
          for(int i = 0 ; i < Name_spaceUsed; i++)
          {
            usb.print(F(" "));
          }
          usb.println(F("|| "));
          //break;

        //3:
          usb.println(F("***************************************||****************************************||"));
         // break;

        //4:
          usb.println(F("                                       ||                                        ||"));
         // break;
    
        //5:
          // LINE ONE : LATITUDE
          usb.print(F("Lat:  "));       // print the label
          for(int i = 0; i< thing1->lat_space; i++)      // print the required space to stay aligned
          {                             //
            usb.print(F(" "));          //
          }                             //
          usb.print(thing1->lat); usb.print(F("                      || "));  // print the rest of the line
          usb.print(F("Lat:  "));       // print the label
          for(int i = 0; i< thing2->lat_space; i++)      // print the required space to stay aligned
          {                             //
            usb.print(F(" "));          //
          }                             //
          usb.print(thing2->lat); usb.println(F("                      || "));  // print the rest of the line
          //break;
          
        //6:
          // LINE TWO : LONGITUDE
          usb.print(F("Lon: "));        // print the label
          for(int i = 0; i< thing1->lon_space; i++)      // print the required space to stay aligned
          {                             //
            usb.print(F(" "));          //
          }                             //
          usb.print(thing1->lon); usb.print(F("                      || "));  // print the rest of the line
          usb.print(F("Lon: "));        // print the label
          for(int i = 0; i< thing2->lon_space; i++)      // print the required space to stay aligned
          {                             //
            usb.print(F(" "));          //
          }                             //
          usb.print(thing2->lon); usb.println(F("                      || "));  // print the rest of the line
          //break;

        //7:
          // LINE THREE : ELEVATION
          usb.print(F("Elev:    "));    // print the label
          for(int i = 0; i < thing1->elev_space; i++)     // print the required space to stay aligned
          {                             //
            usb.print(F(" "));          //
          }                             //
          usb.print(thing1->elev); usb.print(F("                      || "));  // print the rest of the line
          usb.print(F("Elev:    "));    // print the label
          for(int i = 0; i < thing2->elev_space; i++)     // print the required space to stay aligned
          {                             //
            usb.print(F(" "));          //
          }                             //
          usb.print(thing2->elev); usb.println(F("                      || "));  // print the rest of the line
         // break;

        //8:
          // LINE FOUR : BLANK
          usb.println(F("                                       ||                                        ||"));
          //break;

        //9:
          // LINE FIVE : ACC/GYRO/MAG LABELS
          usb.println(F("ACC           GYRO          MAG        || ACC           GYRO          MAG        ||"));
          
        //10:
          // LINE SIX : ACC/GYRO/MAG X VALUES
          usb.print(F("X: "));          // print the label                          //
          for(int i = 0; i < thing1->acc_space.x; i++)    // print the required space to stay aligned //
          {                             //                                          // 1ACC.X
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing1->acc.x);         // print the value                          //
        
          usb.print(F("    X: "));         // print the label                          //
          for(int i = 0; i < thing1->gyro_space.x; i++)   // print the required space to stay aligned //
          {                             //                                          // 1GYRO.X
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing1->gyro.x);        // print the value                          //
        
          usb.print(F("    X: "));         // print the label                          //
          for(int i = 0; i < thing1->mag_space.x; i++)    // print the required space to stay aligned //
          {                             //                                          // 1MAG.X
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing1->mag.x);         // print the value                          //
          usb.print(F(" || "));       // print the rest of the line               //
      
          usb.print(F("X: "));          // print the label                          //
          for(int i = 0; i < thing2->acc_space.x; i++)    // print the required space to stay aligned //
          {                             //                                          // 2ACC.X
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing2->acc.x);         // print the value                          //
        
          usb.print(F("    X: "));         // print the label                          //
          for(int i = 0; i < thing2->gyro_space.x; i++)   // print the required space to stay aligned //
          {                             //                                          // 2GYRO.X
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing2->gyro.x);        // print the value                          //
        
          usb.print(F("    X: "));         // print the label                          //
          for(int i = 0; i < thing2->mag_space.x; i++)    // print the required space to stay aligned //
          {                             //                                          // 2MAG.X
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing2->mag.x);         // print the value                          //
          usb.println(F(" || "));       // print the rest of the line               //
         // break;
        
        //11:
          // LINE SEVEN : ACC/GYRO/MAG Y VALUES
          usb.print(F("Y: "));          // print the label                          //
          for(int i = 0; i < thing1->acc_space.y; i++)    // print the required space to stay aligned //
          {                             //                                          // 1ACC.Y
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing1->acc.y);         // print the value                          //
        
          usb.print(F("    Y: "));         // print the label                          //
          for(int i = 0; i < thing1->gyro_space.y; i++)   // print the required space to stay aligned //
          {                             //                                          // 1GYRO.Y
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing1->gyro.y);        // print the value                          //
        
          usb.print(F("    Y: "));         // print the label                          //
          for(int i = 0; i < thing1->mag_space.y; i++)    // print the required space to stay aligned //
          {                             //                                          // 1MAG.Y
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing1->mag.y);         // print the value                          //
          usb.print(F(" || "));       // print the rest of the line               //
      
          usb.print(F("Y: "));          // print the label                          //
          for(int i = 0; i < thing2->acc_space.y; i++)    // print the required space to stay aligned //
          {                             //                                          // 2ACC.Y
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing2->acc.y);         // print the value                          //
        
          usb.print(F("    Y: "));         // print the label                          //
          for(int i = 0; i < thing2->gyro_space.y; i++)   // print the required space to stay aligned //
          {                             //                                          // 2GYRO.Y
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing2->gyro.y);        // print the value                          //
        
          usb.print(F("    Y: "));         // print the label                          //
          for(int i = 0; i < thing2->mag_space.y; i++)    // print the required space to stay aligned //
          {                             //                                          // 2MAG.Y
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing2->mag.y);         // print the value                          //
          usb.println(F(" || "));       // print the rest of the line               //
         // break;
    
        //12:
          // LINE EIGHT : ACC/GYRO/MAG Z VALUES
          usb.print(F("Z: "));          // print the label                          //
          for(int i = 0; i < thing1->acc_space.z; i++)    // print the required space to stay aligned //
          {                             //                                          // 1ACC.Z
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing1->acc.z);         // print the value                          //
        
          usb.print(F("    Z: "));         // print the label                          //
          for(int i = 0; i < thing1->gyro_space.z; i++)   // print the required space to stay aligned //
          {                             //                                          // 1GYRO.Z
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing1->gyro.z);        // print the value                          //
        
          usb.print(F("    Z: "));         // print the label                          //
          for(int i = 0; i < thing1->mag_space.z; i++)    // print the required space to stay aligned //
          {                             //                                          // 1MAG.Z
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing1->mag.z);         // print the value                          //
          usb.print(F(" || "));       // print the rest of the line               //
      
          usb.print(F("Z: "));          // print the label                          //
          for(int i = 0; i < thing2->acc_space.z; i++)    // print the required space to stay aligned //
          {                             //                                          // 2ACC.Z
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing2->acc.z);         // print the value                          //
        
          usb.print(F("    Z: "));         // print the label                          //
          for(int i = 0; i < thing2->gyro_space.z; i++)   // print the required space to stay aligned //
          {                             //                                          // 2GYRO.Z
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing2->gyro.z);        // print the value                          //
        
          usb.print(F("    Z: "));         // print the label                          //
          for(int i = 0; i < thing2->mag_space.z; i++)    // print the required space to stay aligned //
          {                             //                                          // 2MAG.Z
            usb.print(F(" "));          //                                          //
          }                             //                                          //
          usb.print(thing2->mag.z);         // print the value                          //
          usb.println(F(" || "));       // print the rest of the line               //
          //break;
    
        //13:
          // LINE NINE : BLANK
          usb.println(F("                                       ||                                        ||"));
          //break;
      
        //14:
          // LINE TEN : EULER ANGLE LABEL AND PRESSURE
          usb.print(F("EUL              PRESS: "));   // print the labels
          for(int i = 0; i < thing1->pressure_space; i++)         // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //
          }                                     //
          usb.print(thing1->pressure);              // print the value
          usb.print(F("      || "));            // print the rest of the line
          usb.print(F("EUL              PRESS: "));   // print the labels
          for(int i = 0; i < thing2->pressure_space; i++)         // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //
          }                                     //
          usb.print(thing2->pressure);              // print the value
          usb.println(F("      || "));            // print the rest of the line
          //break;
    
        //15:
          // LINE ELEVEN : EULER ANGLE X AND TEMP
          usb.print(F("X: "));   // print the labels
          for(int i = 0; i < thing1->eul_space.x; i++)            // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //
          }                                     //
          usb.print(thing1->eul.x);                 // print the value
          usb.print(F("        TEMP:     "));        // print the next label
          for(int i = 0; i < thing1->temperature_space; i++)      // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //
          }                                     //
          usb.print(thing1->temperature);           // print the value
          usb.print(F("      || "));            // print the rest of the line
      
          usb.print(F("X: "));   // print the labels
          for(int i = 0; i < thing2->eul_space.x; i++)            // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //
            thing2->eul_space.x--;                  //
          }                                     //
          usb.print(thing2->eul.x);                 // print the value
          usb.print(F("        TEMP:     "));        // print the next label
          for(int i = 0; i < thing2->temperature_space; i++)      // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //
          }                                     //
          usb.print(thing2->temperature);           // print the value
          usb.println(F("      || "));            // print the rest of the line
         // break;
          
        //16:
          // LINE TWELVE : EULER ANGLE Y AND HUMIDITY 
          usb.print(F("Y: "));   // print the labels
          for(int i = 0; i < thing1->eul_space.y; i++)            // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //
          }                                     //
          usb.print(thing1->eul.y);                 // print the value
          usb.print(F("       HUMID:     "));        // print the next label
          for(int i = 0; i < thing1->humidity_space; i++)         // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //
          }                                     //
          usb.print(thing1->humidity);              // print the value
          usb.print(F("      || "));            // print the rest of the line
      
          usb.print(F("Y: "));   // print the labels
          for(int i = 0; i < thing2->eul_space.y; i++)            // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //

          }                                     //
          usb.print(thing2->eul.y);                 // print the value
          usb.print(F("       HUMID:     "));        // print the next label
          for(int i = 0; i < thing2->humidity_space; i++)         // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //
          }                                     //
          usb.print(thing2->humidity);              // print the value
          usb.println(F("      || "));            // print the rest of the line
         // break;
    
        //17:
          // LINE THIRTEEN : EULER ANGLE Z AND PITOT READING
          usb.print(F("Z: "));   // print the labels
          for(int i = 0; i < thing1->eul_space.z; i++)            // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //
          }                                     //
          usb.print(thing1->eul.z);                 // print the value
          usb.print(F("       PITOT:  "));           // print the next label
          for(int i = 0; i < thing1->pitot_space; i++)            // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //
          }                                     //
          usb.print(thing1->pitot);                 // print the value
          usb.print(F("      || "));            // print the rest of the line
      
          usb.print(F("Z: "));   // print the labels
          for(int i = 0; i < thing2->eul_space.z; i++)            // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //
          }                                     //
          usb.print(thing2->eul.z);                 // print the value
          usb.print(F("       PITOT:  "));           // print the next label
          for(int i = 0; i < thing1->pitot_space; i++)            // print the required space for alignment
          {                                     //
            usb.print(F(" "));                  //
          }                                     //
          usb.print(thing2->pitot);                 // print the value
          usb.println(F("      || "));            // print the rest of the line
         // break;

        //18:
          // LINE FOURTEEN : DIVIDER BETWEEEN HUD AND THE STATUS BOX
          usb.println(F("                                       ||                                        ||"));
         // break;

        //19:
          usb.println(F("***************************************||****************************************||"));
         // break;

        //20:
          usb.println(F("                                       ||                                        ||"));
         // break;

        //21:
          // LINE FIFTEEN : LINE 1 OF THE STATUS BOX
          usb.print(thing1->statBuff.line1);          // print the line
          for(int i = 0; i < thing1->statSpace.line1; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.print(F("|| "));                        // print the end frame
          usb.print(thing2->statBuff.line1);          // print the line
          for(int i = 0; i < thing2->statSpace.line1; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.println(F("||"));                        // print the end frame
         // break;

        //22:
          // LINE SIXTEEN : LINE 2 OF THE STATUS BOX
          usb.print(thing1->statBuff.line2);          // print the line
          for(int i = 0; i < thing1->statSpace.line2; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.print(F("|| "));                        // print the end frame
          usb.print(thing2->statBuff.line2);          // print the line
          for(int i = 0; i < thing2->statSpace.line2; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.println(F("||"));                        // print the end frame
          //break;

        //23:
          // LINE SEVENTEEN : LINE 3 OF THE STATUS BOX
          usb.print(thing1->statBuff.line3);          // print the line
          for(int i = 0; i < thing1->statSpace.line3; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.print(F("|| "));                        // print the end frame
          usb.print(thing2->statBuff.line3);          // print the line
          for(int i = 0; i < thing2->statSpace.line3; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.println(F("||"));                        // print the end frame
          //break;
        
        //24:
          // LINE EIGHTEEN : LINE 4 OF THE STATUS BOX
          usb.print(thing1->statBuff.line4);          // print the line
          for(int i = 0; i < thing1->statSpace.line4; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.print(F("|| "));                        // print the end frame
          usb.print(thing2->statBuff.line4);          // print the line
          for(int i = 0; i < thing2->statSpace.line4; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.println(F("||"));                        // print the end frame
          //break;
        
        //25:
          // LINE NINETEEN : LINE 5 OF THE STATUS BOX
          usb.print(thing1->statBuff.line5);          // print the line
          for(int i = 0; i < thing1->statSpace.line5; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.print(F("|| "));                        // print the end frame
          usb.print(thing2->statBuff.line5);          // print the line
          for(int i = 0; i < thing2->statSpace.line5; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.println(F("||"));                        // print the end frame
          //break;
        
        //26:
          // LINE TWENTY : LINE 6 OF THE STATUS BOX
          usb.print(thing1->statBuff.line6);          // print the line
          for(int i = 0; i < thing1->statSpace.line6; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.print(F("|| "));                        // print the end frame
          usb.print(thing2->statBuff.line6);          // print the line
          for(int i = 0; i < thing2->statSpace.line6; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.println(F("||"));                        // print the end frame
          //break;
        
        //27:
          // LINE TWENTY-ONE : LINE 7 OF THE STATUS BOX
          usb.print(thing1->statBuff.line7);          // print the line
          for(int i = 0; i < thing1->statSpace.line7; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.print(F("|| "));                        // print the end frame
          usb.print(thing2->statBuff.line7);          // print the line
          for(int i = 0; i < thing2->statSpace.line7; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.println(F("||"));                        // print the end frame
         // break;
        
        //28:
          // LINE TWENTY-TWO : LINE 8 OF THE STATUS BOX
          usb.print(thing1->statBuff.line8);          // print the line
          for(int i = 0; i < thing1->statSpace.line8; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.print(F("|| "));                        // print the end frame
          usb.print(thing2->statBuff.line8);          // print the line
          for(int i = 0; i < thing2->statSpace.line8; i++)  // print the alignment spaces
          {
            usb.print(F(" "));
          }
          usb.println(F("||"));                        // print the end frame
         // break;
        
        //29:
          // LINE TWENTY-THREE : LINE BOTTOM BORDER OF THE STATUS BOX
          usb.println(F(">>                                     ||>>                                      ||"));
         // break;

        //30:
          usb.println(F("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"));
      //};   
    }                                

    //*************************************************************************************************************
    //*******                                   addStat
    //*************************************************************************************************************
    void addStat(thing *thisThing, uint8_t cmdType, uint8_t cmdVal, bool state)
    {
      statRollover(thisThing);
      thisThing->statBuff.line8 = thisThing->Name;
      thisThing->statBuff.line8 += F(":");
      thisThing->statBuff.line8 += cmdType;
      thisThing->statBuff.line8 += cmdVal;
      thisThing->statBuff.line8 += F(":");
      if(state)
      {
        thisThing->statBuff.line8 += F("success");
      }
      else
      {
        thisThing->statBuff.line8 += F("failed");
      }
    }

    void addStat(thing *thisThing, String cmd, String msg) 
    {
      statRollover(thisThing);
      thisThing->statBuff.line8 = thisThing->Name;
      thisThing->statBuff.line8 += F(":");
      thisThing->statBuff.line8 += cmd;
      thisThing->statBuff.line8 += F(":");
      thisThing->statBuff.line8 += msg;
    }

    void addStat(String cmd, thing *thisThing, bool state) 
    {
      statRollover(thisThing);
      thisThing->statBuff.line8 = thisThing->Name;
      thisThing->statBuff.line8 += F(":");
      thisThing->statBuff.line8 += cmd;
      thisThing->statBuff.line8 += F(":");
      if(state)
      {
        thisThing->statBuff.line8 += F("success");
      }
      else
      {
        thisThing->statBuff.line8 += F("failed");
      }
    }

    
    //*************************************************************************************************************
    //*******                                   statRollover
    //*************************************************************************************************************
    void statRollover(thing *thisThing)
    {
      thisThing->statBuff.line1 = thisThing->statBuff.line2;
      thisThing->statBuff.line2 = thisThing->statBuff.line3;
      thisThing->statBuff.line3 = thisThing->statBuff.line4;
      thisThing->statBuff.line4 = thisThing->statBuff.line5;
      thisThing->statBuff.line5 = thisThing->statBuff.line6;
      thisThing->statBuff.line6 = thisThing->statBuff.line7;
      thisThing->statBuff.line7 = thisThing->statBuff.line8;
    }
    
    //*************************************************************************************************************
    //*******                                 struct Thing intitializeProtothreads
    //*************************************************************************************************************
    void initializeProtothreads(thing *thisThing)
    {
      PT_INIT(&(thisThing->packetGrab_PT));     // initialize the protothread for the initial packetGrab
      PT_INIT(&(thisThing->packetParse_PT));    // initialize the protothread for the packetparsing function
      PT_INIT(&(thisThing->gpsParse_PT));       // initialize the protothread for parsing the GPS sentence and modifying struct members
      PT_INIT(&(thisThing->otherParse_PT));     // initialize the protothread for parsing the othersensor sentence and modifying struct members
      PT_INIT(&(thisThing->gpsWrite_PT));       // initialize the protothread for writing the GPS sentence to the appropriate SD file
      PT_INIT(&(thisThing->otherWrite_PT));     // initialize the protothread for writing the other sensors sentence to the appropriate SD file
      PT_INIT(&(thisThing->parseInput_PT));       // initialize the protothread for figuring out user input
    }

    //*************************************************************************************************************
    //*******                                 struct Thing intitializeProtothreads
    //*************************************************************************************************************
    void runProtothreads(Xbee *xbee, thing *thisThing, XbeeSettings *theseSettings)
    {
      packetGrab_sense(xbee,thisThing);    // call the protothread for the initial packetGrab
      packetParse_sense(thisThing);  // call the protothread for the packetparsing function
      gpsParse_sense(thisThing);        // call the protothread for parsing the GPS sentence and modifying struct members
      otherParse_sense(thisThing);    // call the protothread for parsing the othersensor sentence and modifying struct members
      gpsWrite_sense(thisThing);        // call the protothread for writing the GPS sentence to the appropriate SD file
      otherWrite_sense(thisThing);          // call the protothread for writing the other sensors sentence to the appropriate SD file
      parseInput_sense(xbee,thisThing,theseSettings);  
    }

    void runProtothreads(Xbee *xbee1, Xbee *xbee2, thing *thing1, thing *thing2, XbeeSettings *settings1, XbeeSettings *settings2)
    {
      packetGrab_sense(xbee1,thing1);    // call the protothread for the initial packetGrab, 1
      packetGrab_sense(xbee2,thing2);    // call the protothread for the initial packetGrab, 2
      packetParse_sense(thing1);  // call the protothread for the packetparsing function, 1
      packetParse_sense(thing2);  // call the protothread for the packetparsing function,2 
      gpsParse_sense(thing1);        // call the protothread for parsing the GPS sentence and modifying struct members, 1 
      gpsParse_sense(thing2);        // call the protothread for parsing the GPS sentence and modifying struct members, 2
      otherParse_sense(thing1);    // call the protothread for parsing the othersensor sentence and modifying struct members, 1
      otherParse_sense(thing2);    // call the protothread for parsing the othersensor sentence and modifying struct members, 2
      gpsWrite_sense(thing1);        // call the protothread for writing the GPS sentence to the appropriate SD file, 1
      gpsWrite_sense(thing2);        // call the protothread for writing the GPS sentence to the appropriate SD file, 2 
      otherWrite_sense(thing1);          // call the protothread for writing the other sensors sentence to the appropriate SD file, 1
      otherWrite_sense(thing2);          // call the protothread for writing the other sensors sentence to the appropriate SD file, 2  
      parseInput_sense(xbee1,thing1,settings1);
      parseInput_sense(xbee2,thing2,settings2); 
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
    //*******                                           SETUP
    //*************************************************************************************************************
    void setup() {
      wdt_disable(); // this prevents infinite loops from occuring with the watchdog reset                            // watchdog
      delay(5000);    // used to read setup debug code

      usb.begin(2000000);
      usb.println(F("INITIALIZING"));
    
      pinMode(sdPower, OUTPUT);     //
      pinMode(imuPower, OUTPUT);    // Sets all power transistor gate pins and reset pins to OUTPUT mode
      pinMode(altPower, OUTPUT);    //
                                    //
      digitalWrite(sdPower, HIGH);  //
      digitalWrite(imuPower, HIGH); // closes path to ground on all sensors, and sets the imu rst pin to HIGH
      digitalWrite(altPower, HIGH); //
      delay(250);                   // allows time for the pin operations and sensors to come up 

      one.Name = XBEE1_NAME;        // 
      #ifdef XB2_DEST_ADDR          // apply names to struct elements (using consts defined
        two.Name = XBEE2_NAME;      // in the configuration section at top of sketch
      #endif                        //
      
      long xbeeTime = millis();
      while(XB1_NP != 256 && (millis() - xbeeTime <= 5000))
      {
        XB1_NP = xbee1.begin();
      }
      // include any other xbee begin cycles here
      xbeeTime = millis();
      while(XB2_NP != 256 && (millis() - xbeeTime <= 5000))
      {
        XB2_NP = xbee2.begin();
      }
      // include any other xbee begin cycles here
      if(XB1_NP == 256)
      {
        setupXbee(&xbee1,&xbee_set1); // include all setup features in this fucntion (above)
      }
      #ifdef XB2_DEST_ADDR
        if(XB2_NP == 256)
        {
          setupXbee(&xbee2,&xbee_set2); // include all setup features in this fucntion (above)
        }
      #endif
      
      if(!SD.begin(SD_CS)){         // initializes the SD card
        usb.println(F("SD Card failed to initialize!")); // if the SD card fails, set sdError true and tell someone
        sdError = true;
        fatalError = true;
      }                                 
      else if(!makeFiles(&one)){        // initializes the SD files
        usb.println(F("File Initialization Failed!")); // if makeFiles fails, set makeFileError to true and tell someone
        makeFileError = true;
        fatalError = true;
      }
      #ifdef XB2_DEST_ADDR
        else if(!makeFiles(&two)){
          usb.println(F("File Initialization Failed!")); // if makeFiles fails, set makeFileError to true and tell someone
          makeFileError = true;
          fatalError = true;
        }
      #endif
      else{
        usb.println(F("makefiles worked!"));
      }                       

      initializeProtothreads(&one);       // 
      #ifdef XB2_DEST_ADDR                //
        initializeProtothreads(&two);     // Initialize all the protothreads
      #endif                              //
      PT_INIT(&printHud_PT);              //
      PT_INIT(&userInput_PT);             // 

      reserveMemory(&one);
      #ifdef XB2_DEST_ADDR
        reserveMemory(&two);
      #endif
    
      usb.println(F("Setup complete, entering loop!"));
    
      wdt_enable(16384); // enable watchdog timer
    }
    
    //*************************************************************************************************************
    //*******                                           LOOP
    //*************************************************************************************************************
    void loop() {

      #ifndef XB2_DEST_ADDR
        userInput_sense(&userInput_PT);
        xbee1.protothreadLoop();
        runProtothreads(&xbee1,&one,&xbee_set1);
        HUD_sense(&printHud_PT);
      #else
        userInput_sense(&userInput_PT);
        xbee1.protothreadLoop();
        xbee2.protothreadLoop();
        runProtothreads(&xbee1,&xbee2,&one,&two,&xbee_set1,&xbee_set2);
        HUD_sense(&printHud_PT);
      #endif
         
      wdt_reset();    // watchdog reset
      
    }



  #endif
#else



/**************************************************************************************************************
 * *************************************************************************************************************
 * *************************************************************************************************************
 *                      
 *                                         RECEIVER CODE ABOVE HERE
 *                                      CONFIG ERROR CODE BELOW HERE HERE
 *                                        
 * *************************************************************************************************************                                       
 * *************************************************************************************************************
 * *************************************************************************************************************
 */

  #include <pt.h>               // protothread library
  #include <Adafruit_DotStar.h> // controls LED on ItsyBitsyM4 MCU
  
  // On-board dotstar LED stuff
  // There is only one pixel on the board
  #define NUMPIXELS 1 
  //Use these pin definitions for the ItsyBitsy M4
  #define DATAPIN    8
  #define CLOCKPIN   6
  Adafruit_DotStar px(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
  
  // Serial stuff
  #define usb Serial            // renames Serial as usB
  long baudrate_array[15] = {300,1200,2400,4800,9600,19200,38400,57600,74880,115200,230400,250000,500000,1000000,2000000};
  int baud_rate = 0;
  
  // Protothreads stuff
  static struct pt printstuff;
  static struct pt changebaud;
  static struct pt updateDot;
  long updateDot_timer = 0;
  #define updateDot_thresh 10
  long delayText_stamp = 3000;
  #define delayText_thresh 2000
  int line = 0;
  
  String error_message[35] = { 
    F("                iWs                                 ,W["),
    F("              W@@W.                              g@@["),
    F("            i@@@@@s                           g@@@@W"),
    F("             @@@@@@@W.                       ,W@@@@@@"),
    F("            ]@@@@@@@@@W.   ,_______.       ,m@@@@@@@@i"),
    F("           ,@@@@@@@@@@@@W@@@@@@@@@@@@@@mm_g@@@@@@@@@@["),
    F("           d@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"),
    F("          i@@@@@@@A*~~~~~VM@@@@@@@@@@Af~~~~V*@@@@@@@@@i"),
    F("          @@@@@A~          'M@@@@@@A`         'V@@@@@@b"),
    F("         d@@@*`              Y@@@@f              V@@@@@."),
    F("        i@@A`                 M@@P                 V@@@b"),
    F("       ,@@A                   '@@                   !@@@."),
    F("       W@P                     @[                    '@@W"),
    F("      d@@            ,         ]!                     ]@@b"),
    F("     g@@[          ,W@@s       ]       ,W@@s           @@@i"),
    F("    i@@@[          W@@@@i      ]       W@@@@i          @@@@i"),
    F("   i@@@@[          @@@@@[      ]       @@@@@[          @@@@@i"),
    F("  g@@@@@[          @@@@@!      @[      @@@@@[          @@@@@@i"),
    F(" d@@@@@@@          !@@@P      iAW      !@@@A          ]@@@@@@@i"),
    F("W@@@@@@@@b          '~~      ,Z Yi      '~~          ,@@@@@@@@@"),
    F("'*@@@@@@@@s                  Z`  Y.                 ,W@@@@@@@@A"),
    F("  'M@@@*f**W.              ,Z     Vs               ,W*~~~M@@@f"),
    F("    'M@    'Vs.          ,z~       'N_           ,Z~     d@P"),
    F("   M@@@       ~|-__  __z/` ,gmW@@mm_ '+e_.   __=/`      ,@@@@"),
    F("    'VMW                  g@@@@@@@@@W     ~~~          ,WAf"),
    F("       ~N.                @@@@@@@@@@@!                ,Z`"),
    F("         V.               !M@@@@@@@@f                gf-"),
    F("          'N.               '~***f~                ,Z`"),
    F("            Vc.                                  _Zf"),
    F("              ~e_                             ,gY~"),
    F("                'V=_          -@@D         ,gY~ '"),
    F("                    ~|=__.           ,__z=~`"),
    F("                         '~~~*==Y*f~~~"),
    F(" LOOKS LIKE YOU CRAPPED UP THE CONFIGURATION SECTION, READ THE\n"),
    F("             DOCUMENATION AND TRY AGAIN PLEASE... ")
  };
  
  
  //*************************************************************************************************************
  //*******                          PROTOTHREAD - printStuff_sense - FAULTY CONFIG
  //*************************************************************************************************************
  static int printStuff_sense(struct pt *pt){
    PT_BEGIN(pt);
      PT_WAIT_UNTIL(pt, (millis() - delayText_stamp) >= delayText_thresh);
      usb.println(error_message[line]);
      line++;
    PT_END(pt);
  }
  
  
  //*************************************************************************************************************
  //*******                          PROTOTHREAD - changebuad_sense - FAULTY CONFIG
  //*************************************************************************************************************
  static int changebuad_sense(struct pt *pt){
    PT_BEGIN(pt);
      PT_WAIT_UNTIL(pt, (line > 34));
      line = 0;
      baud_rate++;
      if(baud_rate > 14)
      {
        baud_rate = 0;
        delayText_stamp = millis();
      }
      else
      {
        usb.begin(baudrate_array[baud_rate]);
        usb.println(F("\n\n\n\n\n\n\n\n\n\n\n\n"));
      }
    PT_END(pt);
  }
  
  //*************************************************************************************************************
  //*******                          PROTOTHREAD - updateDot_sense - FAULTY CONFIG
  //*************************************************************************************************************
  static int updateDot_sense(struct pt *pt){
    PT_BEGIN(pt);
      PT_WAIT_UNTIL(pt, (millis() - updateDot_timer) >= updateDot_thresh);
      int red = random(0,255);
      int green = random(0,255);
      int blue = random(0,255);
      px.setPixelColor(0, red, green, blue); // Set the pixel colors
      px.show();              // Refresh strip
      updateDot_timer = millis();
    PT_END(pt);
  }
  
  
  //*************************************************************************************************************
  //*******                                       SETUP - FAULTY CONFIG
  //*************************************************************************************************************
  void setup()
  {
    // initialize the dotstar
    px.begin(); // Initialize pins for output
    px.show();  // Turn all LEDs off ASAP
    randomSeed(analogRead(0)); //initialise the random number generator
  
    PT_INIT(&printstuff);         // Initilize all protothreads
    PT_INIT(&changebaud);         // Initilize all protothreads
    PT_INIT(&updateDot);         // Initilize all protothreads
  
    usb.begin(baudrate_array[baud_rate]);
  }
  
  //*************************************************************************************************************
  //*******                                        LOOP - FAULTY CONFIG
  //*************************************************************************************************************
  void loop()
  {
    printStuff_sense(&printstuff);
    updateDot_sense(&updateDot);
    changebuad_sense(&changebaud);
  }

#endif
 
