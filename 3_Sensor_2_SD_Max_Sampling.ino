#include <pt.h>           // protothread library
#include <Adafruit_BNO055.h> // IMU library
#include <Adafruit_BME280.h> // Altimeter library
#include <SD.h>           // SD library
#include <Adafruit_GPS.h> // GPS library
#include <Wire.h>         // wire library (needed for some sensors)

#define usb Serial       // renames Serial as usb
#define gps Serial2      // renames Serial2 as gps

Adafruit_BNO055 IMU = Adafruit_BNO055(); // IMU OBJECT DEFINITION
Adafruit_BME280 ALT; //  ALT I2C

static struct pt imuRead;     //
static struct pt altRead;     //
static struct pt gpsRead;      // protothread objects
static struct pt otherWrite;  // 
static struct pt gpsWrite;    //


long imuStamp = 0;    // timestamp tracking last time IMU sensor data was gathered
long writeStamp = 0;  // timestamp tracking last time data was written to SD
long altStamp = 0;    // timestamp tracking last time data from altimeter was gathered

String OtherflushTotal = "";  // string object for sensor data (temporary storage)
String GPSflushTotal = "";    // string object for GPU data (temporary storage)
File OtherData;
File GPSdata;

#define imuTime 10          //min refresh period of most IMU sensors (max of 100Hz)
#define altTime 1000        // min refresh period for Altimeter (1 Hz)
#define OtherFlushThresh 5  // number of data pulls from the other sensors before writing to SD
#define GPSFlushThresh 5    // number of gps data pulls before writing to SD


int count = 0;          // counter for number of IMU data pulls
int gpsCount = 0;       // counter for number of GPS data pulls
int flushcount = 0;     // total number of flushes, used to track progress in serial monitor
int gpsFlushcount = 0;  // total number of gps flushes to SD used to track progress in serial monitor

// altimeter variables
float temp = 0;
float pressure = 0;
float humid = 0;

//SPI communication pins, SCK, MISO, and MOSI are common to all SPI sensors, CS pins are unique to each 
// SPI sensor.
#define SPI_SCK 52
#define SPI_MISO 50
#define SPI_MOSI 51
#define SD_CS 44

// GPS stuff
Adafruit_GPS GPS(&gps); // GPS
#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean); 

//*************Setup *************************************************************
void setup() {
  usb.begin(115200); // initialize serial monitor (USB output to computer)

  if(!IMU.begin()){  // initializes the IMU and checks for errors
    usb.println("BNO055 IMU Initialization Error");
    while(1);
  }
  else if(!ALT.begin()){
    usb.println("BME280 Sensor Initialization Error");
    while(1);
  }
  else if(!SD.begin(SD_CS)){
    usb.println("SD Card Initialization Error");
    while(1);
  }
  else if (!makeFiles()){
    usb.println("File Initialization failed");
    while(1);
  }
  
  IMU.setExtCrystalUse(true);  // IMU settings

  // GPS initialization and settings
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // sets NMEA sentence output to RMC & GGA
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);  //
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);  // sets GPS refresh rate at 5Hz (every 200 ms)
  useInterrupt(true);

  PT_INIT(&imuRead);  //initialize the imu sensors protothread variable (100Hz)
  PT_INIT(&altRead);  //initialize the altimeter read protothread variable (1Hz)
  PT_INIT(&gpsRead);  //initialize the GPS read protothread variable (5Hz)
  PT_INIT(&otherWrite);  //initialize the SD write protothread variable (every 5 pulls)
  PT_INIT(&gpsWrite);
  

  usb.println("Initialized!");

  delay(1000);
}

//*************imuRead Protothread************************************************
static int imuSensor(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, millis() - imuStamp >= imuTime);
    getIMU();
  }
  PT_END(pt);
}

//*************altSensor Protothread************************************************
static int altSensor(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, millis() - altStamp >= altTime);
    getAlt();
  }
  PT_END(pt);
}

//*************gpsSensor Protothread************************************************
static int gpsSensor(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, GPS.newNMEAreceived());
    GPSflushTotal = GPSflushTotal + GPS.lastNMEA();
    gpsCount++;
  }
  PT_END(pt);
}

//*************OtherWriteSensor Protothread************************************************
static int OtherWriteSensor(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, OtherFlushThresh == count);
    writeOther();
  }
  PT_END(pt);
}

//*************OtherWriteSensor Protothread************************************************
static int GPSWriteSensor(struct pt *pt){
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, GPSFlushThresh == gpsCount);
    writeGPS();
  }
  PT_END(pt);
}


//*************getIMU Function************************************************
void getIMU(){
  //sensors_event_t BN_Event; // creates a sensor event
  
  imu::Vector<3> BN_acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // creates vectors for each event
  imu::Vector<3> BN_gyro = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);    //
  imu::Vector<3> BN_mag = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  OtherflushTotal = OtherflushTotal  + "\n X: " + BN_acc.x() + "  Y: " + BN_acc.y() + " Z: " + BN_acc.z() + " Gyro X: " + BN_gyro.x() + " Y: " + BN_gyro.y() + " Z: " + BN_gyro.z() + " Mag X: " + BN_mag.x() + " Y: " + BN_mag.y() + " Z: " + BN_mag.z() + " Temp: " + temp + " Press: " + pressure + " RH: " + humid;
  imuStamp = millis();
  count++;
}

//*************getAlt Function************************************************
void getAlt(){
  temp = ALT.readTemperature();
  pressure = ALT.readPressure();
  humid = ALT.readHumidity();

  altStamp = millis();
}

//*************writeOther Function************************************************
void writeOther(){
  OtherData.print(OtherflushTotal);
  OtherData.flush();
  flushcount++;
  usb.print("Flush Number : ");
  usb.print(flushcount);
  usb.print("    Time: ");
  usb.println(millis()/(long)1000);
  OtherflushTotal = "";
  count = 0;
}

//*************writeGPS Function************************************************
void writeGPS(){
  GPSdata.print(GPSflushTotal);
  GPSdata.flush();
  gpsFlushcount++;
  usb.print("GPS Flush Number : ");
  usb.print(gpsFlushcount);
  usb.print("    Time: ");
  usb.println(millis()/(long)1000);
  GPSflushTotal = "";
  gpsCount = 0;
}


//*************MAKEFILES************************************************
bool makeFiles()                            // FUNCTION to create the files on the SD card
{                                           // 
  bool rtn = true;                          // set a boolean flag of rtn to be true

  char gpsfile[15];                         // create a pointer for a character string of 15 for gps string
  char otherdatafile[15];                   // create a pointer for a character string of 15 for other data strings
  
  strcpy(gpsfile, "GPSLOG05.TXT");          // set the string name to the pointer
  strcpy(otherdatafile, "AODATA05.TXT");    // set the string name to the pointer
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
                                            //
  GPSdata = SD.open(gpsfile, FILE_WRITE);   // write a new file for the gps file on the SD card
  if(!GPSdata)                              // if the file failed to be created
  {                                         // 
    usb.print("Failed to create ");         // display failed to create on the serial monitor
    usb.println(gpsfile);                    // followed by the filename
    rtn = false;                            // set the boolean flag rtn to false
  }                                         // end 
                                            //        
  OtherData = SD.open(otherdatafile, FILE_WRITE); // write a new file for the gps file on the SD card
  if(!OtherData)                                // if the file failed to be created
  {                                             // 
    usb.print("Failed to create ");             // display failed to create on the serial monitor
    usb.println(otherdatafile);                 // followed by the filename
    rtn = false;                                // set the boolean flag rtn to false
  }                                             //
                                                //
                                                //
//  Logfile = SD.open(logfilename, FILE_WRITE);   // do the same thing as the last 2 file openings, but
//  if(!Logfile)                                  // for the log file.
//  {                                             //
//    usb.print("Failed to create ");             //
//    usb.println(logfilename);                   //   
//    rtn = false;                                //
//  }                                             //
                                                //
  return rtn;                                   // 
}

//*************SIGNAL (NEEDED FOR GPS)************************************************
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
}

//*************USEINTERRUPT (NEEDED FOR GPS)************************************************
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

//*************LOOOOOOOOPPPPPPP************************************************
void loop() {
  imuSensor(&imuRead);
  altSensor(&altRead);
  gpsSensor(&gpsRead);
  OtherWriteSensor(&otherWrite);
  GPSWriteSensor(&gpsWrite);
}

