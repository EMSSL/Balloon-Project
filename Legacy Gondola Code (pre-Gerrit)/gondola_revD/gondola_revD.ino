// gondola.ino
// This file contains the code for the main program that controls the gondola
// sensors and electronic equipment on the High Altitude Wind Controlled 
// System (HAWCS).

// Libraries
#include <Wire.h>                 // Wire library for I2C/TWI devices
#include <Adafruit_Sensor.h>      // Unified sensor drivers from Adafruit
#include <Adafruit_HMC5883_U.h>   // Magnetometer library
#include <SPI.h>                  // SPI communication library
#include <Adafruit_BMP183.h>      // Barometer, altitude sensor
#include <Adafruit_GPS.h>         // GPS library
#include <SoftwareSerial.h>       // Software serial library
#include <SD.h>                   // SD card library 
#include <avr/sleep.h>            // Sleep library

// Pins, #define because that's the arduino way
#define pin_pitot_read    A0      // analog in for the pitot tube, from pressure transducer
#define pin_accel_readX   A1      // analog in for the x axis accelerometer
#define pin_accel_readY   A2      // analog in for the y axis accelerometer
#define pin_accel_readZ   A3      // analog in for the z axis accelerometer
#define pin_XBEE_rx       14      // xbee receive pin RX
#define pin_XBEE_tx       15      // xbee transmit pin TX
#define pin_GPS_rx        16      // gps receive pin rx
#define pin_GPS_tx        17      // gps transmit pin tx
#define pin_I2C_sda       20      // serial data line for I2C devices
#define pin_I2C_scl       21      // serial clock for I2C devices
#define pin_GPS_pps       22      // gps pulse-per-second pin... do we actually use this???
#define pin_COMP_rp       24      // compass 
#define pin_XBEE_rts      26      // xbee ready to send pin
#define pin_XBEE_cts      28      // xbee clear to send pin
#define pin_ERROUT_comp   29      // compass pin for error
#define pin_XBEE_dtr      30      // data terminal ready pin for the xbee
#define pin_XBEE_rst      31      // xbee reset pin, normally high. pull low to reset
#define pin_ERROUT_gps    32      // gps initialize pin for error
#define pin_ERROUT_accl   33      // accelerometer pin for error
#define pin_ERROUT_sd     34      // sd card initialize pin for error
#define pin_ERROUT_file   35      // sd card file write pin for error
#define pin_ERROUT_xbee   36      // xbee pin for error
#define pin_ERROUT_baro   38      // barometer pin for error
#define pin_bailout       40      // pin to activate the cutdown pin high
#define pin_SD_cd         48      // sd card chip select pin
#define pin_BARO_ss       49      // barometer slave select pin
#define pin_MISO          50      // master in slave out pin for comunication SPI
#define pin_MOSI          51      // master out slave in pin for comunication SPI
#define pin_SCK           52      // serial clock pin for SPI
#define pin_SD_ss         53      // slave select pin for sd card
// missing a cut down circuit pin 
//  need to create new cutdown circuit based on relays
//  need to test to verify the mega can dissipate enough current via a digital pin to trip the relay
//  maybe tie several digital pins together, prevent pack currents using a diode and add currents?
//  maybe thats a dumb idea....



#define flushThresh       9
#define writeThresh       3
#define transThresh       10
#define address 0x1E //0011110b, I2C 7bit address of HMC5883

// Debugging
#define GPSECHO false

// Get world time from GPS and write it to other data file, NaN if no

// Error codes
enum errorCodes 
{
	INITFAIL,
	WRITEFAIL,
	WARN,
	GENERAL,
  FATAL
};


// Global variables
bool tellMe = true;
bool fatalError = false;
bool writeData = false;                 // boolean operator, true == writeData to SD card, false == do not write
bool flushData = false;                 // boolean operator, true == flushData to SD card, false == do not flush
bool compassReady = true;             // Brute for flag for dealing with a non-existant compass object
File GPSdata;                           // file name for the GPS data file to write on the SD card
File OtherData;                         // file name for the other data file to write on the SD card
unsigned long initialTime = millis();   // begin the millis timer on the arduino in case gps time is lost
bool usingInterrupt = false;      
unsigned int flushTimer = 0;            // flush timer counter for the GPS file
unsigned int otherTimer = 0;            // flush timer counter for the otehr data file
unsigned int transTimer = 0;            // timer to transmit data via XBee
unsigned long TNAUT = millis();         // get initial millis time
unsigned long TC = 0;                   // current time value in milliseconds
bool bail_flag = false;                 // cutdown flag set to false
bool BMPdetect = false;                 // does the barometer initialize
bool trans_flag = false;                // transmit data to the computer


// // Global objects
Adafruit_BMP183 barometer = Adafruit_BMP183(pin_BARO_ss);       // Using hardware serial, only cs pin is passed to ctor
Adafruit_GPS GPS(&Serial2);                                     // GPS object on s2 line
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(101); // ctor arg is sensor ID (for I2C)

// // Function declaration
uint8_t parseHex(char c);                 // declare a function to read a Hex value and return the decimal equivalent
void displayError(errorCodes c, int p);   // 
bool makeFiles();                         // declare a function to make two files on the SD card
void initPins();                          // declare a function to initialize pins
void useInterrupt(boolean v);             // declare a function to use the interrupts for the GPS

void setup() 
{
	// Open serial lines
  // Only need to open the line if not done so when the object was instantiated
	Serial.begin(115200);                   // open the serial line to the computer with high baud rate
 
	initPins();                             // run init pins function
	
	 // Init barometer
	 if(!barometer.begin())
	 {
	 	if(tellMe) {Serial.println("Barometer initialization failed");}
    //Serial3.println("Barometer failure");
    displayError(INITFAIL, pin_ERROUT_baro);
    BMPdetect = false;
	 }
  BMPdetect = true;
	
	// Init GPS
   GPS.begin(9600);
   GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Getting RMA and GGA data (see http://www.gpsinformation.org/dale/nmea.htm)
   GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    //GPS will refresh 1 time a second (doesn't mean you will get data that often)
   GPS.sendCommand(PGCMD_NOANTENNA);             //This turns off the atenna status
   useInterrupt(true);
	
	// Init Accelerometer: removed 07/07/2016 by CDY due to the fact that the pitot measures 
	//                     -2.5 to +2.5 V and cannot operate if the AREF pin is set to 3Vo 
	//                     from the accelerometer
  // Using the AREF pin as the voltage reference for the accelerometer
  // analogReference(EXTERNAL);
	
	// Init SD card
	if(!SD.begin(pin_SD_ss))
	{
		if (tellMe){
		  Serial.println("SD Card initialization failed");}
    //Serial3.println("SD card failure");
    fatalError = true;
    displayError(INITFAIL, pin_ERROUT_sd);
	}
	
	// Make files
  if(!makeFiles())
	{
    if (tellMe){
      Serial.println("File initialization failed");}
      //Serial3.println("File creation failure");
      fatalError = true;
      displayError(INITFAIL, pin_ERROUT_file);	
	}
  // Headers
  OtherData.print("Time"); OtherData.print(", "); OtherData.print("rawAcclX"); OtherData.print(", "); OtherData.print("rawAcclY"); OtherData.print(", ");
  OtherData.print("rawAcclZ"); OtherData.print(", "); OtherData.print("pressure"); OtherData.print(", "); OtherData.print("temperature"); OtherData.print(", "); 
  OtherData.print("altitude"); OtherData.print(", "); OtherData.print("rawCompX"); OtherData.print(", "); OtherData.print("rawCompY"); OtherData.print(", "); 
  OtherData.print("rawCompZ"); OtherData.print(", "); OtherData.print("nomHeading"); OtherData.print(", "); OtherData.println("PitotRaw");
	
//	// Init Compass
// Serial.println(compass.begin());
//	if(compass.begin())
//  {
//    if(tellMe){
//      Serial.println("Compass Initialized");}
//      compassReady = true;
//  }
//  else
//  {
//    if(tellMe){
//      Serial.println("No compass found");}
//      //Serial3.println("Compass failure");
//    compassReady = false;
//    if(tellMe){
//      Serial.println("HMC5883 initialization failed");}
//      displayError(INITFAIL, pin_ERROUT_comp);
//  }

  // Init compass
  Wire.begin(); //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  // Init XBee
  Serial3.begin(19200);
  Serial3.println("Serial3 began");
  
  if(tellMe){Serial.println("Setup Complete");}
}

/********************************************************************************************
 * BEGIN LOOP 
 *******************************************************************************************/
void loop() 
{
  if(tellMe){Serial.println("Start Loop");}
  
  flushTimer++;                     // increment the flushTimer counter by one
  otherTimer++;                     // increment the otherTimer counter by one
  transTimer++;                     // increment the transTimer counter by one
  
  if(tellMe){                       // if you wanna know,
    Serial.print("FlushTimer ");    // print flush timer
    Serial.println(flushTimer);     // print the value of flush timer
    Serial.print("OtherTimer ");    // print other timer
    Serial.println(otherTimer);     // print the value of other timer
    Serial.print("TransTimer ");    // print trans timer
    Serial.println(transTimer);     // print the value of trans timer
  }
  
  if (flushTimer > flushThresh)     // if the flushTimer is greater than the flushThreshhold
  {                                 // then
    flushTimer = 0;                 // reset the flushTimer counter
    flushData = true;               // and set the boolean flag for flusing data to true
  }                                 // else do nothing
  
  if (otherTimer > writeThresh)     // if otherTimer is greater than the threshhold to flush to SD card
  {                                 // then
    otherTimer = 0;                 // reset the otherTimer counter
    writeData = true;               // set the boolean flag for writing data to true
  }                                 // else do nothing 
  
  if (transTimer > transThresh)     // transTimer is greater than the threshhold to transmit via xbee
  {                                 // then
    transTimer = 0;                 // reset the transmit timer to zero 
    trans_flag = true;              // and set the transmit flag to high so it transmits
  }                                 // else do nothing
    
  if(tellMe){
    Serial.print("FlushData ");
    Serial.println(flushData);
    Serial.print("WriteData ");
    Serial.println(writeData);
    }  
  if(tellMe){
    Serial.println("Start GPS");
    }
  
  // Get GPS Data 
  String sGpsTime;                  // write the gps time to a string
  if (! usingInterrupt)             // if the software interrupts are NOT being used
  {
    char c = GPS.read();            // get new character string of GPS output
    if (GPSECHO)                    // if GPSECHO is true,
      if (c) Serial.print(c);       // and if c is true, the print c. What is ECHO????
    if(tellMe){Serial.println("GPS Interupt Running");}      
  }
  
  // Parse GPS Data
  if (GPS.newNMEAreceived())        // if newNEMAreceived is true
  {
    char *stringptr = GPS.lastNMEA();                           // pointer to the last NMEA string

    if (GPS.fix)                                                // if the GPS has a fix
    {                                                           //
      if (strstr(stringptr, "RMC") || strstr(stringptr, "GGA")) // if the string is either RMC or GGA data
      {                                                         //
        GPSdata.println(stringptr);                             // print the string to the GPS data file
      }                                                         //
    }                                                           //
    else                                                        //
    {                                                           //
      Serial.println("No GPS fix");                             // otherwise, print to the serial monitor that there is not a fix
      GPSdata.println("You're killin me smalls...");
    }                                                           //
      
    if (GPS.parse(stringptr))         // this also sets the newNMEAreceived() flag to false
    {                                 // if the gps parse flag is high
      sGpsTime += GPS.hour;           // separate out the hour
      sGpsTime += GPS.minute;         // separate out the minute
      sGpsTime += GPS.seconds;        // separate out the seconds
      sGpsTime += GPS.milliseconds;   // separate out the milliseconds
    }                                 //
    else                              //
    {                                 //
      sGpsTime += "NaN";              // otherwise return NaN
    }                                 //
  }                                   //
  else                                //
  {                                   //
     sGpsTime += millis();            // increment the gps time using the arduino millisecond clock onboard     
  }
//
  // Get Accelerometer data
  int rawAcclX = analogRead(pin_accel_readX);
  int rawAcclY = analogRead(pin_accel_readY);
  int rawAcclZ = analogRead(pin_accel_readZ);
  if(tellMe){Serial.print("acclX ");Serial.print(rawAcclX);Serial.println(" m_per_s2");Serial.print("acclY ");Serial.print(rawAcclY);Serial.println(" m_per_s2");Serial.print("acclZ ");Serial.print(rawAcclZ);Serial.println(" m_per_s2");}

  // Get Barometer data
  float pressure = barometer.getPressure();
  float temperature = barometer.getTemperature();  
  float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
  float altitude = barometer.getAltitude(seaLevelPressure);
  if(tellMe){Serial.print("Pressure ");Serial.print(pressure);Serial.println(" N_per_m2");} 
//
////  // fix this 07/02/2016
////  // will not recognize that the compass is not plugged in. think's theres a compass there regardless
////  // Get Compass Data
//  float rawCompX = 0.0;
//  float rawCompY = 0.0;
//  float rawCompZ = 0.0;
//  float nomHeading = 0.0;  
//  Serial.println(compassReady);
//  if(compassReady)
//  {
//    if(tellMe){Serial.println("Compass must be ready");}
//    
//    sensors_event_t event;                  // Compass event obj 
//    sensor_t sensor;
//    compass.getSensor(&sensor); 
//    if(tellMe){Serial.println(sensor.sensor_id);}
//    
//    compass.getEvent(&event);
//    if(tellMe){Serial.println("get event");}
//
//    rawCompX = event.magnetic.x;
//    rawCompY = event.magnetic.y;
//    rawCompZ = event.magnetic.z;
//    nomHeading = atan2(rawCompX, rawCompZ); // Note that this is only a nominal heading. Need to post all 3 to get true heading.
//  }
//  if(tellMe){Serial.print("Nominal Heading");Serial.println(nomHeading);}

// // //        new trial compass code
  int rawCompX,rawCompY,rawCompZ; //triple axis data

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    rawCompX = Wire.read()<<8; //X msb
    rawCompX |= Wire.read(); //X lsb
    rawCompZ = Wire.read()<<8; //Z msb
    rawCompZ |= Wire.read(); //Z lsb
    rawCompY = Wire.read()<<8; //Y msb
    rawCompY |= Wire.read(); //Y lsb
  }

  if(tellMe){
    Serial.print("RawX: ");Serial.print(rawCompX); Serial.println(" uT");
    Serial.print("RawY: ");Serial.print(rawCompY); Serial.println(" uT");
    Serial.print("RawZ: ");Serial.print(rawCompZ); Serial.println(" uT");}

// // //        end new compass code

  float nomHeading = 0.0;  
  nomHeading = atan2(rawCompX, rawCompY);


  // get pitot data
  int pitotVal = analogRead(pin_pitot_read);
  float pitotPrs = (((analogRead(pin_pitot_read)*(5.0 / 1023)) / 5) - 0.5) / 0.2; // kPa
  if(tellMe){Serial.print("Pitot press: ");Serial.print(pitotPrs); Serial.println(" kPa");}

  
  // Write the data to files
  if(writeData)     // if we need to write the data to the other data file, then let's write it
  {
    Serial.println("Wrote Data");
    otherTimer = 0;
    OtherData.print(sGpsTime); OtherData.print(", "); OtherData.print(rawAcclX); OtherData.print(", "); OtherData.print(rawAcclY); OtherData.print(", ");
    OtherData.print(rawAcclZ); OtherData.print(", "); OtherData.print(pressure); OtherData.print(", "); OtherData.print(temperature); OtherData.print(", "); 
    OtherData.print(altitude); OtherData.print(", "); OtherData.print(rawCompX); OtherData.print(", "); OtherData.print(rawCompY); OtherData.print(", "); 
    OtherData.print(rawCompZ); OtherData.print(", "); OtherData.print(nomHeading); OtherData.print(", "); OtherData.println(pitotVal);
    
    writeData = false;
  }

    

  // Flush every ... 5 seconds?
  if(flushData)
  {
    if(tellMe){Serial.println("Flushed");}
    
    flushTimer = 0;
    GPSdata.flush();
    OtherData.flush();
    flushData = false;
  }


  // // // // // // // // //    \/ CUTDOWN \/    // // // // // // // // // 
  // Is everything good? Need to cutdown and bail out yet?
  // missing the cutdown functionality, add from other code here
  TC = millis() - TNAUT;    // calculates the current time (in ms) since beginning the loop
  if (pressure < 69000) { // if pressure is less than threshhold (ie height is greater than 3100 meters),
     if (BMPdetect) {
        if (TC > 10800000) {                // and if the barometer is detected, and the time is greater than 3hours * 3600 * 1000
          OtherData.flush();               // first flush all other data currently stored 
          GPSdata.flush();                 // first flush all other data currently stored 
          OtherData.println("CUTDOWN SYSTEM ACTIVATED!"); 
          OtherData.println("CUTDOWN SYSTEM ACTIVATED!"); 
          OtherData.println("CUTDOWN SYSTEM ACTIVATED!"); 
          GPSdata.println("CUTDOWN SYSTEM ACTIVATED!"); 
          GPSdata.println("CUTDOWN SYSTEM ACTIVATED!"); 
          GPSdata.println("CUTDOWN SYSTEM ACTIVATED!"); 
          OtherData.print(sGpsTime); OtherData.print(", "); OtherData.print(rawAcclX); OtherData.print(", "); OtherData.print(rawAcclY); OtherData.print(", ");
          OtherData.print(rawAcclZ); OtherData.print(", "); OtherData.print(pressure); OtherData.print(", "); OtherData.print(temperature); OtherData.print(", "); 
          OtherData.print(altitude); OtherData.print(", "); OtherData.print(rawCompX); OtherData.print(", "); OtherData.print(rawCompY); OtherData.print(", "); 
          OtherData.print(rawCompZ); OtherData.print(", "); OtherData.print(nomHeading);  OtherData.print(", "); OtherData.println(pitotVal);
          OtherData.flush();
          GPSdata.flush();
          digitalWrite(pin_bailout, HIGH);     // turn the LED on (HIGH is the voltage level)
          delay(30000);                        // wait 30 seconds to ensure the tethers are cut
          digitalWrite(pin_bailout, LOW);      // turn the LED off by making the voltage LOW
          bail_flag = true;                    // set the bail flag to true for the xbee
        }
     }
  }
  // // // // // // // // //    /\ CUTDOWN /\    // // // // // // // // // 



  /************************* Tell the XBee what's up  ******************************/
  
  if (bail_flag)
  {
    // send all the cutdown message to the computer with data of current position, altitude, etc. 
    Serial3.println("CUTDOWN SYSTEM ACTIVATED!");
    Serial3.println("CUTDOWN SYSTEM ACTIVATED!");
    // Serial3.println(stringptr);
    Serial3.print(sGpsTime); Serial3.print(", "); Serial3.print(rawAcclX); Serial3.print(", "); Serial3.print(rawAcclY); Serial3.print(", ");
    Serial3.print(rawAcclZ); Serial3.print(", "); Serial3.print(pressure); Serial3.print(", "); Serial3.print(temperature); Serial3.print(", "); 
    Serial3.print(altitude); Serial3.print(", "); Serial3.print(rawCompX); Serial3.print(", "); Serial3.print(rawCompY); Serial3.print(", "); 
    Serial3.print(rawCompZ); Serial3.print(", "); Serial3.print(nomHeading); OtherData.println(pitotVal);
    Serial3.println("CUTDOWN SYSTEM ACTIVATED!");
    Serial3.println("CUTDOWN SYSTEM ACTIVATED!");
  }

    /********************** SEND TO LAPTOP ************************/

  // send info for the current time step with error messages via xbee to the computer
  if (trans_flag){
    Serial3.print("Time: "); Serial3.println(sGpsTime); 
    // Serial3.print("GPS string: "); Serial3.println(stringptr);
    Serial3.print("Xdd: "); Serial3.println(rawAcclX); 
    Serial3.print("Ydd: "); Serial3.println(rawAcclY); 
    Serial3.print("Zdd: "); Serial3.println(rawAcclZ); 
    Serial3.print("Pressure: "); Serial3.print(pressure); Serial3.println(" Pa");
    Serial3.print("Temperature: "); Serial3.print(temperature); Serial3.println(" degC");
    Serial3.print("Altitude: "); Serial3.print(altitude); Serial3.println(" m");
    Serial3.print("XMagField: "); Serial3.print(rawCompX); Serial3.println(" uT");
    Serial3.print("YMagField: "); Serial3.print(rawCompY); Serial3.println(" uT");
    Serial3.print("ZMagField: "); Serial3.print(rawCompY); Serial3.println(" uT");
    Serial3.print("Heading: "); Serial3.print(nomHeading); Serial3.println(" deg");
    Serial3.print("Pitot pressure: "); Serial3.print(pitotPrs); Serial3.println(" kPa");
    trans_flag = false;
  }
  



  delay(1000);        // now just you wait 1 second Mr. ...
  if(tellMe){Serial.println("End of loop");}
  
}
/**************************************************************************************************************
 * END LOOP
**************************************************************************************************************/



/*************************************************************************************************************
 * ***********************************************************************************************************
 * ***********************************************************************************************************
 * // // // // // FUNCTIONS WRITTEN BELOW // // // // // 
 */

uint8_t parseHex(char c)    
{
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A')+10;
}
//
void displayError(errorCodes code, int pin)
{
	// Based on what the error code is, blink out a signal to a pin.
	// Choose pin based on the error code?
  int timesBlink;
  bool blinkForever = false;
	switch(code)
	{
		case INITFAIL:
			timesBlink = 3;
      // Serial.println("Initialization Failure");
		break;
    case FATAL:
      timesBlink = 0;
      blinkForever = true;
      // Serial.println("Fatal Error");
		default:
		//code
		break;
	}
	for (int i = 0; i < timesBlink; i++)
	{
		digitalWrite(pin,HIGH);
		delay(1000);
		digitalWrite(pin,LOW);
		delay(1000);
	}
  while(blinkForever)
  {
    digitalWrite(pin,HIGH);
    delay(1000);
    digitalWrite(pin,LOW);
    delay(1000);
  }
}

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
		Serial.print("Failed to create ");      // display failed to create on the serial monitor
		Serial.println(gpsfile);                // followed by the filename
		rtn = false;                            // set the boolean flag rtn to false
	}                                         // end 
                                                  //	      
	OtherData = SD.open(otherdatafile, FILE_WRITE); // write a new file for the gps file on the SD card
	if(!OtherData)                                  // if the file failed to be created
	{                                               // 
		Serial.print("Failed to create ");            // display failed to create on the serial monitor
		Serial.println(otherdatafile);                // followed by the filename
		rtn = false;                                  // set the boolean flag rtn to false
	}                                               //    
	                                                //
	return rtn;                                     // 
}                                                 //



void initPins()                       // FUNCTION written to initialize pins on the arduino mega
{                                     // initialize the following pins to the following modes
	//pinMode(pin_ERROUT_baro, OUTPUT);   // set the barometer error pin to output mode
	pinMode(pin_ERROUT_sd, OUTPUT);     // set the SD card error pin to output mode
  pinMode(pin_BARO_ss, OUTPUT);       // set the barometer slave select pin to output mode
  pinMode(pin_SD_ss, OUTPUT);         // set the SD card slave select pin to output mode
  pinMode(pin_XBEE_dtr, OUTPUT);         // set the SD card slave select pin to output mode
}                                     // end function
//
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  #ifdef UDR0
      if (GPSECHO)
        if (c) UDR0 = c;  
  #endif
}

void useInterrupt(boolean v) 
{
  if (v) 
  {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else 
  {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
