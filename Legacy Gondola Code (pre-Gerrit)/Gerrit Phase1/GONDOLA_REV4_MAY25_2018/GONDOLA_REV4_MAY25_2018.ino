#include <Wire.h>
#include <SPI.h>

//GPS and SD libraries (CRITICAL)
#include <Adafruit_GPS.h>
#include <SD.h>

//Sensor Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>

//Extra libraries
#include <utility/imumaths.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define usb; Serial
#define gps Serial2
#define xbee Serial3

#define SPI_SCK 52
#define SPI_MISO 50
#define SPI_MOSI 51
#define SD_CS 44
#define ALT2_CS 53
#define XBEE_RTS 42
#define XBEE_CTS 40
#define XBEE_RST 38
#define CUTDOWN_TRIGGER 30
#define ERROR_LED 32
#define PITOT A15

#define GPS_PWR 22
#define SD_PWR 23
#define IMU_PWR 34
#define ALT1_PWR 26
#define ALT2_PWR 24

#define PITOT_BIAS 16 // current bias is for pitot labelled "b"

#define flushthresh 2 // NUMBER OF LOOPS BEFORE FLUSHING OCCURS
#define ALT_THRESH 6000

#define gndlvlPress (1013.25)

File GPSdata;
File OtherData;
File Logfile;

Adafruit_GPS GPS(&gps); // GPS
#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean); 

Adafruit_BNO055 IMU = Adafruit_BNO055(); // IMU OBJECT DEFINITION
Adafruit_BME280 ALT1; //  ALT2 I2C 
Adafruit_BME280 ALT2(ALT2_CS); // ALT 1 HARDWARE SPI

int globalLoopCount;
int globalFlushCount;
int cutdownTimer;
int cutdownTransmitAttempt;
String GPSflushTotal;
String OtherflushTotal;
String errorCodes;
char prev_gpsfile[15];
char prev_otherdatafile[15];
char gpsfile[15];
char otherdatafile[15];

bool fatalError = false;
bool isError = false;
bool sdError = false;
bool fileError = false;
bool gpsError = false;
bool imuError = false;
bool alt1Error = false;
bool alt2Error = false;
bool xbeeError = false;
bool enablewatchDog = false;
bool isSending = false;

float temp1 = 0;
float press1 = 0;
float humid1 = 0;
float altitude1 = 0;
float temp2 = 0;
float press2 = 0;
float humid2 = 0;
float altitude2 = 0;
float pitotreading = 0;
float windspeed = 0;
#define gndlvlPress (1013.25)

float gpsTime = 0;
float millisTime = 0;
float xbeeTime = 0;

bool testMode = false;

void setup() {
  wdt_reset(); // this prevents infinite loops from occuring with the watchdog reset
  if(testMode){
    usb.begin(115200);
    usb.println("Initializing...");
  }

  digitalWrite(GPS_PWR, HIGH);
  digitalWrite(SD_PWR, HIGH);
  digitalWrite(IMU_PWR, HIGH);
  digitalWrite(ALT1_PWR, HIGH);
  digitalWrite(ALT2_PWR, HIGH);
  digitalWrite(CUTDOWN_TRIGGER, LOW);
  digitalWrite(ERROR_LED, LOW);
  delay(250);
  

  if(testMode){
    usb.println("pins initialized");
  }

  xbee.begin(9600);                                 // initializes xbee
  if(!xbee){          // initializes Xbee if the recall command hasn't been initiated 
      if(testMode){
        usb.println("Xbee failed to initialize!");
      }
      xbeeError = true;
  }
  else{
    xbeeError = false;
    if(testMode){
    usb.println("xbee initialized");
    }
    xbee.write("initializing");
    digitalWrite(XBEE_RTS, LOW);
    digitalWrite(XBEE_RST, HIGH);
  }

  if(!SD.begin(SD_CS)){
    if(testMode){
      usb.println("SD Card failed to initialize!");
    }
    sdError = true;
  }                                 // initializes the SD files
  else if(!makeFiles()){
    if(testMode){
      usb.println("File Initialization Failed!");
    }
    fileError = true;
  }
  else{
    sdError = fileError = false;
    if(testMode){
    usb.println("SD and Files initialized");
    }
  }

  if(!gps){                       // if they fail to initiate.
    if(testMode){
      usb.println("GPS ERROR!");
    }
    gpsError = true;
  }
  else{
    GPS.begin(9600);                                // Setup GPS 
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Getting RMA and GGA data (see http://www.gpsinformation.org/dale/nmea.htm)
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    //GPS will refresh 1 time a second (doesn't mean you will get data that often)
    GPS.sendCommand(PGCMD_NOANTENNA);             
    useInterrupt(true);
    if(testMode){
      usb.println("GPS initialized");
    }
    gpsError = false;
  }

  if(!IMU.begin()){                                 // starts imu
    if(testMode){
      usb.println("BNO055 IMU Initialization Error");
    }
    imuError = true;  
  }
  else{
    imuError = false;
    if(testMode){
      usb.println("BNO055 IMU Initialized");
    }
  }

  if(!ALT1.begin()){                           // starts the first altimeter
    if(testMode){
      usb.println("BME280 Sensor 1 Initialization Error");
    }
    alt1Error = true;
  }
  else{
    alt1Error = false;
   if(testMode){
      usb.println("BME280 Sensor 1 Initialized");
    } 
  }

  if(!ALT2.begin()){                           // starts the first altimeter
    if(testMode){
      usb.println("BME280 Sensor 2 Initialization Error");
    }
    alt2Error = true;
  }
  else{
    alt2Error = false;
   if(testMode){
      usb.println("BME280 Sensor 2 Initialized");
    } 
  }

  if(!testMode){
    ledMessage();
  }

  if(!(sdError || fileError || gpsError || imuError || alt1Error || alt2Error)){
     if(testMode){
      usb.println("Initialization Complete!");
      }
    if(!xbeeError){
      xbee.write("Complete!\n");
    }
  }
  else if(!(sdError || fileError || gpsError)){
    if(testMode){
      usb.println("Complete with sensor errors");
    }
    if(!xbeeError){
      xbee.write("Minor Errors\n");
    }
    sensorFix();
  }
  else{
    fatalErrorhandling();
  }
 if(testMode){
    usb.println("****Leaving Setup!****");
  } 
}

//***************************************************************************************************
//************************************   LOOOOOOP()  **************************************
void loop() {

  wdt_enable(WDTO_8S);
  if(!isSending){  
    if(enablewatchDog){
      wdt_enable(WDTO_15MS);
    }

  
    
    if (isError && ((globalLoopCount % 10) == 0)){
      if(testMode){
        usb.println("leaving loop to sensorFix()");
      }
      sensorFix();
      if(testMode){
        usb.println("leaving sensorFix back to loop()");
      }
    }
  
    globalLoopCount++;
    if(testMode){
      usb.print("Loop#: ");
      usb.println(globalLoopCount);
    }
//                                                 ////////////////////xbee looking for received message
//    if(xbee.available()){                                                                       //
//      char command[7];                                                                          //  
//      while(xbee.available()){                                                                  //
//        command[6] = command[5];                                                                //
//        command[5] = command[4];                                                                //
//        command[4] = command[3];                                                                //
//        command[3] = command[2];                                                                //
//        command[2] = command[1];                                                                //
//        command[1] = command[0];                                                                //
//        command[0] = xbee.read();                                                               //
//        if(command[0]==command[1]==command[2]==command[3]==command[4]==command[5]==command[6]){ //
//          xbeeReceive(command[0]);                                                              //
//        }                                                                                       //
//      }                                                                                         //
//    }                                                                                           //
//    if(xbeeError){                                                                              //
//      if(testMode){                                                                             //
//        usb.println("resetting xbee");                                                          //
//      }                                                                                         //
//      digitalWrite(XBEE_RST, LOW); // resets the xbee by pulling the rst pin low and then high  //
//      delay(250);                                                                               //
//      digitalWrite(XBEE_RST, HIGH);                                                             //
//      delay(250);                                                                               //
//      bool ctsStatus = digitalRead(XBEE_CTS); // checks the CTS pin, if it is high (telling the arduino to stop
//      if(!ctsStatus){               // sending data) then the error has not been cleared        //
//        if(testMode){                                                                           //
//          usb.println("alt1 reset successful");                                                 //
//        }                                                                                       //
//        xbeeError = false;                                                                      //
//      }                                                                                         //
//      else{                                                                                     //
//        if(testMode){                                                                           //
//        usb.println("xbee reset failed");                                                       //
//        }                                                                                       //
//        xbeeError = true;                                                                       //
//      }                                                                                         //
//    }                                                                                            //
//                                          /////////////////////////////////////////////////////////
    //GPS data aquisition                                    
    if(GPS.newNMEAreceived()){
      if(testMode){
        usb.println("new GPS sentence detected");
      }
      String flerp = GPS.lastNMEA();
      GPSflushTotal = GPSflushTotal + "\n" + flerp;
      if(testMode){
        usb.println("leaving loop to gpscheck()");
      }
      bool gotTime = gpscheck(flerp);                                // checks for errors and gets time stamp for
      if(testMode){
        usb.println("leaving gpsCheck() to loop midway through gps gathering");
      }
      int milliCheck = millis() ;                                               // later use.
      while(!GPS.newNMEAreceived() && ((millis() - milliCheck) <= 3000)){}
      if (!gotTime){                                              // waits for next sentence, then adds it to 
        if(testMode){
          usb.println("didn't get gpstime first time");
        }
        flerp = GPS.lastNMEA();
        GPSflushTotal = GPSflushTotal + "\n" + flerp;
        if(testMode){
         usb.println("leaving loop to gpscheck()");
        }
        gotTime = gpscheck(flerp);
        if(testMode){
          usb.println("leaving gpsCheck() to loop bottom of gps gathering");
        }
        if(!gotTime){
          gpsTime = gpsTime + ((millis()- millisTime)/1000); // uses the millis() function to add
        }                                                   // to last GPS fix time if the gpscheck()
      }                                                     // function doesn't produce a new GPS time
      else{
        if(testMode){
          usb.println("got GPS time from first time");
        }
        GPSflushTotal = GPSflushTotal + "\n" + GPS.lastNMEA();
      }
      GPSflushTotal = GPSflushTotal + "\n";
      if(GPSflushTotal.length() < 10){                // checks for lack of sentences coming from GPS
        gpsError = true;                              // even without satellite fix, empty sentences should
        if(testMode){
          usb.println("gps info too short, gpsError is now true");
          usb.println("GPS info = ");
          usb.println(GPSflushTotal);
        }
      }
    }

//                                               ////////////////////xbee looking for received message
//    if(xbee.available()){                                                                       //
//      char command[7];                                                                          //  
//      while(xbee.available()){                                                                  //
//        command[6] = command[5];                                                                //
//        command[5] = command[4];                                                                //
//        command[4] = command[3];                                                                //
//        command[3] = command[2];                                                                //
//        command[2] = command[1];                                                                //
//        command[1] = command[0];                                                                //
//        command[0] = xbee.read();                                                               //
//        if(command[0]==command[1]==command[2]==command[3]==command[4]==command[5]==command[6]){ //
//          xbeeReceive(command[0]);                                                              //
//        }                                                                                       //
//      }                                                                                         //
//    }                                                                                           //
//    if(xbeeError){                                                                              //
//      if(testMode){                                                                             //
//        usb.println("resetting xbee");                                                          //
//      }                                                                                         //
//      digitalWrite(XBEE_RST, LOW); // resets the xbee by pulling the rst pin low and then high  //
//      delay(250);                                                                               //
//      digitalWrite(XBEE_RST, HIGH);                                                             //
//      delay(250);                                                                               //
//      bool ctsStatus = digitalRead(XBEE_CTS); // checks the CTS pin, if it is high (telling the arduino to stop
//      if(!ctsStatus){               // sending data) then the error has not been cleared        //
//        if(testMode){                                                                           //
//          usb.println("alt1 reset successful");                                                 //
//        }                                                                                       //
//        xbeeError = false;                                                                      //
//      }                                                                                         //
//      else{                                                                                     //
//        if(testMode){                                                                           //
//        usb.println("xbee reset failed");                                                       //
//        }                                                                                       //
//        xbeeError = true;                                                                       //
//      }                                                                                         //
//    }                                                                                            //
//     
    
    OtherflushTotal = OtherflushTotal + globalLoopCount+ "," + gpsTime + ",";
  
    sensors_event_t BN_accelEvent,BN_gyroEvent,BN_magEvent,BN_eulerEvent, BN_gravEvent; // CREATES SENSOR EVENTS FOR THE IMU
      imu::Vector<3> BN_acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // creates vectors for each event
      imu::Vector<3> BN_gyro = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      imu::Vector<3> BN_mag = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      imu::Vector<3> BN_eul = IMU.getVector(Adafruit_BNO055::VECTOR_EULER);
      imu::Vector<3> BN_grav = IMU.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    
    if(imuError){
      usb.println("buildstring imu error");
      OtherflushTotal = OtherflushTotal + "e,e,e,e,e,e,e,e,e,e,e,e,e,e,e";
    }
    else{
      if(testMode){
        usb.println("buildstring: no imu error string build");
      }
              // builds the IMU data into a parse-able string.
        OtherflushTotal = OtherflushTotal + BN_acc.x() + "," + BN_acc.y() + "," + BN_acc.z();
        OtherflushTotal = OtherflushTotal + "," + BN_gyro.x() + "," + BN_gyro.y() + "," + BN_gyro.z();
        OtherflushTotal = OtherflushTotal + "," + BN_mag.x() + "," + BN_mag.y() + "," + BN_mag.z();
        OtherflushTotal = OtherflushTotal + "," + BN_eul.x() + "," + BN_eul.y() + "," + BN_eul.z();
        OtherflushTotal = OtherflushTotal + "," + BN_grav.x() + "," + BN_grav.y() + "," + BN_grav.z();
    }
    // checks for IMU errors, the IMU readings will be taken in the buildString() function
      uint8_t system_status, self_test_results, system_error;     
      system_status = self_test_results = system_error = 0;
      IMU.getSystemStatus(&system_status, &self_test_results, &system_error);
      if(testMode){
        usb.print("IMU system_error = ");
        usb.println(system_error);
      }
      if((system_error != 0) || (BN_acc.x()+BN_acc.y()+BN_acc.z()+BN_gyro.x()+BN_gyro.y()+BN_gyro.z()+BN_mag.x()+BN_mag.y()+BN_mag.z()+BN_eul.x()+BN_eul.y()+BN_eul.z()+BN_grav.x()+BN_grav.y()+BN_grav.z() == 0.00)){
        imuError = true;
        if(testMode){
          usb.println("imu error occured");
        }
      }
      else{
        if(testMode){
          usb.println("imu error didn't occur");
        }
        imuError = false;
      }

//                                                   ////////////////////xbee looking for received message
//    if(xbee.available()){                                                                       //
//      char command[7];                                                                          //  
//      while(xbee.available()){                                                                  //
//        command[6] = command[5];                                                                //
//        command[5] = command[4];                                                                //
//        command[4] = command[3];                                                                //
//        command[3] = command[2];                                                                //
//        command[2] = command[1];                                                                //
//        command[1] = command[0];                                                                //
//        command[0] = xbee.read();                                                               //
//        if(command[0]==command[1]==command[2]==command[3]==command[4]==command[5]==command[6]){ //
//          xbeeReceive(command[0]);                                                              //
//        }                                                                                       //
//      }                                                                                         //
//    }                                                                                           //
//    if(xbeeError){                                                                              //
//      if(testMode){                                                                             //
//        usb.println("resetting xbee");                                                          //
//      }                                                                                         //
//      digitalWrite(XBEE_RST, LOW); // resets the xbee by pulling the rst pin low and then high  //
//      delay(250);                                                                               //
//      digitalWrite(XBEE_RST, HIGH);                                                             //
//      delay(250);                                                                               //
//      bool ctsStatus = digitalRead(XBEE_CTS); // checks the CTS pin, if it is high (telling the arduino to stop
//      if(!ctsStatus){               // sending data) then the error has not been cleared        //
//        if(testMode){                                                                           //
//          usb.println("alt1 reset successful");                                                 //
//        }                                                                                       //
//        xbeeError = false;                                                                      //
//      }                                                                                         //
//      else{                                                                                     //
//        if(testMode){                                                                           //
//        usb.println("xbee reset failed");                                                       //
//        }                                                                                       //
//        xbeeError = true;                                                                       //
//      }                                                                                         //
//    }                                                                                            //
//     
  
    // Takes altimeter 1 readings if there is no identified error    
      if(!alt1Error){
        temp1 = ALT1.readTemperature();
        press1 = ALT1.readPressure();
        humid1 = ALT1.readHumidity();
        altitude1 = ALT1.readAltitude(gndlvlPress);
  
        if((press1 > 108380) || !(press1 == press1) || (press1 < 0)){ // checks for potential erros in 
          alt1Error = true;                                           // 108380 Pa is highest atmospheric
          if(testMode){
            usb.println("alt1 error detected");
          }
        }                                                             // press. ever recorded.
        else{
          alt1Error = false;
          if(testMode){
            usb.println("no alt1 errors");
          }
        }
      }
      
    // ALT1 portion
    if(alt1Error){
      usb.println("buildstring alt1 error");
      OtherflushTotal = OtherflushTotal + ",e,e,e,e";
    }
    else{
      usb.println("buildstring: no alt1 error string build");
      OtherflushTotal = OtherflushTotal + "," + temp1 + "," + press1 + "," + humid1 + "," + altitude1;
    }

//                                                 ////////////////////xbee looking for received message
//    if(xbee.available()){                                                                       //
//      char command[7];                                                                          //  
//      while(xbee.available()){                                                                  //
//        command[6] = command[5];                                                                //
//        command[5] = command[4];                                                                //
//        command[4] = command[3];                                                                //
//        command[3] = command[2];                                                                //
//        command[2] = command[1];                                                                //
//        command[1] = command[0];                                                                //
//        command[0] = xbee.read();                                                               //
//        if(command[0]==command[1]==command[2]==command[3]==command[4]==command[5]==command[6]){ //
//          xbeeReceive(command[0]);                                                              //
//        }                                                                                       //
//      }                                                                                         //
//    }                                                                                           //
//    if(xbeeError){                                                                              //
//      if(testMode){                                                                             //
//        usb.println("resetting xbee");                                                          //
//      }                                                                                         //
//      digitalWrite(XBEE_RST, LOW); // resets the xbee by pulling the rst pin low and then high  //
//      delay(250);                                                                               //
//      digitalWrite(XBEE_RST, HIGH);                                                             //
//      delay(250);                                                                               //
//      bool ctsStatus = digitalRead(XBEE_CTS); // checks the CTS pin, if it is high (telling the arduino to stop
//      if(!ctsStatus){               // sending data) then the error has not been cleared        //
//        if(testMode){                                                                           //
//          usb.println("alt1 reset successful");                                                 //
//        }                                                                                       //
//        xbeeError = false;                                                                      //
//      }                                                                                         //
//      else{                                                                                     //
//        if(testMode){                                                                           //
//        usb.println("xbee reset failed");                                                       //
//        }                                                                                       //
//        xbeeError = true;                                                                       //
//      }                                                                                         //
//    }                                                                                            //
     
  
    // Takes altimeter 2 readings if there is no identified error
      if(!alt2Error){
        temp2 = ALT1.readTemperature();
        press2 = ALT1.readPressure();
        humid2 = ALT1.readHumidity();
        altitude2 = ALT1.readAltitude(gndlvlPress);
  
        if((press2 > 108380) || !(press2 == press2) || (press2 < 0)){ // checks for potential erros in 
          alt2Error = true;
          if(testMode){
            usb.println("alt2 error detected");
          }
        }
        else{
          alt2Error = false;
          if(testMode){
            usb.println("no alt2 errors");
          }
        }
      }
  
    // ALT2 portion of string build
    if(alt2Error){
      usb.println("buildstring alt2 error");
      OtherflushTotal = OtherflushTotal + ",e,e,e,e";
    }
    else{
      usb.println("buildstring: no alt2 error string build");
      OtherflushTotal = OtherflushTotal + "," + temp2 + "," + press2 + "," + humid2 + "," + altitude2;
    }

//                                               ////////////////////xbee looking for received message
//    if(xbee.available()){                                                                       //
//      char command[7];                                                                          //  
//      while(xbee.available()){                                                                  //
//        command[6] = command[5];                                                                //
//        command[5] = command[4];                                                                //
//        command[4] = command[3];                                                                //
//        command[3] = command[2];                                                                //
//        command[2] = command[1];                                                                //
//        command[1] = command[0];                                                                //
//        command[0] = xbee.read();                                                               //
//        if(command[0]==command[1]==command[2]==command[3]==command[4]==command[5]==command[6]){ //
//          xbeeReceive(command[0]);                                                              //
//        }                                                                                       //
//      }                                                                                         //
//    }                                                                                           //
//    if(xbeeError){                                                                              //
//      if(testMode){                                                                             //
//        usb.println("resetting xbee");                                                          //
//      }                                                                                         //
//      digitalWrite(XBEE_RST, LOW); // resets the xbee by pulling the rst pin low and then high  //
//      delay(250);                                                                               //
//      digitalWrite(XBEE_RST, HIGH);                                                             //
//      delay(250);                                                                               //
//      bool ctsStatus = digitalRead(XBEE_CTS); // checks the CTS pin, if it is high (telling the arduino to stop
//      if(!ctsStatus){               // sending data) then the error has not been cleared        //
//        if(testMode){                                                                           //
//          usb.println("alt1 reset successful");                                                 //
//        }                                                                                       //
//        xbeeError = false;                                                                      //
//      }                                                                                         //
//      else{                                                                                     //
//        if(testMode){                                                                           //
//        usb.println("xbee reset failed");                                                       //
//        }                                                                                       //
//        xbeeError = true;                                                                       //
//      }                                                                                         //
//    }                                                                                            //
     

  
    for(int r = 1; r <= 10; r++){                               // averages 10 pitot readings over
           pitotreading = pitotreading + (float)analogRead(PITOT);  // 1/10th of a second
           delay(10);
        }
        pitotreading = pitotreading / 10;
        pitotreading = pitotreading - 1024/2 - PITOT_BIAS;          //sets pitot value range to -512 to 
                                                                    // 512 and accounts for 0 bias
        float windspeed = sqrt((2*abs(pitotreading)*(2000/512))/1.225); // uses dynamic pressure-freestream 
                                                                        //velocity equation and conversion of
                                                                        //byte values to pressure values in 
                                                                        //Pascals to find the velocity of 
                                                                        //the pitot tube
        if(windspeed < 0){            // loop checks for any negative velocity values (corresponding to motion 
          windspeed = windspeed*-1;       //opposite the positive line of motion), and changes the respective 
        }
  
    OtherflushTotal = OtherflushTotal + "," + pitotreading + "," + windspeed + "::\n";
  
//                                                 ////////////////////xbee looking for received message
//    if(xbee.available()){                                                                       //
//      char command[7];                                                                          //  
//      while(xbee.available()){                                                                  //
//        command[6] = command[5];                                                                //
//        command[5] = command[4];                                                                //
//        command[4] = command[3];                                                                //
//        command[3] = command[2];                                                                //
//        command[2] = command[1];                                                                //
//        command[1] = command[0];                                                                //
//        command[0] = xbee.read();                                                               //
//        if(command[0]==command[1]==command[2]==command[3]==command[4]==command[5]==command[6]){ //
//          xbeeReceive(command[0]);                                                              //
//        }                                                                                       //
//      }                                                                                         //
//    }                                                                                           //
//    if(xbeeError){                                                                              //
//      if(testMode){                                                                             //
//        usb.println("resetting xbee");                                                          //
//      }                                                                                         //
//      digitalWrite(XBEE_RST, LOW); // resets the xbee by pulling the rst pin low and then high  //
//      delay(250);                                                                               //
//      digitalWrite(XBEE_RST, HIGH);                                                             //
//      delay(250);                                                                               //
//      bool ctsStatus = digitalRead(XBEE_CTS); // checks the CTS pin, if it is high (telling the arduino to stop
//      if(!ctsStatus){               // sending data) then the error has not been cleared        //
//        if(testMode){                                                                           //
//          usb.println("alt1 reset successful");                                                 //
//        }                                                                                       //
//        xbeeError = false;                                                                      //
//      }                                                                                         //
//      else{                                                                                     //
//        if(testMode){                                                                           //
//        usb.println("xbee reset failed");                                                       //
//        }                                                                                       //
//        xbeeError = true;                                                                       //
//      }                                                                                         //
//    }                                                                                            //
     
  
    errorCheck();
  
    if((((altitude1 > ALT_THRESH) && (altitude2 > ALT_THRESH)) || ((altitude1 > ALT_THRESH) && alt2Error) || ((altitude2 > ALT_THRESH) && alt1Error) || (alt1Error && alt2Error)) && (cutdownTimer == 0)){
      cutdownTimer = millis();
      if(!xbeeError){
        String cutdown = "";
        cutdown = cutdown + "cutdown timer started, send '$$$$$$' every 5 minutes to keep cutdown from occuring";
        sendMessage(cutdown);
        if(testMode){
          usb.println("cutdown timer started, send '$$$$$$' every 5 minutes to keep cutdown from occuring");  
        }
      }
    }
    else if((((altitude1 > ALT_THRESH) && (altitude2 > ALT_THRESH)) || ((altitude1 > ALT_THRESH) && alt2Error) || ((altitude2 > ALT_THRESH) && alt1Error) || (alt1Error && alt2Error)) && ((millis()- cutdownTimer) > 30000)){
      if(testMode){
        usb.println("***********CUTDOWN INITIATED*************");
      }
      if(!xbeeError){
        String cutdown = "";
        cutdown = cutdown + "***********CUTDOWN INITIATED*************";
        sendMessage(cutdown);
        delay(50);
        sendMessage(GPSflushTotal);
        delay(50);
        sendMessage(OtherflushTotal);
        delay(50);
        sendMessage(errorCodes);
        delay(50);
      }
      GPSdata.print(GPSflushTotal);
      delay(50);
      OtherData.print(OtherflushTotal);
      delay(50);
      Logfile.print(errorCodes);
      delay(50);
      GPSdata.flush();
      delay(50);
      OtherData.flush();
      delay(50);
      Logfile.flush();
      delay(50);
  
      digitalWrite(IMU_PWR, LOW);
      digitalWrite(ALT1_PWR, LOW);
      digitalWrite(ALT2_PWR, LOW);
  
      GPSflushTotal = "";
      OtherflushTotal = "";
      errorCodes = "";
      wdt_reset();
      digitalWrite(CUTDOWN_TRIGGER, HIGH);
      while(1){
        if(GPS.newNMEAreceived()){
          GPSflushTotal = GPSflushTotal + "\n" + GPS.lastNMEA();
          while(!GPS.newNMEAreceived()){}
          GPSflushTotal = GPSflushTotal + "\n" + GPS.lastNMEA();
        }
        GPSdata.print(GPSflushTotal);
        delay(50);
        GPSdata.flush();
        delay(50);
        sendMessage(GPSflushTotal);
        delay(50);
        GPSflushTotal = "";
      } 
      
      //delay(30000);
      //digitalWrite(CUTDOWN_TRIGGER, LOW);
    }
    else if (!(((altitude1 > ALT_THRESH) && (altitude2 > ALT_THRESH)) || ((altitude1 > ALT_THRESH) && alt2Error) || ((altitude2 > ALT_THRESH) && alt1Error) || (alt1Error && alt2Error))){
      cutdownTimer = 0;
      if(testMode){
        usb.println("cutdown timer reset");
      }
    }
    else{
      if(testMode){
        usb.print("cutdownTimer = "); usb.println(millis() - cutdownTimer);
      }
    }
  
    if(globalLoopCount % flushthresh == 0){
      sdError = fatalError = !SD.exists(otherdatafile);
      if(!(sdError || fileError || gpsError || imuError || alt1Error || alt2Error)){
       if(testMode){
          usb.println("flushing!");
        }
        if(!xbeeError){
          xbee.write("flush\n");
          sendMessage(GPSflushTotal);
          delay(50);
          sendMessage(OtherflushTotal);
          delay(50);
          sendMessage(errorCodes);
          delay(50);
        }
      GPSdata.print(GPSflushTotal);
      delay(50);
      OtherData.print(OtherflushTotal);
      delay(50);
      Logfile.print(errorCodes);
      delay(50);
      GPSdata.flush();
      delay(50);
      OtherData.flush();
      delay(50);
      Logfile.flush();
      delay(50);
  
      GPSflushTotal = "";
      OtherflushTotal = "";
      errorCodes = "";
      
    }
    else if(!(sdError || fileError || gpsError)){
      if(testMode){
        usb.println("flushing");
      }
      if(!xbeeError){
          xbee.write("flush\n");
          sendMessage(GPSflushTotal);
          delay(50);
          sendMessage(OtherflushTotal);
          delay(50);
          sendMessage(errorCodes);
          delay(50);
        }
      GPSdata.print(GPSflushTotal);
      delay(50);
      OtherData.print(OtherflushTotal);
      delay(50);
      Logfile.print(errorCodes);
      delay(50);
      GPSdata.flush();
      delay(50);
      OtherData.flush();
      delay(50);
      Logfile.flush();
      delay(50);
  
      GPSflushTotal = "";
      OtherflushTotal = "";
      errorCodes = "";
      sensorFix();
    }
    else{
      if(testMode){
        usb.println("fatal error prior to SD write");
      }
      fatalErrorhandling();
    }
    }
    delay(300);
  }
  wdt_reset();
}

//***************************************************************************************************
//************************************   MAKE FILES FUNCTION   **************************************
bool makeFiles()                            // FUNCTION to create the files on the SD card
{
  usb.println("entering makeFiles()");// 
  bool rtn = true;                          // set a boolean flag of rtn to be true

// commented out, filenames set to global vars  ***********************************
//  char gpsfile[15];                         // create a pointer for a character string of 15 for gps string
//  char otherdatafile[15];                   // create a pointer for a character string of 15 for other data strings
  
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
  Logfile = SD.open("LOGFILE.TXT", FILE_WRITE);   // do the same thing as the last 2 file openings, but
  if(!Logfile)                                  // for the log file.
  {                                             //
    usb.print("Failed to create LOGFILE.TXT");             //
                     //   
    rtn = false;                                //
  }                                             //
                                                //
  return rtn; 
  usb.println("leaving makeFiles()");// 
}


//***************************************************************************************************
//**************************************  SENSOR FIX FUNCTION  *************************************
void sensorFix(){
  usb.println("entering sensorfix");
  if(alt1Error){
    usb.println("resetting alt1");
    digitalWrite(ALT1_PWR, LOW);  // resets the altimeter via the transistor gate
    delay(250);
    digitalWrite(ALT1_PWR, HIGH);
    delay(250);

    if(!ALT1.begin()){            // re-checks the status of the altimeter
      usb.println("alt1 reset failed");
      alt1Error = true;
    }
    else{
      usb.println("alt1 reset successful");
      alt1Error = false;
    }
  }
  
  if(alt2Error){
    usb.println("resetting alt2");
    digitalWrite(ALT2_PWR, LOW);  // resets the altimeter via the transistor gate
    delay(250);
    digitalWrite(ALT2_PWR, HIGH);
    delay(250);

    if(!ALT2.begin()){          // re-checks the status of the altimeter
      usb.println("alt2 reset failed");
      alt2Error = true;
    }
    else{
      usb.println("alt2 reset successful");
      alt2Error = false;
    }
  }
  
  if(imuError){
    usb.println("resetting imu");
    digitalWrite(IMU_PWR, LOW); // resets the imu by pulling the transistor gate pin low and then high
    delay(250);
    digitalWrite(IMU_PWR, HIGH);
    delay(250);
    if(!IMU.begin()){
      imuError = true;
    }
    else{
      uint8_t system_status, self_test_results, system_error;     // checks for potential errors in sensor
      system_status = self_test_results = system_error = 0;
      IMU.getSystemStatus(&system_status, &self_test_results, &system_error);
      if(system_error != 0){
        usb.println("imu reset failed");
        imuError = true;
      }
      else{
        usb.println("imu reset successful");
        imuError = false;
      }
    }
  }
    
  if(xbeeError){
    usb.println("resetting xbee");
    digitalWrite(XBEE_RST, LOW); // resets the xbee by pulling the rst pin low and then high
    delay(250);
    digitalWrite(XBEE_RST, HIGH);
    delay(250);
    bool ctsStatus = digitalRead(XBEE_CTS); // checks the CTS pin, if it is high (telling the arduino to stop
    if(!ctsStatus){               // sending data) then the error has not been cleared
      usb.println("alt1 reset successful");
      xbeeError = false;
    }
    else{
      usb.println("xbee reset failed");
      xbeeError = true;
    }
  }
  usb.println("leaving sensorfix()");
}


//***************************************************************************************************
//*********************************  FATAL ERROR HANDLING FUNCTION  *********************************
void fatalErrorhandling(){
  if(testMode){
    usb.println("entering fatalErrorhandling()");
  }
  
  if(sdError){                                            // error handling for SD card error
    if(testMode){
      usb.println("fatalerrorhandling resetting SD card..");
    }
    digitalWrite(SD_PWR, LOW);
    delay(250);
    digitalWrite(SD_PWR, HIGH);
    delay(250);

    GPSdata = SD.open(gpsfile, FILE_WRITE);                 // attempts to re-open current files
    Logfile = SD.open("LOGFILE.TXT", FILE_WRITE);
    OtherData = SD.open(otherdatafile, FILE_WRITE);
                                                           // if this does not work...
    if((!GPSdata) || (!OtherData) || (!Logfile) || (!SD.exists(otherdatafile))){
      if(testMode){  
        usb.println("fatalerrorhandling: sd card restart failed, reset");
      }                                       
      sdError = true;                                             //reset the arduino             
    }
    else{
      if(testMode){
        usb.println("fatalerrorhandling: sd card reset successful");
      }
      sdError = false;                                        
    }        
  }

  if(fileError){
    if(testMode){
      usb.println("re-initializing file");
    }
    digitalWrite(SD_PWR, LOW);                          // resets SD card reader
    delay(250);
    digitalWrite(SD_PWR, HIGH);
    delay(250);

    strcpy(prev_gpsfile, gpsfile);                     // updates the previous gps and other file
    strcpy(prev_otherdatafile, otherdatafile);         // names prior to calling makeFiles()
                                                           // attempts to creat new files
                                                           // if this does not work...
    if(!makeFiles()){
      if(testMode){
        usb.println("fatalerrorhandling: makefiles failed after sd reset, reset arduino"); 
      }                                         
      fileError = true;                                             //reset the arduino             
    }
    else{                                                   // if reset is successful, set error flag
                                                            // to false, and dump error message and 
      fileError = false;                                    // linking filenames to each file on SD
      if(testMode){                                                      // card.
        usb.println("SD card and file successfully reset");
      }
      Logfile.print("\n fatalErrorHandling reset of SD card initiated on loop# ");
      Logfile.print(globalLoopCount);
      Logfile.print(".\nError Codes: ");
      Logfile.print(errorCodes);
      Logfile.print("\nPlease refer to files ");
      Logfile.print(prev_gpsfile);
      Logfile.print(" and ");
      Logfile.print(prev_otherdatafile);
      Logfile.println(" for previous data related to this flight run.");
      
      GPSdata.print("\n fatalErrorHandling reset of SD card initiated on loop# ");
      GPSdata.print(globalLoopCount);
      GPSdata.print("\nPlease refer to files ");
      GPSdata.print(prev_gpsfile);
      GPSdata.println(" for previous data related to this flight run.");

      OtherData.print("\n fatalErrorHandling reset of SD card initiated on loop# ");
      OtherData.print(globalLoopCount);
      OtherData.print("\nPlease refer to files ");
      OtherData.print(prev_gpsfile);
      OtherData.println(" for previous data related to this flight run.");                                    
    }
  }

  if(gpsError){
    usb.println("restarting gps");
    digitalWrite(GPS_PWR, LOW);
    delay(250);
    digitalWrite(GPS_PWR, HIGH);
    delay(250);

    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Getting RMA and GGA data (see http://www.gpsinformation.org/dale/nmea.htm)
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    //GPS will refresh 1 time a second (doesn't mean you will get data that often)
    GPS.sendCommand(PGCMD_NOANTENNA);             
    useInterrupt(true);

    int gpsErrorTimer = millis();
    while((!GPS.newNMEAreceived()) && ((millis()-gpsErrorTimer) < 3000)){}
    String gpsErrTest = "";
    if(GPS.newNMEAreceived()){
      gpsErrTest = gpsErrTest + GPS.lastNMEA();
    }
    if(gpsErrTest.length() > 4){
      if(testMode){
        usb.println("gps restart worked");
      }
      gpsError = false;
    }
    else{
      if(testMode){
        usb.println("gps restart did not work");
      }
      gpsError = true;
    }
  }
  usb.println("leaving fatalerrorhandling()");

  if(sdError || fileError || gpsError){
    if(testMode){
      usb.println("failed to fix errors, restarting");
    }
    reset();
  }
}

//***************************************************************************************************
//**************************************  RESET FUNCTION  *******************************************
void reset(){
  if(testMode){
    usb.println("entering reset()");
  }

  delay(100);
  
  enablewatchDog = true;
  if(testMode){
    usb.println("leaving reset()");
  }
  loop(); 
}

//***************************************************************************************************
//**************************************  ERROR CHECK FUNCTION  *************************************
void errorCheck(){
  if(testMode){
    usb.println("entering errorcheck()");
  }
  if(sdError || gpsError || fileError){        // determines if there is a fatal error
    fatalError = true;
    isError = true;
  }
  else if(alt1Error || alt2Error || imuError || xbeeError){   // if not, determines if there is a 
    isError = true;                                                         // non-fatal error
  }
  else{
    isError = false;                                                        // if no errors are indicated, then 
    fatalError = false;                                                     // clears isError and fatalError
  }
  errorCodes = "";
  errorCodes = errorCodes + "\n" + globalLoopCount + "," + gpsTime;
  if(isError){
    if(sdError){
      errorCodes = errorCodes + ",1";
    }
    if(fileError){
      errorCodes = errorCodes + ",2";
    }
    if(gpsError){
      errorCodes = errorCodes + ",3";
    }
    if(alt1Error && alt2Error){
      errorCodes = errorCodes + ",4";
    }
    if(alt1Error && !alt2Error){
      errorCodes = errorCodes + ",5";
    }
    if(alt2Error && !alt1Error){
      errorCodes = errorCodes + ",6";
    }
    if(imuError){
      errorCodes = errorCodes + ",7";
    }
    if(xbeeError){
      errorCodes = errorCodes + ",9";
    }
  }
  else{
    errorCodes = errorCodes + ",0";
  }
  if(testMode){
    usb.print("error codes = ");  usb.println(errorCodes);
    usb.println("leaving errorcheck");
  }
}

////***************************************************************************************************
////**************************************  XBEE RECEIVE FUNCTION  *************************************
//void xbeeReceive(char command){
//  if(testMode){
//    usb.println("entering xbee receive()");
//  }
//
//         // checks to see if the command reaches the minimum length to be 
//  if(command == "%"){    // considered relevant (all commands are at least 6 chars
//    xbeeTime = millis();                  // six percents = an xbee comms ping to see if xbee can 
//    if(testMode){
//      usb.println("xbee ping received");
//    }
//  }                                       // still receive stuff from ground
//  else if(command == "?"){  // six question marks asks for status of system,
//    errorCheck();                             // build most recent error codes and send 
//    sendMessage(errorCodes);
//    xbeeTime = millis();
//    if(testMode){
//      usb.println("system status command received");
//      usb.println(errorCodes);
//    }
//    else if (!xbeeError){
//      sendMessage(errorCodes);
//    }
//  }
//  else if(command == "$"){   // six dollar signs = acknowledge, also used to prevent
//    cutdownTransmitAttempt = 0;      // cutdown from firing if there is a sensor malfunction
//    String acknowledge = "";
//    acknowledge = acknowledge + "\nacknowledged";
//    sendMessage(acknowledge);              // must be send every 5 minutes to reset cutdown timer
//    xbeeTime = millis();
//    if(testMode){
//      usb.println("acknowledge command received");
//    }
//  }
//  else if(command == "^"){   // initiates a cutdown procedure, regardless of sensors
//    String acknowledge = "";
//    acknowledge = acknowledge + "\n********************\nCUTDOWN COMMAND RECEIVED! CUTTING NOW!\n***************\n";
//    sendMessage(acknowledge);
//    if(testMode){
//      usb.println("\n********************\nCUTDOWN COMMAND RECEIVED! CUTTING NOW!\n***************\n");
//    }
//    errorCodes = errorCodes + "\n**************\nCUTDOWN FIRING, COMMENCE FINAL SAVE AND SEND\n************\n";
//    // flushes last bit of data to sd 
//    GPSdata.print(GPSflushTotal);
//    delay(50);
//    OtherData.print(OtherflushTotal);
//    delay(50);
//    Logfile.print(errorCodes);
//    delay(50);
//    GPSdata.flush();
//    delay(50);
//    OtherData.flush();
//    delay(50);
//    Logfile.flush();
//    delay(50);
//
//    if(testMode){
//      usb.println(GPSflushTotal);
//      usb.println(OtherflushTotal)
//      usb.println(errorCodes);
//    }
//
//    // relays same data to the ground
//    sendMessage(GPSflushTotal);
//    delay(50);
//    sendMessage(OtherflushTotal);
//    delay(50);
//    sendMessage(errorCodes);
//    delay(50);
//
//    GPSflushTotal = "";
//    OtherflushTotal = "";
//    errorCodes = "";
//
//    // turns off non-essential recovery sensors
//    digitalWrite(IMU_PWR, LOW);
//    digitalWrite(ALT1_PWR, LOW);
//    digitalWrite(ALT2_PWR, LOW);
//
//    // fires cutdown for 30 seconds, to ensure that if the nichrome doesn't break, then the battery doesn't flame on.
//    if(testMode){
//      usb.println("CUTDOWN TRIGGER IS FIRING");
//    }
//    wdt_reset();
//    digitalWrite(CUTDOWN_TRIGGER, HIGH);
//    while(1){
//      if(GPS.newNMEAreceived()){
//        GPSflushTotal = GPSflushTotal + "\n" + GPS.lastNMEA();
//        while(!GPS.newNMEAreceived()){}
//        GPSflushTotal = GPSflushTotal + "\n" + GPS.lastNMEA();
//      }
//      GPSdata.print(GPSflushTotal);
//      delay(50);
//      GPSdata.flush();
//      delay(50);
//      sendMessage(GPSflushTotal);
//      delay(50);
//      GPSflushTotal = "";
//    }
//    //delay(30000);
////    if(testMode){
////      usb.println("CUTDOWN TRIGGER IS TURNING OFF");
////    }
//    //digitalWrite(CUTDOWN_TRIGGER, LOW);
////    while(1){
////      freeFallOperation();
////    }                               
//  }
//  else{                                       // catch all that asks for a repeat if the program does
//    String errormess = "";
//    errormess = errormess + "do not understand, please try again";
//    sendMessage(errormess);                       // not recognize the command.
//    if(testMode){
//      usb.println(command);
//    }
//    xbeeTime = millis();
//  }
//  
//  if((millis() - xbeeTime) > 300000){         // all of the commands reset the xbeeTime variable, or the
//    if(testMode){
//      usb.println("ping time exceeded, xbeeError is now true (from xbeeReceive)");
//    }
//    xbeeError = true;                         // time since last contact with ground. If this timer reaches
//  }                                           // 5 minutes, then it is assumed that contact is lost, and 
//  if(testMode){
//    usb.println("leaving xbeeReceive()");
//  }
//}

//***************************************************************************************************
//********************************* SEND MESSAGE FUNCTION *******************************************
void sendMessage(String derp){  // derp is the GPS data, lerp is the other sensor data                          // GPSflushTotal and OtherflushTotal,
  isSending = true;                                           // respectively
  if(testMode){                               
    usb.println("sendMessage is being used");  // debug mode status message
  }
  int k = 0;    // tracks character location in sting
  int cts_count = 0;    // tracks number of consecutive attempts to send with no response from xbee
  digitalWrite(XBEE_RTS, LOW);    // RTS (ready to send) tells the xbee to start manipulating CTS
  delay(10);                      // to control incoming traffic from the arduino serial output.
  while(k<derp.length()){
    while((digitalRead(XBEE_CTS) == LOW) && (k<derp.length())){
      xbee.write(derp[k]);
      usb.write(derp[k]);
      k++;
    }
    
    int blerp = (digitalRead(XBEE_CTS)); // while sending, check the CTS pin, and if it is high,
    if(blerp == 1){                      // count up the cts_count variable by 1
      cts_count++;
      if(testMode){
        usb.print(cts_count);usb.print(",");
      }
    }
    else{
      cts_count = 0;                      // otherwise, reset the cts_count
    }
    
    if (cts_count >= 100){                // if CTS is held high for too long, when something is wrong           
        Logfile.print(globalLoopCount);    // with the xbee. log the error with loop# and time
        Logfile.print(",");                         // and indicate the problem to the program
        Logfile.print(gpsTime);
        Logfile.print(",");
        Logfile.println("Xbee failed to send and was reset"); 
        xbeeError = true;
        goto ABORT_SEND; 
    }
    
  }
  ABORT_SEND:
  if(testMode){
    usb.println("leaving sendMessage()");
  }
  isSending = false;
}

//***************************************************************************************************
//**************************************  GPS CHECK FUNCTION  *******************************************
bool gpscheck(String newNMEA){
  usb.println("entering gpscheck()");
  if(newNMEA.length() < 4){
    return false;
    usb.println("NMEA is too short");
  }
  else{
    char sentenceType[4];                                   // this will hold the 3 char sentence type
                                               // the char array format of sentenceType
    
    int dollarSign = newNMEA.indexOf('$');                          // sets a location index

    usb.print("var dollarsign = "); usb.println(dollarSign);
    
    for(int h = 0; h<= 3; h++){                             // reads in specific chars after the $ symbol
      sentenceType[h] = newNMEA.charAt((dollarSign + 3) + h);
    }
    String sentenceString = "";
    sentenceString = sentenceString + sentenceType;
    

     usb.print("var sentenceType = "); usb.println(sentenceType);
    
    if((sentenceString.indexOf("GGA") >= 0) || sentenceString.indexOf("RMC") >=0 ){                            // if the sentence type is GGA or RMC
      int comma1 = newNMEA.indexOf(',');                        // determine index locations of 1st and
      int comma2 = newNMEA.indexOf(',', (comma1 + 1));          // 2nd comma, and build a substring of 
      String newtime = newNMEA.substring((comma1+1), comma2);              // the content between those commas
      if(testMode){
        usb.print("newtime = "); usb.println(newtime);
      }
      if(newtime.length() > 4){
        gpsTime = newtime.toFloat();
        millisTime = millis();
        usb.print("var gpsTime = "); usb.println(gpsTime);
        return true;
      }
    }                                                           // this is the UTC time (see NMEA sentence
    else{                                                       // structure
      return false;                                             // otherwise, return false because all other
      usb.println("no suitable NMEA sentence found");
    }                                                           // sentences gathered do have have time
  }
  usb.println("leaving gpscheck()");
}

//***************************************************************************************************
//********************************* CRITICAL GPS-RELATED FUNCTIONS **********************************
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


//***************************************************************************************************
//**************************************  LED MESSAGE FUNCTION  *************************************
void ledMessage(){
  if(sdError){                              // 2 shorts for SD card error
    for(int w = 1; w <= 2; w++){
      digitalWrite(ERROR_LED, HIGH);
      delay(250);
      digitalWrite(ERROR_LED, LOW);
      delay(250);
    }
  }
  else if(fileError){                       // 3 shorts for file creation error
    for(int w = 1; w <= 3; w++){
      digitalWrite(ERROR_LED, HIGH);
      delay(250);
      digitalWrite(ERROR_LED, LOW);
      delay(250);
    }
  }
  else if(gpsError){                             // 4 shorts for GPS error
    for(int w = 1; w <= 4; w++){
      digitalWrite(ERROR_LED, HIGH);
      delay(250);
      digitalWrite(ERROR_LED, LOW);
      delay(250);
    }
  }
  else if(alt1Error && alt2Error){              // 5 shorts for both altimeters having an error
    for(int w = 1; w <= 5; w++){
      digitalWrite(ERROR_LED, HIGH);
      delay(250);
      digitalWrite(ERROR_LED, LOW);
      delay(250);
    }
  }
  else if(alt1Error && !alt2Error){             // 1 long, 2 shorts for alt1 error
    digitalWrite(ERROR_LED, HIGH);
    delay(1000);
    digitalWrite(ERROR_LED, LOW);
    delay(500);
    digitalWrite(ERROR_LED, HIGH);
    delay(250);
    digitalWrite(ERROR_LED, LOW);
    delay(500);
    digitalWrite(ERROR_LED, HIGH);
    delay(250);
    digitalWrite(ERROR_LED, LOW);
    delay(500);
  }
  else if(alt2Error && !alt1Error){             // 2 longs, 1 short for alt2 error
    digitalWrite(ERROR_LED, HIGH);
    delay(1000);
    digitalWrite(ERROR_LED, LOW);
    delay(500);
    digitalWrite(ERROR_LED, HIGH);
    delay(1000);
    digitalWrite(ERROR_LED, LOW);
    delay(500);
    digitalWrite(ERROR_LED, HIGH);
    delay(250);
    digitalWrite(ERROR_LED, LOW);
    delay(500);
  }
  else if(imuError){                            // 3 longs for IMU error
    digitalWrite(ERROR_LED, HIGH);
    delay(1000);
    digitalWrite(ERROR_LED, LOW);
    delay(500);
    digitalWrite(ERROR_LED, HIGH);
    delay(1000);
    digitalWrite(ERROR_LED, LOW);
    delay(500);
    digitalWrite(ERROR_LED, HIGH);
    delay(1000);
    digitalWrite(ERROR_LED, LOW);
    delay(500);
  }
//  else if(pitotError){                          // 1 short, 2 longs for pitot error
//    digitalWrite(ERROR_LED, HIGH);
//    delay(250);
//    digitalWrite(ERROR_LED, LOW);
//    delay(500);
//    digitalWrite(ERROR_LED, HIGH);
//    delay(1000);
//    digitalWrite(ERROR_LED, LOW);
//    delay(500);
//    digitalWrite(ERROR_LED, HIGH);
//    delay(1000);
//    digitalWrite(ERROR_LED, LOW);
//    delay(500);
//  }
  else if(xbeeError){                           // 2 shorts, 1 long for xbee error
    digitalWrite(ERROR_LED, HIGH);
    delay(250);
    digitalWrite(ERROR_LED, LOW);
    delay(500);
    digitalWrite(ERROR_LED, HIGH);
    delay(250);
    digitalWrite(ERROR_LED, LOW);
    delay(500);
    digitalWrite(ERROR_LED, HIGH);
    delay(1000);
    digitalWrite(ERROR_LED, LOW);
    delay(500);
  }
  else{
    digitalWrite(ERROR_LED, HIGH);
    delay(250);
    digitalWrite(ERROR_LED, LOW);
  }
}
