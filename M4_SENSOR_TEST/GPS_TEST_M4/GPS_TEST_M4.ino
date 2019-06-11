#include <Adafruit_Sensor.h>  // master sensor library
#include <Adafruit_GPS.h>     // GPS library

#define usb Serial            // renames Serial as usb
#define gps Serial1           // renames Serial2 as gps

Adafruit_GPS GPS(&gps);       // GPS object constructor

String gpsStuff = "";         // placeholder for NMEA sentences
bool usingInterrupts;
#define GPSECHO

void setup() {
  delay(10000);
  usb.begin(115200);
  usb.println(F("GPS TEST...setting up"));
  gpsStuff.reserve(350);      // statically allocates memory for a dynamic memory object
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Getting RMA and GGA data (see http://www.gpsinformation.org/dale/nmea.htm)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);    //
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);    //GPS will refresh 5 times a second (doesn't mean you will get data that often)
  GPS.sendCommand(PGCMD_NOANTENNA);             // kills the antenna status update
  useInterrupt(true);
  usb.println(F("GPS TEST: setup complete!"));
}

void loop() {
//  if(gps.available())
//  {
//    while(gps.available()){
//    char c = GPS.read();
//    usb.print(c);  
//    }
//  }
//  if(GPS.newNMEAreceived())
//  {
//    gpsStuff = GPS.lastNMEA();
//    usb.println(gpsStuff);
//  }
//  else
//  {
//    //usb.println(F("NOT YET"));
//  }
}

//SIGNAL(TIMER0_COMPA_vect) {
//  char c = GPS.read();
//  // if you want to debug, this is a good time to do it!
//  if (GPSECHO)
//    if (c) UDR0 = c;  
//    // writing direct to UDR0 is much much faster than usb.print 
//    // but only one character can be written at a time. 
//}

void useInterrupt(boolean v) {
  if(v)
  {
    usingInterrupts = true;
  }
  else
  {
    usingInterrupts = false;
  }
  
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
}



void serialEventRun()
{
  while(gps.available()){
    char c = GPS.read();
    #ifdef GPSECHO
      Serial.print(c);
    #endif
  }
}
