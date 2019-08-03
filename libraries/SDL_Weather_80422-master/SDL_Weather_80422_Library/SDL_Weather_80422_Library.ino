/*
  SDL_Weather_80422.cpp - Library for SwitchDoc Labs WeatherRack.
  SparkFun Weather Station Meters
  Argent Data Systems
  Created by SwitchDoc Labs July 27, 2014.
  Released into the public domain.
  Version 1.1 - updated constants to suppport 3.3V
  
  Supports WeatherPiArduino Board www.switchdoc.com

  REV - 0:   -  ,     -    , Code as obtained from: https://github.com/switchdoclabs/SDL_Weather_80422
  REV - A: Yoder, 8/11/2018, Modifications for S. Agrawal
  
*/


/*
 
PIN OUTS, REV - A
GND STATION GREEN   (Pin 1)   -> UNO A0 (analog pin 0)
GND STATION GREEN   (Pin 1)   -> 10kOhm resistor -> UNO 5V
GND STATION YELLOW  (Pin 2)   -> UNO GND
GND STATION RED     (Pin 3)   -> UNO 2 (digital pin 2)
GND STATION BLACK   (Pin 4)   -> UNO GND

*/

// libraries required
#include <Wire.h>               // wire library to communicate
#include <Time.h>               // time library to get times
#include "SDL_Weather_80422.h"  // custom library from vendor

// pin defines
#define pinLED     13   // LED connected to digital pin 13
#define pinAnem    2  // Anenometer connected to pin 18 - Int 5 - Mega   / Uno pin 2
#define pinRain    3  // Anenometer connected to pin 2 - Int 0 - Mega   / Uno Pin 3 
#define intAnem    0  // int 0 (check for Uno)
#define intRain    1  // int 1
// IN ADDITION, A0 (analog in pin 0) must be connected!!!

// initialize SDL_Weather_80422 library
SDL_Weather_80422 weatherStation(pinAnem, pinRain, intAnem, intRain, A0, SDL_MODE_INTERNAL_AD);

// DUNNO
uint8_t i;

// DEFINE variables as float for math
float currentWindSpeed;                               // wind speed
float currentWindGust;                                // wind gust
unsigned long t0;                                     // time

// setup function
void setup()
{
  Serial.begin(57600);                                // open serial port with computer, baud = 57600
  Serial.println("-----------");                      // print header
  Serial.println("WeatherArduino SDL_Weather_80422 Class Test");
  Serial.println("Version 1.1");
  Serial.println("-----------");
  Serial.println("Time [s] Speed [m/s] Direction [deg]");
       
  weatherStation.setWindMode(SDL_MODE_SAMPLE, 0.5);   // wind speed is averaged with this time (moving average filter)
  t0 = millis();                                      // get initial time from millis timer      
}

// run run run!
void loop()
{
  //Serial.println("----------------");                                   // print line of dashes inbetween readings
  currentWindSpeed = 0.44704*weatherStation.current_wind_speed()/1.6;     // get wind speed and convert to meters/second
  currentWindGust = 0.44704*weatherStation.get_wind_gust()/1.6;           // get wind gust and convert to meters/second

  // print results
  Serial.print(millis() - t0);
  Serial.print(", ");
  Serial.print(currentWindSpeed);
  Serial.print(", ");
  Serial.println(weatherStation.current_wind_direction());
  delay(50);
}
