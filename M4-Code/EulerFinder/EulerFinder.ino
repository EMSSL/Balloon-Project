/*! \file EulerFinder.ino
 *  \author Gerrit Motes
 *  \date September 10 2019
 *  \version 1.0
 *  \brief EMSSL Balloon Project Euler/Quaternion Finder
 *  \details 
 *  
 *  The BNO055 Absolute Orientation IMU does not contain any information about
 *  rotation sequences used to determine its output Euler Angles. Because of this,
 *  a method for determining the rotation sequence in the rotational environment used
 *  (high angular displacement about Z, low oscillating angular displacement about X
 *  and Y) is needed to ensure accurate data processing. 
 *  
 *  The method used will involve comparing measured quaternion data to measured euler
 *  angle data, cycling through 12 unique conversions from quaternions to euler angles,
 *  using the 12 unique intrinsic rotation sequences, and matching the appropriate
 *  output angles to the measured output angles from the BNO055.
 */

// libraries
#include <Wire.h>             // wire library (needed for some sensors)
#include <Adafruit_Sensor.h>  // master sensor library
#include <Adafruit_BNO055.h>  // IMU library

#define usb Serial            // renames Serial as usb
#define imuPower 12
Adafruit_BNO055 IMU = Adafruit_BNO055(55,BNO055_ADDRESS_B);
char* outputText;

void setup() {
  delay(5000);    // used to read setup debug code
  usb.begin(115200);

  pinMode(imuPower, OUTPUT);
  digitalWrite(imuPower, HIGH);
  delay(250);

  int i = 0;
  while(!IMU.begin() && i < 100){
    i++;
  }
  if(i > 99){
    usb.println(F("IMU has derped for the 100th time, eff dis ish!"));
    while(true){}
  }
  else{
    usb.println(F("IMU is functional!"));
  }
  usb.println(F("DATA DATA DATA\n\n EULER ANGLES X,Y,Z  :  QUATERNION STUFF\n"));
}

void loop() {
  delay(100);

  imu::Vector<3> BN_eul = IMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Quaternion BN_quat = IMU.getQuat();
  
  sprintf(outputText,"%f,%f,%f:%f,%f,%f%f",BN_eul.x(),BN_eul.y(),BN_eul.z(),BN_quat.w(),BN_quat.x(),BN_quat.y(),BN_quat.z());
  usb.println(outputText);
}
