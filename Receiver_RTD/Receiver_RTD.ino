#include <SPI.h>              // SPI digital comms library (needed for SD card)
#include <SD.h>               // SD library
#include <pt.h>               // protothread library
#include <xbeeAPI.h>          // xbee library (900HP S3B models in API SPI mode)

#define usb Serial            // renames Serial as usb

//Xbee1 OBJECT DEFINITION
#define XB1_SS (5)                          // M4 PIN FOR XBEE SPI SLAVE SELECT
#define XB1_ATTN (7)                        // M4 PIN FOR XBEE ATTN 
#define XB1_CMD (9)                         // M4 PIN NUMBER FOR COMMAND SIGNAL
#define XB1_LOC_COMMAND_DIO ('0')           // DIO NUMBER FOR LOCAL XBEE COMMAND PIN
#define XB1_LOC_CUTDOWN_DIO ('5')           // DIO NUMBER FOR LOCAL XBEE CUTDOWN PIN
#define XB1_DEST_COMMAND_DIO ('0')          // DIO NUMBER FOR DESTINATION COMMAND PIN
#define XB1_DEST_CUTDOWN_DIO ('5')          // DIO NUMBER FOR DESTINATION CUTDOWN
#define XB1_DEST_ADDR (0x0013A200417E3816)  // DESTINATION XBEE 64-BIT ADDRESS 
#define XB1_SPI_CLK (3000000)               // SPI BUS FREQUENCY FOR XBEE DATA
#define XB1_PRIMARY_MODE (2)                // 0 = FAST, 1 = ACCURATE, 2 = RECEIVER
#define XB1_MAX_PAYLOAD_SIZE 256                 // cannot exceed NP (256)
Xbee xbee = Xbee(XB1_SS, XB1_ATTN);         // CONSTRUCTOR FOR XBEE OBJECT
uint16_t XP1_NP;                            // for storing the xbee's NP AT value;

//SPI communication pins, SCK, MISO, and MOSI are common to all SPI sensors, CS pins are unique to each 
// SPI sensor.
#define SD_CS 2

File OtherData;                 // SD file for sensor data
File GPSdata;                   // SD file for GPS data

Display gondola_display("Gondola");    // diplay object for the gondola

String gpsTotal;                // final string used to write to GPSdata
String otherTotal;              // final string used to write to OtherData

uint8_t received_compressed1[256];    // compressed byte array
uint8_t received_compressed2[256];    // compressed byte array
uint8_t received_compressed3[256];    // compressed byte array

uint16_t rec_comp_index[3];          // compressed index

uint8_t received_decompressed1[500]; // decompressed byte array
uint8_t received_decompressed2[500]; // decompressed byte array
uint8_t received_decompressed3[500]; // decompressed byte array
uint16_t rec_decomp_index[3];         // decompressed index 1

uint8_t activePackets_compressed;     // used to keep track of how many packets are around
uint8_t activePackets_decompressed;   // used to keep track of how many decomp'd packets are around
        
bool dataType[3];           // used to mark the type of data received (false = gps, true = other);
bool classified[3];         // used to track which buffers have been classified

uint8_t command_buf[20];    // grabs the command frames from the xbee buffer
uint8_t command_ind;        // grabs the command frame index from the xbee library

long gpsInfo_stamp = 0;     // used to track the last time GPS info was grabbed from an incoming string

static struct pt getreceivedPT;
static struct pt decompressPT;
static struct pt classifyPT;
static struct pt getGPSInfoPT;

//*************************************************************************************************************
//*******                                   PROTOTHREAD getReceived_sense
//*************************************************************************************************************
static int getReceived_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, xbee.data_received_length && (activePackets_compressed < 3));
    getNewPacket(activePackets_compressed++);
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD decompress_sense
//*************************************************************************************************************
static int decompress_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt,activePackets_compressed && (activePackets_decompressed < 3));
    decompControl();
    activePackets_compressed--;
    activePackets_decompressed++;
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD classify_sense
//*************************************************************************************************************
static int classify_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt,activePackets_decompressed && !(classified[0] & classified[1] & classified[2]));
    classify();
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                   PROTOTHREAD getGPSInfo_sense
//*************************************************************************************************************
static int getGPSInfo_sense(struct pt *pt){
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt,((millis() - gpsInfo_stamp > 900) && (classified[0] && !dataType[0]) || (classified[1] && !dataType[1]) || (classified[2] && !dataType[2])));
    getGPSInfo();
    gpsInfo_stamp = millis();
  PT_END(pt);
}

//*************************************************************************************************************
//*******                                getNewPacket
//*************************************************************************************************************
void getNewPacket(uint8_t activePackets)
{
  rec_comp_index[activePackets] = (uint16_t)xbee.data_received_length;
  switch(activePackets)
  {
    case 0:
      for(int i = 0; i < rec_comp_index[activePackets]; i++)
      {
        received_compressed1[i] = xbee.data_received[i];
      }
      break;
    case 1:
      for(int i = 0; i < rec_comp_index[activePackets]; i++)
      {
        received_compressed2[i] = xbee.data_received[i];
      }
      break;
    case 2:
      for(int i = 0; i < rec_comp_index[activePackets]; i++)
      {
        received_compressed3[i] = xbee.data_received[i];
      }
      break;  
  };
  xbee.data_received_length = 0;
}

//*************************************************************************************************************
//*******                                decompControl
//*************************************************************************************************************
void decompControl()
{
  if(rec_comp_index[0])
  {
    rec_decomp_index[0] = decompress(received_compressed1, received_decompressed1, rec_comp_index[0]);
    matchCaseDecompress(received_decompressed3, rec_decomp_index[1]);
    rec_comp_index[0] = 0;
  }
  else if(rec_comp_index[1])
  {
    rec_decomp_index[1] = decompress(received_compressed2, received_decompressed2, rec_comp_index[1]);
    matchCaseDecompress(received_decompressed3, rec_decomp_index[1]);
    rec_comp_index[2] = 0;
  }
  else if(rec_comp_index[2])
  {
    rec_decomp_index[2] = decompress(received_compressed2, received_decompressed3, rec_comp_index[2]);
    matchCaseDecompress(received_decompressed3, rec_decomp_index[2]);
    rec_comp_index[2] = 0;
  }
}

//*************************************************************************************************************
//*******                                classify
//*************************************************************************************************************
void classify()
{
  bool matches = true;
  
  if(!classified[0])
  {
   for(int i = 0; i < 6; i++)
    {
      switch(received_decompressed1[i])
      {
        case '$':
          break;
        case 'G':
          break;
        case 'P':
          break;
        case 'R':
          break;
        case 'M':
          break;
        case 'C':
          break;
        case 'A':
          break;
        default:
          matches =& false;
      };
    }
    dataType[0] = matches;
    classified[0] = true;
  }
  else if(!classified[1])
  {
    for(int i = 0; i < 6; i++)
    {
      switch(received_decompressed2[i])
      {
        case '$':
          break;
        case 'G':
          break;
        case 'P':
          break;
        case 'R':
          break;
        case 'M':
          break;
        case 'C':
          break;
        case 'A':
          break;
        default:
          matches =& false;
      };
    }
    dataType[1] = matches;
    classified[1] = true;
  }
  else if(!classified[2])
  {
    for(int i = 0; i < 6; i++)
    {
      switch(received_decompressed3[i])
      {
        case '$':
          break;
        case 'G':
          break;
        case 'P':
          break;
        case 'R':
          break;
        case 'M':
          break;
        case 'C':
          break;
        case 'A':
          break;
        default:
          matches =& false;
      };
    }
    dataType[2] = matches;
    classified[2] = true;
  }
}

//*************************************************************************************************************
//*******                                getGPSInfo
//*************************************************************************************************************
void getGPSInfo()
{ 
  uint8_t active_frame;               // keeps track of which frame we are working on
  if(classified[0] && !dataType[0])
  {
    active_frame = 0;
    uint16_t prior_stringLength = byteArray2string(received_decompressed1, rec_comp_index[0], &gpsTotal); 
  }
  else if(classified[1] && !dataType[1])
  {
    active_frame = 1;
    uint16_t prior_stringLength = byteArray2string(received_decompressed2, rec_comp_index[1], &gpsTotal);
  }
  else if(classified[2] && !dataType[2])
  {
    active_frame = 2;
    uint16_t prior_stringLength = byteArray2string(received_decompressed3, rec_comp_index[2], &gpsTotal);
  }
  
  uint16_t money_index = gpsTotal.indexOf('$',prior_stringLength);
  String gps_sentence = gpsTotal.substring(money_index + 1, money_index + 7);
  if(gps_sentence.equals("GPGGA")
  {
    
  }
  else if (gps_sentence.equals("GPRMC")
  {
    
  }
}

//*************************************************************************************************************
//*******                                charArray2string
//*************************************************************************************************************
uint16_t charArray2string(uint8_t input_array[], uint16_t input_size, String *output_string)
{
  uint16_t current_stringLength = *output_string.length();
  for(int i = 0; i < input_size; i++)
  {
    *output_string += input_array[i];
  }
  return current_stringLength;
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
//*******                                           SETUP
//*************************************************************************************************************
void setup() {
  gpsTotal.reserve(1500);
  OtherTotal.reserve(1500);
   
  usb.begin(115200);
  usb.println(F("INITIALIZING.."));

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

  PT_INIT(&getreceivedPT);
  
  usb.println(F("Setup complete, entering loop!"));

  wdt_enable(16384);        // enable the watchdown to catch any software lockups
}

//*************************************************************************************************************
//*******                                           LOOP
//*************************************************************************************************************
void loop() {
  // put your main code here, to run repeatedly:

}

//*************************************************************************************************************
//*******                                 CLASSDEF :: DISPLAY
//*************************************************************************************************************
Class Display{
  public:
    Display(String);
    static void printStats();
    void printCommandLine();
    void setVerbosity(uint8_t);
    
  private:
    static uint8_t num_of_disps;
    String disp_name; 
    String latitude;
    String longitude;
    String elevation;
    uint8_t fix_quality;
    uint8_t sats_tracked;
    String horiz_dop;
}

void Display(String disp_Name)
{
  num_of_disps++;
  
  Display::disp_name = disp_Name;
  Display::latitude = "no info available yet";
  Display::longitude = "no info available yet";
  Display::elevation = "no info available yet";
  Display::fix_quality = 0;
  Display::sats_tracked = 0;
}
