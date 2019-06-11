#include <xbeeAPI.h>
#include <SD.h>
#include <pt.h>

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

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
