#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <pt.h>           // libraries
#include <SPI.h>


#include "xbeeAPI.h"


/*******DEBUG DEFINITIONS, DEFINE IN SKETCH TO DEBUG SPEICIFIC FUNCTIONS*******/
//  #define debug_all - turns on debugging for all functions in library
//  #define debug_Xbee
//  #define debug_begin
//  #define debug_setSPI_clockFreq
//  #define debug_setSPI_bitOrder
//  #define debug_setSPI_mode
//  #define debug_setDestinationAddress
//  #define debug_setCommandInterruptPin
//  #define debug_setCutdownInterruptPin
//  #define debug_setLocalCommand_DIO
//  #define debug_setLocalCutdown_DIO
//  #define debug_setDestinationCommand_DIO
//  #define debug_setDestinationCutdown_DIO
//  #define debug_sendFrame
//  #define debug_sendPayload
//  #define debug_printStats
//  #define debug_sendAvailable
//  #define debug_messageWaiting
//  #define debug_protothreadLoop
//  #define debug_setMaxPayloadSize
//  #define debug_resetStatus
//  #define debug_clearBuffers
//  #define debug_sendCommand
//  #define debug_setMode
//  #define debug_receivePacket_sense
//  #define debug_parsePacket_sense
//  #define debug_commandMode_sense
//  #define debug_commandModeTO_sense
//  #define debug_fastMode_sense
//  #define debug_accModeLevel0_sense
//  #define debug_accModeLevel1_sense
//  #define debug_garbageCollect_sense
//  #define debug_command_ISR
//  #define debug_cutdown_ISR
//  #define debug_checkSum
  #define debug_recSPI
//  #define debug_parsePacket
//  #define debug_exitCommandMode
//  #define debug_setDestinationCommand_State
//  #define debug_setDestinationCutdown_State

#ifdef debug_all
  #define debug_Xbee
  #define debug_begin
  #define debug_setSPI_clockFreq
  #define debug_setSPI_bitOrder
  #define debug_setSPI_mode
  #define debug_setDestinationAddress
  #define debug_setCommandInterruptPin
  #define debug_setCutdownInterruptPin
  #define debug_setLocalCommand_DIO
  #define debug_setLocalCutdown_DIO
  #define debug_setDestinationCommand_DIO
  #define debug_setDestinationCutdown_DIO
  #define debug_sendFrame
  #define debug_sendPayload
  #define debug_printStats
  #define debug_sendAvailable
  #define debug_messageWaiting
  #define debug_protothreadLoop
  #define debug_setMaxPayloadSize
  #define debug_resetStatus
  #define debug_clearBuffers
  #define debug_sendCommand
  #define debug_setMode
  #define debug_receivePacket_sense
  #define debug_parsePacket_sense
  #define debug_commandMode_sense
  #define debug_commandModeTO_sense
  #define debug_fastMode_sense
  #define debug_accModeLevel0_sense
  #define debug_accModeLevel1_sense
  #define debug_garbageCollect_sense
  #define debug_command_ISR
  #define debug_cutdown_ISR
  #define debug_checkSum
  #define debug_recSPI
  #define debug_parsePacket
  #define debug_exitCommandMode
  #define debug_setDestinationCommand_State
  #define debug_setDestinationCutdown_State
#endif

/***************************************************************************
 ************************* PUBLIC FUNCTIONS ********************************
 ***************************************************************************/
/***************************************************************************
CONSTRUCTOR
***************************************************************************/
Xbee::Xbee(uint8_t slave_select, uint8_t attention_pin)
{
  #ifdef debug_Xbee
    Serial.println(F("constructor: called"));
  #endif

 _XBEE_SS = slave_select;
 _XBEE_ATTN = attention_pin;
 _XBEE_CMD = NULL;
 _XBEE_CUTDWN = NULL;

 pinMode(_XBEE_SS,OUTPUT);
 pinMode(_XBEE_ATTN, INPUT_PULLUP);
 digitalWrite(_XBEE_SS, HIGH);

 PT_INIT(&receivePacket_PT);
 PT_INIT(&parsePacket_PT);
 PT_INIT(&commandMode_PT);
 PT_INIT(&commandModeTO_PT);
 PT_INIT(&fastMode_PT);
 PT_INIT(&accModeLevel0_PT);
 PT_INIT(&accModeLevel1_PT);
 PT_INIT(&garbageCollect_PT);

 #ifdef debug_Xbee
   Serial.println(F("constructor: leaving"));
 #endif
}

/***************************************************************************
BEGIN
***************************************************************************/
uint16_t Xbee::begin(void)
{
  #ifdef debug_begin
    Serial.println(F("begin : called"));
  #endif

  activity_counter = millis();

  _MODE = 1;
  _SPI_DATAMODE = SPI_MODE0;
  _SPI_CLK_FREQ = 500000;
  _SPI_BITORDER = MSBFIRST;

  commandMode = false;
  commandMode_TO_threshold = 5000;
  data_received_length = 0;
  command_receive_length = -1;
  _maxPackets = 0;
  lastPacket_TIME = 0;
  errorStatus = 0x00;
  uint8_t send_header_init[17] = {0x7E, 0x00, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE, 0x00, 0x40};
  for(int i = 0; i < 17; i++)
  {
    _send_header[i] = send_header_init[i];
  }

  #ifdef debug_begin
    Serial.print(F("begin : send_header = "));
    for(int i = 0; i < 17; i++)
    {
      if(send_header_init[i] <= 0x0F)
      {
        Serial.print(F("0"));
      }
      Serial.print(send_header_init[i]);
      Serial.print(F(" "));
    }
    Serial.println("");
  #endif

  uint8_t tester[8] = {0x7E, 0x00, 0x04, 0x08, 0x01, 0x4E, 0x50, 0x58};
  uint8_t tester_reply[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  long begin_to = 0;
  bool attn_state = true;
  int begin_index = -1;

  SPI.begin();

  SPI.beginTransaction(SPISettings(_SPI_CLK_FREQ,_SPI_BITORDER,_SPI_DATAMODE));
  digitalWrite(_XBEE_SS,LOW);
  SPI.transfer(tester,8);
  digitalWrite(_XBEE_SS,HIGH);
  SPI.endTransaction();

  begin_to = millis();
  while(attn_state && (millis()-begin_to)<=5000)
  {
    attn_state = digitalRead(_XBEE_ATTN);
  }

  #ifdef debug_begin
    Serial.print(F("begin : receive while loop exited, attn_state = "));
    Serial.println(attn_state);
  #endif

  if(!attn_state)
  {
    SPI.beginTransaction(SPISettings(_SPI_CLK_FREQ,_SPI_BITORDER,_SPI_DATAMODE));
    digitalWrite(_XBEE_SS,LOW);
    SPI.transfer(tester_reply,15);
    digitalWrite(_XBEE_SS,HIGH);
    SPI.endTransaction();

    for(int i = 0; i < 15; i++)
    {
      if(tester_reply[i] == uint8_t(0x7E))
      {
        #ifdef debug_begin
          Serial.println(F("begin : start byte found on reply!"));
        #endif

        begin_index = i;
        break;
      }
    }

    if(begin_index  < 0 || begin_index > 4)
    {
      #ifdef debug_begin
        Serial.println(F("begin : begin_index kickout, return false"));
      #endif

      return false;
    }

    uint8_t reply_status = tester_reply[begin_index+7];
    uint8_t np_msb = tester_reply[begin_index+8];
    uint8_t np_lsb = tester_reply[begin_index+9];

    #ifdef debug_begin
      Serial.print(F("begin : reply_status = "));
      Serial.println(reply_status);
      Serial.print(F("begin : np_msb = "));
      Serial.println(np_msb);
      Serial.print(F("begin : np_lsb = "));
      Serial.println(np_lsb);
    #endif


    if(!reply_status)
    {
      #ifdef debug_begin
        Serial.println(F("begin : success, returning _NP"));
      #endif

      _NP = (np_msb*256) + np_lsb;
      max_payload_size = _NP;
      return _NP;
    }
    else
    {
      #ifdef debug_begin
        Serial.println(F("begin : reply status != 0, return false"));
      #endif

      return false;
    }
  }
  #ifdef debug_begin
    Serial.println(F("begin : attn_state cause return false"));
  #endif

  return false;
}

/***************************************************************************
setSPI_clockFreq
***************************************************************************/
void Xbee::setSPI_clockFreq(uint32_t freq)
{
  #ifdef debug_setSPI_clockFreq
    Serial.println(F("debug_setSPI_clockFreq : called"));
  #endif

  activity_counter = millis();
 _SPI_CLK_FREQ = freq;

 #ifdef debug_setSPI_clockFreq
   Serial.println(F("debug_setSPI_clockFreq : exited"));
 #endif
}

/***************************************************************************
setSPI_bitOrder
***************************************************************************/
void Xbee::setSPI_bitOrder(BitOrder bitorder)
{
  #ifdef debug_setSPI_bitOrder
    Serial.println(F("setSPI_bitOrder :  called"));
  #endif

  activity_counter = millis();
 _SPI_BITORDER = bitorder;

 #ifdef debug_setSPI_bitOrder
  Serial.println(F("setSPI_bitOrder :  exit"));
 #endif
}

/***************************************************************************
setSPI_mode
***************************************************************************/
void Xbee::setSPI_mode(uint8_t spi_mode)
{
  #ifdef debug_setSPI_mode
    Serial.println(F("setSPI_mode : called"));
  #endif

  activity_counter = millis();
 _SPI_DATAMODE = spi_mode;

 #ifdef debug_setSPI_mode
   Serial.println(F("setSPI_mode : exit"));
 #endif
}

/***************************************************************************
setDestinationAddress
***************************************************************************/
void Xbee::setDestinationAddress(uint64_t addr)
{
  #ifdef debug_setDestinationAddress
    Serial.println(F("setDestinationAddress : called"));
  #endif

  activity_counter = millis();
 uint32_t upperAddy = (uint32_t)(addr >> 32);
 uint32_t lowerAddy = (uint32_t)(addr);
 for(int i = 3; i >= 0; i--)
 {
   uint32_t bitMask = 0b11111111 << ((3 - i) * 8);
   _ADDR[i+4] = (uint8_t)((lowerAddy & bitMask) >> ((3 - i) * 8));
   _ADDR[i] = (uint8_t)((upperAddy & bitMask) >> ((3 - i) * 8));
 }

 for(int i = 0; i < 8; i++)
 {
   _send_header[i+5] = _ADDR[i];
 }

 #ifdef debug_setDestinationAddress
   Serial.println(F("setDestinationAddress : exit"));
 #endif
}

/***************************************************************************
setCommandInterruptPin                                      not functional
***************************************************************************/
void Xbee::setCommandInterruptPin(uint8_t pin)
{
  #ifdef debug_setCommandInterruptPin
    Serial.println(F("setCommandInterruptPin : called"));
  #endif

  activity_counter = millis();
 _XBEE_CMD = pin;
 pinMode(_XBEE_CMD,INPUT_PULLUP);
 //attachInterrupt(digitalPinToInterrupt(_XBEE_CMD),Xbee::command_ISR,RISING); // brokend for some reason

 #ifdef debug_setCommandInterruptPin
   Serial.println(F("setCommandInterruptPin : exit"));
 #endif
}

/***************************************************************************
setCutdownInterruptPin                                    not functional
***************************************************************************/
void Xbee::setCutdownInterruptPin(uint8_t pin)
{
  #ifdef debug_setCutdownInterruptPin
    Serial.println(F("setCutdownInterruptPin :  called"));
  #endif

  activity_counter = millis();
 _XBEE_CUTDWN = pin;
 pinMode(_XBEE_CUTDWN,INPUT_PULLUP);
 //attachInterrupt(digitalPinToInterrupt(_XBEE_CMD),Xbee::cutdown_ISR,RISING);  // broken for some reason

 #ifdef debug_setCutdownInterruptPin
   Serial.println(F("setCutdownInterruptPin :  exit"));
 #endif
}

/***************************************************************************
setLocalCommand_DIO
***************************************************************************/
void Xbee::setLocalCommand_DIO(uint8_t dio_pin)
{
  #ifdef debug_setLocalCommand_DIO
    Serial.println(F("setLocalCommand_DIO :  called"));
  #endif

  activity_counter = millis();
  _LOCAL_COMMAND = dio_pin;

  #ifdef debug_setLocalCommand_DIO
    Serial.println(F("setLocalCommand_DIO :  exit"));
  #endif
}

/***************************************************************************
setLocalCutdown_DIO
***************************************************************************/
void Xbee::setLocalCutdown_DIO(uint8_t dio_pin)
{
  #ifdef debug_setLocalCutdown_DIO
    Serial.println(F("setLocalCutdown_DIO : called"));
  #endif

  activity_counter = millis();
  _LOCAL_CUTDOWN = dio_pin;

  #ifdef debug_setLocalCutdown_DIO
    Serial.println(F("setLocalCutdown_DIO : exit"));
  #endif
}

/***************************************************************************
setDestinationCommand_DIO
***************************************************************************/
void Xbee::setDestinationCommand_DIO(uint8_t dio_pin)
{
  #ifdef debug_setDestinationCommand_DIO
    Serial.println(F("setDestinationCommand_DIO : called"));
  #endif

  activity_counter = millis();
 _DEST_COMMAND[0] = 'D';
 _DEST_COMMAND[1] = dio_pin;

 #ifdef debug_setDestinationCommand_DIO
   Serial.println(F("setDestinationCommand_DIO : exit"));
 #endif
}

/***************************************************************************
setDestinationCutdown_DIO
***************************************************************************/
void Xbee::setDestinationCutdown_DIO(uint8_t dio_pin)
{
  #ifdef debug_setDestinationCutdown_DIO
    Serial.println(F("setDestinationCutdown_DIO : called"));
  #endif

  activity_counter = millis();
 _DEST_CUTDOWN[0] = 'D';
 _DEST_CUTDOWN[1] = dio_pin;

 #ifdef debug_setDestinationCutdown_DIO
   Serial.println(F("setDestinationCutdown_DIO : exit"));
 #endif
}

/***************************************************************************
sendFrame
***************************************************************************/
void Xbee::sendFrame(uint8_t sendBuffer[], uint16_t length_of_buffer)
{
  #ifdef debug_sendFrame
    Serial.println(F("sendFrame : called"));
    for(int i = 0; i < length_of_buffer; i++)
    {
      if(sendBuffer[i] <= 0x0F)
      {
        Serial.print(F("0"));
      }
      Serial.print(sendBuffer[i],HEX);
      Serial.print(F(" "));
    }
    Serial.println("");
  #endif

  activity_counter = millis();
  _sending  = true;
  if(length_of_buffer <= (_NP + 18))
  {
    #ifdef debug_sendFrame
      Serial.println(F("sendFrame : length_of_buffer acceptable, sending frame"));
    #endif

    SPI.beginTransaction(SPISettings(_SPI_CLK_FREQ,_SPI_BITORDER,_SPI_DATAMODE));
    digitalWrite(_XBEE_SS,LOW);
    SPI.transfer(sendBuffer,length_of_buffer);
    digitalWrite(_XBEE_SS,HIGH);
    SPI.endTransaction();
  }
  if(_MODE == 0)
  {
    #ifdef debug_sendFrame
      Serial.println(F("sendFrame : mode = 0 so _sending = false before exit"));
    #endif

    _sending = false;
  }
  #ifdef debug_sendFrame
    Serial.println(F("sendFrame : exit"));
  #endif
}

/***************************************************************************
sendPayload
***************************************************************************/
void Xbee::sendPayload(uint8_t sendBuffer[], uint16_t length_of_buffer)
{
  #ifdef debug_sendPayload
    Serial.println(F("sendPayload : called"));
  #endif

  activity_counter = millis();
  _sendPayload  = true;

  if(length_of_buffer <= max_payload_size)
  {
    #ifdef debug_sendPayload
      Serial.println(F("sendPayload : length of buff < max_payload_size, sending as-is"));
    #endif

    uint16_t frame_length = 14 + length_of_buffer;
    _send_header[2] = (uint8_t)frame_length;
    _send_header[1] = (uint8_t)(frame_length >> 8);

    for(int i = 0; i< 17; i++)
    {
      _send_buffer[i] = _send_header[i];
    }
    for(int i = 0; i < length_of_buffer; i++)
    {
      _send_buffer[i+17] = sendBuffer[i];
    }

    #ifdef debug_sendPayload
      Serial.print(F("sendPayload : sendBuffer with header = "));
      for(int i = 0; i < frame_length+3; i++)
      {
        if(_send_buffer[i] <= 0x0F)
        {
          Serial.print(F("0"));
        }
        Serial.print(_send_buffer[i],HEX);
        Serial.print(F(" "));
      }
      Serial.println(F(""));
    #endif

    uint8_t intermed_buff[length_of_buffer+14];
    for(int i = 0; i < length_of_buffer+14; i++)
    {
      intermed_buff[i] = _send_buffer[i+3];
    }
    _send_buffer[length_of_buffer + 17] = Xbee::checkSum(intermed_buff,(length_of_buffer+14));

    #ifdef debug_sendPayload
      Serial.print(F("sendPayload : intermed_buff = "));
      for(int i = 0; i < length_of_buffer+14; i++)
      {
        if(intermed_buff[i] <= 0x0F)
        {
          Serial.print(F("0"));
        }
        Serial.print(intermed_buff[i],HEX);
        Serial.print(F(" "));
      }
      Serial.println(F(""));
      Serial.print(F("sendPayload : chksum = "));
      Serial.println(_send_buffer[length_of_buffer+17],HEX);
    #endif

    Xbee::sendFrame(_send_buffer,length_of_buffer+18);
  }

  _sendPayload = false;

  #ifdef debug_sendPayload
    Serial.println(F("sendPayload : exit"));
  #endif
}



/***************************************************************************
printStats
***************************************************************************/
void Xbee::printStats(void)
{

 activity_counter = millis();
 Serial.print(F("xbee ss pin = "));
 Serial.println(_XBEE_SS);
 Serial.print(F("xbee attn pin = "));
 Serial.println(_XBEE_ATTN);
 Serial.print(F("xbee cmdpin = "));
 Serial.println(_XBEE_CMD);
 Serial.print(F("xbee cutdown = "));
 Serial.println(_XBEE_CUTDWN);
 Serial.print(F("xbee _SPI_CLK_FREQ = "));
 Serial.println(_SPI_CLK_FREQ);
 Serial.print(F("xbee _NP = "));
 Serial.println(_NP);

 Serial.print(F("xbee max_payload_size = "));
 Serial.println(max_payload_size);
 Serial.print(F("Mode = "));
 Serial.println(_MODE);

 Serial.print(F("xbee _LOCAL_COMMAND = "));
 Serial.println(_LOCAL_COMMAND);

 Serial.print(F("xbee _LOCAL_CUTDOWN = "));
 Serial.println(_LOCAL_CUTDOWN);

 Serial.print(F("xbee _DEST_COMMAND = "));
 Serial.print((char)_DEST_COMMAND[0]);
 Serial.println(_DEST_COMMAND[1]);

 Serial.print(F("xbee _DEST_CUTDOWN = "));
 Serial.print((char)_DEST_CUTDOWN[0]);
 Serial.println(_DEST_CUTDOWN[1]);

 Serial.print(F("xbee _ADDR = "));
 for(int i = 0; i<8; i++)
 {
  if(_ADDR[i] <= 0x0F)
  {
    Serial.print(F("0"));
  }
  Serial.print(_ADDR[i],HEX);
  Serial.print(' ');
 }
 return;
}

/***************************************************************************
sendAvailable
***************************************************************************/
bool Xbee::sendAvailable(void)
{
  #ifdef debug_sendAvailable
    Serial.println(F("sendAvailable : called and returning"));
  #endif

  return !(_sendPayload || _sending);
}


/***************************************************************************
messageWaiting
***************************************************************************/
uint16_t Xbee::messageWaiting(void)
{
  #ifdef debug_messageWaiting
    Serial.println(F("messageWaiting : called and returning"));
  #endif

  activity_counter = millis();
 return data_received_length;
}

/***************************************************************************
protothreadLoop
***************************************************************************/
void Xbee::protothreadLoop(void)
{
  #ifdef debug_protothreadLoop
    Serial.println(F("protothreadLoop : called"));
  #endif

  Xbee::receivePacket_sense(&receivePacket_PT);
  Xbee::parsePacket_sense(&parsePacket_PT);
  Xbee::commandMode_sense(&commandMode_PT);
  Xbee::commandModeTO_sense(&commandModeTO_PT);
  Xbee::fastMode_sense(&fastMode_PT);
  Xbee::accModeLevel0_sense(&accModeLevel0_PT);
  Xbee::accModeLevel1_sense(&accModeLevel1_PT);
  Xbee::garbageCollect_sense(&garbageCollect_PT);

  #ifdef debug_protothreadLoop
    Serial.println(F("protothreadLoop : exit"));
  #endif
}

/***************************************************************************
setMaxPayloadSize                                         NOT TESTED
***************************************************************************/
void Xbee::setMaxPayloadSize(int size_bytes)
{
  #ifdef debug_setMaxPayloadSize
    Serial.println(F("setMaxPayloadSize : called"));
  #endif

  activity_counter = millis();
  max_payload_size= (uint16_t) size_bytes;
  //_PAYLOAD_STRING.reserve(size_bytes);          // not needed

  #ifdef debug_setMaxPayloadSize
    Serial.println(F("setMaxPayloadSize : exit"));
  #endif
}

/***************************************************************************
resetStatus                                         NOT TESTED
***************************************************************************/
void Xbee::resetStatus(void)
{
  #ifdef debug_resetStatus
    Serial.println(F("resetStatus : called"));
  #endif

  activity_counter = millis();
  _sendPayload = false;
  _packetReceived = false;
  _packetParsed = false;
  _send_confirmed = false;
  _sending  = false;

  #ifdef debug_resetStatus
    Serial.println(F("resetStatus : exit"));
  #endif
}

/***************************************************************************
clearBuffers                                         NOT TESTED
***************************************************************************/
void Xbee::clearBuffers(void)
{
  #ifdef debug_clearBuffers
    Serial.println(F("clearBuffers : called"));
  #endif

  activity_counter = millis();
  _data_received_length = 0;
  data_received_length = 0;
  command_receive_length = -1;

  for(int i = 0; i < 274; i++)
  {
    if(i<20)
    {
      command_receive_buffer[i] = 0x00;
      data_received[i] = 0x00;
      _receive_buffer[i] = 0x00;
      _send_buffer[i] = 0x00;
    }
    else if(i >=20 && i < 256)
    {
      data_received[i] = 0x00;
      _receive_buffer[i] = 0x00;
      _send_buffer[i] = 0x00;
    }
    else
    {
      _receive_buffer[i] = 0x00;
      _send_buffer[i] = 0x00;
    }
  }

  #ifdef debug_clearBuffers
    Serial.println(F("clearBuffers : exit"));
  #endif
}

/***************************************************************************
sendCommand                                         NOT TESTED
***************************************************************************/
bool Xbee::sendCommand(uint8_t command_type, uint8_t command_val)
{
  uint8_t mode = _MODE;
  Xbee::setMode(1);
  //uint8_t sendMode = _send_header[4];
  //_MODE = 1;
  //_send_header[4] = 0x01;


  bool functionState = false;
  bool command_state = false;
  bool command_response = false;

  #ifdef debug_sendCommand
    Serial.println(F("sendCommand : entering resetStatus from sendCommand"));
  #endif

  Xbee::resetStatus();

  #ifdef debug_sendCommand
    Serial.println(F("sendCommand : returning to sendCommand from resetStatus"));
  #endif

  _data_received_length = 0;
  _packetReceived = false;

  uint8_t frame[20];
  for(int i = 0; i < 17; i++)
  {
    frame[i] = _send_header[i];
  }
  frame[17] = command_type;
  frame[18] = command_val;
  frame[1] = 0x00;
  frame[2] = 0x10;

  #ifdef debug_sendCommand
    Serial.print(F("sendCommand : frame with header = "));
    for(int i = 0; i < 20; i++)
    {
      if(frame[i] <= 0x0F)
      {
        Serial.print(F("0"));
      }
      Serial.print(frame[i],HEX);
      Serial.print(F(" "));
    }
    Serial.println(F(""));
  #endif

  uint8_t subframe[16];
  for(int i = 0; i < 16; i++)
  {
    subframe[i] = frame[i+3];
  }
  frame[19] = Xbee::checkSum(subframe,16);

  #ifdef debug_sendCommand
    Serial.print(F("sendCommand : subframe = "));
    for(int i = 0; i < 16; i++)
    {
      if(subframe[i] <= 0x0F)
      {
        Serial.print(F("0"));
      }
      Serial.print(subframe[i],HEX);
      Serial.print(F(" "));
    }
    Serial.println(F(""));
    Serial.print(F("sendCommand : subframe checksum = "));
    Serial.println(frame[19],HEX);
  #endif

  long command_timer = millis();
  while((!command_state) && (millis() - command_timer <= 5000))
  {
    #ifdef debug_sendCommand
      Serial.println(F("sendCommand : setting commandState to true"));                                  //DEBUG
    #endif

    command_state = Xbee::setDestinationCommand_State(true);
  }
  _packetReceived = false;
  _data_received_length = 0;

  #ifdef debug_sendCommand
    Serial.println(F("sendCommand : exiting setCommandState Loop"));                                  //DEBUG
  #endif

  if(command_state)
  {
    #ifdef debug_sendCommand
      Serial.println(F("sendCommand : commandState is true!"));
      delay(5000);                                 // DEBUG
    #endif

    sendFrame(frame,20);
    command_timer = millis();
    while(_data_received_length == 0 && millis()-command_timer < 1000)
    {
      Xbee::receivePacket_sense(&receivePacket_PT);
      Xbee::parsePacket_sense(&parsePacket_PT);
    }
    if(_data_received_length)
    {
      #ifdef debug_sendCommand
        Serial.println(F("sendCommand : while loop done, send confirmed!"));                                      // DEBUG
      #endif

      command_response = _send_confirmed;
    }
    #ifdef debug_sendCommand
      else
      {
        Serial.println(F("sendCommand : while loop done, _data_received_length = 0"));
      }                                      // DEBUG
    #endif
  }
  #ifdef debug_sendCommand
    else
    {
      Serial.println(F("sendCommand : failed to enter commandState"));
    }                                      // DEBUG
  #endif

  command_timer = millis();
  _packetReceived = false;
  _data_received_length = 0;
  command_state = false;
  while((!command_state) && (millis() - command_timer <= 5000))
  {
    #ifdef debug_sendCommand
      Serial.println(F("sendCommand : setting commandState to false"));                                 // DEBUG
    #endif

    command_state = Xbee::setDestinationCommand_State(false);
  }

  #ifdef debug_sendCommand
    if(command_state)
    {
      Serial.println(F("sendCommand : confirmed commandState to false!"));
      delay(5000);
    }
    else
    {
      Serial.println(F("sendCommand : failed to set commandState to false"));
    }                                 // DEBUG
  #endif

  Xbee::setMode(mode);
  //_MODE = mode;
  //_send_header[4] = sendMode;

  #ifdef debug_sendCommand
    Serial.println(F("sendCommand : exit"));                                 // DEBUG
  #endif

  return functionState;
}

/***************************************************************************
setMode                                         NOT TESTED
***************************************************************************/
void Xbee::setMode(uint8_t mode)
{
  #ifdef debug_setMode
    Serial.println(F("setMode : called"));
  #endif

  _MODE = mode;
  switch (_MODE)
  {
    case 0:
      _send_header[4] = 0x00;
      break;
    case 1:
      _send_header[4] = 0x01;
      break;
    case 2:
      _send_header[4] = 0x01;
      break;
  }

  #ifdef debug_setMode
    Serial.print(F("setMode : _send_header[4] = 0x0"));
    Serial.println(_send_header[4],HEX);
    Serial.println(F("setMode : exit"));
  #endif
}


/***************************************************************************
 ************************ PRIVATE FUNCTIONS ********************************
 ***************************************************************************/
 /***************************************************************************
 receivePacket_sense      PROTOTHREAD
 ***************************************************************************/
int Xbee::receivePacket_sense(struct pt *pt)
{
  #ifdef debug_receivePacket_sense
    Serial.println(F("receivePacket_sense : called"));
  #endif

  PT_BEGIN(pt);
  while(1)
  {
    PT_WAIT_UNTIL(pt, (_MODE > 0 && digitalRead(_XBEE_ATTN) == LOW));
    _packetReceived = Xbee::recSPI();

    #ifdef debug_receivePacket_sense
      Serial.println(F("receivePacket_sense: triggered"));
      Serial.print(F("_packetReceived = "));
      Serial.println(_packetReceived);
    #endif
  }
  PT_END(pt);

  #ifdef debug_receivePacket_sense
    Serial.println(F("receivePacket_sense : exit"));
  #endif
}

/***************************************************************************
parsePacket_sense      PROTOTHREAD
***************************************************************************/
int Xbee::parsePacket_sense(struct pt *pt)
{
  #ifdef debug_parsePacket_sense
    Serial.println(F("parsePacket_sense : called"));
  #endif

 PT_BEGIN(pt);
 //while (1) {
   PT_WAIT_UNTIL(pt, (_packetReceived));
   _packetParsed = Xbee::parsePacket();

   #ifdef debug_parsePacket_sense
    Serial.println(F("parsePacket_sense : triggered"));
    Serial.print(F("_packetParsed = "));
    Serial.println(_packetParsed);
   #endif
 //}
 PT_END(pt);

 #ifdef debug_parsePacket_sense
  Serial.println(F("parsePacket_sense : exit"));
 #endif
}

/***************************************************************************
commandMode_sense      PROTOTHREAD
***************************************************************************/
int Xbee::commandMode_sense(struct pt *pt)
{
  #ifdef debug_commandMode_sense
    Serial.println(F("commandMode_sense : called"));
  #endif



  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, (_XBEE_CMD != NULL && !(digitalRead(_XBEE_CMD))));
    commandMode = true;

    uint8_t mode = _MODE;
    Xbee::setMode(1);

    #ifdef debug_commandMode_sense
      Serial.println(F("commandMode_sense : triggered"));
      Serial.print(F("commandMode_sense : commandMode = "));
      Serial.println(commandMode);
    #endif

    _packetReceived = false;
    _packetParsed = false;
    command_receive_length = 0;
    _data_received_length = 0;
    clearBuffers();
    commandMode = true;
    commandMode_TO = millis();
    while(commandMode && command_receive_length  < 1)
    {
      Xbee::commandModeTO_sense(&commandModeTO_PT);
      Xbee::receivePacket_sense(&receivePacket_PT);
      Xbee::parsePacket_sense(&parsePacket_PT);
    }

    #ifdef debug_commandMode_sense
      Serial.println(F("commandMode_sense : while loop exited"));
      Serial.print(F("commandMode_sense : _packetReceived ="));
      Serial.println(_packetReceived);
      Serial.print(F("commandMode_sense : _packetParsed ="));
      Serial.println(_packetParsed);
      Serial.print(F("commandMode_sense : command_receive_length = "));
      Serial.println(command_receive_length);
      if(command_receive_length > 0)
      {
        Serial.print(F("commandMode_sense : command_receive_buffer = "));
        for(int i = 0; i < command_receive_length; i++)
        {
          if(command_receive_buffer[i] < 0x10)
          {
            Serial.print(F("0"));
          }
          Serial.print(command_receive_buffer[i],HEX);
          Serial.print(F(" "));
        }
        Serial.println(F(""));
      }
    #endif

    if(_packetParsed)
    {
      _packetParsed = false;
      _packetReceived = false;
      _data_received_length = 0;
    }
    if(commandMode)
    {
      Xbee::exitCommandMode();
    }

    Xbee::setMode(mode);
  }
  PT_END(pt);

  #ifdef debug_commandMode_sense
    Serial.println(F("commandMode_sense : exit"));
  #endif
}

/***************************************************************************
commandModeTO_sense      PROTOTHREAD
***************************************************************************/
int Xbee::commandModeTO_sense(struct pt *pt)
{
  #ifdef debug_commandModeTO_sense
    Serial.println(F("commandModeTO_sense : called"));
  #endif

 PT_BEGIN(pt);
 while (1) {
   PT_WAIT_UNTIL(pt, (commandMode && (millis() - commandMode_TO > commandMode_TO_threshold)));
   commandMode = !(Xbee::exitCommandMode());

   #ifdef debug_commandModeTO_sense
     Serial.println(F("commandModeTO_sense : triggered"));
     Serial.print(F("commandModeTO_sense : commandMode = "));
     Serial.println(commandMode);
   #endif
 }
 PT_END(pt);

 #ifdef debug_commandModeTO_sense
   Serial.println(F("commandModeTO_sense : exit"));
 #endif
}

/***************************************************************************
fastMode_sense      PROTOTHREAD
***************************************************************************/
int Xbee::fastMode_sense(struct pt *pt)
{
  #ifdef debug_fastMode_sense
    Serial.println(F("fastMode_sense : called"));
  #endif

 PT_BEGIN(pt);
 while (1) {
   PT_WAIT_UNTIL(pt, (!commandMode && !_MODE && (_packetReceived || _packetParsed)));
   Xbee::resetStatus();
   if(errorStatus > 0x03)
   {
     errorStatus = 0x03;
   }
   #ifdef debug_fastMode_sense
     Serial.println(F("fastMode_sense : triggered, entering reset status"));
   #endif
 }
 PT_END(pt);

 #ifdef debug_fastMode_sense
   Serial.println(F("fastMode_sense : exit"));
 #endif
}

/***************************************************************************
accModeLevel0_sense      PROTOTHREAD
***************************************************************************/
int Xbee::accModeLevel0_sense(struct pt *pt)
{
  #ifdef debug_accModeLevel0_sense
    Serial.println(F("accModeLevel0_sense : called"));
  #endif

 PT_BEGIN(pt);
// while (1) {
   PT_WAIT_UNTIL(pt, (!commandMode && _MODE && (_sending || _sendPayload)));
   #ifdef debug_accModeLevel0_sense
     Serial.println(F("accModeLevel0_sense : triggered"));
   #endif

   Xbee::accModeLevel1_sense(&accModeLevel1_PT);
 //}
 PT_END(pt);

 #ifdef debug_accModeLevel0_sense
   Serial.println(F("accModeLevel0_sense : exit"));
 #endif
}

/***************************************************************************
accModeLevel1_sense      PROTOTHREAD
***************************************************************************/
int Xbee::accModeLevel1_sense(struct pt *pt)
{
  #ifdef debug_accModeLevel1_sense
    Serial.println(F("accModeLevel1_sense : called"));
  #endif

 PT_BEGIN(pt);
 while (1) {
   PT_WAIT_UNTIL(pt, (_send_confirmed));

   #ifdef debug_accModeLevel1_sense
     Serial.println(F("accModeLevel1_sense : triggered"));
   #endif

   Xbee::resetStatus();
 }
 PT_END(pt);

 #ifdef debug_accModeLevel1_sense
   Serial.println(F("accModeLevel1_sense : exit"));
 #endif
}

/***************************************************************************
garbageCollect_sense      PROTOTHREAD
***************************************************************************/
int Xbee::garbageCollect_sense(struct pt *pt)
{
  #ifdef debug_garbageCollect_sense
    Serial.println(F("garbageCollect_sense : called"));
  #endif

 PT_BEGIN(pt);
 while (1) {
   PT_WAIT_UNTIL(pt, (!commandMode && millis() - activity_counter > 5000));

   #ifdef debug_garbageCollect_sense
     Serial.println(F("garbageCollect_sense : triggered"));
   #endif

   Xbee::resetStatus();
   Xbee::clearBuffers();
 }
 PT_END(pt);

 #ifdef debug_garbageCollect_sense
   Serial.println(F("garbageCollect_sense : exit"));
 #endif
}



/***************************************************************************
command_ISR         INTERRUPT SERVICE ROUTINE             NOT FINISHED
***************************************************************************/
void Xbee::command_ISR(void)
{
  commandMode = true;
  detachInterrupt(digitalPinToInterrupt(_XBEE_CMD));
  commandMode_TO = millis();
}

/***************************************************************************
cutdown_ISR         INTERRUPT SERVICE ROUTINE           NOT FINISHED
***************************************************************************/
void Xbee::cutdown_ISR(void)
{
  cutdownMode = true;
  detachInterrupt(digitalPinToInterrupt(_XBEE_CUTDWN));
}

/***************************************************************************
checkSum                                                NOT TESTED
***************************************************************************/
uint8_t Xbee::checkSum(uint8_t bufferArray[], uint16_t buffer_size)
{
  #ifdef debug_checkSum
    Serial.println(F("checkSum : called"));
    Serial.print(F("bufferArray = "));
    for(int i = 0; i < buffer_size; i++)
    {
      if(bufferArray[i] <= 0x0F)
      {
        Serial.print(F("0"));
      }
      Serial.print(bufferArray[i],HEX);
      Serial.print(F(" "));
    }
    Serial.println(F(""));
  #endif

  activity_counter = millis();

  uint8_t checkSum = 0;
  for(int i = 0; i < buffer_size; i++)
  {
    checkSum += bufferArray[i];
  }

  #ifdef debug_checkSum
    Serial.print(F("checkSum : checksum = "));
    Serial.println((0xFF - checkSum),HEX);
    Serial.println(F("checkSum : exit"));
  #endif

  return (uint8_t)(0xFF - checkSum);
}

/***************************************************************************
recSPI                                                      NOT TESTED
***************************************************************************/
bool Xbee::recSPI(void)
{
  #ifdef debug_recSPI
    Serial.println(F("recSPI : called"));
  #endif

  activity_counter = millis();

  if(_MODE == 0 && !commandMode)
  {
    #ifdef debug_recSPI
      Serial.println(F("recSPI : _MODE = 0, return true"));
    #endif

    return true;
  }

  #ifdef debug_recSPI
    else
    {
      Serial.println(F("recSPI : _MODE != 0"));
    }
  #endif


  bool functionState = false;

  uint8_t val;
  SPI.beginTransaction(SPISettings(_SPI_CLK_FREQ,_SPI_BITORDER,_SPI_DATAMODE));
  for(int i = 0; i < 274; i++)
  {
    digitalWrite(_XBEE_SS,LOW);
    val = SPI.transfer(0);
    digitalWrite(_XBEE_SS,HIGH);
    if(val == 0x7E)
    {
      #ifdef debug_recSPI
        Serial.println(F("recSPI : 0x7E found!"));
      #endif

      break;
    }
    else if(i == 273)
    {
      #ifdef debug_recSPI
        Serial.println(F("recSPI : 0x7E not found, return false"));
      #endif

      return functionState;
    }
  }

  uint8_t length_msb;
  uint8_t length_lsb;
  uint16_t frame_length;
  digitalWrite(_XBEE_SS,LOW);
  length_msb = SPI.transfer(0);
  length_lsb = SPI.transfer(0);
  digitalWrite(_XBEE_SS,HIGH);
  frame_length = (length_msb*0x0100) + length_lsb;

  #ifdef debug_recSPI
    Serial.print(F("recSPI : frame_length = "));
    Serial.println(frame_length);
  #endif

  for(int i = 0; i < frame_length; i++)
  {
    _receive_buffer[i] = 0x00;
  }
  digitalWrite(_XBEE_SS,LOW);
  SPI.transfer(_receive_buffer,frame_length);
  uint8_t frame_chksum = SPI.transfer(0);
  digitalWrite(_XBEE_SS,HIGH);
  SPI.endTransaction();

  #ifdef debug_recSPI
    Serial.print(F("frame_chksum = "));
    Serial.println(frame_chksum,HEX);
    Serial.print(F("recSPI : _receive_buffer = "));
    for(int i = 0; i < frame_length; i++)
    {
      if(_receive_buffer[i] <= 0x0F)
      {
        Serial.print(F("0"));
      }
      Serial.print(_receive_buffer[i],HEX);
      Serial.print(F(" "));
    }
    Serial.println(F(""));
  #endif

  uint8_t buffer_chksum = Xbee::checkSum(_receive_buffer,frame_length);

  #ifdef debug_recSPI
    Serial.print(F("recSPI : buffer_chksum = "));
    Serial.println(buffer_chksum,HEX);
  #endif

  if(buffer_chksum == frame_chksum){
    #ifdef debug_recSPI
      Serial.println(F("recSPI : checksum match, receive Success!"));
    #endif

    functionState = true;
    _data_received_length = frame_length;
  }

  #ifdef debug_recSPI
    else
    {
      Serial.println(F("recSPI : checksums don't match, return false"));
    }
  #endif

  return functionState;
}

/***************************************************************************
parsePacket                                                     NOT TESTED
***************************************************************************/
bool Xbee::parsePacket(void)
{
  #ifdef debug_parsePacket
    Serial.println(F("parsePacket : called"));
  #endif

  activity_counter = millis();
  bool functionState = false;
  uint8_t frametype = _receive_buffer[0];

  #ifdef debug_parsePacket
    Serial.print(F("parsePacket : frametype = "));
    Serial.println(frametype,HEX);
  #endif

  if(!commandMode){

    #ifdef debug_parsePacket
      Serial.println(F("parsePacket : not in command mode"));
    #endif

    switch (_MODE)
    {
      case 0:
        #ifdef debug_parsePacket
          Serial.println(F("parsePacket : mode = 0"));
        #endif

        _send_confirmed = true;
        functionState = true;
        break;
      case 1:
        #ifdef debug_parsePacket
          Serial.println(F("parsePacket : mode = 1"));
        #endif

        if(frametype == 0x8B &&_receive_buffer[5] == 0x00)
        {
          #ifdef debug_parsePacket
            Serial.println(F("parsePacket : send confirmation received"));
          #endif
          _data_received_length = 0;
          _send_confirmed = true;
          functionState = true;
        }
        else
        {
          #ifdef debug_parsePacket
            Serial.println(F("parsePacket : not a send confirmation"));
          #endif

          _send_confirmed = false;
          functionState = true;
        }
        break;
      case 2:
        #ifdef debug_parsePacket
          Serial.println(F("parsePacket : mode = 2"));
        #endif

        if(frametype == 0x90)
        {
          #ifdef debug_parsePacket
            Serial.println(F("parsePacket : data packet received"));
          #endif

          data_received_length = _data_received_length;
          for(int i = 12; i < data_received_length; i++)
          {
            data_received[i-12] = _receive_buffer[i];
          }
          functionState = true;

          #ifdef debug_parsePacket
            Serial.print(F("parsePacket : data_received = "));
            for(int i = 0; i < data_received_length - 12 ; i++)
            {
              if(data_received[i] <= 0x0F)
              {
                Serial.print(F("0"));
              }
              Serial.print(data_received[i],HEX);
              Serial.print(F(" "));
            }
            Serial.println(F(""));
          #endif
        }
        break;

    }
  }
  else if(frametype == 0x90 && _data_received_length <= 20)
  {
    #ifdef debug_parsePacket
      Serial.println(F("parsePacket : commandMode = true and command received"));
    #endif

    command_receive_length = _data_received_length;
    for(int i = 0; i < command_receive_length; i++)
    {
      command_receive_buffer[i] = _receive_buffer[i];
    }

    #ifdef debug_parsePacket
      Serial.print(F("parsePacket : command_receive_buffer = "));
      for(int i = 0; i < command_receive_length; i++)
      {
        if(command_receive_buffer[i] <= 0x0F)
        {
          Serial.print(F("0"));
        }
        Serial.print(command_receive_buffer[i],HEX);
        Serial.print(F(" "));
      }
      Serial.println(F(""));
    #endif

    long exitTimer = millis();

    #ifdef debug_parsePacket
      Serial.println(F("parsePacket : exiting commandMode"));
    #endif

    while(!functionState && millis() - exitTimer < 5000)
    {
      functionState = Xbee::exitCommandMode();
    }
    if(!functionState)
    {
      #ifdef debug_parsePacket
        Serial.println(F("parsePacket : failed to exit commandMode"));
      #endif

      errorStatus = 0x01;
    }
    #ifdef debug_parsePacket
      else
      {
        Serial.println(F("parsePacket : commandMode exited successfully"));
      }
    #endif

  }
  #ifdef debug_parsePacket
    else
    {
      Serial.println(F("parsePacket : unknown parsing problem"));

      for(int i = 0; i < _data_received_length; i++)
      {
        if(_receive_buffer[i] < 0x10)
        {
          Serial.print(F("0"));
        }
        Serial.print(_receive_buffer[i],HEX);
        Serial.print(F(" "));
      }
      Serial.println(F(""));
    }
    Serial.println(F("parsePacket : exit"));
  #endif

  return functionState;
}

/***************************************************************************
 exitCommandMode                                               NOT functional
 ***************************************************************************/
 bool Xbee::exitCommandMode(void)
 {
   #ifdef debug_exitCommandMode
    Serial.println(F("exitCommandMode : called"));
   #endif

   uint8_t mode = _MODE;
   Xbee::setMode(1);

   activity_counter = millis();
   bool functionState = false;


   uint8_t exit_header[9] = {0x7E, 0x00, 0x05, 0x08, 0x01, 0x44, 0x00, 0x05, 0x00};
   exit_header[6] = _LOCAL_COMMAND;
   uint8_t subframe[5];

   #ifdef debug_exitCommandMode
    Serial.println(F("exitCommandMode : subframe = "));
   #endif

   for(int i = 0; i<5; i++)
   {
     subframe[i] = exit_header[i+3];

     #ifdef debug_exitCommandMode
      if(subframe[i] <= 0x0F)
      {
        Serial.print(F("0"));
        Serial.print(subframe[i],HEX);
        Serial.print(F(" "));
      }
     #endif
   }

   #ifdef debug_exitCommandMode
    Serial.println(F(""));
   #endif

   exit_header[8] = checkSum(subframe,5);

   #ifdef debug_exitCommandMode
    Serial.println(F("exitCommandMode : checksum = "));
    Serial.println(exit_header[8],HEX);
   #endif

   Xbee::sendFrame(exit_header,9);
   long timerTO;
   timerTO = millis();
   long recTO_max = 500;
   while((!(_packetReceived)) && ((millis() - timerTO) < recTO_max))
   {
     Xbee::receivePacket_sense(&receivePacket_PT);
   }

   if(_packetReceived)
   {
     #ifdef debug_exitCommandMode
      Serial.println(F("exitCommandMode : response packet received"));
     #endif

     uint8_t frametype = _receive_buffer[0];
     if(frametype == 0x88 && _receive_buffer[4] == 0x00)
     {
       #ifdef debug_exitCommandMode
        Serial.println(F("exitCommandMode : response packet confirmed"));
       #endif

       functionState = true;
     }

     #ifdef debug_exitCommandMode
       else
       {
        Serial.println(F("exitCommandMode : response packet not-confirmed"));
       }
     #endif
   }
   #ifdef debug_exitCommandMode
     else
     {
      Serial.println(F("exitCommandMode : no response packet received"));
     }
    Serial.println(F("exitCommandMode : exit"));
   #endif

   _packetReceived = false;
   _packetParsed = false;


   Xbee::setMode(mode);
   return functionState;
 }

/***************************************************************************
 setDestinationCommand_State                            NOT TESTED
 ***************************************************************************/
bool Xbee::setDestinationCommand_State(bool pinState)
{
  #ifdef debug_setDestinationCommand_State
    Serial.println(F("setDestinationCommand_State : called"));
  #endif

  uint8_t mode = _MODE;
  Xbee::setMode(1);

  activity_counter = millis();
  bool functionState = false;
  uint8_t frame[20] = {0x7E, 0x00, 0x10, 0x17, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE, 0x02, 0x44, 0x00, 0x00, 0x00};

  #ifdef debug_setDestinationCommand_State
    Serial.println(F("setDestinationCommand_State : frame generated"));                                         //  DEBUG
  #endif

  for(int i = 5; i <= 12; i++)
  {
    frame[i] = _ADDR[i-5];
  }

  #ifdef debug_setDestinationCommand_State
    Serial.println(F("setDestinationCommand_State : address added"));                                           // DEBUG
  #endif

  frame[17] = _DEST_COMMAND[1];

  if(pinState > 0)
  {
    frame[18] = 0x05;

    #ifdef debug_setDestinationCommand_State
      Serial.println(F("setDestinationCommand_State : pinstate = 0x05"));                                       // DEBUG
    #endif
  }
  else
  {
    frame[18] = 0x04;

    #ifdef debug_setDestinationCommand_State
      Serial.println(F("setDestinationCommand_State : pinstate = 0x04"));                                       // DEBUG
    #endif
  }

  uint8_t frame_checksummed[16];

  for(int i = 0; i<16; i++)
  {
    frame_checksummed[i] = frame[i+3];
  }
  frame[19] = Xbee::checkSum(frame_checksummed,16);

  #ifdef debug_setDestinationCommand_State
    Serial.print(F("setDestinationCommand_State : frame_checksummed = "));
    for(int i = 0; i < 16; i++)                                                 //
    {                                                                           //
      if(frame_checksummed[i] < 0x0F)                                                          //
      {                                                                         // DEBUG
        Serial.print("0");                                                      //
      }                                                                         //
      Serial.print(frame_checksummed[i],HEX);                                               //
      Serial.print(F(" "));                                                     //
    }                                                                           //
    Serial.println("");
    Serial.print(F("setDestinationCommand_State : checksum = "));                                             //
    Serial.println(frame[19],HEX);                                              //
    Serial.print(F("setDestinationCommand_State : frame = "));                                                                            //
    for(int i = 0; i < 20; i++)                                                 //
    {                                                                           //
      if(frame[i] < 0x0F)                                                          //
      {                                                                         // DEBUG
        Serial.print("0");                                                      //
      }                                                                         //
      Serial.print(frame[i],HEX);                                               //
      Serial.print(F(" "));                                                     //
    }                                                                           //
    Serial.println("");                                                         //
  #endif


  Xbee::sendFrame(frame,20);

  unsigned long recTO_max = 1000;
  unsigned long recTO = millis();
  while(!_packetReceived && (millis()-recTO) <= recTO_max)
  {
    Xbee::receivePacket_sense(&receivePacket_PT);
  }

  if(_packetReceived && _receive_buffer[0] == 0x97 && _receive_buffer[14] == 0x00){
    #ifdef debug_setDestinationCommand_State
      Serial.println(F("setDestinationCommand_State : response confirm"));
    #endif

    functionState = true;
    destination_command_status = pinState;
  }

  #ifdef debug_setDestinationCommand_State
  else if(_packetReceived && _receive_buffer[0] == 0x97)
  {
    Serial.println(F("setDestinationCommand_State : response received, not confirmed"));
  }
  else if(_packetReceived)
  {
    Serial.println(F("setDestinationCommand_State : packet received, but not response"));
    Serial.print(F("setDestinationCutdown_State : _receive_buffer = "));
    for(int i = 0; i < _data_received_length; i++)
    {
      if(_receive_buffer[i] <= 0x0F)
      {
        Serial.print(F("0"));
      }
      Serial.print(_receive_buffer[i],HEX);
      Serial.print(F(" "));
    }
    Serial.println(F(""));
  }
  else
  {
    Serial.println(F("setDestinationCommand_State : no packet received, timeout occured"));
  }
  #endif

  Xbee::setMode(mode);

  #ifdef debug_setDestinationCommand_State
    Serial.println(F("setDestinationCommand_State : exit"));
  #endif

  return functionState;
}

/***************************************************************************
 setDestinationCutdown_State                            NOT TESTED
 ***************************************************************************/
bool Xbee::setDestinationCutdown_State(bool pinState)
{
  #ifdef debug_setDestinationCutdown_State
    Serial.println(F("setDestinationCutdown_State : called"));
  #endif

  uint8_t mode = _MODE;
  Xbee::setMode(1);

  activity_counter = millis();
  bool functionState = false;
  uint8_t frame[20] = {0x7E, 0x00, 0x10, 0x17, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE, 0x02, 0x44, 0x00, 0x00, 0x00};

  for(int i = 5; i <= 12; i++)
  {
    frame[i] = _ADDR[i-5];
  }

  frame[17] = _DEST_CUTDOWN[1];

  if(pinState > 0)
  {
    frame[18] = 0x05;
  }
  else
  {
    frame[18] = 0x04;
  }

  uint8_t frame_checksummed[16];
  for(int i = 0; i<16; i++)
  {
    frame_checksummed[i] = frame[i+3];
  }
  frame[19] = Xbee::checkSum(frame_checksummed,16);

  #ifdef debug_setDestinationCommand_State
    Serial.print(F("setDestinationCutdown_State : frame_checksummed = "));
    for(int i = 0; i < 16; i++)                                                 //
    {                                                                           //
      if(frame_checksummed[i] < 0x0F)                                                          //
      {                                                                         // DEBUG
        Serial.print("0");                                                      //
      }                                                                         //
      Serial.print(frame_checksummed[i],HEX);                                               //
      Serial.print(F(" "));                                                     //
    }                                                                           //
    Serial.println("");
    Serial.print(F("setDestinationCutdown_State : checksum = "));                                             //
    Serial.println(frame[19],HEX);                                              //
    Serial.print(F("setDestinationCutdown_State : frame = "));                                                                            //
    for(int i = 0; i < 20; i++)                                                 //
    {                                                                           //
      if(frame[i] < 0x0F)                                                          //
      {                                                                         // DEBUG
        Serial.print("0");                                                      //
      }                                                                         //
      Serial.print(frame[i],HEX);                                               //
      Serial.print(F(" "));                                                     //
    }                                                                           //
    Serial.println("");                                                         //
  #endif

  Xbee::sendFrame(frame,20);

  unsigned long recTO_max = 500;
  unsigned long recTO = millis();
  while(!_packetReceived && (millis()-recTO) <= recTO_max)
  {
    Xbee::receivePacket_sense(&receivePacket_PT);
  }
  if(_packetReceived && _receive_buffer[0] == 0x97 && _receive_buffer[14] == 0x00){
    #ifdef debug_setDestinationCutdown_State
      Serial.println(F("setDestinationCutdown_State : packet received and confirmed"));
    #endif

    functionState = true;
    destination_cutdown_status = pinState;
  }

  #ifdef debug_setDestinationCommand_State
  else if(_packetReceived && _receive_buffer[0] == 0x97)
  {
    Serial.println(F("setDestinationCutdown_State : response received, not confirmed"));
  }
  else if(_packetReceived)
  {
    Serial.println(F("setDestinationCutdown_State : packet received, but not response"));
    Serial.print(F("setDestinationCutdown_State : _receive_buffer = "));
    for(int i = 0; i < _data_received_length; i++)
    {
      if(_receive_buffer[i] <= 0x0F)
      {
        Serial.print(F("0"));
      }
      Serial.print(_receive_buffer[i],HEX);
      Serial.print(F(" "));
    }
    Serial.println(F(""));
  }
  else
  {
    Serial.println(F("setDestinationCutdown_State : no packet received, timeout occured"));
  }
  #endif

  #ifdef debug_setDestinationCutdown_State
    Serial.println(F("setDestinationCutdown_State : exit"));
  #endif

  Xbee::setMode(mode);
  return functionState;
}
