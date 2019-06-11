#ifndef xbee_h
#define xbee_h

#include "Arduino.h"
#include <pt.h>           // libraries
#include <SPI.h>

#ifndef FAST_TRANSMIT             //  used for operating mode
  #define FAST_TRANSMIT 0         //  read receipts not needed
#endif                            //
#ifndef ACCURATE_TRANSMIT         //
  #define ACCURATE_TRANSMIT 1     //  read receipt required before next packet sends
#endif                            //
#ifndef RECEIVER                  //
  #define RECEIVER 2              //  operates in receive mode
#endif                            //

// ERROR STATUSES
#ifndef CMD_NOEXIT
  #define CMD_NOEXIT 0X01
#endif
#ifndef ACCMODE_TO
  #define ACCMODE_TO 0X02
#endif
#ifndef REC_FASTMODE
  #define REC_FASTMODE 0X03
#endif
#ifndef REC_FAIL
  #define REC_FAIL 0X04
#endif


/*  CLASS DEFINITION FOR XBEE OBJECTS
    CURRENTLY DESIGNED FOR DATA RELAY AND RECEIPT OF SOME BASIC COMMANDS

*/
class Xbee{
  public:
     uint8_t data_received[256];
     int data_received_length;
     uint8_t command_receive_buffer[20];
     int command_receive_length;
     bool destination_command_status;
     bool destination_cutdown_status;
     uint8_t errorStatus;

    Xbee(uint8_t slave_select, uint8_t attention_pin);

    void setSPI_clockFreq(uint32_t freq);
    void setSPI_bitOrder(BitOrder bitorder);
    void setSPI_mode(uint8_t spi_mode);

    uint16_t begin(void);
    void setMode(uint8_t mode);
    void setMaxPayloadSize(int size_bytes);

    void setDestinationAddress(uint64_t addr);
    void setCommandInterruptPin(uint8_t pin);
    void setLocalCommand_DIO(uint8_t dio_pin);
    void setCutdownInterruptPin(uint8_t pin);
    void setLocalCutdown_DIO(uint8_t dio_pin);

    void setDestinationCommand_DIO(uint8_t dio_pin);
    void setDestinationCutdown_DIO(uint8_t dio_pin);

    bool sendAvailable(void);
    void sendFrame(uint8_t sendBuffer[], uint16_t length_of_buffer);
    void sendPayload(uint8_t sendBuffer[], uint16_t length_of_buffer);
    bool sendCommand(uint8_t command_type, uint8_t command_val);

    uint16_t messageWaiting(void);

    void protothreadLoop(void);

    void printStats(void);
    void resetStatus(void);
    void clearBuffers(void);

  private:
    bool _sendPayload; // indicates sending a string
    bool _packetReceived; // indicates a packet was received outside of command mode
    bool _packetParsed;   // indicates a packet was successfully parsed
    bool _send_confirmed;
    bool cutdownMode;
    bool _sending;

    uint8_t _XBEE_SS;
    uint8_t _XBEE_ATTN;
    uint8_t _XBEE_CMD;
    uint8_t _XBEE_CUTDWN;
    uint8_t _ADDR[8];
    uint8_t _MODE;
    uint8_t _LOCAL_COMMAND;
    uint8_t _LOCAL_CUTDOWN;
    uint8_t _DEST_COMMAND[2];
    uint8_t _DEST_CUTDOWN[2];
    uint8_t _SPI_DATAMODE;
    uint8_t _receive_buffer[274];
    uint8_t _send_buffer[274];
    uint8_t _send_header[17];
    uint8_t _maxPackets;
    uint16_t _NP;
    uint16_t _data_received_length;
    uint16_t max_payload_size;
    uint32_t _SPI_CLK_FREQ;

    long lastPacket_TIME;
    #define replyTO_THRESH 250L;    // timeout for a packet receipt (to prevent lockup of data pipeline)
    long commandMode_TO_threshold;
    long activity_counter;

    String _PAYLOAD_STRING;

    BitOrder _SPI_BITORDER;

    volatile long commandMode_TO;
    volatile bool commandMode;

    // protothread object
    struct pt receivePacket_PT;
    struct pt parsePacket_PT;
    struct pt commandMode_PT;
    struct pt commandModeTO_PT;
    struct pt fastMode_PT;
    struct pt accModeLevel0_PT;
    struct pt accModeLevel1_PT;
    struct pt garbageCollect_PT;

    // protothreads
    int receivePacket_sense(struct pt *pt);
    int parsePacket_sense(struct pt *pt);
    int commandMode_sense(struct pt *pt);
    int commandModeTO_sense(struct pt *pt);
    int fastMode_sense(struct pt *pt);
    int accModeLevel0_sense(struct pt *pt);
    int accModeLevel1_sense(struct pt *pt);
    int garbageCollect_sense(struct pt *pt);

    // Interrupt service routines
    void command_ISR(void);
    void cutdown_ISR(void);

    // functions
    bool setDestinationCommand_State(bool pinState);
    bool setDestinationCutdown_State(bool pinState);

    uint8_t checkSum(uint8_t intermed_buff_1[], uint16_t buffer_size);

    bool recSPI();
    bool parsePacket(void);
    bool exitCommandMode(void);

};
#endif
