/* Message encoding and decoding system for a lightweight communication protocol */
/* This communication protocol is intended for small to medium sized microcontrollers  */
/* Written by Ing. Segers Laurent*/
//#include "Arduino.h"
#ifdef __cplusplus
extern "C" {
#endif

#ifndef MESSAGEENCODER_H
#define MESSAGEENCODER_H

#include <stdint.h>
/* One can also use the stack as a sniffing tool */
/* Therefore, the flag "ENABLE_IP_SNIFFER" must be set */
//#define ENABLE_IP_SNIFFER
//#define PrintVerboseMessages
/* define some constants */
#define BUF_LENGTH          64
#define MAX_MESSAGE_LENGTH  63
#define PACKET_TYPE         6
#define PACKET_DELIMITER    250
#define HEADER_LENGTH       12   //-- declare the length of the header (in bytes)
#define MAX_PAYLOAD         MAX_MESSAGE_LENGTH-HEADER_LENGTH

// Next header types
#define ICMP                58
#define UDP                 17
#define ACK                 143 // unassigned in the list of protocols

#define BROADCAST_ADDRESS   0xFFFF
/**********************************************************************
 * packet frame format
 *
 * packetType | messagelength |  Next Header  | message ID |    CRC    |    Header_delimiter  | empty bytes | senderID |  receiverID |    Data
 *
 *   1 byte        1 byte           1 byte        1 byte       1 byte        1 byte             2 bytes     2 bytes      2 bytes      n bytes
 **********************************************************************/
 /* The total length of the message + header may not exceed 64 bytes*/
/*---------------------------------------------------------*/
/* First define our message structure */
typedef struct
{
    uint16_t senderID;
    uint16_t receiverID;
} Socket_t;
/*---------------------------------------------------------*/
/* define the struct to keep all information of one packet */
typedef struct
{
  uint8_t packetType;
  uint8_t messageLength;
  uint8_t next_header;
  uint8_t Message_ID;
  uint8_t CRC_Check;
  uint8_t Header_delimiter;
  uint8_t empty_byte2;
  uint8_t empty_byte3;
  Socket_t IP_info;
} Socket_info_t;
/*---------------------------------------------------------*/
/* Define some functions that will allow us build and parse the messages */
///
/// \brief IP_BufferDataByte: This function allows one to buffer data from the underlying hardware
/// byte by byte into the IP-buffer. This functions should always be followed by the function IP_read.
/// \param databyte: The input byte from the underlying receiver hardware
///
void IP_BufferDataByte(char databyte);
///
/// \brief IP_Read: The function reads the current state of the buffer and returns a datapacket if a full
/// IP-packet has been detected. This function should always be used after IP_BufferDataBye.
/// \param socket: A pointer to a structure containing the ID of the sender and receiver.
/// \param data: A pointer to an array of char, the size must be at least equal to MAX_PAYLOAD.
/// \return: Returns the amount of received bytes in char* data
///
int IP_Read(Socket_info_t* socket, char* data);
///
/// \brief IP_Setup: Setups the HOST_ID and resets the IP-buffers.
/// \param IP: The ID of the current HOST.
/// \param streamCallbackHandler: function pointer to a callback function that interacts with the hardware
///  to transmit all the bytes of the generated packet
///
void IP_Setup(uint16_t IP,void (*streamCallbackHandler)(char*,uint8_t));

#ifndef ENABLE_IP_SNIFFER // a sniffer is only allowed to read packets!!
///
/// \brief IP_Write: Builds up the the complete stream which will be transmitted to the underlying hardware.
/// \param socket: A pointer to a structure containing the ID of the sender and receiver. Only the receiver ID
/// will be used. The senderID will be set to the current HOST ID.
/// \param data: The userdata to be transmitted. The maximum length is equal to MAX_PAYLOAD
/// \param stream: The stream which will be transmitted to the underlying hardware (transmission)
/// \param datalength: The length of the userdata
/// \param packetID: the ID of the current sending packet (pass by reference!)
/// \return Returns the length of the complete packet (length of stream)
///
int IP_Write(Socket_t* socket, char* data, char* stream, int datalength,int* packetID);
///
/// \brief IP_Ack: Sends an acknowledge of packet reception to the sender of the corresponding packet
/// \param socket: the socket which has been formed as communication between sender en receiver (this)
///
void IP_Ack(Socket_info_t* socket);
#endif
/// do we want to print messages in the terminal?
#ifdef PrintVerboseMessages // use this to debug the packets on a desktop
#define IN  0
#define OUT 1
void PrintMessageInfo(volatile  Socket_info_t* socket, char* message,int in_out);
#endif

#endif // MESSAGEENCODER_H

#ifdef __cplusplus
}
#endif
