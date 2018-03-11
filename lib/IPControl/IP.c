/* Communication stack implementation file */
/* Written by Ing. Segers Laurent */
#include "IP.h"
#ifdef PrintVerboseMessages
#include <stdio.h>
#endif

/* build a buffer with a writeindex */
/* first make a union of the IP_header and the complete databuffer*/
/* this structure is intended for buffering every databyte before decoding */
typedef union
{
    Socket_info_t socket_info;
    char datastream[HEADER_LENGTH];
} IP_info_t;
/*----------------------------------------------------------------------------*/
typedef struct
{
    int16_t writeIndex;
    int16_t readIndex;
    uint8_t datastream[BUF_LENGTH]; // mimic a ring buffer
} IP_Ringbuffer_t;
/*----------------------------------------------------------------------------*/
/*---------------------------------------------------------*/
/* Define a union which will map the MessageHeader_t structure onto a array of char */
/* The array of char is the most basic received datatype from communication lines */
typedef union
{
    char datastream[HEADER_LENGTH];
    Socket_info_t header;
} Message_t;
/*---------------------------------------------------------*/
volatile IP_Ringbuffer_t IP_buffer;
/* Store our own HostID */
volatile uint16_t HOST_ID;
volatile uint8_t message_count; // to keep track of the Message ID -> incremented by each message
volatile Message_t OutGoingMessage;
volatile IP_info_t ReceiveIP_info;

int IP_FindStartByte();
void CopyInput2Stream(int startIndex, int length, char* output);
/*----------------------------------------------------------------------------*/
// declare here the callback function for transmitting the output stream
void (*IPWriteStream_Callback)(char* data,uint8_t len);

/*----------------------------------------------------------------------------*/
void IP_BufferDataByte(char databyte)
{
    IP_buffer.datastream[IP_buffer.writeIndex] = databyte;
    //-- continue writing in the buffer until the end, then return
    if (IP_buffer.writeIndex==BUF_LENGTH-1) IP_buffer.writeIndex=0;
    else IP_buffer.writeIndex++;
}
/*----------------------------------------------------------------------------*/
int IP_FindStartByte()
{
    unsigned int start_byte_found;
    int16_t current_readIndex;

    start_byte_found=0;
    current_readIndex = IP_buffer.readIndex;
    while((IP_buffer.writeIndex!=current_readIndex)&&(start_byte_found==0))
    {
      if (IP_buffer.datastream[current_readIndex]==PACKET_TYPE)
      {
          IP_buffer.readIndex = current_readIndex;
          return 1;
      }
      //-- update pointers
      if (current_readIndex==BUF_LENGTH-1) current_readIndex=0;
      else current_readIndex++;
    }
    return -1;
}
/*----------------------------------------------------------------------------*/
void CopyInput2Stream(int startIndex, int length, char* output)
{
  int i;
  uint8_t readIndex = startIndex%BUF_LENGTH;
  for (i=0;i<length;i++)
  {
    output[i] = IP_buffer.datastream[readIndex];
    if (readIndex==BUF_LENGTH-1) readIndex=0;
    else readIndex++;
  }
}
/*----------------------------------------------------------------------------*/
int IP_Read(Socket_info_t* socket, char* data)
{
  short int i;
  short int message_len;
  uint8_t data_length;
  uint8_t CRC_DATA = 0;

  if (IP_FindStartByte())
  {
    // first check if we have enough bytes to decode
    message_len = IP_buffer.writeIndex-IP_buffer.readIndex;
    if (message_len<0) message_len = message_len+BUF_LENGTH;
    if (message_len>=HEADER_LENGTH) // we have more than one header length of data, try parsing data
    {
      // decode header here
      CopyInput2Stream(IP_buffer.readIndex, HEADER_LENGTH,(char*)ReceiveIP_info.datastream);
      //CopyHeader2InfoStruct();
      // do we have enough bytes to process a complete packet?
      if (ReceiveIP_info.socket_info.Header_delimiter==PACKET_DELIMITER)
      {
        if (message_len>=ReceiveIP_info.socket_info.messageLength)
        {
          /* Calculate the CRC check of all bytes -> must be zero in order to accept the packet */
          /* start with the header */
          for (i=0;i<HEADER_LENGTH;i++) CRC_DATA = CRC_DATA^ReceiveIP_info.datastream[i];
          /* continue with the data payload */
          data_length = ReceiveIP_info.socket_info.messageLength-HEADER_LENGTH;
          CopyInput2Stream(IP_buffer.readIndex+HEADER_LENGTH,data_length, data);
          for (i=0;i<data_length;i++) CRC_DATA = CRC_DATA^data[i];
          /* if CRC != 0 -> drop packet (faulty), else accept */
          //printf("Value of the CRC = %02x\r\n", (uint8_t)CRC_DATA);
          if (CRC_DATA==0)
          {
            *socket =  ReceiveIP_info.socket_info;
            IP_buffer.readIndex = IP_buffer.readIndex+ReceiveIP_info.socket_info.messageLength;
            if (IP_buffer.readIndex>=BUF_LENGTH) IP_buffer.readIndex=IP_buffer.readIndex-BUF_LENGTH;
            return data_length;
          }
          else
          {
            //just move on with the next packet
            if (IP_buffer.readIndex==BUF_LENGTH-1) IP_buffer.readIndex=0;
            else IP_buffer.readIndex++;
          }
        }
      }
      else
      {
        //just move on with the next packet
        if (IP_buffer.readIndex==BUF_LENGTH-1) IP_buffer.readIndex=0;
        else IP_buffer.readIndex++;
      }
    }
  }
  return -1;
}
/*----------------------------------------------------------------------------*/
void IP_Setup(uint16_t IP, void (*streamCallbackHandler)(char*,uint8_t))
{
    HOST_ID = (IP>0?IP:0);
    IP_buffer.writeIndex = 0;
    IP_buffer.readIndex = 0;
    message_count = 0;
    IPWriteStream_Callback = streamCallbackHandler;
}
/*----------------------------------------------------------------------------*/
#ifndef ENABLE_IP_SNIFFER // a sniffer is only allowed to read packets!!
int IP_Write(Socket_t* socket, char* data, char* stream, int datalength,int* packetID)
{
    uint16_t i;
    uint8_t CRC = 0;
    if ((datalength>0)&&(datalength<MAX_PAYLOAD))
    {
        *packetID = message_count;
        socket->senderID=HOST_ID;
        OutGoingMessage.header.packetType = PACKET_TYPE;
        OutGoingMessage.header.IP_info=*socket;
        OutGoingMessage.header.Message_ID=message_count;
        OutGoingMessage.header.next_header=UDP;
        message_count++;
        OutGoingMessage.header.messageLength=datalength + HEADER_LENGTH;
        OutGoingMessage.header.CRC_Check = 0;
        OutGoingMessage.header.Header_delimiter=PACKET_DELIMITER;
        OutGoingMessage.header.empty_byte2=0;
        OutGoingMessage.header.empty_byte3=0;

        for(i=0;i<HEADER_LENGTH;i++)
        {
          CRC =CRC^OutGoingMessage.datastream[i];
          stream[i] = OutGoingMessage.datastream[i];
        }

        for (i=0;i<datalength;i++)
        {
            stream[HEADER_LENGTH+i]=data[i];
            CRC = CRC^data[i];
        }
        stream[4] = CRC;
        OutGoingMessage.header.CRC_Check = CRC;
        //-- stream our data to the underlying UART
        if (IPWriteStream_Callback!=0)
        {
          IPWriteStream_Callback(stream,OutGoingMessage.header.messageLength);
        }
#ifdef PrintVerboseMessages
        PrintMessageInfo(&OutGoingMessage.header, data,OUT);
#endif
        return OutGoingMessage.header.messageLength; // success!
    }
    return 0;
}
/*----------------------------------------------------------------------------*/
void IP_Ack(Socket_info_t* socket)
{
    //char stream[HEADER_LENGTH];
    uint16_t i;
    uint8_t CRC = 0;
    char stream[HEADER_LENGTH];
    OutGoingMessage.header.Message_ID = socket->Message_ID;
    OutGoingMessage.header.IP_info.receiverID = socket->IP_info.senderID;
    OutGoingMessage.header.IP_info.senderID = HOST_ID;
    OutGoingMessage.header.messageLength=HEADER_LENGTH;
    OutGoingMessage.header.CRC_Check=0;
    OutGoingMessage.header.next_header = ACK;
    OutGoingMessage.header.packetType = PACKET_TYPE;
    OutGoingMessage.header.Header_delimiter=PACKET_DELIMITER;
    OutGoingMessage.header.empty_byte2=0;
    OutGoingMessage.header.empty_byte3=0;
    for (i=0;i<HEADER_LENGTH;i++)
    {
        CRC = CRC^OutGoingMessage.datastream[i];
        stream[i] = OutGoingMessage.datastream[i];
    }
    stream[4] = CRC;
    OutGoingMessage.header.CRC_Check = CRC;
    if (IPWriteStream_Callback!=0)
    {
      IPWriteStream_Callback(stream,HEADER_LENGTH);
    }
#ifdef PrintVerboseMessages
    PrintMessageInfo(&OutGoingMessage.header, 0,OUT);
#endif

}
#endif
/* Some helper functions */
/*----------------------------------------------------------------------------*/
#ifdef PrintVerboseMessages
/*----------------------------------------------------------------------------*/
inline void PrintMessageInfo(volatile  Socket_info_t* socket, char* message,int in_out)
{
		int i;
		if (socket->next_header==UDP)
		{
				printf("Sender ID: %04X ++ Receiver ID: %04X ++ Message ID: %d ++ Message: ", socket->IP_info.senderID, socket->IP_info.receiverID, socket->Message_ID);
				// print ingoing our outgoging information
				if (in_out==IN) printf("\e[32m <-- \e[0m");
				else if (in_out==OUT) printf("\e[31m --> \e[0m");
				for (i=0;i<socket->messageLength-HEADER_LENGTH;i++)
				{
					printf("%02X ",(uint8_t)message[i]);
				}
				printf("\r\n");
		}
    else if (socket->next_header == ACK)
    {
        if (in_out==OUT)
        {
            printf("\e[31mSender ID: %04X ++ Receiver ID: %04X ++ Message ID: %d --> ACK\e[0m\r\n", socket->IP_info.senderID, socket->IP_info.receiverID, socket->Message_ID);
        }
        else
        {
            printf("\e[32mSender ID: %04X ++ Receiver ID: %04X ++ Message ID: %d <-- ACK\e[0m\r\n", socket->IP_info.senderID, socket->IP_info.receiverID, socket->Message_ID);
        }
    }
}
/*----------------------------------------------------------------------------*/
#endif
