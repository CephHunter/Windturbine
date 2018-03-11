#include "IPControl.h"

/* Store our own HostID */
volatile uint16_t HOST_ID;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void IPControl_Setup(uint16_t ID,void (*streamCallbackHandler)(char*,uint8_t))
{
  IP_Setup(ID,streamCallbackHandler);
  HOST_ID=ID;
}
//------------------------------------------------------------------------------
#ifndef ENABLE_IP_SNIFFER
int IPControl_Write(Socket_t* socket, char* data, char* stream, int datalength)
{
  int packetID;
  return IP_Write(socket, data, stream, datalength,&packetID);
}
#endif
//------------------------------------------------------------------------------
int IPControl_Read(Socket_t* socket, char* data)
{
  Socket_info_t s;
  // parse here the data -> do we have a packet for us?
  int message_len = IP_Read(&s, data);
  if (message_len>=0)
  {
    if (s.next_header==UDP)
    {
      if (s.IP_info.receiverID==HOST_ID) // we do ACK for point to point communication
      {
        *socket=s.IP_info;
#ifdef PrintVerboseMessages
        PrintMessageInfo(&s, data,IN);
#endif
// send ACK
#ifndef ENABLE_IP_SNIFFER
        IP_Ack(&s);
#endif
        return message_len;
      }
      else if (s.IP_info.receiverID==BROADCAST_ADDRESS) // on broadcast we do not ACK back
      {
        *socket=s.IP_info;
#ifdef PrintVerboseMessages
        PrintMessageInfo(&s, data,IN);
#endif
        return message_len;
      }
    }
    else if (s.next_header==ACK) // remove the message from pending ACK list
    {
      //-- get message ID to remove from pending list
#ifdef PrintVerboseMessages
      PrintMessageInfo(&s, 0,IN);
#endif
    }
  }
  return 0;
}
//------------------------------------------------------------------------------
