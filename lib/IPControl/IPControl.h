#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IPCONTROL_H
#define IPCONTROL_H

#include "IP.h"
#include <stdint.h>


/// \brief IPControl_Setup: Setups the HOST_ID and resets the IPControl-buffers, this function calls function IP_Setup.
/// \param IP: The ID of the current HOST.
/// \param streamCallbackHandler: function pointer to a callback function that interacts with the hardware
///  to transmit all the bytes of the generated packet
///
void IPControl_Setup(uint16_t ID,void (*streamCallbackHandler)(char*,uint8_t));
///
/// \brief IPControl_Read: The function reads the current state of the buffer and returns a datapacket if a full
/// IP-packet has been detected. This function should always be used after IP_BufferDataBye (IP.h).
/// \param socket: A pointer to a structure containing the ID of the sender and receiver.
/// \param data: A pointer to an array of char, the size must be at least equal to MAX_PAYLOAD.
/// \return: Returns the amount of received bytes in char* data
///
int IPControl_Read(Socket_t* socket, char* data);

#ifndef ENABLE_IP_SNIFFER
///
/// \brief IPControl_Write: Builds up the the complete stream which will be transmitted to the underlying hardware.
/// \param socket: A pointer to a structure containing the ID of the sender and receiver. Only the receiver ID
/// will be used. The senderID will be set to the current HOST ID.
/// \param data: The userdata to be transmitted. The maximum length is equal to MAX_PAYLOAD
/// \param stream: The stream which will be transmitted to the underlying hardware (transmission)
/// \param datalength: The length of the userdata
/// \return Returns the length of the complete packet (length of stream)
///

int IPControl_Write(Socket_t* socket, char* data, char* stream, int datalength);
#endif

#endif

#ifdef __cplusplus
}
#endif