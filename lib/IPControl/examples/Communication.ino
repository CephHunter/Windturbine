// include the library here (IP)
//#include "IP.h"
#include "IPControl.h"

// define a LED pin
#define LEDpin 13
//-- define the connection between sender and receiver
Socket_t connection; 
// declare a stream variable array here
char stream[64];
//-- keep track of time
uint64_t previoustime;

void UART_receive();
void UART_Send(char* data, uint8_t len);
int LED_state = 0;
int rpm = 0;
double windSpeed=0;
//-----------------------------------------------
void setup() 
{
  Serial.begin(9600);
  pinMode(LEDpin,OUTPUT);  
  // who am I (number)?
  IPControl_Setup(8,UART_Send);  
  // reset time
  previoustime = 0;
}
//-----------------------------------------------
void loop() 
{
  // receive part of the code
  char receiveData[64];
  UART_receive();
  int datalen = IPControl_Read(&connection, receiveData);
  if (datalen>0)
  {
    if (receiveData[5]=='N') LED_state = 1;//digitalWrite(LEDpin,HIGH);
    if (receiveData[5]=='F') LED_state = 0;//digitalWrite(LEDpin,LOW);  
  }
  if (LED_state==0)
  {
    digitalWrite(LEDpin,LOW);
  }
  else 
  {
    digitalWrite(LEDpin,HIGH);
  }
  
  // send data every 1 seconds
  if (millis()-previoustime>=1000)
  {
    // write here a stream of characters (string)
    char message_out[64];
    /*if (LED_state==0)
    {
      strcpy(message_out,"LED=OFF");
    }
    else 
    {
      strcpy(message_out,"LED=ON");  
    }*/

    String message = "2;" + String(rpm) + ";"+String(windSpeed) + ";1;12.5";    
    message.toCharArray(message_out, 64);
    
    rpm++;
    windSpeed+=0.5;
    if(rpm>30){rpm=0;}
    if(windSpeed>10){windSpeed = 0;}
    // calculate the length of the string to be sent
    int stringlength = strlen(message_out);
    //-- who is going to receive our messages?
    connection.receiverID = 1;
    IPControl_Write(&connection, message_out, stream, stringlength);
    previoustime = millis();
  }
}
//-----------------------------------------------
//-----------------------------------------------
//-----------------------------------------------
// HELPER FUNCTIONS FOR IP (must be here)
//-----------------------------------------------
//-----------------------------------------------
//-----------------------------------------------
void UART_Send(char* data, uint8_t len)
{
  Serial.write(data,len);  
}
//-----------------------------------------------
void UART_receive()
{
  char incomingByte;
  while(Serial.available() > 0) 
  {
    // read the incoming byte:
    incomingByte = Serial.read();
    IP_BufferDataByte(incomingByte);
  }
}
//-----------------------------------------------
