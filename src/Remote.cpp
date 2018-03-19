#include <arduino.h>

// --------------------------------
//      Include control values
// --------------------------------
#include <ini.h>

// -----------------------
//      For RF module
// -----------------------
#include <IPControl.h>
// define the connection between sender and receiver
Socket_t connection; 
// declare a stream variable array here
char stream[64];
void UART_receive();
void UART_Send(char* data, uint8_t len);

// ---------------------
//      Define pins
// ---------------------
// ==== Arduino Uno ====
// Reserved pins (RF module): 0, 1
// Reserved pins (SD card): 10, 11, 12, 13
// Reserved pins (Display): A4, A5
// UART (TX, RX): 0, 1
// SPI (SS, MOSI, MISO, SCK): 10, 11, 12, 13
// I2C (SDA, SCL): A4, A5
// interrupt pins: 2, 3
// PWM pins: 3, 5, 6, 9, 10, 11
#define POT A0
#define driveSwitch A1
#define brakeSwitch A2

// ----------------------------------
//      Declare global variables
// ----------------------------------
int potValue = 0;
int drive = 0;
int brake = 0;

// ---------------------------
//      Declare functions
// ---------------------------

// ---------------
//      Setup
// ---------------
void setup() {
    pinMode(POT, INPUT);
    pinMode(driveSwitch, INPUT);
    pinMode(brakeSwitch, INPUT);

    //---- Initialize RF module ----// 
    // who am I ()? 107, 108, 109
    // admin = 107
    IPControl_Setup(108, UART_Send);  
}

// --------------
//      Loop
// --------------
void loop() {
    potValue = analogRead(POT);
    drive = digitalRead(driveSwitch);
    brake = digitalRead(brakeSwitch);

    // =========================
    //      Receive RF data
    // =========================
    char receiveData[64];
    UART_receive();
    int datalen = IPControl_Read(&connection, receiveData);
    if (datalen > 0) {
        // Do something
    }

    // ======================
    //      Send RF data
    // ======================
    // write here a stream of characters (string)
    char message_out[64];
    sprintf(message_out, "%d;%d;%d", potValue, drive, brake);
    // calculate the length of the string to be sent
    int stringlength = strlen(message_out);
    //-- who is going to receive our messages?
    connection.receiverID = 107;
    IPControl_Write(&connection, message_out, stream, stringlength);
}

// --------------------------
//      Helper functions
// --------------------------
void UART_Send(char* data, uint8_t len) {
    Serial.write(data, len);  
}

void UART_receive() {
    char incomingByte;
    while (Serial.available() > 0) {
        // read the incoming byte:
        incomingByte = Serial.read();
        IP_BufferDataByte(incomingByte);
    }
}