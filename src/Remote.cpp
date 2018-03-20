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

// -------------------------
//      For LCD display
// -------------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 16, 2);

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
#define brakeDIR A3

// ----------------------------------
//      Declare global variables
// ----------------------------------
int potValue = 0;
int drive = 0;
int brake = 0;
int DIR = 0;
uint32_t prevtime = 0;
double turbine_voltage = 0;
double turbine_current = 0;
double battery_voltage = 0;
double output_current = 0;

// ---------------------------
//      Declare functions
// ---------------------------
String addTrailingSpaces(String text, int TotalLength = 16);
double clip(double n, double lower, double upper);
double roundDepth(double digit, int precision = 0);

// ---------------
//      Setup
// ---------------
void setup() {
    pinMode(POT, INPUT);
    pinMode(driveSwitch, INPUT);
    pinMode(brakeSwitch, INPUT);
    pinMode(brakeDIR, INPUT);
    Serial.begin(9600);

    //---- Initialize RF module ----// 
    // who am I ()? 107, 108, 109
    // admin = 107
    IPControl_Setup(108, UART_Send);  

    //---- Initiate lcd connection ----//
    lcd.begin();
    //---- Turn on backlight ----//
    lcd.backlight();
}

// --------------
//      Loop
// --------------
void loop() {
    // =========================
    //      Receive RF data
    // =========================
    char receiveData[64];
    UART_receive();
    int datalen = IPControl_Read(&connection, receiveData);
    if (datalen > 0) {
        int data[10];
        int counter = 0;
        char* strpart = strtok(receiveData, ";");
        while (strpart != NULL) {
            data[counter] = atoi(strpart);
            strpart = strtok(NULL, ";");
            counter += 1;
        }

        turbine_voltage = data[0] / 100.;
        turbine_current = data[1] / 100.;
        battery_voltage = data[2] / 100.;
        output_current  = data[3] / 100.;

        // -----------------------------
        //      Display data on LCD
        // -----------------------------
        lcd.setCursor(0,0);
        lcd.print(addTrailingSpaces("U1:" + String(turbine_voltage), 9));
        lcd.setCursor(9,0);
        lcd.print(addTrailingSpaces("I1:" + String(turbine_current), 7));
        lcd.setCursor(0,1);
        lcd.print(addTrailingSpaces("U2:" + String(battery_voltage), 9));
        lcd.setCursor(9,1);
        lcd.print(addTrailingSpaces("I2:" + String(output_current), 7));
    }

    if (millis() - prevtime > 500) {
        prevtime = millis();

        potValue = analogRead(POT);
        if (digitalRead(driveSwitch) == HIGH) {
            drive = 1;
        } else {
            drive = 0;
        }
        if (digitalRead(brakeSwitch) == HIGH) {
            brake = 1;
        } else {
            brake = 0;
        }
        if (digitalRead(brakeDIR) == HIGH) {
            DIR = 1;
        } else {
            DIR = 0;
        }

        // ======================
        //      Send RF data
        // ======================
        // write here a stream of characters (string)
        char message_out[64];
        sprintf(message_out, "%d;%d;%d;%d;0", potValue, drive, brake, DIR);
        Serial.println(message_out);
        // calculate the length of the string to be sent
        int stringlength = strlen(message_out);
        //-- who is going to receive our messages?
        connection.receiverID = 107;
        IPControl_Write(&connection, message_out, stream, stringlength);
    }
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

String addTrailingSpaces(String text, int TotalLength) {
    String res = text;
    int blocksToAdd = TotalLength - text.length();
    if (blocksToAdd < 0) {blocksToAdd = 0;}
    for (uint8_t i = 0; i < blocksToAdd; i++) {
        res += " ";
    }
    return res;
}

double clip(double n, double lower, double upper) {
    return max(lower, min(n, upper));
}

double roundDepth(double digit, int precision) {
    return round(digit * pow(10, precision)) / pow(10, precision);
}