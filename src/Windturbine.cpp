#include <arduino.h>

// -----------------------
//      For RF module
// -----------------------
#include <IPControl.h>
#define LEDpin 13
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
//      For SD card
// ---------------------
#include <SPI.h>
#include <SD.h>
File myFile;

// -------------------------
//      For timekeeping
// -------------------------
#include <Time.h>
#include <TimeLib.h>
#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message

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

// ==== Arduino mega ====
// Reserved pins (RF module): 0, 1
// Reserved pins (SD card): 10, 11, 12, 13, 50, 51, 52, 53
// Reserved pins (Display): 20, 21
// UART (TX0, RX0, TX3, RX3, TX2, RX2, TX1, RX1): 0, 1, 14, 15, 16, 17, 18, 19
// SPI (SS, MOSI, MISO, SCK): 53, 51, 50, 52
// I2C (SDA, SCL): 20, 21
// interrupt pins: 2, 3, 18, 19, 20, 21
// PWM pins: 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45, 46

#define sense_manometer 2
#define sense_turbine 3
#define current_sense_turbine A2
#define voltage_sense_turbine A3
#define current_sense_output A0
#define voltage_sense_battery A1
#define turbine_to_bat_switch 4
#define battery_output_switch 5
#define dummy_load_switch 22
// #define StepperDriverPot A3
#define Stepper_EN 23
// #define Stepper_CW 6
#define Stepper_CLK 6

// ----------------------------------
//      Declare global variables
// ----------------------------------
volatile uint16_t count_manometer = 0;
volatile uint16_t count_turbine = 0;
uint32_t preTime = 0;
uint32_t curTime = 0;
uint16_t tickLength = 1000;     // Display update rate in ms
float WindSpeedToFillBar = 15;   // m/s
double max_battery_charge_current = 2.1;
double battery_float_charge_current = 0.7;
double battery_max_idle_voltage = 13.5;
double battery_min_idle_voltage = 10.5;
uint8_t turbine_to_bat_switch_pwm = 0;
// uint8_t battery_output_switch_pwm = 0;
double battery_SOC = 0;

// ---------------------------
//      Declare functions
// ---------------------------
String formatTime();
String addTrailingSpaces(String text, int TotalLength = 16);
void processSyncMessage();
time_t requestSync();
void manometerInterrupt();
void turbineInterrupt();
double clip(double n, double lower, double upper);
double batteryChargeVoltageDrop(double currentIn, double currentOut);
double batteryDischargeVoltageDrop(double currentIn, double currentOut);
double calcBatterySOC(double batVoltage, double currentIn, double currentOut);

// ---------------
//      Setup
// ---------------
void setup() {
    pinMode(sense_manometer, INPUT);
    pinMode(sense_turbine, INPUT);
    pinMode(current_sense_turbine, INPUT);
    pinMode(voltage_sense_turbine, INPUT);
    pinMode(current_sense_output, INPUT);
    pinMode(voltage_sense_battery, INPUT);
    pinMode(turbine_to_bat_switch, OUTPUT);
    pinMode(battery_output_switch, OUTPUT);
    pinMode(dummy_load_switch, OUTPUT);
    pinMode(Stepper_EN, OUTPUT);
    pinMode(Stepper_CLK, OUTPUT);
    Serial.begin(9600);
    // Serial1.begin(9600);

    //---- Initialize RF module ----//
    pinMode(LEDpin, OUTPUT);  
    // who am I (number)?
    IPControl_Setup(8, UART_Send);  

    //---- Initiate lcd connection ----//
    lcd.begin();
    //---- Turn on backlight ----//
    lcd.backlight();

    //---- Initialise SD card ----//
    Serial.print("Initializing SD card...");
    pinMode(53, OUTPUT);
    if (!SD.begin(53)) {
        Serial.println("initialization failed!");
        return;
    }
    Serial.println("initialization done.");

    myFile = SD.open("log.txt", FILE_WRITE);
    myFile.println("---------------------------------");
    myFile.println("            New Log");
    myFile.println("---------------------------------");
    myFile.println("DD:MM:YYYY HH:MM:SS; v (m/s)");
    myFile.println("----------------------------");
    myFile.close();

    //---- Sync time with computer ----//
    setSyncProvider( requestSync );  //set function to call when sync required
    Serial.println("Waiting for sync message");

    //---- Add interrupts ----//
    attachInterrupt(digitalPinToInterrupt(sense_manometer), manometerInterrupt, RISING );
    attachInterrupt(digitalPinToInterrupt(sense_turbine), turbineInterrupt, RISING );
}

// --------------
//      Loop
// --------------
void loop() {
    // ===================
    //      Sync time
    // ===================
    if (Serial.available()) {
      processSyncMessage();
    }

    // =========================
    //      Receive RF data
    // =========================
    char receiveData[64];
    UART_receive();
    int datalen = IPControl_Read(&connection, receiveData);
    if (datalen > 0) {
        if (receiveData[5] == 'N') digitalWrite(LEDpin, HIGH);
        if (receiveData[5] == 'F') digitalWrite(LEDpin, LOW);
    }


    // ===============================
    //      Stepper drive control
    // ===============================
    // uint16_t potVal = analogRead(StepperDriverPot);
    // if (potVal < 15) {
    //     digitalWrite(Stepper_EN, HIGH);
    // } else {
    //     digitalWrite(Stepper_EN, LOW);
    // }

    // digitalWrite(Stepper_CW, LOW);
    // int speed = potVal * 30;
    // tone(Stepper_CLK, speed);
    // Serial.println(speed);

    //Serial.println(1.0 * speed / 1600 * 60 * 3);

    // ======================
    //      Process data
    // ======================
    curTime = millis();
    if ((curTime - preTime) >= tickLength) {
        // ---------------------------------
        //      Reset counter variables
        // ---------------------------------
        preTime = curTime;
        uint16_t current_manometer_count = count_manometer;
        uint16_t current_turbine_count = count_turbine;
        count_manometer = 0;
        count_turbine = 0;

        // -------------------------
        //      Calc wind speed
        // -------------------------
        float WSpeed = 0;
        // Serial.println(current_manometer_count);
        if (current_manometer_count != 0) {
            WSpeed = (current_manometer_count / tickLength * 1000);
        }

        // -------------------------------------
        //      Measure voltage and current
        // -------------------------------------
        double turbine_voltage = analogRead(voltage_sense_turbine) * 0.0311 - 0.0309;
        double turbine_current = analogRead(current_sense_turbine) * 0.00417 - 0.14326;
        double battery_voltage = analogRead(voltage_sense_battery) * 0.0315 - 0.0586;
        double output_current = analogRead(current_sense_output) * 0.00460 - 0.16617;
        turbine_voltage = abs(turbine_voltage);
        turbine_current = abs(turbine_current);
        battery_voltage = abs(battery_voltage);
        output_current  = abs(output_current );

        // --------------------------------
        //      Control battery charge
        // --------------------------------
        //---- Define regulator variables ----//
        double max_voltage = battery_max_idle_voltage + batteryChargeVoltageDrop(turbine_current, output_current);
        double min_voltage = battery_min_idle_voltage + batteryDischargeVoltageDrop(turbine_current, output_current);
        double target_battery_current = battery_float_charge_current;
        if (battery_voltage < battery_max_idle_voltage) {
            target_battery_current = max_battery_charge_current;
        }
    
        //---- Limit turbine current ----//
        if (battery_voltage < max_voltage) {
            digitalWrite(dummy_load_switch, LOW);
            double battery_current_vs_target = turbine_current / target_battery_current;

            if (battery_current_vs_target < 0.05) {
                turbine_to_bat_switch_pwm = 255;
            } else {
                if (turbine_to_bat_switch_pwm == 0) {turbine_to_bat_switch_pwm = 1;}
                turbine_to_bat_switch_pwm = clip(round(turbine_to_bat_switch_pwm / battery_current_vs_target), 0, 255);
            }

            analogWrite(turbine_to_bat_switch, turbine_to_bat_switch_pwm);

        } else {
            digitalWrite(turbine_to_bat_switch, LOW);
            turbine_to_bat_switch_pwm = 0;

            //---- Manage dummy load ----//
            if (turbine_voltage > max_voltage) {
                digitalWrite(dummy_load_switch, HIGH);
            } else {
                digitalWrite(dummy_load_switch, LOW);
            }
        }

        //---- Manage output ----//
        if (battery_voltage > min_voltage) {
            digitalWrite(battery_output_switch, HIGH);
        } else {
            digitalWrite(battery_output_switch, LOW);
        }

        //---- Get battery state of charge ----//
        battery_SOC = calcBatterySOC(battery_voltage, turbine_current, output_current);

        // -----------------------------
        //      Display data on LCD
        // -----------------------------
        // lcd.setCursor(0,0);
        // lcd.print(addTrailingSpaces("V:" + String(WSpeed), 8));
        // lcd.setCursor(8,0);
        // lcd.print(addTrailingSpaces("RPM:" + String(count_turbine * 3 * 1000 / tickLength), 8));

        // String bar = "";
        // for (int i = 0; i < floor(WSpeed * 16 / WindSpeedToFillBar ); i++) {
        //   bar += char(255);
        // }
        // lcd.setCursor(0,1);
        // lcd.print(addTrailingSpaces(bar));

        lcd.setCursor(0,0);
        lcd.print(addTrailingSpaces("U1:" + String(turbine_voltage), 9));
        lcd.setCursor(9,0);
        lcd.print(addTrailingSpaces("I1:" + String(turbine_current), 7));
        lcd.setCursor(0,1);
        lcd.print(addTrailingSpaces("U2:" + String(battery_voltage), 9));
        lcd.setCursor(9,1);
        lcd.print(addTrailingSpaces("I2:" + String(output_current), 7));

        // --------------------------------------
        //      Store a line in the log file
        // --------------------------------------
        myFile = SD.open("log.txt", FILE_WRITE);
        if (myFile) {
            myFile.println(formatTime() + String(WSpeed));
            // Serial.println(formatTime() + String(WSpeed));
        } else {
            // Serial.println("error opening log.txt");
        }
        myFile.close();

        // ----------------------
        //      Send RF data
        // ----------------------
        // write here a stream of characters (string)
        char message_out[64] = "This is a test";
        // calculate the length of the string to be sent
        int stringlength = strlen(message_out);
        //-- who is going to receive our messages?
        connection.receiverID = 1;
        IPControl_Write(&connection, message_out, stream, stringlength);        
    }
}

// --------------------------
//      Helper functions
// --------------------------
String formatTime() {
    String res = "";

    if ( day() < 10 ) { res += "0"; }
    res += String( day() ) + "/";

    if ( month() < 10 ) { res += "0"; }
    res += String( month() ) + "/";

    if ( year() < 10 ) { res += "000"; }
    else if ( year() < 100 ) { res += "00"; }
    else if ( year() < 1000 ) { res += "0"; }
    res += String( year() ) + " ";

    if ( hour() < 10 ) { res += "0"; }
    res += String( hour() ) + ":";

    if ( minute() < 10 ) { res += "0"; }
    res += String( minute() ) + ":";

    if ( second() < 10 ) { res += "0"; }
    res += String( second() ) + "; ";

    return res;
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

void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

    if(Serial.find(TIME_HEADER)) {
        pctime = Serial.parseInt();
        if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
            setTime(pctime); // Sync Arduino clock to the time received on the serial port
        }
    }
}

time_t requestSync() {
    Serial.write(TIME_REQUEST);
    return 0; // the time will be sent later in response to serial mesg
}

void manometerInterrupt() {
    count_manometer += 1;
}

void turbineInterrupt() {
    count_turbine += 1;
}

double clip(double n, double lower, double upper) {
    return max(lower, min(n, upper));
}

double mosfetVoltageDrop(double current) {
    return current * 0.2;
}

double batteryChargeVoltageDrop(double currentIn, double currentOut) {
    double batCurrent = currentIn - currentOut;

    if (batCurrent > 0.2) {
        return 0.7;
    } else {
        return 0;
    }
}

double batteryDischargeVoltageDrop(double currentIn, double currentOut) {
    double batCurrent = currentIn - currentOut;

    if (batCurrent < -0.2) {
        return -0.4;
    } else {
        return 0;
    }
}

double calcBatterySOC(double batVoltage, double currentIn, double currentOut) {
    double realVoltage = batVoltage + batteryChargeVoltageDrop(currentIn, currentOut) + batteryDischargeVoltageDrop(currentIn, currentOut);
    double range = battery_max_idle_voltage - battery_min_idle_voltage;

    return (batVoltage - battery_min_idle_voltage) / range;
}

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