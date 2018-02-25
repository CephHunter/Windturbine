#include <arduino.h>

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
// Reserved pins (TX, RX): 0, 1
// Reserved pins (SD card): 10, 11, 12, 13
// Reserved pins (Display): A4, A5
// interrupt pins: 2, 3
// PWM pins: 3, 5, 6, 9, 10, 11
#define sense_manometer 2
#define sense_turbine 3
#define current_sense_turbine A2
#define voltage_sense_turbine A3
#define current_sense_output A0
#define voltage_sense_battery A1
#define turbine_to_bat_switch 6
#define battery_output_switch 5
#define dummy_load_switch 4
// #define StepperDriverPot A3
#define Stepper_EN 8
// #define Stepper_CW 6
#define Stepper_CLK 9

// ----------------------------------
//      Declare global variables
// ----------------------------------
volatile uint16_t count_manometer = 0;
volatile uint16_t count_turbine = 0;
uint32_t preTime = 0;
uint32_t curTime = 0;
uint16_t diff = 0;
uint8_t tickRate = 1000;     // Display update rate in ms
float WindSpeedToFillBar = 15;   // m/s

// ---------------------------
//      Declare functions
// ---------------------------
String formatTime(uint32_t t);
String addTrailingSpaces(String text, int TotalLength = 16);
void processSyncMessage();
time_t requestSync();
void manometerInterrupt();
void turbineInterrupt();

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

    //---- Initiate lcd connection ----//
    lcd.begin();
    //---- Turn on backlight ----//
    lcd.backlight();

    //---- Initialise SD card ----//
    Serial.print("Initializing SD card...");
    if (!SD.begin(10)) {
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
    diff = curTime - preTime;
    if (diff >= tickRate) {
        preTime = curTime;

        // -------------------------
        //      Calc wind speed
        // -------------------------
        float WSpeed = 0;
        if (count_manometer != 0) {
            WSpeed = (count_manometer * tickRate * 3) * 0.0306 - 1.22;
        }

        // -------------------------------------
        //      Measure voltage and current
        // -------------------------------------
        double turbine_voltage = abs(analogRead(voltage_sense_turbine) * 0.0311 - 0.0309);
        double turbine_current = abs(analogRead(current_sense_turbine) * 0.00417 - 0.14326);
        double battery_voltage = abs(analogRead(voltage_sense_battery) * 0.0315 - 0.0586);
        double output_current = abs(analogRead(current_sense_output) * 0.00460 - 0.16617);

        // ---------------------------------
        //      Control battery charger
        // ---------------------------------
        // digitalWrite(turbine_to_bat_switch, LOW);
        // digitalWrite(battery_output_switch, LOW);
        // digitalWrite(dummy_load_switch, LOW);
        if (turbine_voltage > 11.5 && battery_voltage < 13) {
            
        } else {
            digitalWrite(turbine_to_bat_switch, LOW);
        }

        // ---------------------------------
        //      Display data on the LCD
        // ---------------------------------
        // lcd.setCursor(0,0);
        // lcd.print(addTrailingSpaces("V:" + String(WSpeed), 8));
        // lcd.setCursor(8,0);
        // lcd.print(addTrailingSpaces("RPM:" + String(count_turbine * 3 * tickRate), 8));

        // String bar = "";
        // for (int i = 0; i < floor(WSpeed * 16 / WindSpeedToFillBar ); i++) {
        //   bar += char(255);
        // }
        // lcd.setCursor(0,1);
        // lcd.print(addTrailingSpaces(bar));

        lcd.setCursor(0,0);
        lcd.print(addTrailingSpaces("U1:" + String(turbine_voltage), 8));
        lcd.setCursor(8,0);
        lcd.print(addTrailingSpaces("I1:" + String(turbine_current), 8));
        lcd.setCursor(0,1);
        lcd.print(addTrailingSpaces("U2:" + String(battery_voltage), 8));
        lcd.setCursor(8,1);
        lcd.print(addTrailingSpaces("I2:" + String(output_current), 8));

        // --------------------------------------
        //      Store a line in the log file
        // --------------------------------------
        myFile = SD.open("log.txt", FILE_WRITE);
        if (myFile) {
            myFile.println(formatTime(curTime) + String(WSpeed));
            Serial.println(formatTime(curTime) + String(WSpeed));
        } else {
            Serial.println("error opening test.txt");
        }
        myFile.close();

        // ---------------------------------
        //      Reset counter variables
        // ---------------------------------
        count_manometer = 0;
        count_turbine = 0;
    }
}

// --------------------------
//      Helper functions
// --------------------------
String formatTime(uint32_t t) {
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
