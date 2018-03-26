#include <arduino.h>
#include <util/atomic.h>

// --------------------------------
//      Include control values
// --------------------------------
#include <ini.h>
#include <Shared functions.h>

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
//      For SD card
// ---------------------
#include <SPI.h>
#include <SD.h>
File myFile;

// -------------------------
//      For timekeeping
// -------------------------
// #include <Time.h>
#include <TimeLib.h>
// #include <Wire.h>    // Already included
#include <DS1307RTC.h>
#define TIME_HEADER  "T"   // Header tag for serial time sync message
// #define TIME_REQUEST  7    // ASCII bell character requests a time sync message

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
#define Stepper_EN 23
#define Stepper_DIR 24
#define Stepper_CLK 6
#define generator_drive_switch 25
#define brake_switch 26
#define brake_limit_switch 27

// ----------------------------------
//      Declare global variables
// ----------------------------------
//---- Counters ----//
volatile uint16_t count_manometer = 0;
volatile uint16_t count_turbine = 0;
uint16_t current_manometer_count = 0;
uint16_t current_turbine_count = 0;
//---- Measurements ----//
double turbine_voltage = 0;
double turbine_current = 0;
double battery_voltage = 0;
double output_current =  0;
double generatedPower = 0;
double WSpeed = 0;  // Wind speed
uint16_t turbineRPM = 0;
double battery_SOC = 0; // Battery state of charge
//---- Control variables ----//
uint32_t preTime = 0;   // Used to periodicly run a codeblock
uint32_t curTime = 0;   // ^
uint8_t turbine_to_bat_switch_pwm = 0;
uint8_t turbine_status = 0;         // 1 == turning, 0 == not turning
uint8_t brakeStepperStatus = 0;     // 0 == do nothing, 1 == close brake, 2 == open brake
uint8_t stopTurbineStatus = 0;      // 0 == do nothing, 1 == bring turbine to a complete stop
uint8_t turbineBoostStatus = 0;     // 0 == do nothing, 1 == boost turbine
uint32_t stepperStartTime = 0;      // Used to control the sweep of the stepper
//---- Other ----//
#define UINT32_MAX 4294967295

// ---------------------------
//      Declare functions
// ---------------------------
String formatTime();
unsigned long processSyncMessage();
// time_t requestSync();
void manometerInterrupt();
void turbineInterrupt();
double batteryChargeVoltageDrop(double currentIn, double currentOut);
double batteryDischargeVoltageDrop(double currentIn, double currentOut);
double calcBatterySOC(double batVoltage, double currentIn, double currentOut);
void freqSweep(uint16_t startFreq, uint16_t endFreq, uint32_t startTime, uint16_t duration);
uint8_t openBrake(uint32_t startTime);
uint8_t closeBrake(uint32_t startTime);
uint8_t stopTurbine(uint32_t startTime);
uint8_t startTurbine(uint32_t startTime);
void disableStepper();

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
    pinMode(Stepper_DIR, OUTPUT);
    pinMode(Stepper_CLK, OUTPUT);
    pinMode(generator_drive_switch, OUTPUT);
    pinMode(brake_switch, OUTPUT);
    pinMode(brake_limit_switch, INPUT);
    Serial.begin(9600);
    Serial3.begin(9600);

    //---- Initialize RF module ----// 
    // who am I ()? 107, 108, 109
    // admin = 107
    IPControl_Setup(107, UART_Send);  

    // //---- Initiate lcd connection ----//
    // lcd.begin();
    // //---- Turn on backlight ----//
    // lcd.backlight();

    //---- Initialise SD card ----//
    Serial.print("Initializing SD card...");
    pinMode(53, OUTPUT);
    if (!SD.begin(53)) {
        Serial.println("initialization failed!");
        return;
    }
    Serial.println("initialization done.");

    myFile = SD.open(SDcardFileName, FILE_WRITE);
    myFile.println("---------------------------------");
    myFile.println("            New Log");
    myFile.println("---------------------------------");
    myFile.println("time;turbine_status;brakeStepperStatus;turbine_voltage;turbine_current;battery_voltage;output_current;WSpeed;turbineRPM;generatedPower;battery_SOC");
    myFile.println("---------------------------------");
    myFile.close();

    //---- Sync time with computer ----//
    setSyncProvider( RTC.get );  //set function to call when sync required
    if (timeStatus() != timeSet) 
        Serial.println("Unable to sync with the RTC");
    else
        Serial.println("RTC has set the system time"); 

    //---- Add interrupts ----//
    attachInterrupt(digitalPinToInterrupt(sense_manometer), manometerInterrupt, RISING );
    attachInterrupt(digitalPinToInterrupt(sense_turbine), turbineInterrupt, RISING );

    //---- Open the brake ----//
    brakeStepperStatus = 2;
    stepperStartTime = millis();
    while (brakeStepperStatus == 2) {
        brakeStepperStatus = openBrake(stepperStartTime);
    }
}

// --------------
//      Loop
// --------------
void loop() {
    // ===================
    //      Sync time
    // ===================
    if (Serial.available()) {
        time_t t = processSyncMessage();
        if (t != 0) {
            RTC.set(t);   // set the RTC and the system time to the received value
            setTime(t);          
        }
    }

    // =========================
    //      Receive RF data
    // =========================
    char receiveData[64];
    UART_receive();
    int datalen = IPControl_Read(&connection, receiveData);
    if (datalen > 0) {
        // Serial.println(receiveData);

        switch (stringIdentifier(receiveData)) {
            case 90748:     // SEND
                {
                    String message = "3;" + String(turbineRPM) + ";" + String(WSpeed) + ";" + String(turbine_status) + ";" + String(generatedPower) + ";" + String(now());
                    char message_out[64];
                    message.toCharArray(message_out, 64);
                    int stringlength = strlen(message_out);
                    connection.receiverID = 1;
                    IPControl_Write(&connection, message_out, stream, stringlength);
                    break;
                }
            case 921404:    // START
                {
                    break;
                }
                
            case 92270:     // STOP
                {
                    break;
                }
            default:
                {
                    int data[10];
                    int counter = 0;
                    char* strpart = strtok(receiveData, ";");
                    while (strpart != NULL) {
                        data[counter] = stringToInt(strpart);
                        strpart = strtok(NULL, ";");
                        counter += 1;
                    }

                    if (data[3] == 1) {
                        digitalWrite(Stepper_DIR, stepperDIRvalToCloseBrake);
                    } else {
                        digitalWrite(Stepper_DIR, !stepperDIRvalToCloseBrake);
                    }

                    if (data[0] < stepperENminPotValue) {
                        digitalWrite(Stepper_EN, !stepperValToEnable);
                    } else {
                        digitalWrite(Stepper_EN, stepperValToEnable);
                    }

                    int speed = data[0] * stepperPotValueMultiplier;
                    tone(Stepper_CLK, speed);
                    // Serial.println(speed);

                    if (data[1] == 1) {
                        digitalWrite(generator_drive_switch, generatorDriveSwitchValToActivate);
                    } else {
                        digitalWrite(generator_drive_switch, !generatorDriveSwitchValToActivate);
                    }

                    if (data[2] == 1 && !(digitalRead(brake_limit_switch) == HIGH && data[3] == 0)) {
                        digitalWrite(brake_switch, brakeSwitchValToActivate);
                    } else {
                        digitalWrite(brake_switch, !brakeSwitchValToActivate);
                    }
                }
        }
        
    }

    // ====================================
    //      Control brake limit switch
    // ====================================
    if (digitalRead(brake_limit_switch) == HIGH && bitRead(PORTD, brake_switch) == brakeSwitchValToActivate && bitRead(PORTD, Stepper_DIR) == !stepperDIRvalToCloseBrake) {
        digitalWrite(brake_switch, LOW);
    }

    // ======================
    //      Process data
    // ======================
    curTime = millis();
    if ((curTime - preTime) >= tickLength) {
        // ---------------------------------
        //      Reset counter variables
        // ---------------------------------
        preTime = curTime;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            current_manometer_count = count_manometer;
            current_turbine_count = count_turbine;
            count_manometer = 0;
            count_turbine = 0;
        }
       
        // -------------------------------------
        //      Measure voltage and current
        // -------------------------------------
        turbine_voltage = analogRead(voltage_sense_turbine) * 0.0311 - 0.0309;
        turbine_current = analogRead(current_sense_turbine) * 0.00417 - 0.14326;
        battery_voltage = analogRead(voltage_sense_battery) * 0.0315 - 0.0586;
        output_current = analogRead(current_sense_output) * 0.00460 - 0.16617;
        turbine_voltage = abs(turbine_voltage);
        turbine_current = abs(turbine_current);
        battery_voltage = abs(battery_voltage);
        output_current  = abs(output_current );

        // ---------------------------------------------------------
        //      Calc wind speed, turbineRPM and generated power
        // ---------------------------------------------------------
        if (current_manometer_count != 0) {
            WSpeed = (current_manometer_count * 1000. / tickLength * 0.0835 + 1.0722);
        } else {
            WSpeed = 0;
        }

        turbineRPM = (int)(current_turbine_count * 60. / turbineEncoderTheeth * 1000 / tickLength);

        if (turbine_status == 1) {
            generatedPower = turbine_voltage * turbine_current;
        } else {
            generatedPower = 0;
        }

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

            if (battery_current_vs_target < bat_current_threshold) {
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
        // lcd.print(addTrailingSpaces("RPM:" + String(current_turbine_count * 3 / 2 * 1000 / tickLength), 8));

        // String bar = "";
        // for (int i = 0; i < floor(WSpeed * 16 / WindSpeedToFillBar ); i++) {
        //   bar += char(255);
        // }
        // lcd.setCursor(0,1);
        // lcd.print(addTrailingSpaces(bar));

        // lcd.setCursor(0,0);
        // lcd.print(addTrailingSpaces("U1:" + String(turbine_voltage), 9));
        // lcd.setCursor(9,0);
        // lcd.print(addTrailingSpaces("I1:" + String(turbine_current), 7));
        // lcd.setCursor(0,1);
        // lcd.print(addTrailingSpaces("U2:" + String(battery_voltage), 9));
        // lcd.setCursor(9,1);
        // lcd.print(addTrailingSpaces("I2:" + String(output_current), 7));

        // --------------------------------------
        //      Store a line in the log file
        // --------------------------------------
        myFile = SD.open(SDcardFileName, FILE_WRITE);
        if (myFile) {
            myFile.println(
                String(now()) + ";" +
                String(turbine_status) + ";" +
                String(brakeStepperStatus) + ";" +
                String(turbine_voltage) + ";" +
                String(turbine_current) + ";" +
                String(battery_voltage) + ";" +
                String(output_current ) + ";" +
                String(WSpeed) + ";" +
                String(turbineRPM) + ";" +
                String(generatedPower) + ";" +
                String(battery_SOC) + ";"
            );
        } else {
            Serial.print("error opening ");
            Serial.println(SDcardFileName);
        }
        myFile.close();

        // -------------------------------------------
        //      Send RF data to remote controller
        // -------------------------------------------
        String message = String(turbine_voltage) + ";" +
            String(turbine_current) + ";" + String(battery_voltage) + ";" + String(output_current) + ";" +
            String(WSpeed) + ";" + String(turbineRPM) + ";0";
        char message_out[64];
        message.toCharArray(message_out, 64);
        int stringlength = strlen(message_out);
        connection.receiverID = 108;
        uint8_t prevSize = 0;
        while (Serial3.available() != prevSize) {
            prevSize == Serial3.available();
            delayMicroseconds(serialWaitTime);
        }
        IPControl_Write(&connection, message_out, stream, stringlength);
    }
}

// -----------------------
//      Brake control
// -----------------------
uint8_t openBrake(uint32_t startTime) {
    if (digitalRead(brake_limit_switch) == LOW) {
        digitalWrite(brake_switch, brakeSwitchValToActivate);
        digitalWrite(Stepper_DIR, !stepperDIRvalToCloseBrake);
        digitalWrite(Stepper_EN, stepperValToEnable);
        freqSweep(sweepStartSpeed, brakeOpeningSpeed, startTime, brakeSpeedSweepTime);
        return 2;
    }
    disableStepper();
    digitalWrite(brake_switch, !brakeSwitchValToActivate);
    return 0;
}

uint8_t closeBrake(uint32_t startTime) {
    digitalWrite(brake_switch, brakeSwitchValToActivate);
    digitalWrite(Stepper_DIR, stepperDIRvalToCloseBrake);
    digitalWrite(Stepper_EN, stepperValToEnable);
    freqSweep(sweepStartSpeed, brakeClosingSpeed, startTime, brakeSpeedSweepTime);

    uint16_t usedPulses = (map( clip(millis() - startTime, 0, brakeSpeedSweepTime), 
        0, brakeSpeedSweepTime, sweepStartSpeed, brakeClosingSpeed ) + sweepStartSpeed) / 2
        * clip(millis() - startTime, 0, brakeSpeedSweepTime) / 1000
        + brakeClosingSpeed * (clip(millis() - startTime, brakeSpeedSweepTime, UINT32_MAX) 
        - brakeSpeedSweepTime) / 1000;
    
    if (usedPulses > pulsesToCloseBrake) {
        disableStepper();
        return 0;
    }
    return 1;
}

uint8_t stopTurbine(uint32_t startTime) {
    digitalWrite(brake_switch, brakeSwitchValToActivate);
    if (brakeStepperStatus == 1) {
        brakeStepperStatus = closeBrake(startTime);
    }
    if (turbineRPM == 0) return 0;
    return 1;
}

// -------------------------------
//      Turbine drive control
// -------------------------------
uint8_t startTurbine(uint32_t startTime) {
    digitalWrite(generator_drive_switch, generatorDriveSwitchValToActivate);
    digitalWrite(Stepper_DIR, stepperDIRvalToBoostTurbine);
    digitalWrite(Stepper_EN, stepperValToEnable);
    freqSweep(sweepStartSpeed, turbineBoostSpeed, startTime, turbineBoostSweepTime);
    if (millis() - startTime > turbineBoostSweepTime) {
        disableStepper();
        return 0;
    } 
    return 1;
}

// --------------------------------
//      Disable stepper driver
// --------------------------------
void disableStepper() {
    digitalWrite(Stepper_EN, !stepperValToEnable);
    noTone(Stepper_CLK);
}

// --------------------------
//      Helper functions
// --------------------------
void freqSweep(uint16_t startFreq, uint16_t endFreq, uint32_t startTime, uint16_t duration) {
    double fraction = clip((millis() - startTime) * 1. / duration, 0, 1);
    uint16_t currentFreq = (endFreq - startFreq) * fraction + startFreq;
    tone(Stepper_CLK, currentFreq);
}

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

unsigned long processSyncMessage() {
    unsigned long pctime = 0L;
    const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

    if(Serial.find(TIME_HEADER)) {
        pctime = Serial.parseInt();
        return pctime;
        if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
            pctime = 0L; // return 0 to indicate that the time is not valid
        }
    }
    return pctime;
}

// time_t requestSync() {
//     Serial.write(TIME_REQUEST);
//     return 0; // the time will be sent later in response to serial mesg
// }

void manometerInterrupt() {
    count_manometer += 1;
}

void turbineInterrupt() {
    count_turbine += 1;
}

double mosfetVoltageDrop(double current) {
    return current * mosfet_resistance;
}

double batteryChargeVoltageDrop(double currentIn, double currentOut) {
    double batCurrent = currentIn - currentOut;

    if (batCurrent > bat_current_correction_threshold) {
        return bat_charge_voltage_correction;
    } else {
        return 0;
    }
}

double batteryDischargeVoltageDrop(double currentIn, double currentOut) {
    double batCurrent = currentIn - currentOut;

    if (batCurrent < -bat_current_correction_threshold) {
        return -bat_discharge_voltage_correction;
    } else {
        return 0;
    }
}

double calcBatterySOC(double batVoltage, double currentIn, double currentOut) {
    double realVoltage = batVoltage + batteryChargeVoltageDrop(currentIn, currentOut) + batteryDischargeVoltageDrop(currentIn, currentOut);
    double range = battery_max_idle_voltage - battery_min_idle_voltage;

    return (realVoltage - battery_min_idle_voltage) / range;
}

void UART_Send(char* data, uint8_t len) {
    Serial3.write(data, len);  
}

void UART_receive() {
    char incomingByte;
    while (Serial3.available() > 0) {
        // read the incoming byte:
        incomingByte = Serial3.read();
        IP_BufferDataByte(incomingByte);
    }
}