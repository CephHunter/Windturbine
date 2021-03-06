#include <arduino.h>

const uint16_t tickLength = 1000;       // Data measure/process rate in ms (Windturbine.cpp)
const uint16_t tickLength2 = 350;       // Data measure/process rate in ms (Remote.cpp)
const uint8_t WindSpeedToFillBar = 15;   // m/s
const double max_battery_charge_current = 2.1;
const double battery_float_charge_current = 0.7;
const double battery_max_idle_voltage = 13.5;
const double battery_min_idle_voltage = 11.5;

const double bat_current_threshold = 0.05;
const double bat_current_correction_threshold = 0.2;
const double bat_charge_voltage_correction = 0.7;
const double bat_discharge_voltage_correction = -0.4;

// const double mosfet_resistance = 0.2;

const uint8_t stepperDIRvalToCloseBrake = 0;
const uint8_t stepperDIRvalToBoostTurbine = 1;
const uint8_t stepperValToEnable = 0;
const uint8_t stepperENminPotValue = 2;
const uint8_t stepperPotValueMultiplier = 13;
const uint8_t generatorDriveSwitchValToActivate = 1;
const uint8_t brakeSwitchValToActivate = 1;

const uint8_t turbineEncoderTeeth = 28;    // number of teeth on the encode wheel
const uint8_t anemometerEncoderTeeth = 20;

#define SDcardFileName "log.txt"

// const uint16_t serialWaitTime = 600; // microseconds

uint8_t allowSelfStart = 0;
const uint8_t sweepStartSpeed = 200;         // pulses per second, 1600 = 1 rotation
const uint16_t brakeOpeningSpeed = 1600;
const uint16_t brakeClosingSpeed = 1600;
const uint16_t pulsesToCloseBrake = 11200;   // 1600 = 1 rotation
const uint16_t openBrakepulsesTimeout = 16000;
const uint16_t brakeSpeedSweepTime = 1000;      // Time to get from sweepStartSpeed to tartget speed in milliseconds
const uint16_t turbineBoostSpeed = 800;
const uint32_t turbineBoostSweepTime = 180000;

const double windSpeedThresholdToStartTurbine = 4;  // m/s
const uint16_t windSpeedThresholdTimeToStartTurbine = 10000; // #ms over windspeed treshold before turbine start
const double maxAllowedTurbineRPM = 150;
const double maxAllowedWinspeed = 10; // m/s

bool selfSend = true;
int autoOrManualSelect = 1;     // 1 == manual; 2 == auto