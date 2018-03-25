#include <arduino.h>

const uint16_t tickLength = 1000;     // Display update rate in ms
const uint8_t WindSpeedToFillBar = 15;   // m/s
const double max_battery_charge_current = 2.1;
const double battery_float_charge_current = 0.7;
const double battery_max_idle_voltage = 13.5;
const double battery_min_idle_voltage = 10.5;

const double bat_current_threshold = 0.05;
const double bat_current_correction_threshold = 0.2;
const double bat_charge_voltage_correction = 0.7;
const double bat_discharge_voltage_correction = -0.4;

const double mosfet_resistance = 0.2;

const bool StepperDIRvalForHigh = 1;
const uint8_t StepperENminPotValue = 3;
const uint8_t stepperPotValueMultiplier = 30;
const bool generatorDriveSwitchValForHigh = 1;
const bool brakeSwitchValForHigh = 1;

const uint8_t turbineEncoderTheeth = 40;

#define SDcardFileName "log.text"

const uint16_t serialWaitTime = 600; // microseconds