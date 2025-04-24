#ifndef VOLT_H
#define VOLT_H

#include "Arduino.h"

#define ANALOG_VOLT A1

extern float adc_voltage;
extern float in_voltage;

extern float R1;
extern float R2;

extern float ref_voltage;

extern int adc_value;

float get_voltage();

#endif
