// Script for processing voltage sensor data

#include "voltage.h"

float adc_voltage=0.0;
float in_voltage=0.0;

float R1=30000.0;
float R2=7500.0;

float ref_voltage=5.0;

int adc_value=0;

float get_voltage(){

  adc_value=analogRead(ANALOG_VOLT);

  adc_voltage=(adc_value * ref_voltage) /1024.0;

  in_voltage=adc_voltage*(R1+R2)/R2;

  return in_voltage+0.5;

  
}
