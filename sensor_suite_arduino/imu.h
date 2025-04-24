#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

extern uint16_t BNO055_SAMPLERATE_DELAY_MS;

extern Adafruit_BNO055 bno;


void init_imu();

void get_reading(float* vals);

#endif
