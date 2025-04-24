#include "imu.h"

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);


void init_imu(){
if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(100);
}


void get_reading(float* vals){
  
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  vals[0]=orientationData.orientation.x;
  vals[1]=orientationData.orientation.y;
  vals[2]=orientationData.orientation.z;
  
  
}
