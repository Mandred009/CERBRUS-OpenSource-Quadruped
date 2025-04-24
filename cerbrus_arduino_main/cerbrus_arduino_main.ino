// Arduino Script for main servo controller arduino mega

#include "quad_servo.h"
#include <Servo.h>

#define CALI_BUTTON 2

int cali_flag=0; // Check if calibration button is on

float input_values[20]; //starting and ending value is -1 for checking correct transfer
byte* ddata = reinterpret_cast<byte*>(&input_values);
size_t pcDataLen= sizeof(input_values);
bool newData=false;

Servo base_servo;
Servo effector_servo;

// The mouth arm servo limits
const int effector_servo_min=50;
const int effector_servo_max=130;

const int base_servo_min=0;
const int base_servo_max=180;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  Serial1.begin(1000000);
  st.pSerial = &Serial1;
  pinMode(CALI_BUTTON,INPUT_PULLUP);
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.setTimeout(100);

  base_servo.attach(5);
  effector_servo.attach(6);
  
  delay(1000);
  
}


void loop(){
  int button=digitalRead(CALI_BUTTON);

  if(button==0){
    cali_flag=1-cali_flag;
    digitalWrite(LED_BUILTIN,cali_flag);
    delay(1000);
  }
  if(cali_flag==1){
      calibration_pos();
    }
    else{
      checkForNewData();
  if(newData==true){
    if(input_values[0]==-1.0 && input_values[19]==-1.0){
      stable_state(all_leg,4,input_values[1],input_values[2],input_values[3],input_values[4],
                  input_values[5],input_values[6],input_values[7],input_values[8],
                  input_values[9],input_values[10],input_values[11],input_values[12]
                  ,input_values[13],input_values[14],input_values[15],input_values[16]);

      move_mouth_servos(int(trunc(input_values[17])),int(trunc(input_values[18])));
    }
    
    newData=false;
  }
      
    }
  
  
}

// Function to check for new data from the serial port
void checkForNewData() {
  if (Serial.available() >= pcDataLen && newData == false) {
    for (byte n = 0; n < pcDataLen; n++) {
      ddata[n] = Serial.read();
    }
    while (Serial.available() > 0) {
      byte dumpByte = Serial.read();
    }
    newData = true;
  }
}

// Function to move the servos
void move_mouth_servos(int pot1, int pot2){

  if(pot1>=0 and pot1<=1023 and pot2>=0 and pot2<=1023){
      int base_servo_angle=map(pot1,0,1023,base_servo_min,base_servo_max);
      int effector_servo_angle=map(pot2,0,1023,effector_servo_min,effector_servo_max);
    
      base_servo.write(base_servo_angle);
      effector_servo.write(effector_servo_angle);
  }

  
  
}
