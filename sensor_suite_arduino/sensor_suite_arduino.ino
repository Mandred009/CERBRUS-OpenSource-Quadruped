// Arduino mega script for sensor suite arduino
//Additional pins active 30 to 45, A1 to A15

#include "imu.h"
#include "voltage.h"
#include "additional_pins.h"
#include "pitches.h"
#include <SoftwareSerial.h>

SoftwareSerial HC12(10, 11);

float imu_vals[3];

const byte numChars = 128; 
char receivedChars[numChars]="null";  
boolean newData = false;


char receivedCmd;

const int no_of_controller_vals=17; // Change this if you want to input more values from the controller

String split_vals[no_of_controller_vals];

int start_flag=0;

int blue_flag=0;
int green_flag=0;
int laser_flag=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  HC12.begin(9600);
  
  pinMode(LED_BUILTIN,OUTPUT);
  init_imu();
  Serial.setTimeout(500);
  
  init_additional(); // Initialize additonal pins
 
  delay(100);
  

}

void loop() {
  // put your main code here, to run repeatedly:

  recvWithStartEndMarkers();
  Serial.print(receivedChars);
  if(start_flag==0){
    extractLastValue(receivedChars);
  }
  splitString(receivedChars,'|',split_vals,no_of_controller_vals);
  if(split_vals[6]=="0"){
    blue_flag=1-blue_flag;
  }
  if(split_vals[7]=="0"){
    green_flag=1-green_flag;
  }
  if(split_vals[8]=="0"){
    laser_flag=1-laser_flag;
  }
  
  newData=false;
  
  
  get_reading(imu_vals);
  
  Serial.print(",");
  Serial.print(imu_vals[0]);
  Serial.print(",");
  Serial.print(imu_vals[1]);
  Serial.print(",");
  Serial.print(imu_vals[2]);
  Serial.print(",");
  Serial.println(get_voltage(),2);

  delay(80);

  if(blue_flag==1){
    digitalWrite(BLUE_LIGHT,HIGH);
  }
  else{
    digitalWrite(BLUE_LIGHT,LOW);
  }
  
  if(green_flag==1){
    digitalWrite(GREEN_LIGHT,HIGH);
  }
  else{
    digitalWrite(GREEN_LIGHT,LOW);
  }
  
  if(laser_flag==1){
    digitalWrite(LASER,HIGH);
  }
  else{
    digitalWrite(LASER,LOW);
  }

}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (HC12.available() > 0 && newData == false) {
        rc = HC12.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void extractLastValue(String inputData) {
  // Remove the square brackets from the input string
  inputData.remove(0, 1);  // Remove the first character '['
  inputData.remove(inputData.length() - 1, 1);  // Remove the last character ']'
  
  // Find the position of the last '|'
  int lastPipeIndex = inputData.lastIndexOf('|');
  
  // Extract the last value (after the last '|')
  String lastValue = inputData.substring(lastPipeIndex + 1);
  
  lastValue.trim();

  if(lastValue=="on" && start_flag==0)
  {
    play_intro();
    start_flag=1;
  }
}

void splitString(String data, char delimiter, String result[], int maxParts) {
  int count = 0;
  int start = 0;
  int end = data.indexOf(delimiter);

  while (end != -1 && count < maxParts - 1) {
    result[count++] = data.substring(start, end);
    start = end + 1;
    end = data.indexOf(delimiter, start);
  }

  // Add the last part
  if (count < maxParts) {
    result[count++] = data.substring(start);
  }

}
