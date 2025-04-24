// Script for Arduino Mega in the controller

#include <SoftwareSerial.h>

SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin

#define JOY_X_R A1
#define JOY_Y_R A2
#define JOY_BUTTON_R 2

#define JOY_X_L A3
#define JOY_Y_L A4
#define JOY_BUTTON_L 3

#define A_BUTTON 22
#define B_BUTTON 24
#define C_BUTTON 26
#define D_BUTTON 28

#define BUTTON_1 52
#define BUTTON_2 50
#define BUTTON_3 48
#define BUTTON_4 46

#define MANDRED_LED 8

#define SWITCH 13

void setup() {
  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600);               // Serial port to HC12
  pinMode(JOY_BUTTON_R,INPUT);
  digitalWrite(JOY_BUTTON_R,HIGH);

  pinMode(JOY_BUTTON_L,INPUT);
  digitalWrite(JOY_BUTTON_L,HIGH);

  pinMode(MANDRED_LED,OUTPUT);
  digitalWrite(MANDRED_LED,255);

  pinMode(A_BUTTON,INPUT);
  digitalWrite(A_BUTTON,HIGH);
  pinMode(B_BUTTON,INPUT);
  digitalWrite(B_BUTTON,HIGH);
  pinMode(C_BUTTON,INPUT);
  digitalWrite(C_BUTTON,HIGH);
  pinMode(D_BUTTON,INPUT);
  digitalWrite(D_BUTTON,HIGH);
  pinMode(BUTTON_1,INPUT);
  digitalWrite(BUTTON_1,HIGH);
  pinMode(BUTTON_2,INPUT);
  digitalWrite(BUTTON_2,HIGH);
  pinMode(BUTTON_3,INPUT);
  digitalWrite(BUTTON_3,HIGH);
  pinMode(BUTTON_4,INPUT);
  digitalWrite(BUTTON_4,HIGH);
  
  pinMode(SWITCH,INPUT);
  digitalWrite(SWITCH,HIGH);
  
}

void loop() {
  int xr= analogRead(JOY_X_R);
  int yr= analogRead(JOY_Y_R);
  int buttonr= digitalRead(JOY_BUTTON_R);

  int xl= analogRead(JOY_X_L);
  int yl= analogRead(JOY_Y_L);
  int buttonl= digitalRead(JOY_BUTTON_L);

  int A_button=digitalRead(A_BUTTON);
  int B_button=digitalRead(B_BUTTON);
  int C_button=digitalRead(C_BUTTON);
  int D_button=digitalRead(D_BUTTON);

  int button_1=digitalRead(BUTTON_1);
  int button_2=digitalRead(BUTTON_2);
  int button_3=digitalRead(BUTTON_3);
  int button_4=digitalRead(BUTTON_4);

  int switch_state=digitalRead(SWITCH);

  HC12.print('<');  // Start of data packet
  HC12.print('[');
  HC12.print(xr);    // Send X value
  HC12.print('|');  // Separator
  HC12.print(yr);    // Send Y value
  HC12.print('|');  // Separator
  HC12.print(buttonr); // Send button value
  HC12.print('|');
  HC12.print(xl);    // Send X value
  HC12.print('|');  // Separator
  HC12.print(yl);    // Send Y value
  HC12.print('|');  // Separator
  HC12.print(buttonl); // Send button value
  HC12.print('|');
  HC12.print(A_button); 
  HC12.print('|');
  HC12.print(B_button); 
  HC12.print('|');
  HC12.print(C_button); 
  HC12.print('|');
  HC12.print(D_button); 
  HC12.print('|');
  HC12.print(button_1); 
  HC12.print('|');
  HC12.print(button_2); 
  HC12.print('|');
  HC12.print(button_3); 
  HC12.print('|');
  HC12.print(button_4); 
  HC12.print('|');
  if(switch_state==0){
    HC12.print("on");
  }
  else{
    HC12.print("off");
  }
  
  HC12.print(']');
  HC12.print('>');  // End of data packet
  HC12.println();   // New line (to signify end of message)
  delay(50);

 
}
