// Arduino nano controller telemetry script
// Channel 002 and 9600 Baud for HC-12

#include <SoftwareSerial.h>

SoftwareSerial HC12(2, 3);


const byte numChars = 128; 
char receivedChars[numChars]="null";  
boolean newData = false;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  HC12.begin(9600);

  delay(100);
  

}

void loop() {
  // put your main code here, to run repeatedly:

  recvWithStartEndMarkers();
  Serial.println(receivedChars);
  newData=false;

  delay(100);



}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '(';
    char endMarker = ')';
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
