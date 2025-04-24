// Arduino nano onboard telemetry script
// Channel 002 and 9600 Baud for HC-12

#include <Wire.h>
#include <SPI.h>

#include <SoftwareSerial.h>

SoftwareSerial HC12(3, 2);


String receivedData="";

const byte numChars=128;
char receivedChars[numChars]="null";
boolean newData=false;

#define MAX_VALUES 8  // Maximum number of values you expect

String extractedValues[MAX_VALUES]= {""};  // Array to hold the extracted values
int extractedCount = 0;  // Keeps track of the number of extracted values


void setup () 
{
  
  Serial.begin(9600);
  HC12.begin(9600);
  while(true){
    String cmd_="null";
    if(Serial.available()>0){
      cmd_=Serial.readStringUntil('\n');
      cmd_.trim();
    }
    if(cmd_=="start"){
      break;
    }
  }

    
}  // end of setup

void loop () 
{

  recvWithStartEndMarkers();
  newData=false;

  if(countOccurrences(receivedChars,'(')==0 && countOccurrences(receivedChars,')')==0 && countOccurrences(receivedChars,',')==MAX_VALUES-1)
  {
    send_data();

  
    extractValues(receivedChars);  // Extract values from the string
    
      
    extractedCount=0;
  }
  
  

} 


void send_data(){
  HC12.print('(');
  for(auto i:receivedChars){
    HC12.print(i);
  }
  HC12.print(')');
  HC12.println();
  delay(100);
  
}

void extractValues(const String& input) {
  String temp = input;
  temp.trim();  // Remove any leading or trailing spaces
  
  // Ensure the input starts with '(' and ends with ')'
    // Remove the parentheses
    temp.remove(0, 1);  // Remove the first '('
    temp.remove(temp.length() - 1, 1);  // Remove the last ')'
    
    int startIndex = 0;
    int endIndex;
    extractedCount = 0;  // Reset extracted count

    // Extract values from the string, using commas as delimiters
    while ((endIndex = temp.indexOf(',', startIndex)) != -1 && extractedCount < MAX_VALUES) {
      extractedValues[extractedCount++] = temp.substring(startIndex, endIndex);
      startIndex = endIndex + 1;
    }

    // Extract the last value if there's no comma at the end
    if (startIndex < temp.length() && extractedCount < MAX_VALUES) {
      extractedValues[extractedCount++] = temp.substring(startIndex);
    }

}

// Function to check te incoming serial stream and its enclosing bytes that is '()' in this case
void recvWithStartEndMarkers(){
  static boolean recvInProgress=false;
  static byte ndx=0;
  char startMarker='(';
  char endMarker=')';
  char rc;

  while(Serial.available()>0 && newData==false){
    rc=Serial.read();

    if(recvInProgress==true){
      if(rc!=endMarker){
        receivedChars[ndx]=rc;
        ndx++;
        if(ndx>=numChars){
          ndx=numChars-1;
        }
      }
      else{
        receivedChars[ndx]='\0';
        recvInProgress=false;
        ndx=0;
        newData=true;
      }
    }
    else if(rc==startMarker){
      recvInProgress=true;
    }
  }
}

// Function to count character occurances in a string. For data check
int countOccurrences(const String& input, char target) {
    int count = 0;
    for (int i = 0; i < input.length(); i++) {
        if (input[i] == target) {
            count++;
        }
    }
    return count;
}
