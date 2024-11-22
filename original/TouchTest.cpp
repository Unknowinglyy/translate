#include <TouchScreen.h>

// Initialize the touchscreen with the given pins
TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0);

bool sendData = false;

void setup() {
  // Start the serial communication
  Serial.begin(9600);

}

void loop() {

  if(Serial.available() >0){
    char command = Serial.read();
    if(command == 'S'){
      sendData = true;
    }
    else if(command == 'E'){
      sendData = false;
    }
  }

  if(sendData){
    // Get the touch point
    TSPoint p = ts.getPoint();


    Serial.print(p.x);
    Serial.print(",");
    Serial.print(p.y);
    Serial.print(",");
    Serial.print(p.z);
    Serial.println();

    // Small delay to avoid flooding the serial monitor
    delay(1000);
  }
}