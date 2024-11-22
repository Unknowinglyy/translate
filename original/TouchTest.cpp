#include <TouchScreen.h>

// Initialize the touchscreen with the given pins
TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0);

void setup() {
  // Start the serial communication
  Serial.begin(9600);
}

void loop() {

  if(Serial.available() > 0){
    char c = Serial.read();

    if(c == 'S'){
      // Get the touch point
      TSPoint p = ts.getPoint();

      Serial.write(p.x);
      Serial.write(",");
      Serial.write(p.y);
      Serial.write(",");
      Serial.write(p.z);
      Serial.print("\n");
    }
  }

  // Small delay to avoid flooding the serial monitor
  delay(1000);
}