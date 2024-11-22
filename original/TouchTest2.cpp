#include <TouchScreen.h>

// Initialize the touchscreen with the given pins
TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0);

void setup() {
  // Start the serial communication
  Serial.begin(9600);
}

void loop() {
  TSPoint p = ts.getPoint();
  Serial.println("100,200,300");  // Simple test data
  delay(100);
}
