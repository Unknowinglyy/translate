#include <TouchScreen.h>

// Initialize the touchscreen with the given pins
TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0);

void setup() {
  // Start the serial communication
  Serial.begin(9600);
}

void loop() {
  TSPoint p = ts.getPoint();
  if (p.z > 0) {
    Serial.print((int)p.x);  // Cast to integer for clean output
    Serial.print(",");
    Serial.print((int)p.y);
    Serial.print(",");
    Serial.println((int)p.z);
  }
  delay(100);
}
