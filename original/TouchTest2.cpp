#include <TouchScreen.h>

// Initialize the touchscreen with the given pins
TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0);

void setup() {
  // Start the serial communication
  Serial.begin(9600);
}

void loop() {
  // Get the touch point
  TSPoint p = ts.getPoint();

  // Check if there is a valid touch
  if (p.z == 0) {
    Serial.print(p.x);
    Serial.print(",");
    Serial.print(p.y);
    Serial.print(",");
    Serial.print(p.z);
    Serial.println();
  }

  // Small delay to avoid flooding the serial monitor
  delay(100);
}
