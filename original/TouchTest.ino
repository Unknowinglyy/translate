#include <TouchScreen.h>

// Initialize the touchscreen with the given pins
TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0);

void setup() {
  // Start the serial communication
  Serial.begin(9600);
}

void loop() {
  TSPoint p = ts.getPoint();

  String data = String(p.x) + "," + String(p.y) + "," + String(p.z) + "\n";

  Serial.print(data);

  // Small delay to avoid flooding the serial monitor
  delay(1000);
}