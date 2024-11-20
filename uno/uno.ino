void setup() {
  // Initialize pin A0 as output
  pinMode(A0, OUTPUT);
  
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming byte
    char command = Serial.read();

    // If the command is '1', turn on the LED
    if (command == '1') {
      digitalWrite(A0, HIGH);
    }
    // If the command is '0', turn off the LED
    else if (command == '0') {
      digitalWrite(A0, LOW);
    }
  }
}