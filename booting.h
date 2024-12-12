#define MIN_PULSE_LENGTH 1000  // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000  // Maximum pulse length in µs

Servo esc;

void displayInstructions() {
  Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
  Serial.println("\t1 : Calibrate");
  Serial.println("\t2 : Run Blower Station");
  Serial.println("\t3 : Shutdown System");
}

void bootsetup() {
  displayInstructions();
  esc.attach(16, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
}
