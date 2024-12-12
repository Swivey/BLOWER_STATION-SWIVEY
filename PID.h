#ifndef PID_H
#define PID_H
#include "pin.h"

// Parameter PID
float Tf = 0.0039;       // Filter time constant
float Kp = 3.8450;       // Gain proportional
float Ki = 15.6710;      // Gain integral
float Kd = 0.0013;       // Gain derivative

const int tolerance = 1.0;       // Toleransi error

// Variabel global PID
static double integral = 0;
static double lastError = 0;
static double filteredDerivative = 0;
unsigned long lastTime = 0;

void initializePID() {
  integral = 0;
  lastError = 0;
  filteredDerivative = 0;
}

float computePID(float setpoint, float measuredValue) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  // Ubah ms ke detik
  lastTime = currentTime;

  if (deltaTime > 0) {
    // Hitung error
    double error = setpoint - measuredValue;

    // Terapkan toleransi (zona mati untuk error kecil)

    // Hitung integral (dengan batasan untuk mencegah akumulasi berlebihan)
    integral += error * deltaTime;
    integral = constrain(integral, ((305) / Ki) * -1, ((305) / Ki));

    // Hitung derivatif dengan filter
    double derivative = (error - lastError) / deltaTime;
    filteredDerivative = (derivative + Tf * filteredDerivative) / (1 + Tf / deltaTime);

    // Hitung output PID
    double rawOutput = (Kp * error) + (Ki * integral) + (Kd * filteredDerivative);

    lastError = error;

    return rawOutput;
  }

  return 0.0;
}


#endif
