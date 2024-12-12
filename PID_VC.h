#ifndef PID_VC_H
#define PID_VC_H
#include "pin.h"

// KONSTANTA PID

double KpVC = 0.6;
double KiVC = 20;
double KdVC = 0.05;

const int maxChangePerCycle = 10.0;
static double integralVC;
static double lastErrorVC = 0;

unsigned long lcon;
double dtcon;

void inisialisasiPID() {
  integralVC = 0;
  lastError = 0;
}


float calculatePID(float setpoint, float measuredValue){
  double error = setpoint - measuredValue;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;

  if (deltaTime > 0) {
    integral += error * deltaTime;
    integral = constrain(integral, 0, 100);

    double derivative = (error - lastError) / deltaTime;
    double rawOutput = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;
    lastTime = currentTime;

    // Derating: Batasi perubahan output
    static double previousOutput = 0;
    int change = rawOutput - previousOutput;

    // Jika perubahan melebihi batas maksimum, kurangi ke nilai maksimum
    if (abs(change) > maxChangePerCycle) {
      rawOutput = previousOutput + (change > 0 ? maxChangePerCycle : -maxChangePerCycle);
    }

    previousOutput = rawOutput;  // Simpan output untuk iterasi berikutnya
    return rawOutput;
  }
  return 0;
}


#endif
