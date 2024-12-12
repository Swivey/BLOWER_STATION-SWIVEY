#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <Arduino.h>

#define NUM_SAMPLES 10

float readPressure() {
  analogReadResolution(12);

  static float readings[NUM_SAMPLES];
  static int readIndex = 0;
  static float total = 0;
  static float average = 0;

  raw = analogRead(Honeywell);
  voltage = raw / 4095.0 * 3.3;
  voltage = voltage * (5.0 / 3.3);
  mbar = ((voltage - 0.5) / 4.0) * 200.0 - 100.0;
  cmh2o = mbar * 1.01972;
  float baseline_pressure = 10;
  cmh2o = cmh2o - baseline_pressure;

  total -= readings[readIndex];
  readings[readIndex] = cmh2o;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % NUM_SAMPLES;
  average = total / NUM_SAMPLES;

  float alpha = 0.5;
  filteredPressure = (alpha * average) + ((1 - alpha) * filteredPressure);

  return filteredPressure * 0.93 + 0.83;
}
#endif
