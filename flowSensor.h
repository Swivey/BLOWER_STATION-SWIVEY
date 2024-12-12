#include <SensirionI2cSfmSf06.h>
SensirionI2cSfmSf06 sensor;


float aFlow1 = 0.0;
float aTemperature1 = 0.0;
uint16_t aStatusWord1 = 0u;


void sensorflowinit(){
  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.begin();

   sensor.begin(Wire, SFM3019_I2C_ADDR_2E); // Sensor 1
   sensor.stopContinuousMeasurement();
   delay(1000);
   sensor.startO2ContinuousMeasurement();

}

void sensorflow() {
  sensor.readMeasurementData(aFlow1, aTemperature1, aStatusWord1);

}
