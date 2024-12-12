#ifndef EXVALVE_H
#define EXVALVE_H

#include "pressureSensor.h"

double measuredPressure = readPressure();
double targetPressure;

double integralV = 0, prevV = 0, outputV = 0;
double errorV;

//double KpV = 3.2, KiV = 4.4, KdV = 0.22; 
double KpV = 0.5, KiV = 0.5, KdV = 0.2; 

double dt,last_time_v;
double lastInput;
int SampleTime = 20; //0.1 sec


double PIDexvalve(double error) {
    //if(inAuto){
      unsigned long now = millis();
      dt = (now - last_time_v);
      if (dt >= SampleTime){

        double proportional = error;

        integralV += error * dt / 1000.0;
        if ((outputV >= 255 && error > 0) || (outputV <= 0 && error < 0)) {
              integralV -= error * dt / 1000.0;
          }

        double derivative = (measuredPressure - lastInput) / (dt / 1000.0);

        outputV = (KpV * proportional) + (KiV * integralV) - (KdV * derivative);
        //outputV = (KiV * integralV);
        outputV = constrain(outputV, 0, 255);

        prevV = error;
        lastInput = measuredPressure;
        last_time_v = now;

      }
    
    return outputV;
}

void runExvalve() {  
    // Only run when in expiratory phase;
    errorV = targetPressure - measuredPressure;
    outputV = PIDexvalve(errorV);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, outputV);

}

#endif
