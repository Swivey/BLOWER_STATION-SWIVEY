#ifndef PIN_H
#define PIN_H

const int escPin = 16;
const int Honeywell = A1;  // Pin sensor tekanan

const int minThrottle = 1130;
const int maxThrottle = 2000;

double raw, mbar, voltage, cmh2o;

char data;

int VoltageESC = 0;
int filteredPressure = 0;

//pin exvalve
int in3 = 19;
int in4 = 17;
int enB = 18;

//pin VPWS
const int enA = 21;
int in1 = 22;
int in2 = 20;

//relay
const int relayPin = 11 ;

#endif
