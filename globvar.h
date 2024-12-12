#ifndef GLOBVAR_H
#define GLOBVAR_H

#include <Arduino.h>

// Extern variable declarations
extern int mappressure, mappressuregrafik, mapflow, lastflow;
extern int mapvolume, mapI, mapE, mapPIP, mapPEEP, mapVT, mapIT, mapRR, mapIF, mapF, mapFIO2, mappressure1;
extern int activePageId;
extern bool calibrationDone, systemShutdown, RDone, SDone;
extern double PIP, PEEP, inspirationTime, expirationTime;
extern bool modePC, isInspiration;
extern int RR;
extern float inspR, expR;
extern int counter, oxygen, inspiratoryFlow, tidalVolume;
extern unsigned long currentTime;
extern unsigned long phaseStartTime

#endif // GLOBALS_H
