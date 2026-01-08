#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <Arduino.h>

extern bool autoPilotEnabled;

void initAutoPilot();

void runAutoPilotLogic(long distFront, long distLeft, long distRight, float roll, float pitch);

#endif