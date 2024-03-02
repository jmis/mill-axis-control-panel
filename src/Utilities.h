#ifndef UTILITIES_H
#define UTILITIES_H

#include <FastAccelStepper.h>

using namespace std;

void configureButton(int activatePin, int groundPin, const function<void()>& onPress, const function<void()>& onRelease);
void configureEncoder(int pinA, int pinB, int pinClick, int powerPin, int groundPin, const function<void(int)>& onMove);
void configurePotentiometer(int feedRatePin, int powerPin, int groundPin, const function<void(int)>& percentChanged);
FastAccelStepper* configureStepper(int stepPin, int stepGround, int dirPin, int dirGround);

#endif