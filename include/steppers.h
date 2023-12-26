#pragma once

#include <Arduino.h>
#include <AccelStepper.h>

extern AccelStepper *steppers[8];
extern AccelStepper *h_steppers[4];
extern AccelStepper *m_steppers[4];

void initializeSteppers();