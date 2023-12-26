#include "steppers.h"
#include "AccelStepper.h"
#include "config.h"

#if BROKEN_PCB
AccelStepper x1m(17, 20, STEPS_PER_REVOLUTION);//broken maple cutoff from black rev 1 pcb
#else
AccelStepper x1m(26, 27, STEPS_PER_REVOLUTION);//normal green
#endif
AccelStepper x1h(14, 25, STEPS_PER_REVOLUTION);
AccelStepper x2m(4,  5,  STEPS_PER_REVOLUTION);
AccelStepper x2h(2,  3,  STEPS_PER_REVOLUTION);
AccelStepper x3m(10, 11, STEPS_PER_REVOLUTION);
AccelStepper x3h(12, 13, STEPS_PER_REVOLUTION);
AccelStepper x4m(8,  9,  STEPS_PER_REVOLUTION);
AccelStepper x4h(7,  6,  STEPS_PER_REVOLUTION);

AccelStepper *steppers[] = {&x1m, &x1h, &x2m, &x2h, &x3m, &x3h, &x4m, &x4h};
AccelStepper *h_steppers[] = {&x1h, &x2h, &x3h, &x4h};
AccelStepper *m_steppers[] = {&x1m, &x2m, &x3m, &x4m};

// Initialize steppers in your CPP file
void initializeSteppers() {
    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        steppers[i]->setPinModesDriver();
        steppers[i]->setMaxSpeed(STEPPER_DEFAULT_SPEED);
        steppers[i]->setAcceleration(STEPPER_DEFAULT_ACCEL);
        steppers[i]->setCurrentPosition((int)(STEPS_PER_REVOLUTION * STEPPER_DEFAULT_POS_FRACTION));
        steppers[i]->moveToSingleRevolution(0, -1);
    }
}