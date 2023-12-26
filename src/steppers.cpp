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

AccelStepper *steppers[] = {&x1m, &x2m, &x3m, &x4m, &x1h, &x2h, &x3h, &x4h};
AccelStepper *h_steppers[] = {&x1h, &x2h, &x3h, &x4h};
AccelStepper *m_steppers[] = {&x1m, &x2m, &x3m, &x4m};

// Initialize steppers in your CPP file
void initializeSteppers() {
    for (int i = 0; i < NUM_STEPPERS_H; i++)
    {
        h_steppers[i]->setPinModesDriver();
        h_steppers[i]->setMaxSpeed(STEPPER_DEFAULT_SPEED);
        h_steppers[i]->setAcceleration(STEPPER_DEFAULT_ACCEL);
        h_steppers[i]->setCurrentPosition((int)(STEPS_PER_REVOLUTION * STEPPER_DEFAULT_POS_FRACTION));
        h_steppers[i]->moveToSingleRevolution(0, -1);
    }

    for (int i = 0; i < NUM_STEPPERS_M; i++)
    {
        m_steppers[i]->setPinModesDriver();
        m_steppers[i]->setMaxSpeed(STEPPER_DEFAULT_SPEED);
        m_steppers[i]->setAcceleration(STEPPER_DEFAULT_ACCEL);
        m_steppers[i]->setCurrentPosition((int)(STEPS_PER_REVOLUTION * STEPPER_DEFAULT_POS_FRACTION));
        m_steppers[i]->moveToSingleRevolution(0, 1);
    }
}