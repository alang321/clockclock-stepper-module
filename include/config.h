#pragma once

#define DEBUG false

// start to be configured
#define BROKEN_PCB false // pcb 16
#define I2C_ADDRESS 12 // [12;17], 12 is at top left from clockface, row first
// end

#define NUM_STEPPERS 8
#define NUM_STEPPERS_H 4
#define NUM_STEPPERS_M 4
#define STEPPER_DEFAULT_SPEED 700
#define STEPPER_DEFAULT_ACCEL 300
#define STEPS_PER_REVOLUTION 4320 //360*12
#define STEPPER_DEFAULT_POS_FRACTION 0.5 //at 6o'clock position

#define I2C_SDA_PIN 15
#define I2C_SCL_PIN 16

#if BROKEN_PCB
  #define ENABLE_PIN 18 // broken pcb
#else
  #define ENABLE_PIN 17
#endif

#define MAX_SPEED 800
#define MIN_SPEED 5
#define MAX_ACCEL 500
#define MIN_ACCEL 5

#define CMD_QUEUE_LENGTH 300