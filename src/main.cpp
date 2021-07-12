#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>

#define NUM_STEPPERS 8
#define NUM_STEPPERS_H 4
#define NUM_STEPPERS_M 4
#define NUM_CLOCKS 4
#define MOTOR_SPEED 1200

void i2cReceive(int numBytesReceived);

const uint8_t address = 10; //10-16, 10 is at top left from clockface, row first
const int sda = 15;
const int scl = 16;

const int fullRev = 360*12;

struct move_data {
  uint16_t speed; //2 bytes
  uint16_t position; //2bytes
  int8_t dir; // -1, 0, 1 1byte
  uint8_t sub_id; //1byte
  bool is_minute_pointer; //1byte    =   7bytes/32bytes max

  move_data(uint16_t position, int8_t dir, uint8_t sub_id, bool is_minute_pointer, uint16_t apeed = MOTOR_SPEED)      
  {      
    this->speed = speed;  
    this->position = position;     
    this->dir = dir;      
    this->sub_id  = sub_id;   
    this->is_minute_pointer = is_minute_pointer;   //true minute hand - false hour hand
  }     
};

move_data move_i2c;

//step pin, dir pin, hall pin, hall offset, number of steps per revolution
AccelStepper x1m(26, 27, fullRev);
AccelStepper x1h(22, 25, fullRev);
AccelStepper x2m(4,  5,  fullRev);
AccelStepper x2h(2,  3,  fullRev);
AccelStepper x3m(10, 11, fullRev);
AccelStepper x3h(12, 13, fullRev);
AccelStepper x4m(8,  9,  fullRev);
AccelStepper x4h(7,  6,  fullRev);

AccelStepper *allSteppers[] = {&x1m, &x1h, &x2m, &x2h, &x3m, &x3h, &x4m, &x4h};

AccelStepper *hSteppers[] = {&x1h, &x2h, &x3h, &x4h};
AccelStepper *mSteppers[] = {&x1m, &x2m, &x3m, &x4m};

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);

  for(int i = 0; i < NUM_STEPPERS; i++){
    allSteppers[i]->setPinModesDriver();
    allSteppers[i]->setMaxSpeed(1100);
    allSteppers[i]->setAcceleration(1100);
  }
  
  //Wire.begin(sda, scl, address); //sda, scl, adress
  Wire.setSCL(scl);
  Wire.setSDA(sda);
  Wire.begin(address); 
  Wire.onReceive(i2cReceive);
}

void i2cReceive(int numBytesReceived) {
  Wire.readBytes( (byte*) &move_i2c, numBytesReceived);
  
  AccelStepper *stepper;

  //minute or hour pointer
  if(move_i2c.is_minute_pointer){
    stepper = mSteppers[move_i2c.sub_id];
  }
  else{
    stepper = hSteppers[move_i2c.sub_id];
  }

  stepper->setMaxSpeed(move_i2c.speed);

  //movement direction, shortest, left or right
  if(move_i2c.dir == 0){
    stepper->moveToShortestPath(move_i2c.position);
  }else{
    stepper->moveToSingleRevolutionDir(move_i2c.position, move_i2c.dir);
  }
}

// the loop function runs over and over again forever
void loop() {
  for(int i = 0; i < NUM_STEPPERS; i++){
	  allSteppers[i]->run();
  }
}




  
