#include <Arduino.h>
#include <AccelStepper.h>

#define NUM_STEPPERS 8
#define NUM_STEPPERS_H 4
#define NUM_STEPPERS_M 4
#define NUM_CLOCKS 4

void i2cReceive(int numBytesReceived);

const int address = 10;

const int fullRev = 360*12;

struct move_data {
  uint16_t position;
  int8_t dir; // -1, 0, 1
  uint8_t sub_id;
  bool minute_pointer;

  move_data(uint16_t position, int8_t dir, uint8_t sub_id, bool minute_pointer)      
  {      
    this->position = position;     
    this->dir = dir;      
    this->sub_id  = sub_id;   
    this->minute_pointer = minute_pointer;   
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
  
  Wire.begin(address);
  Wire.onReceive(i2cReceive);
}

void i2cReceive(int numBytesReceived) {
  Wire.readBytes( (byte*) &move_i2c, numBytesReceived);
  
  //todo : implement dir
  //todo : implement set speed
  if(move_i2c.minute_pointer){
	mSteppers[move_i2c.sub_id]->moveToShortestPath(move_i2c.position)
  }
  else{
	hSteppers[move_i2c.sub_id]->moveToShortestPath(move_i2c.position)
  }
}

// the loop function runs over and over again forever
void loop() {
  for(int i = 0; i < NUM_STEPPERS; i++){
	allSteppers[i]->run();
  }
}




  
