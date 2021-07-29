#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>

#define NUM_STEPPERS 8
#define NUM_STEPPERS_H 4
#define NUM_STEPPERS_M 4
#define LED 33
#define NUM_CLOCKS 4
#define STEPPER_DEFAULT_SPEED 1100
#define STEPPER_DEFAULT_ACCEL 1100
#define STEPPER_DEFAULT_POS_FRACTION 0.5 //at 6o'clock position

void blink(int how_often);
void i2cReceive(int numBytesReceived);
void blink(int how_often, int delay_t);

const int fullRev = 360*12;

//step pin, dir pin, hall pin, hall offset, number of steps per revolution
//pcb rev 2
AccelStepper x1m(26, 27, fullRev);//normal green
//AccelStepper x1m(1, 0, fullRev);//broken maple cutoff from black rev 1 pcb
AccelStepper x1h(14, 25, fullRev);
AccelStepper x2m(4,  5,  fullRev);
AccelStepper x2h(2,  3,  fullRev);
AccelStepper x3m(10, 11, fullRev);
AccelStepper x3h(12, 13, fullRev);
AccelStepper x4m(8,  9,  fullRev);
AccelStepper x4h(7,  6,  fullRev);

//pcb rev 1
//AccelStepper x1m(12, 13, fullRev);
//AccelStepper x1h(10, 11, fullRev);
//AccelStepper x2m(9,  8,  fullRev);
//AccelStepper x2h(7,  6,  fullRev);
//AccelStepper x3m(3,  2,  fullRev);
//AccelStepper x3h(4,  5,  fullRev);
//AccelStepper x4m(30, 31, fullRev);
//AccelStepper x4h(29, 28, fullRev);

AccelStepper *allSteppers[] = {&x1m, &x1h, &x2m, &x2h, &x3m, &x3h, &x4m, &x4h};

AccelStepper *hSteppers[] = {&x1h, &x2h, &x3h, &x4h};
AccelStepper *mSteppers[] = {&x1m, &x2m, &x3m, &x4m};

struct move_data {
  uint16_t speed; //2 bytes
  uint16_t position; //2bytes
  int8_t dir; // -1, 0, 1 1byte
  uint8_t sub_id; //1byte
  bool is_minute_pointer; //1byte    =   7bytes/32bytes max


  move_data(uint16_t position, int8_t dir, uint8_t sub_id, bool is_minute_pointer, uint16_t speed)      
  {      
    this->speed = speed;   
    this->position = position;     
    this->dir = dir;      
    this->sub_id  = sub_id;   
    this->is_minute_pointer = is_minute_pointer;   
  }       
};

move_data move_i2c = {0, 0, 0, 0, 0};

const uint8_t address = 11; //1-16, 10 is at top left from clockface, row first
//i2c pins used, from the circuit board
const int sda = 15;
const int scl = 16;

bool i2c_flag = false;

// the setup function runs once when you press reset or power the board
void setup() {
  for(int i = 0; i < NUM_STEPPERS; i++){
    allSteppers[i]->setPinModesDriver();
    allSteppers[i]->setMaxSpeed(STEPPER_DEFAULT_SPEED);
    allSteppers[i]->setAcceleration(STEPPER_DEFAULT_ACCEL);
    allSteppers[i]->setCurrentPosition((int)(fullRev*STEPPER_DEFAULT_POS_FRACTION));
    allSteppers[i]->moveToShortestPath(0);
  }   

  //Initialize as i2c slave
  Wire.setSCL(scl);
  Wire.setSDA(sda);  
  Wire.setClock(100000);  
  Wire.begin(address); 
  Wire.onReceive(i2cReceive);
}

// the loop function runs over and over again forever
void loop() {    
  if(i2c_flag){
    i2c_flag = false;
  
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
  for(int i = 0; i < NUM_STEPPERS; i++){
	  allSteppers[i]->run();
  }
}

void i2cReceive(int numBytesReceived) {
  i2c_flag = true;
  Wire.readBytes( (byte*) &move_i2c, numBytesReceived);
}