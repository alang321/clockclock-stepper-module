#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <cppQueue.h>

#define NUM_STEPPERS 8
#define NUM_STEPPERS_H 4
#define NUM_STEPPERS_M 4
#define LED 33
#define STEPPER_DEFAULT_SPEED 700
#define STEPPER_DEFAULT_ACCEL 300
#define STEPS_PER_REVOLUTION 4320 //360*12
#define STEPPER_DEFAULT_POS_FRACTION 0.5 //at 6o'clock position

#define CMD_COUNT 6
#define CMD_QUEUE_LENGTH 16

#define I2C_ADDRESS 15//[12;17], 12 is at top left from clockface, row first
#define I2C_SDA_PIN 15
#define I2C_SCL_PIN 16
#define ENABLE_PIN 17

//i2c handlers
void i2c_receive(int numBytesReceived);
void i2c_request();

//command handlers
void set_speed_handler();
void set_accel_handler();
void moveTo_handler();
void move_handler();
void stop_handler();
void falling_pointer_handler();

#pragma region accel stepper defintion

//step pin, dir pin, number of steps per revolution
//pcb rev 2
AccelStepper x1m(26, 27, STEPS_PER_REVOLUTION);//normal green
//AccelStepper x1m(17, 20, STEPS_PER_REVOLUTION);//broken maple cutoff from black rev 1 pcb
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

#pragma endregion

#pragma region i2c commands

enum cmd_identifier {set_speed = 0, set_accel = 1, moveTo = 2, move = 3, stop = 4, falling_pointer = 5};

struct set_speed_datastruct {
  uint16_t speed; //2bytes
  uint8_t stepper_id; //1byte
};

struct set_accel_datastruct {
  uint16_t accel; //2bytes
  uint8_t stepper_id; //1byte
};

struct moveTo_datastruct {
  uint16_t position; //2bytes
  int8_t dir; // -1 ccw, 0 shortest path, 1 cw 1byte
  uint8_t stepper_id; //1byte
};

struct move_datastruct {
  uint16_t distance; //2bytes
  int8_t dir; // -1 ccw, 1 cw 1byte
  uint8_t stepper_id; //1byte  
};

struct stop_datastruct {
  uint8_t stepper_id; //1byte
};

struct falling_pointer_datastruct {
  uint8_t stepper_id; //1byte
};

cppQueue	set_speed_queue(sizeof(set_speed_datastruct), CMD_QUEUE_LENGTH, FIFO, true);
cppQueue	set_accel_queue(sizeof(set_accel_datastruct), CMD_QUEUE_LENGTH, FIFO, true);
cppQueue	moveTo_queue(sizeof(moveTo_datastruct), CMD_QUEUE_LENGTH, FIFO, true);
cppQueue	move_queue(sizeof(move_datastruct), CMD_QUEUE_LENGTH, FIFO, true);
cppQueue	stop_queue(sizeof(stop_datastruct), CMD_QUEUE_LENGTH, FIFO, true);
cppQueue	falling_pointer_queue(sizeof(falling_pointer_datastruct), CMD_QUEUE_LENGTH, FIFO, true);

cppQueue i2c_cmd_queues[] = {set_speed_queue, set_accel_queue, moveTo_queue, move_queue, stop_queue, falling_pointer_queue};

typedef void (*i2c_cmd_handler) ();

i2c_cmd_handler i2c_cmd_handlers[] = {set_speed_handler, set_accel_handler, moveTo_handler, move_handler, stop_handler, falling_pointer_handler};

#pragma endregion

#pragma region setup and loop
// the setup function runs once when you press reset or power the board
void setup() {
  for(int i = 0; i < NUM_STEPPERS_H; i++){
    h_steppers[i]->setPinModesDriver();
    h_steppers[i]->setMaxSpeed(STEPPER_DEFAULT_SPEED);
    h_steppers[i]->setAcceleration(STEPPER_DEFAULT_ACCEL);
    h_steppers[i]->setCurrentPosition((int)(STEPS_PER_REVOLUTION*STEPPER_DEFAULT_POS_FRACTION));
    h_steppers[i]->moveToSingleRevolutionDir(0, -1);
  }

  for(int i = 0; i < NUM_STEPPERS_M; i++){
    m_steppers[i]->setPinModesDriver();
    m_steppers[i]->setMaxSpeed(STEPPER_DEFAULT_SPEED);
    m_steppers[i]->setAcceleration(STEPPER_DEFAULT_ACCEL);
    m_steppers[i]->setCurrentPosition((int)(STEPS_PER_REVOLUTION*STEPPER_DEFAULT_POS_FRACTION));
    m_steppers[i]->moveToSingleRevolutionDir(0, 1);
  }

  delay(5);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  //Initialize as i2c slave
  Wire.setSCL(I2C_SCL_PIN);
  Wire.setSDA(I2C_SDA_PIN);  
  //Wire.setClock(100000);  breaks the i2c bus for some reason
  Wire.begin(I2C_ADDRESS); 
  Wire.onReceive(i2c_receive);
  Wire.onRequest(i2c_request);
}

// the loop function runs over and over again forever
void loop() {
  for(int i = 0; i < CMD_COUNT; i++){
	  if(!i2c_cmd_queues[i].isEmpty()){
      i2c_cmd_handlers[i]();
    }
  }
  for(int i = 0; i < NUM_STEPPERS; i++){
	  steppers[i]->run();
  }
}

#pragma endregion

#pragma region i2c handlers

void i2c_receive(int numBytesReceived) {
  uint8_t cmd_id = 0;
  Wire.readBytes((byte*) &cmd_id, 1);

  byte i2c_buffer[numBytesReceived - 1];

  Wire.readBytes((byte*) &i2c_buffer, numBytesReceived - 1);
  i2c_cmd_queues[cmd_id].push(&i2c_buffer);
}

void i2c_request() {
  byte is_running_bitmap = 0; // 1 if its still running to target
  for(int i = 0; i < NUM_STEPPERS; i++){
    if(steppers[i]->isRunning()){
      bitSet(is_running_bitmap, i);
    }
  }
  Wire.write(is_running_bitmap);
}

#pragma endregion

#pragma region command handlers

void set_speed_handler(){
  int cmd_id = set_speed;
  
  set_speed_datastruct set_speed_data;
  i2c_cmd_queues[0].pop(&set_speed_data);

  steppers[set_speed_data.stepper_id]->setMaxSpeed(set_speed_data.speed);
}

void set_accel_handler(){
  int cmd_id = set_accel;
  
  set_accel_datastruct set_accel_data;
  i2c_cmd_queues[cmd_id].pop(&set_accel_data);

  steppers[set_accel_data.stepper_id]->setMaxSpeed(set_accel_data.accel);
}

void moveTo_handler(){
  int cmd_id = moveTo;
  
  moveTo_datastruct moveTo_data;
  i2c_cmd_queues[cmd_id].pop(&moveTo_data);

  //movement direction, shortest, left or right
  if(moveTo_data.dir == 0){ //fix the handling of directions
    steppers[moveTo_data.stepper_id]->moveToShortestPath(moveTo_data.position);
  }else{
    steppers[moveTo_data.stepper_id]->moveToSingleRevolutionDir(moveTo_data.position, moveTo_data.dir);
  }
}

void move_handler(){
  int cmd_id = move;
  
  move_datastruct move_data;
  i2c_cmd_queues[cmd_id].pop(&move_data);

  steppers[move_data.stepper_id]->move(move_data.distance * move_data.dir);
}

void stop_handler(){
  int cmd_id = stop;

  stop_datastruct stop_data;
  i2c_cmd_queues[cmd_id].pop(&stop_data);

  steppers[stop_data.stepper_id]->stop();
}

void falling_pointer_handler(){
  //todo : implement
}

#pragma endregion