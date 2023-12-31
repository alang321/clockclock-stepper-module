#pragma once

#include <Arduino.h>
#include "config.h"

#define STEPPER_ID_MIN -3
#define STEPPER_ID_MAX (NUM_STEPPERS - 1)
#define NUM_STEPPER_IDS (STEPPER_ID_MAX - STEPPER_ID_MIN + 1)

#define CMD_COUNT 9

#define MAX_COMMAND_LENGTH 8 //max length of a command data in bytes + 1

enum cmd_identifier {enable_driver = 0, set_speed = 1, set_accel = 2, moveTo = 3, moveTo_extra_revs = 4, move = 5, stop = 6, wiggle = 7, moveTo_min_steps = 8};
enum stepper_selector {selector_minute = -3, selector_hour = -2, selector_all = -1};

#pragma region Packet data structs

#pragma pack(push, 1) // exact fit - no padding

struct enable_driver_datastruct {
    uint8_t cmd_id; //1bytes
    bool enable; //1bytes # true enables the driver, false disables it
};

struct set_speed_datastruct {
    uint8_t cmd_id; //1bytes
    uint16_t speed; //2bytes
    int8_t stepper_id; // -1  all, -2 hour steps, -3 minute steps, 1byte 
};

struct set_accel_datastruct {
    uint8_t cmd_id; //1bytes
    uint16_t accel; //2bytes
    int8_t stepper_id; // -1  all, -2 hour steps, -3 minute steps, 1byte 
};

struct moveTo_datastruct {
    uint8_t cmd_id; //1bytes
    int16_t position; //2bytes
    int8_t dir; // -1 ccw, 0 shortest path, 1 cw 1byte
    int8_t stepper_id; // -1  all, -2 hour steps, -3 minute steps, 1byte 
};

struct moveTo_extra_revs_datastruct { // moveTo but with an extra variable that allows rotating the stepper a variable extra times before reaching  target destination
    uint8_t cmd_id; //1bytes
    int16_t position; //2bytes
    int8_t dir; // -1 ccw, 1 cw 1byte
    uint8_t extra_revs; //1bytes
    int8_t stepper_id; // -1  all, -2 hour steps, -3 minute steps, 1byte 
};

struct moveTo_min_steps_datastruct {
    uint8_t cmd_id; //1bytes
    int16_t position; //2bytes
    int8_t dir; // -1 ccw, 1 cw 1byte
    uint16_t min_steps; //2bytes
    int8_t stepper_id; // -1  all, -2 hour steps, -3 minute steps, 1byte 
};

struct move_datastruct {
    uint8_t cmd_id; //1bytes
    uint16_t distance; //2bytes
    int8_t dir; // -1 ccw, 1 cw 1byte
    int8_t stepper_id; // -1  all, -2 hour steps, -3 minute steps, 1byte 
};

struct stop_datastruct {
    uint8_t cmd_id; //1bytes
    int8_t stepper_id; // -1  all, -2 hour steps, -3 minute steps, 1byte 
};

struct wiggle_datastruct {
    uint8_t cmd_id; //1bytes
    uint16_t distance; //2bytes
    int8_t dir; // -1 ccw, 1 cw 1byte
    int8_t stepper_id; // -1  all, -2 hour steps, -3 minute steps, 1byte 
};

#pragma pack(pop)

#pragma endregion

#pragma region Abstract Packet Class

//abstract class for packet data
class CommandPacket{
public:
    byte buffer[MAX_COMMAND_LENGTH];
    int bufferLength = 0;
    bool valid = false;
    const uint8_t commandID = 0;

    CommandPacket();
    CommandPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength);
    virtual bool executeCommand() = 0;
protected:
    bool verifyChecksum();
    virtual bool parseData() = 0;
    bool isStepperIdValid(int8_t stepper_id);
};

#pragma endregion

#pragma region Command Packet classes

class EnableDriverPacket : public CommandPacket{
public:
    const uint8_t commandID = enable_driver;

    EnableDriverPacket();
    EnableDriverPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength);

    bool executeCommand() override;

private:
    bool parseData() override;

    enable_driver_datastruct data;
};

class SetSpeedPacket : public CommandPacket{
public:
    const uint8_t commandID = set_speed;

    SetSpeedPacket();
    SetSpeedPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength);

    bool executeCommand() override;

private:
    bool parseData() override;
    set_speed_datastruct data;
};

class SetAccelPacket : public CommandPacket{
public:
    const uint8_t commandID = set_accel;

    SetAccelPacket();
    SetAccelPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength);

    bool executeCommand() override;

private:
    bool parseData() override;
    set_accel_datastruct data;
};

class MoveToPacket : public CommandPacket{
public:
    const uint8_t commandID = moveTo;

    MoveToPacket();
    MoveToPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength);

    bool executeCommand() override;

private:    
    bool parseData() override;
    moveTo_datastruct data;
};

class MoveToExtraRevsPacket : public CommandPacket{
public:
    const uint8_t commandID = moveTo_extra_revs;

    MoveToExtraRevsPacket();
    MoveToExtraRevsPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength);

    bool executeCommand() override;

private:    
    bool parseData() override;
    moveTo_extra_revs_datastruct data;
};

class MoveToMinStepsPacket : public CommandPacket{
public:
    const uint8_t commandID = moveTo_min_steps;

    MoveToMinStepsPacket();
    MoveToMinStepsPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength);

    bool executeCommand() override;

private:
    bool parseData() override;
    moveTo_min_steps_datastruct data;
};

class MovePacket : public CommandPacket{
public:
    const uint8_t commandID = move;

    MovePacket();
    MovePacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength);

    bool executeCommand() override;

private:
    bool parseData() override;
    move_datastruct data;
};

class StopPacket : public CommandPacket{
public:
    const uint8_t commandID = stop;

    StopPacket();
    StopPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength);

    bool executeCommand() override;

private:
    bool parseData() override;
    stop_datastruct data;
};

class WigglePacket : public CommandPacket{
public:
    const uint8_t commandID = wiggle;

    WigglePacket();
    WigglePacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength);

    bool executeCommand() override;

private:
    bool parseData() override;
    wiggle_datastruct data;
};

#pragma endregion

#pragma region Command Queues

struct CommandData{
    byte buffer[MAX_COMMAND_LENGTH];
    uint8_t bufferLength;
    uint8_t commandID;
    bool hasExecuted = true; //an object with this set to false is returned when the queue is empty
};

class CommandQueue{
public:
    bool pushCommand(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength); //returns true if overwrote a command that hasnt been executed yet
    //this returns a reference to the buffer of the next command to be executed and the command id
    const CommandData& popCommand();
    bool isEmpty();

private:
    //array of command data for each item in the entire queue length
    CommandData commands[CMD_QUEUE_LENGTH];
    uint16_t current_execute_index;
    uint16_t current_push_index;

    CommandData invalid_command = {{0}, 0, 0, false};
};

#pragma endregion

