#pragma once

#include <Arduino.h>
#include "config.h"
#include "packet_handlers.h"

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
