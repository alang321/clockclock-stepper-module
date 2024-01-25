#include <Arduino.h>
#include "config.h"
#include "command_queue.h"

bool CommandQueue::pushCommand(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength){
    commands[current_push_index].bufferLength = bufferLength;
    memcpy(commands[current_push_index].buffer, buffer, bufferLength);
    commands[current_push_index].commandID = static_cast<uint8_t>(commands[current_push_index].buffer[0]);
    
    if(!commands[current_push_index].hasExecuted){
        //if the command at the current_push_index has not been executed yet, push the execute index forward 
        //this keeps the queue fifo but allows overwriting of commands that have not been executed yet
        //if the queue is full
        current_execute_index = (current_execute_index + 1) % CMD_QUEUE_LENGTH; 

        current_push_index = (current_push_index + 1) % CMD_QUEUE_LENGTH;

        return true;
    }
    else{
        commands[current_push_index].hasExecuted = false;

        current_push_index = (current_push_index + 1) % CMD_QUEUE_LENGTH;

        return false;
    }
}

bool CommandQueue::isEmpty(){
    return commands[current_execute_index].hasExecuted;
}

const CommandData& CommandQueue::popCommand(){
    if(!isEmpty()){
        commands[current_execute_index].hasExecuted = true;

        uint16_t temp = current_execute_index;
        current_execute_index = (current_execute_index + 1) % CMD_QUEUE_LENGTH;

        return commands[temp];
    }
#if DEBUG
    Serial.println("Command queue is empty");
    Serial.println("Returning invalid command");
    //print push and execute index
    Serial.print("Push index: ");
    Serial.println(current_push_index);
    Serial.print("Execute index: ");
    Serial.println(current_execute_index);
#endif
    return invalid_command;
}
