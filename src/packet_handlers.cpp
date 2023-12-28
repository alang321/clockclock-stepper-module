#include "packet_handlers.h"
#include <Arduino.h>
#include <AccelStepper.h>
#include "config.h"
#include "steppers.h"

bool isCommandIDValid(uint8_t cmd_id){
    return cmd_id >= 0 && cmd_id <= CMD_COUNT;
}

#pragma region Abstract Packet Class

CommandPacket::CommandPacket(){
}

//abstract class for packet data
CommandPacket::CommandPacket(byte (&buffer)[MAX_COMMAND_LENGTH - 1], uint8_t bufferLength, uint8_t commandID){
    memcpy(this->buffer, buffer, bufferLength);
    this->bufferLength = bufferLength;
    this->_commandID = commandID;
}

bool CommandPacket::hasValidChecksum(){
    uint8_t checksum = 0;
    for(int i = 0; i < bufferLength - 1; i++){
        checksum += buffer[i];
    }
    checksum += _commandID;
#if DEBUG
    bool checksum_valid = checksum == buffer[bufferLength - 1];
    if(!checksum_valid){
        Serial.println("Invalid checksum");
        Serial.print("Checksum: ");
        Serial.println(checksum);
        Serial.print("Checksum byte: ");
        Serial.println(buffer[bufferLength - 1]);
    }
    return checksum_valid;
#else
    return checksum == buffer[bufferLength - 1];
#endif
}

bool CommandPacket::isStepperIdValid(int8_t stepper_id){
    return stepper_id >= STEPPER_ID_MIN && stepper_id <= STEPPER_ID_MAX;
}

#pragma endregion

#pragma region Enable Driver Packet

EnableDriverPacket::EnableDriverPacket() : CommandPacket(){}

EnableDriverPacket::EnableDriverPacket(byte (&buffer)[MAX_COMMAND_LENGTH - 1], uint8_t bufferLength) : CommandPacket(buffer, bufferLength, commandID){}

bool EnableDriverPacket::parseData(){
    if (!(bufferLength == sizeof(data) + 1 && hasValidChecksum())) 
    {
        valid = false;
        return false;
    }   

    memcpy(&data, buffer, sizeof(data));

    //check if data is in valid range
    valid = true;
    return true;
}

bool EnableDriverPacket::executeCommand(){
    if(valid){
        digitalWrite(ENABLE_PIN, data.enable);
        return true;
    }
    return false;
}

#pragma endregion

#pragma region Set Speed Packet

SetSpeedPacket::SetSpeedPacket() : CommandPacket(){}

SetSpeedPacket::SetSpeedPacket(byte (&buffer)[MAX_COMMAND_LENGTH - 1], uint8_t bufferLength) : CommandPacket(buffer, bufferLength, commandID){}

bool SetSpeedPacket::parseData(){
    if (!(bufferLength == sizeof(data) + 1 && hasValidChecksum())) {
        valid = false;
        return false;
    }

    memcpy(&data, buffer, sizeof(data));

    //check if data is in valid range
    if(data.speed < MIN_SPEED || data.speed > MAX_SPEED) {
        valid = false;
        return false;
    }

    if(!isStepperIdValid(data.stepper_id)) {
        valid = false;
        return false;
    }

    valid = true;
    return true;
}

bool SetSpeedPacket::executeCommand(){
    if(valid){
        switch(data.stepper_id)
        {
            case selector_all:
            for(int i = 0; i < NUM_STEPPERS; i++){
                steppers[i]->setMaxSpeed(data.speed);
            }
            break;

            case selector_hour:
            for(int i = 0; i < NUM_STEPPERS_H; i++){
                h_steppers[i]->setMaxSpeed(data.speed);
            }
            break;

            case selector_minute:
            for(int i = 0; i < NUM_STEPPERS_M; i++){
                m_steppers[i]->setMaxSpeed(data.speed);
            }
            break;
            
            default: //all the other stepper ids selecting individual steppers
            steppers[data.stepper_id]->setMaxSpeed(data.speed);
            break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region Set Accel Packet

SetAccelPacket::SetAccelPacket() : CommandPacket(){}

SetAccelPacket::SetAccelPacket(byte (&buffer)[MAX_COMMAND_LENGTH - 1], uint8_t bufferLength) : CommandPacket(buffer, bufferLength, commandID){}

bool SetAccelPacket::parseData(){
    if (!(bufferLength == sizeof(data) + 1 && hasValidChecksum()))  {
        valid = false;
        return false;
    }

    memcpy(&data, buffer, sizeof(data));

    //check if data is in valid range
    if(data.accel < MAX_ACCEL || data.accel > MAX_ACCEL)  {
        valid = false;
        return false;
    }

    if(!isStepperIdValid(data.stepper_id))  {
        valid = false;
        return false;
    }

    valid = true;
    return true;
}

bool SetAccelPacket::executeCommand(){
    if(valid){
        switch(data.stepper_id)
        {
        case selector_all:{
            float c0 = steppers[0]->setAcceleration(data.accel);
            for(int i = 1; i < NUM_STEPPERS; i++){
                steppers[i]->setAcceleration(data.accel, c0);
            }
        }break;

        case selector_hour:{
            float c0 = h_steppers[0]->setAcceleration(data.accel);
            for(int i = 1; i < NUM_STEPPERS_H; i++){
                h_steppers[i]->setAcceleration(data.accel, c0);
            }
            }break;

        case selector_minute:{
            float c0 = m_steppers[0]->setAcceleration(data.accel);
            for(int i = 1; i < NUM_STEPPERS_M; i++){
                m_steppers[i]->setAcceleration(data.accel, c0);
            }
            }break;

        default: //all the other stepper ids selecting individual steppers
            steppers[data.stepper_id]->setAcceleration(data.accel);
            break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region MoveTo Packet

MoveToPacket::MoveToPacket() : CommandPacket(){}

MoveToPacket::MoveToPacket(byte (&buffer)[MAX_COMMAND_LENGTH - 1], uint8_t bufferLength) : CommandPacket(buffer, bufferLength, commandID){}

bool MoveToPacket::parseData(){
    if (!(bufferLength == sizeof(data) + 1 && hasValidChecksum())) {
        valid = false;
        return false;
    }

    memcpy(&data, buffer, sizeof(data));

    //check if data is in valid range
    if(!isStepperIdValid(data.stepper_id))  {
        valid = false;
        return false;
    }
    if (data.dir != 1 && data.dir != -1 && data.dir != 0) {
        valid = false;
        return false;
    }

    valid = true;
    return true;
}

bool MoveToPacket::executeCommand(){
    if(valid){
        switch(data.stepper_id)
        {
        case selector_all:
            for(int i = 0; i < NUM_STEPPERS; i++){
            steppers[i]->moveToSingleRevolution(data.position, data.dir);
            }
            break;

        case selector_hour:
            for(int i = 0; i < NUM_STEPPERS_H; i++){
            h_steppers[i]->moveToSingleRevolution(data.position, data.dir);
            }
            break;

        case selector_minute:
            for(int i = 0; i < NUM_STEPPERS_M; i++){
            m_steppers[i]->moveToSingleRevolution(data.position, data.dir);
            }
            break;

        default: //all the other stepper ids selecting individual steppers
            steppers[data.stepper_id]->moveToSingleRevolution(data.position, data.dir);
            break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region MoveTo Extra Revs Packet

MoveToExtraRevsPacket::MoveToExtraRevsPacket() : CommandPacket(){}

MoveToExtraRevsPacket::MoveToExtraRevsPacket(byte (&buffer)[MAX_COMMAND_LENGTH - 1], uint8_t bufferLength) : CommandPacket(buffer, bufferLength, commandID){}

bool MoveToExtraRevsPacket::parseData(){
    if (!(bufferLength == sizeof(data) + 1 && hasValidChecksum()))  {
        valid = false;
        return false;
    }

    memcpy(&data, buffer, sizeof(data));

    //check if data is in valid range
    if(!isStepperIdValid(data.stepper_id))  {
        valid = false;
        return false;
    }
    if (data.dir != 1 && data.dir != -1 && data.dir != 0)  {
        valid = false;
        return false;
    }

    valid = true;
    return true;
}

bool MoveToExtraRevsPacket::executeCommand(){
    if(valid){
        switch(data.stepper_id)
        {
            case selector_all:
                for(int i = 0; i < NUM_STEPPERS; i++){
                    steppers[i]->moveToExtraRevolutions(data.position, data.dir, data.extra_revs);
                }
            break;

            case selector_hour:
                for(int i = 0; i < NUM_STEPPERS_H; i++){
                    h_steppers[i]->moveToExtraRevolutions(data.position, data.dir, data.extra_revs);
                }
            break;

            case selector_minute:
                for(int i = 0; i < NUM_STEPPERS_M; i++){
                    m_steppers[i]->moveToExtraRevolutions(data.position, data.dir, data.extra_revs);
                }
            break;

            default: //all the other stepper ids selecting individual steppers
                steppers[data.stepper_id]->moveToExtraRevolutions(data.position, data.dir, data.extra_revs);
                break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region MoveTo Min Steps Packet

MoveToMinStepsPacket::MoveToMinStepsPacket() : CommandPacket(){}

MoveToMinStepsPacket::MoveToMinStepsPacket(byte (&buffer)[MAX_COMMAND_LENGTH - 1], uint8_t bufferLength) : CommandPacket(buffer, bufferLength, commandID){}

bool MoveToMinStepsPacket::parseData(){
    if (!(bufferLength == sizeof(data) + 1 && hasValidChecksum())) {
        valid = false;
        return false;
    }

    memcpy(&data, buffer, sizeof(data));

    //check if data is in valid range
    if(!isStepperIdValid(data.stepper_id)) {
        valid = false;
        return false;
    }
    if (data.dir != 1 && data.dir != -1 && data.dir != 0) {
        valid = false;
        return false;
    }

    valid = true;
    return true;
}

bool MoveToMinStepsPacket::executeCommand(){
    if(valid){
        switch(data.stepper_id)
        {
            case selector_all:
                for(int i = 0; i < NUM_STEPPERS; i++){
                    steppers[i]->moveToMinSteps(data.position, data.dir, data.min_steps);
                }
            break;

            case selector_hour:
                for(int i = 0; i < NUM_STEPPERS_H; i++){
                    h_steppers[i]->moveToMinSteps(data.position, data.dir, data.min_steps);
                }
            break;

            case selector_minute:
                for(int i = 0; i < NUM_STEPPERS_M; i++){
                    m_steppers[i]->moveToMinSteps(data.position, data.dir, data.min_steps);
                }
            break;

            default: //all the other stepper ids selecting individual steppers
                steppers[data.stepper_id]->moveToMinSteps(data.position, data.dir, data.min_steps);
                break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region Move Packet

MovePacket::MovePacket() : CommandPacket(){}

MovePacket::MovePacket(byte (&buffer)[MAX_COMMAND_LENGTH - 1], uint8_t bufferLength) : CommandPacket(buffer, bufferLength, commandID){}

bool MovePacket::parseData(){
    if (!(bufferLength == sizeof(data) + 1 && hasValidChecksum())) {
        valid = false;
        return false;
    }

    memcpy(&data, buffer, sizeof(data));

    //check if data is in valid range
    if(!isStepperIdValid(data.stepper_id)) {
        valid = false;
        return false;
    }
    if (data.dir != 1 && data.dir != -1 && data.dir != 0) {
        valid = false;
        return false;
    }

    valid = true;
    return true;
}

bool MovePacket::executeCommand(){
    if(valid){
        switch(data.stepper_id)
        {
            case selector_all:
                for(int i = 0; i < NUM_STEPPERS; i++){
                    steppers[i]->moveTarget(data.distance * data.dir);
                }
            break;

            case selector_hour:
                for(int i = 0; i < NUM_STEPPERS_H; i++){
                    h_steppers[i]->moveTarget(data.distance * data.dir);
                }
            break;

            case selector_minute:
                for(int i = 0; i < NUM_STEPPERS_M; i++){
                    m_steppers[i]->moveTarget(data.distance * data.dir);
                }
            break;

            default: //all the other stepper ids selecting individual steppers
                steppers[data.stepper_id]->move(data.distance * data.dir);
                break;
        }
        return true;
    }
    return false;
}


#pragma endregion

#pragma region Stop Packet

StopPacket::StopPacket() : CommandPacket(){}

StopPacket::StopPacket(byte (&buffer)[MAX_COMMAND_LENGTH - 1], uint8_t bufferLength) : CommandPacket(buffer, bufferLength, commandID){}

bool StopPacket::parseData(){
    if (!(bufferLength == sizeof(data) + 1 && hasValidChecksum())) {
        valid = false;
        return false;
    }

    memcpy(&data, buffer, sizeof(data));

    //check if data is in valid range
    if(!isStepperIdValid(data.stepper_id)) {
        valid = false;
        return false;
    }

    valid = true;
    return true;
}

bool StopPacket::executeCommand(){
    if(valid){
        switch(data.stepper_id)
        {
            case selector_all:
                for(int i = 0; i < NUM_STEPPERS; i++){
                    steppers[i]->stop();
                }
            break;

            case selector_hour:
                for(int i = 0; i < NUM_STEPPERS_H; i++){
                    h_steppers[i]->stop();
                }
            break;

            case selector_minute:
                for(int i = 0; i < NUM_STEPPERS_M; i++){
                    m_steppers[i]->stop();
                }
            break;
            
            default: //all the other stepper ids selecting individual steppers
                steppers[data.stepper_id]->stop();
                break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region Wiggle Packet

WigglePacket::WigglePacket() : CommandPacket(){}

WigglePacket::WigglePacket(byte (&buffer)[MAX_COMMAND_LENGTH - 1], uint8_t bufferLength) : CommandPacket(buffer, bufferLength, commandID){}

bool WigglePacket::parseData(){
    if (!(bufferLength == sizeof(data) + 1 && hasValidChecksum())) {
        valid = false;
        return false;
    }

    memcpy(&data, buffer, sizeof(data));

    //check if data is in valid range
    if(!isStepperIdValid(data.stepper_id)) {
        valid = false;
        return false;
    }
    if (data.dir != 1 && data.dir != -1) {
        valid = false;
        return false;
    }

    valid = true;
    return true;
}

bool WigglePacket::executeCommand(){
    if(valid){
        switch(data.stepper_id)
        {
            case selector_all:
                for(int i = 0; i < NUM_STEPPERS; i++){
                    steppers[i]->wiggle(data.distance * data.dir);
                }
            break;

            case selector_hour:
                for(int i = 0; i < NUM_STEPPERS_H; i++){
                    h_steppers[i]->wiggle(data.distance * data.dir);
                }
            break;

            case selector_minute:
                for(int i = 0; i < NUM_STEPPERS_M; i++){
                    m_steppers[i]->wiggle(data.distance * data.dir);
                }
            break;

            default: //all the other stepper ids selecting individual steppers
                steppers[data.stepper_id]->wiggle(data.distance * data.dir);
                break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region Command Queue

bool CommandQueue::pushCommand(uint8_t cmd_id, byte (&buffer)[MAX_COMMAND_LENGTH - 1], uint8_t bufferLength){
    commands[current_push_index].bufferLength = bufferLength;
    memcpy(commands[current_push_index].buffer, buffer, bufferLength);
    commands[current_push_index].commandID = cmd_id;
    
    if(!commands[current_push_index].hasExecuted){
        //if the command at the current_push_index has not been executed yet, push the execute index forward 
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

#pragma endregion