#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "steppers.h"
#include "packet_handlers.h"

// i2c handlers
void i2c_receive(int numBytesReceived);
void i2c_request();

CommandQueue i2c_cmd_queue;

#pragma region setup and loop
// the setup function runs once when you press reset or power the board
void setup()
{
    // initialize steppers
    initializeSteppers();

    // set enable_pin to high so no weird behaviour happens during mcu startup (has external pull down)
    delay(5);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);

    // Initialize as i2c slave
    Wire.setSCL(I2C_SCL_PIN);
    Wire.setSDA(I2C_SDA_PIN);
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(i2c_receive);
    Wire.onRequest(i2c_request);

#if DEBUG
    Serial.begin(9600);
    Serial.println("Setup done");
#endif
}

// the loop function runs over and over again forever
void loop()
{
    if(!i2c_cmd_queue.isEmpty()){
        CommandData next_cmd_data = i2c_cmd_queue.popCommand();

#if DEBUG
        if (!next_cmd_data.hasExecuted)
        {
            Serial.println("invalid command packet, this shouldnt happen here");
            return;
        }
#endif
        
        //call the correct packet handler for each command id
        switch (next_cmd_data.commandID)
        {
            case enable_driver:
            {
                EnableDriverPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
                if(packet.parseData()){
                    packet.executeCommand();
                }
                break;
            }
            case set_speed:
            {
                SetSpeedPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
                if(packet.parseData()){
                    packet.executeCommand();
                }
                break;
            }
            case set_accel:
            {
                SetAccelPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
                if(packet.parseData()){
                    packet.executeCommand();
                }
                break;
            }
            case moveTo:
            {
                MoveToPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
                if(packet.parseData()){
                    packet.executeCommand();
                }
                break;
            }
            case moveTo_extra_revs:
            {
                MoveToExtraRevsPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
                if(packet.parseData()){
                    packet.executeCommand();
                }
                break;
            }
            case move:
            {
                MovePacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
                if(packet.parseData()){
                    packet.executeCommand();
                }
                break;
            }
            case stop:
            {
                StopPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
                if(packet.parseData()){
                    packet.executeCommand();
                }
                break;
            }
            case wiggle:
            {
                WigglePacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
                if(packet.parseData()){
                    packet.executeCommand();
                }
                break;
            }
            case moveTo_min_steps:
            {
                MoveToMinStepsPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
                if(packet.parseData()){
                    packet.executeCommand();
                }
                break;
            }

            default:
#if DEBUG
                Serial.println("Invalid command ID received, this shouldnt happen here, ignoring command");
#endif
                break;
        }
    }

    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        steppers[i]->run();
    }
}

#pragma endregion

#pragma region i2c handlers

void i2c_receive(int numBytesReceived)
{
    if (numBytesReceived > 0 && numBytesReceived <= MAX_COMMAND_LENGTH)
    {
        uint8_t cmd_id = 0;
        Wire.readBytes((byte *)&cmd_id, 1);

        if (isCommandIDValid(cmd_id))
        {
            byte i2c_buffer[MAX_COMMAND_LENGTH - 1];

            Wire.readBytes((byte *)&i2c_buffer, numBytesReceived - 1);
            i2c_cmd_queue.pushCommand(cmd_id, i2c_buffer, numBytesReceived - 1);
        }
#if DEBUG
        else
        {
            Serial.println("Invalid command ID received");
        }
#endif
    }
#if DEBUG
    else
    {
        Serial.println("Invalid command byte length");
    }
#endif
}

void i2c_request()
{
    byte is_running_bitmap = 0; // 1 if its still running to target
    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        if (steppers[i]->isRunning())
        {
            is_running_bitmap = (1 << i) | is_running_bitmap;
        }
    }
    
    Wire.write(is_running_bitmap);
}

#pragma endregion
