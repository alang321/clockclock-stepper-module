#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "steppers.h"
#include "packet_handlers.h"
#include "command_queue.h"

// i2c handlers
void i2c_receive(int numBytesReceived);
void i2c_request();

CommandQueue i2c_cmd_queue;
volatile byte is_running_bitmap_shadow = 0;

#pragma region setup and loop

// the setup function runs once when you press reset or power the board
void setup()
{
    delay(5);
    initializeSteppers();
    
    // set enable_pin to high so no weird behaviour happens during mcu startup (has external pull down)
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);

    // Initialize as i2c slave
    Wire.setSCL(I2C_SCL_PIN);
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setClock(I2C_CLOCK_SPEED);
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
    CommandData next_cmd_data;
    bool should_process = false;

    noInterrupts();
    if (!i2c_cmd_queue.isEmpty())
    {
        next_cmd_data = i2c_cmd_queue.popCommand();
        should_process = true;
    }
    interrupts();


    if (should_process)
    {
#if DEBUG
        if (!next_cmd_data.hasExecuted)
        {
            Serial.println("invalid command packet, this shouldnt happen here");
            return;
        }
#endif
        // call the correct packet handler for each command id, these parse the buffer, check the checksum, check if the command is valid and then execute the command
        switch (next_cmd_data.commandID)
        {
        case enable_driver:
        {
            EnableDriverPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
            packet.executeCommand();
            break;
        }
        case set_speed:
        {
            SetSpeedPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
            packet.executeCommand();
            break;
        }
        case set_accel:
        {
            SetAccelPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
            packet.executeCommand();
            break;
        }
        case moveTo:
        {
            MoveToPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
            packet.executeCommand();
            break;
        }
        case moveTo_extra_revs:
        {
            MoveToExtraRevsPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
            packet.executeCommand();
            break;
        }
        case move:
        {
            MovePacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
            packet.executeCommand();
            break;
        }
        case stop:
        {
            StopPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
            packet.executeCommand();
            break;
        }
        case wiggle:
        {
            WigglePacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
            packet.executeCommand();
            break;
        }
        case moveTo_min_steps:
        {
            MoveToMinStepsPacket packet(next_cmd_data.buffer, next_cmd_data.bufferLength);
            packet.executeCommand();
            break;
        }

        default:
#if DEBUG
            Serial.println("Invalid command ID received, this can happen here due to interference, ignoring command");
#endif
            break;
        }
    }

    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        steppers[i]->run();
    }

    byte current_running_bitmap = 0;
    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        current_running_bitmap |= (steppers[i]->isRunning() << i);
    }
    is_running_bitmap_shadow = current_running_bitmap;
}

#pragma endregion

#pragma region i2c handlers

void i2c_receive(int numBytesReceived)
{
    if (numBytesReceived >= 2 && numBytesReceived <= MAX_COMMAND_LENGTH)
    {
        static byte i2c_buffer[MAX_COMMAND_LENGTH];
        Wire.readBytes((byte *)&i2c_buffer, numBytesReceived);
        i2c_cmd_queue.pushCommand(i2c_buffer, numBytesReceived);
    }
    else
    {
        // clear the bytes form the buffer
        byte discard_buffer[numBytesReceived];
        Wire.readBytes((byte *)&discard_buffer, numBytesReceived);
#if DEBUG
        Serial.println("Invalid command byte length");
#endif
    }
}

void i2c_request()
{
    Wire.write(is_running_bitmap_shadow);
}

#pragma endregion
