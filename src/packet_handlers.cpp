#include "packet_handlers.h"
#include <Arduino.h>
#include <AccelStepper.h>
#include "config.h"
#include "steppers.h"

bool isStepperIDValid(int8_t stepper_id)
{
    return stepper_id >= STEPPER_ID_MIN && stepper_id <= STEPPER_ID_MAX;
}

bool isCommandIDValid(uint8_t command_id)
{
    return command_id >= CMD_ID_MIN && command_id <= CMD_ID_MAX;
}

#pragma region Abstract Packet Class

CommandPacket::CommandPacket() {}

// abstract class for packet data
CommandPacket::CommandPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength)
{
    memcpy(this->buffer, buffer, bufferLength);
    this->bufferLength = bufferLength;

    valid = verifyChecksum();
}

bool CommandPacket::verifyChecksum()
{
    uint8_t checksum = 0;
    for (int i = 0; i < bufferLength - 1; i++)
    {
        checksum += buffer[i];
    }
    return checksum == buffer[bufferLength - 1];
}

#pragma endregion

#pragma region Enable Driver Packet

EnableDriverPacket::EnableDriverPacket() : CommandPacket() {}

EnableDriverPacket::EnableDriverPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength) : CommandPacket(buffer, bufferLength)
{
    valid = parseData();
}

bool EnableDriverPacket::parseData()
{
    if (valid)
    {
        if (!(bufferLength == sizeof(data)))
            return false;

        memcpy(&data, buffer, sizeof(data));

        if (data.cmd_id != commandID)
            return false;

        return true;
    }
    return false;
}

bool EnableDriverPacket::executeCommand()
{
    if (valid)
    {
        digitalWrite(ENABLE_PIN, data.enable);
        return true;
    }
    return false;
}

#pragma endregion

#pragma region Set Speed Packet

SetSpeedPacket::SetSpeedPacket() : CommandPacket() {}

SetSpeedPacket::SetSpeedPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength) : CommandPacket(buffer, bufferLength)
{
    valid = parseData();
}

bool SetSpeedPacket::parseData()
{
    if (valid)
    {
        if (!(bufferLength == sizeof(data)))
            return false;

        memcpy(&data, buffer, sizeof(data));

        if (data.cmd_id != commandID)
            return false;
        if (data.speed < MIN_SPEED || data.speed > MAX_SPEED)
            return false;
        if (!isStepperIDValid(data.stepper_id))
            return false;

        return true;
    }
    return false;
}

bool SetSpeedPacket::executeCommand()
{
    if (valid)
    {
        switch (data.stepper_id)
        {
        case selector_all:
            for (int i = 0; i < NUM_STEPPERS; i++)
            {
                steppers[i]->setMaxSpeed(data.speed);
            }
            break;

        case selector_hour:
            for (int i = 0; i < NUM_STEPPERS_H; i++)
            {
                h_steppers[i]->setMaxSpeed(data.speed);
            }
            break;

        case selector_minute:
            for (int i = 0; i < NUM_STEPPERS_M; i++)
            {
                m_steppers[i]->setMaxSpeed(data.speed);
            }
            break;

        default: // all the other stepper ids selecting individual steppers
            steppers[data.stepper_id]->setMaxSpeed(data.speed);
            break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region Set Accel Packet

SetAccelPacket::SetAccelPacket() : CommandPacket() {}

SetAccelPacket::SetAccelPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength) : CommandPacket(buffer, bufferLength) 
{
    valid = parseData();
}

bool SetAccelPacket::parseData()
{
    if (valid)
    {
        if (!(bufferLength == sizeof(data)))
            return false;

        memcpy(&data, buffer, sizeof(data));

        if (data.cmd_id != commandID)
            return false;
        if (data.accel < MAX_ACCEL || data.accel > MAX_ACCEL)
            return false;
        if (!isStepperIDValid(data.stepper_id))
            return false;

        return true;
    }
    return false;
}

bool SetAccelPacket::executeCommand()
{
    if (valid)
    {
        switch (data.stepper_id)
        {
        case selector_all:
        {
            float c0 = steppers[0]->setAcceleration(data.accel);
            for (int i = 1; i < NUM_STEPPERS; i++)
            {
                steppers[i]->setAcceleration(data.accel, c0);
            }
        }
        break;

        case selector_hour:
        {
            float c0 = h_steppers[0]->setAcceleration(data.accel);
            for (int i = 1; i < NUM_STEPPERS_H; i++)
            {
                h_steppers[i]->setAcceleration(data.accel, c0);
            }
        }
        break;

        case selector_minute:
        {
            float c0 = m_steppers[0]->setAcceleration(data.accel);
            for (int i = 1; i < NUM_STEPPERS_M; i++)
            {
                m_steppers[i]->setAcceleration(data.accel, c0);
            }
        }
        break;

        default: // all the other stepper ids selecting individual steppers
            steppers[data.stepper_id]->setAcceleration(data.accel);
            break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region MoveTo Packet

MoveToPacket::MoveToPacket() : CommandPacket() {}

MoveToPacket::MoveToPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength) : CommandPacket(buffer, bufferLength) 
{
    valid = parseData();
}

bool MoveToPacket::parseData()
{
    if (valid)
    {
        if (!(bufferLength == sizeof(data)))
            return false;

        memcpy(&data, buffer, sizeof(data));

        if (data.cmd_id != commandID)
            return false;

        // check if data is in valid range
        if (!isStepperIDValid(data.stepper_id))
            return false;

        if (data.dir != 1 && data.dir != -1 && data.dir != 0)
            return false;

        return true;
    }
    return false;
}

bool MoveToPacket::executeCommand()
{
    if (valid)
    {
        switch (data.stepper_id)
        {
        case selector_all:
            for (int i = 0; i < NUM_STEPPERS; i++)
            {
                steppers[i]->moveToSingleRevolution(data.position, data.dir);
            }
            break;

        case selector_hour:
            for (int i = 0; i < NUM_STEPPERS_H; i++)
            {
                h_steppers[i]->moveToSingleRevolution(data.position, data.dir);
            }
            break;

        case selector_minute:
            for (int i = 0; i < NUM_STEPPERS_M; i++)
            {
                m_steppers[i]->moveToSingleRevolution(data.position, data.dir);
            }
            break;

        default: // all the other stepper ids selecting individual steppers
            steppers[data.stepper_id]->moveToSingleRevolution(data.position, data.dir);
            break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region MoveTo Extra Revs Packet

MoveToExtraRevsPacket::MoveToExtraRevsPacket() : CommandPacket() {}

MoveToExtraRevsPacket::MoveToExtraRevsPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength) : CommandPacket(buffer, bufferLength)
{
    valid = parseData();
}

bool MoveToExtraRevsPacket::parseData()
{
    if (valid)
    {
        if (!(bufferLength == sizeof(data)))
            return false;

        memcpy(&data, buffer, sizeof(data));

        if (data.cmd_id != commandID)
            return false;
        if (!isStepperIDValid(data.stepper_id))
            return false;
        if (data.dir != 1 && data.dir != -1 && data.dir != 0)
            return false;

        return true;
    }
    return false;
}

bool MoveToExtraRevsPacket::executeCommand()
{
    if (valid)
    {
        switch (data.stepper_id)
        {
        case selector_all:
            for (int i = 0; i < NUM_STEPPERS; i++)
            {
                steppers[i]->moveToExtraRevolutions(data.position, data.dir, data.extra_revs);
            }
            break;

        case selector_hour:
            for (int i = 0; i < NUM_STEPPERS_H; i++)
            {
                h_steppers[i]->moveToExtraRevolutions(data.position, data.dir, data.extra_revs);
            }
            break;

        case selector_minute:
            for (int i = 0; i < NUM_STEPPERS_M; i++)
            {
                m_steppers[i]->moveToExtraRevolutions(data.position, data.dir, data.extra_revs);
            }
            break;

        default: // all the other stepper ids selecting individual steppers
            steppers[data.stepper_id]->moveToExtraRevolutions(data.position, data.dir, data.extra_revs);
            break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region MoveTo Min Steps Packet

MoveToMinStepsPacket::MoveToMinStepsPacket() : CommandPacket() {}

MoveToMinStepsPacket::MoveToMinStepsPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength) : CommandPacket(buffer, bufferLength) 
{
    valid = parseData();
}

bool MoveToMinStepsPacket::parseData()
{
    if (valid)
    {
        if (!(bufferLength == sizeof(data)))
            return false;

        memcpy(&data, buffer, sizeof(data));

        if (data.cmd_id != commandID)
            return false;
        if (!isStepperIDValid(data.stepper_id))
            return false;
        if (data.dir != 1 && data.dir != -1 && data.dir != 0)
            return false;

        return true;
    }
    return false;
}

bool MoveToMinStepsPacket::executeCommand()
{
    if (valid)
    {
        switch (data.stepper_id)
        {
        case selector_all:
            for (int i = 0; i < NUM_STEPPERS; i++)
            {
                steppers[i]->moveToMinSteps(data.position, data.dir, data.min_steps);
            }
            break;

        case selector_hour:
            for (int i = 0; i < NUM_STEPPERS_H; i++)
            {
                h_steppers[i]->moveToMinSteps(data.position, data.dir, data.min_steps);
            }
            break;

        case selector_minute:
            for (int i = 0; i < NUM_STEPPERS_M; i++)
            {
                m_steppers[i]->moveToMinSteps(data.position, data.dir, data.min_steps);
            }
            break;

        default: // all the other stepper ids selecting individual steppers
            steppers[data.stepper_id]->moveToMinSteps(data.position, data.dir, data.min_steps);
            break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region Move Packet

MovePacket::MovePacket() : CommandPacket() {}

MovePacket::MovePacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength) : CommandPacket(buffer, bufferLength) 
{
    valid = parseData();
}

bool MovePacket::parseData()
{
    if (valid)
    {
        if (!(bufferLength == sizeof(data)))
            return false;

        memcpy(&data, buffer, sizeof(data));

        if (data.cmd_id != commandID)
            return false;
        if (!isStepperIDValid(data.stepper_id))
            return false;
        if (data.dir != 1 && data.dir != -1 && data.dir != 0)
            return false;

        return true;
    }
    return false;
}

bool MovePacket::executeCommand()
{
    if (valid)
    {
        switch (data.stepper_id)
        {
        case selector_all:
            for (int i = 0; i < NUM_STEPPERS; i++)
            {
                steppers[i]->moveTarget(data.distance * data.dir);
            }
            break;

        case selector_hour:
            for (int i = 0; i < NUM_STEPPERS_H; i++)
            {
                h_steppers[i]->moveTarget(data.distance * data.dir);
            }
            break;

        case selector_minute:
            for (int i = 0; i < NUM_STEPPERS_M; i++)
            {
                m_steppers[i]->moveTarget(data.distance * data.dir);
            }
            break;

        default: // all the other stepper ids selecting individual steppers
            steppers[data.stepper_id]->move(data.distance * data.dir);
            break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region Stop Packet

StopPacket::StopPacket() : CommandPacket() {}

StopPacket::StopPacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength) : CommandPacket(buffer, bufferLength) 
{
    valid = parseData();
}

bool StopPacket::parseData()
{
    if (valid)
    {
        if (!(bufferLength == sizeof(data)))
            return false;

        memcpy(&data, buffer, sizeof(data));

        if (data.cmd_id != commandID)
            return false;
        if (!isStepperIDValid(data.stepper_id))
            return false;

        return true;
    }
    return false;
}

bool StopPacket::executeCommand()
{
    if (valid)
    {
        switch (data.stepper_id)
        {
        case selector_all:
            for (int i = 0; i < NUM_STEPPERS; i++)
            {
                steppers[i]->stop();
            }
            break;

        case selector_hour:
            for (int i = 0; i < NUM_STEPPERS_H; i++)
            {
                h_steppers[i]->stop();
            }
            break;

        case selector_minute:
            for (int i = 0; i < NUM_STEPPERS_M; i++)
            {
                m_steppers[i]->stop();
            }
            break;

        default: // all the other stepper ids selecting individual steppers
            steppers[data.stepper_id]->stop();
            break;
        }
        return true;
    }
    return false;
}

#pragma endregion

#pragma region Wiggle Packet

WigglePacket::WigglePacket() : CommandPacket() {}

WigglePacket::WigglePacket(byte (&buffer)[MAX_COMMAND_LENGTH], uint8_t bufferLength) : CommandPacket(buffer, bufferLength) 
{
    valid = parseData();
}

bool WigglePacket::parseData()
{
    if (valid)
    {
        if (!(bufferLength == sizeof(data)))
            return false;

        memcpy(&data, buffer, sizeof(data));

        if (data.cmd_id != commandID)
            return false;
        if (!isStepperIDValid(data.stepper_id))
            return false;
        if (data.dir != 1 && data.dir != -1)
            return false;

        return true;
    }
    return false;
}

bool WigglePacket::executeCommand()
{
    if (valid)
    {
        switch (data.stepper_id)
        {
        case selector_all:
            for (int i = 0; i < NUM_STEPPERS; i++)
            {
                steppers[i]->wiggle(data.distance * data.dir);
            }
            break;

        case selector_hour:
            for (int i = 0; i < NUM_STEPPERS_H; i++)
            {
                h_steppers[i]->wiggle(data.distance * data.dir);
            }
            break;

        case selector_minute:
            for (int i = 0; i < NUM_STEPPERS_M; i++)
            {
                m_steppers[i]->wiggle(data.distance * data.dir);
            }
            break;

        default: // all the other stepper ids selecting individual steppers
            steppers[data.stepper_id]->wiggle(data.distance * data.dir);
            break;
        }
        return true;
    }
    return false;
}

#pragma endregion
