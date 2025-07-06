/**
 * @file RfidSensor.cpp
 * @brief Implements the RfidSensor class.
 *
 * Simulates RFID card detection and event generation in the Modest IoT Nano-framework.
 * Randomly selects from predefined RFID codes to simulate real-world behavior.
 *
 * @author Angel Velasquez
 * @date March 22, 2025
 * @version 0.1
 */

/*
 * This file is part of the Modest IoT Nano-framework (C++ Edition).
 * Copyright (c) 2025 Angel Velasquez
 *
 * Licensed under the Creative Commons Attribution-NoDerivatives 4.0 International (CC BY-ND 4.0).
 * You may use, copy, and distribute this software in its original, unmodified form, provided
 * you give appropriate credit to the original author (Angel Velasquez) and include this notice.
 * Modifications, adaptations, or derivative works are not permitted.
 *
 * Full license text: https://creativecommons.org/licenses/by-nd/4.0/legalcode
 */

#include "RfidSensor.h"
#include <Arduino.h>

const Event RfidSensor::RFID_DETECTED_EVENT = Event(RFID_DETECTED_EVENT_ID);

RfidSensor::RfidSensor(int pin, unsigned long scanInterval, EventHandler *eventHandler)
    : Sensor(pin, eventHandler), scanInterval(scanInterval), lastScan(0), codeCount(0)
{
    lastDetection.isValid = false;
}

void RfidSensor::addRfidCode(const String &code)
{
    if (codeCount < 10)
    {
        availableCodes[codeCount] = code;
        codeCount++;
    }
}

void RfidSensor::simulateScan()
{
    if (codeCount > 0)
    {
        int index = random(0, codeCount);
        lastDetection.rfidCode = availableCodes[index];
        lastDetection.scanType = "ENTRY";
        lastDetection.isValid = true;

        lastScan = millis();

        // Trigger RFID detection event
        on(RFID_DETECTED_EVENT);
    }
}

void RfidSensor::update()
{
    if (millis() - lastScan >= scanInterval)
    {
        simulateScan();
    }
}

RfidData RfidSensor::getLastDetection() const
{
    return lastDetection;
}
