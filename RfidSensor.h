#ifndef RFID_SENSOR_H
#define RFID_SENSOR_H

/**
 * @file RfidSensor.h
 * @brief Declares the RfidSensor class.
 *
 * A concrete sensor class in the Modest IoT Nano-framework for handling RFID card detection.
 * Generates RFID_DETECTED_EVENT when a card is detected.
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

#include "Sensor.h"
#include <Arduino.h>

struct RfidData
{
    String rfidCode;
    String scanType;
    bool isValid;
};

class RfidSensor : public Sensor
{
private:
    unsigned long lastScan;
    unsigned long scanInterval;
    String availableCodes[10];
    int codeCount;
    RfidData lastDetection;

public:
    static const int RFID_DETECTED_EVENT_ID = 11; ///< Unique ID for RFID detection event.
    static const Event RFID_DETECTED_EVENT;       ///< Predefined event for RFID detection.

    /**
     * @brief Constructs an RFID sensor.
     * @param pin The GPIO pin for RFID sensor (if applicable).
     * @param scanInterval Interval between RFID scans in milliseconds.
     * @param eventHandler Optional handler to receive RFID events (default: nullptr).
     */
    RfidSensor(int pin, unsigned long scanInterval = 5000, EventHandler *eventHandler = nullptr);

    /**
     * @brief Adds an RFID code to the list of available codes for simulation.
     * @param code The RFID code to add.
     */
    void addRfidCode(const String &code);

    /**
     * @brief Simulates RFID scanning, randomly selecting from available codes.
     */
    void simulateScan();

    /**
     * @brief Gets the last detected RFID data.
     * @return RfidData structure with latest detection info.
     */
    RfidData getLastDetection() const;

    /**
     * @brief Updates the sensor, checking for new RFID detections.
     */
    void update();
};

#endif // RFID_SENSOR_H
