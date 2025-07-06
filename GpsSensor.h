#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

/**
 * @file GpsSensor.h
 * @brief Declares the GpsSensor class.
 *
 * A concrete sensor class in the Modest IoT Nano-framework for handling GPS data.
 * Generates GPS_DATA_EVENT when new location data is available.
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
#include <TinyGPSPlus.h>
#include <Arduino.h>

struct GpsData
{
    double latitude;
    double longitude;
    bool isValid;
    String timestamp;
};

class GpsSensor : public Sensor
{
private:
    TinyGPSPlus gps;
    GpsData lastValidData;
    unsigned long lastUpdate;
    unsigned long updateInterval;
    int nmeaIndex;

    static const char* nmeaSentences[];
    static const int nmeaCount;

public:
    static const int GPS_DATA_EVENT_ID = 10; ///< Unique ID for GPS data event.
    static const Event GPS_DATA_EVENT;       ///< Predefined event for GPS data updates.

    /**
     * @brief Constructs a GPS sensor (simulated).
     * @param rxPin Ignored in simulation.
     * @param txPin Ignored in simulation.
     * @param updateInterval Minimum interval between GPS updates in milliseconds.
     * @param eventHandler Optional handler to receive GPS events (default: nullptr).
     */
    GpsSensor(int rxPin, int txPin, unsigned long updateInterval = 10000, EventHandler *eventHandler = nullptr);

    /**
     * @brief Simulates GPS data, triggers events when new data is available.
     */
    void update();

    /**
     * @brief Gets the last valid GPS data.
     * @return GpsData structure with latest coordinates and validity status.
     */
    GpsData getLastData() const;

    /**
     * @brief Checks if GPS has valid location data.
     * @return True if GPS has valid fix, false otherwise.
     */
    bool hasValidFix() const;
};

#endif // GPS_SENSOR_H