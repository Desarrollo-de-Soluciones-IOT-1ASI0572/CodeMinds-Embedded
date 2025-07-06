/**
 * @file GpsSensor.cpp
 * @brief Implements the GpsSensor class (simulated NMEA).
 *
 * Simulates GPS data by injecting NMEA sentences from a static array.
 * Generates events when new valid location data is available.
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

#include "GpsSensor.h"
#include <Arduino.h>

const char* GpsSensor::nmeaSentences[] = {
    "$GPGGA,172914.049,2327.985,S,05150.410,W,1,12,1.0,0.0,M,0.0,M,,*60\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,172914.049,A,2327.985,S,05150.410,W,009.7,025.9,060622,000.0,W*74\r\n",
    "$GPGGA,172915.049,2327.982,S,05150.409,W,1,12,1.0,0.0,M,0.0,M,,*6E\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,172915.049,A,2327.982,S,05150.409,W,009.7,025.9,060622,000.0,W*7A\r\n",
    "$GPGGA,172916.049,2327.980,S,05150.408,W,1,12,1.0,0.0,M,0.0,M,,*6E\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,172916.049,A,2327.980,S,05150.408,W,009.7,025.9,060622,000.0,W*7A\r\n"
    // Puedes agregar más frases si lo deseas
};
const int GpsSensor::nmeaCount = sizeof(GpsSensor::nmeaSentences) / sizeof(GpsSensor::nmeaSentences[0]);
const Event GpsSensor::GPS_DATA_EVENT = Event(GPS_DATA_EVENT_ID);

GpsSensor::GpsSensor(int rxPin, int txPin, unsigned long updateInterval, EventHandler *eventHandler)
    : Sensor(-1, eventHandler), updateInterval(updateInterval), lastUpdate(0), nmeaIndex(0)
{
    lastValidData.isValid = false;
}

void GpsSensor::update()
{
    // Simula la llegada de una frase NMEA cada intervalo
    if (millis() - lastUpdate > updateInterval)
    {
        const char* nmea = nmeaSentences[nmeaIndex];
        for (size_t i = 0; i < strlen(nmea); ++i) {
            gps.encode(nmea[i]);
        }
        nmeaIndex = (nmeaIndex + 1) % nmeaCount;

        if (gps.location.isValid())
        {
            lastValidData.latitude = gps.location.lat();
            lastValidData.longitude = gps.location.lng();
            lastValidData.isValid = true;

            // Generate timestamp
            time_t now = time(nullptr);
            struct tm timeinfo;
            char buf[30];
            gmtime_r(&now, &timeinfo);
            strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
            lastValidData.timestamp = String(buf);

            lastUpdate = millis();

            // Trigger GPS data event
            on(GPS_DATA_EVENT);
        }
    }
}

GpsData GpsSensor::getLastData() const
{
    return lastValidData;
}

bool GpsSensor::hasValidFix() const
{
    return lastValidData.isValid && gps.location.isValid();
}