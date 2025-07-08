#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

/**
 * @file GPSSensor.h
 * @brief GPS sensor for Wokwi using NMEA data from custom chip
 *
 * Sensor GPS que procesa datos NMEA desde un chip personalizado de Wokwi,
 * extrayendo latitud, longitud y velocidad para el tracking device.
 *
 * @author CodeMinds (Wokwi Implementation)
 * @date July 07, 2025
 * @version 2.1
 */

#include "ModestIoT.h"

struct GPSData
{
    double latitude;
    double longitude;
    double speed;
    bool isValid;
    String timestamp;
};

class GPSEventIds
{
public:
    static const int GPS_DATA_READY = 1;
};

class GPSCommandIds
{
public:
    static const int SEND_GPS_DATA = 10;
};

class GPSSensor : public EventHandler
{
private:
    HardwareSerial *gpsSerial;
    GPSData lastData;
    EventHandler *deviceHandler;
    unsigned long lastUpdate;
    unsigned long updateInterval;
    String nmeaBuffer;

    // NMEA parsing methods
    bool parseNMEA(const String &nmea);
    bool parseGPGGA(const String &sentence);
    bool parseGPRMC(const String &sentence);
    double parseCoordinate(const String &coord, const String &direction);
    String getTimestamp();

public:
    GPSSensor(int rxPin, int txPin, EventHandler *handler);
    void update();
    GPSData getLastData() const;
    void on(Event event) override;
    ~GPSSensor();
};

#endif // GPS_SENSOR_H
