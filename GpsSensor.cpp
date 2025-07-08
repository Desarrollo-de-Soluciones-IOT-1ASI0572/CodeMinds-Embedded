/**
 * @file GPSSensor.cpp
 * @brief Implementation of GPS sensor for Wokwi using NMEA data
 */

#include "GPSSensor.h"
#include <WiFi.h>
#include <time.h>

GPSSensor::GPSSensor(int rxPin, int txPin, EventHandler *handler)
    : deviceHandler(handler), lastUpdate(0), updateInterval(2000)
{
    gpsSerial = new HardwareSerial(2);
    gpsSerial->begin(9600, SERIAL_8N1, rxPin, txPin);

    lastData.isValid = false;
    lastData.latitude = 0.0;
    lastData.longitude = 0.0;
    lastData.speed = 0.0;
    nmeaBuffer = "";

    Serial.println("GPS Sensor inicializado para Wokwi");
}

void GPSSensor::update()
{
    if (millis() - lastUpdate < updateInterval)
    {
        return;
    }

    // Leer datos NMEA del custom chip de Wokwi
    while (gpsSerial->available() > 0)
    {
        char c = gpsSerial->read();

        if (c == '\n')
        {
            // Procesar la línea NMEA completa
            if (parseNMEA(nmeaBuffer))
            {
                lastData.timestamp = getTimestamp();

                Serial.print("GPS - Latitud: ");
                Serial.print(lastData.latitude, 6);
                Serial.print(", Longitud: ");
                Serial.print(lastData.longitude, 6);
                Serial.print(", Velocidad: ");
                Serial.println(lastData.speed, 2);

                // Notificar al dispositivo principal
                Event gpsEvent(GPSEventIds::GPS_DATA_READY);
                deviceHandler->on(gpsEvent);

                lastUpdate = millis();
            }
            nmeaBuffer = "";
        }
        else if (c != '\r')
        {
            nmeaBuffer += c;
        }
    }
}

bool GPSSensor::parseNMEA(const String &nmea)
{
    if (nmea.startsWith("$GPGGA"))
    {
        return parseGPGGA(nmea);
    }
    else if (nmea.startsWith("$GPRMC"))
    {
        return parseGPRMC(nmea);
    }
    return false;
}

bool GPSSensor::parseGPGGA(const String &sentence)
{
    // Parsear $GPGGA,time,lat,N/S,lon,E/W,quality,numSat,hdop,alt,M,geoidal,M,dgpsAge,dgpsID*checksum
    int commaIndex[14];
    int commaCount = 0;

    // Encontrar todas las comas
    for (int i = 0; i < sentence.length() && commaCount < 14; i++)
    {
        if (sentence.charAt(i) == ',')
        {
            commaIndex[commaCount++] = i;
        }
    }

    if (commaCount < 6)
        return false;

    // Extraer latitud (campo 2)
    String latStr = sentence.substring(commaIndex[1] + 1, commaIndex[2]);
    String latDir = sentence.substring(commaIndex[2] + 1, commaIndex[3]);

    // Extraer longitud (campo 4)
    String lonStr = sentence.substring(commaIndex[3] + 1, commaIndex[4]);
    String lonDir = sentence.substring(commaIndex[4] + 1, commaIndex[5]);

    // Extraer quality (campo 6)
    String qualityStr = sentence.substring(commaIndex[5] + 1, commaIndex[6]);

    if (latStr.length() > 0 && lonStr.length() > 0 && qualityStr.toInt() > 0)
    {
        lastData.latitude = parseCoordinate(latStr, latDir);
        lastData.longitude = parseCoordinate(lonStr, lonDir);
        lastData.isValid = true;
        return true;
    }

    return false;
}

bool GPSSensor::parseGPRMC(const String &sentence)
{
    // Parsear $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,magVar,magVarDir*checksum
    int commaIndex[12];
    int commaCount = 0;

    for (int i = 0; i < sentence.length() && commaCount < 12; i++)
    {
        if (sentence.charAt(i) == ',')
        {
            commaIndex[commaCount++] = i;
        }
    }

    if (commaCount < 7)
        return false;

    // Extraer status (campo 2)
    String status = sentence.substring(commaIndex[1] + 1, commaIndex[2]);
    if (status != "A")
        return false; // A = Active, V = Void

    // Extraer velocidad (campo 7)
    String speedStr = sentence.substring(commaIndex[6] + 1, commaIndex[7]);
    if (speedStr.length() > 0)
    {
        lastData.speed = speedStr.toFloat() * 1.852; // Convertir nudos a km/h
        return true;
    }

    return false;
}

double GPSSensor::parseCoordinate(const String &coord, const String &direction)
{
    if (coord.length() < 4)
        return 0.0;

    // Formato: DDMM.MMMM o DDDMM.MMMM
    int dotIndex = coord.indexOf('.');
    if (dotIndex == -1)
        return 0.0;

    String degreeStr, minuteStr;

    if (dotIndex == 4)
    { // Latitud: DDMM.MMMM
        degreeStr = coord.substring(0, 2);
        minuteStr = coord.substring(2);
    }
    else if (dotIndex == 5)
    { // Longitud: DDDMM.MMMM
        degreeStr = coord.substring(0, 3);
        minuteStr = coord.substring(3);
    }
    else
    {
        return 0.0;
    }

    double degrees = degreeStr.toFloat();
    double minutes = minuteStr.toFloat();
    double result = degrees + (minutes / 60.0);

    if (direction == "S" || direction == "W")
    {
        result = -result;
    }

    return result;
}

String GPSSensor::getTimestamp()
{
    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);
    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", timeinfo);
    return String(buffer);
}

GPSData GPSSensor::getLastData() const
{
    return lastData;
}

void GPSSensor::on(Event event)
{
    // No procesa eventos directamente
}

GPSSensor::~GPSSensor()
{
    delete gpsSerial;
}
