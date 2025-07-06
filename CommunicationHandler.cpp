/**
 * @file CommunicationHandler.cpp
 * @brief Implements the CommunicationHandler class.
 *
 * Manages WiFi connectivity and HTTP communication for data transmission
 * in the Modest IoT Nano-framework. Handles both GPS tracking and RFID scan data.
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

#include "CommunicationHandler.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <Arduino.h>

const Command CommunicationHandler::SEND_GPS_DATA_COMMAND = Command(SEND_GPS_DATA_COMMAND_ID);
const Command CommunicationHandler::SEND_RFID_DATA_COMMAND = Command(SEND_RFID_DATA_COMMAND_ID);
const Command CommunicationHandler::CONNECT_WIFI_COMMAND = Command(CONNECT_WIFI_COMMAND_ID);

CommunicationHandler::CommunicationHandler(const String &ssid, const String &password,
                                           const String &trackingUrl, const String &rfidUrl,
                                           const String &deviceId)
    : wifiSSID(ssid), wifiPassword(password), trackingEndpoint(trackingUrl),
      rfidEndpoint(rfidUrl), deviceId(deviceId), recordId(1), isConnected(false)
{
}

void CommunicationHandler::handle(Command command)
{
    if (command == CONNECT_WIFI_COMMAND)
    {
        connectToWiFi();
    }
    // Note: GPS and RFID data commands are handled externally with actual data
}

bool CommunicationHandler::connectToWiFi()
{
    Serial.print("Conectando a WiFi");
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\nConectado!");
        isConnected = true;
        return true;
    }
    else
    {
        Serial.println("\nFallo en conexión WiFi");
        isConnected = false;
        return false;
    }
}

bool CommunicationHandler::sendGpsData(const GpsData &gpsData)
{
    if (!isConnected || !gpsData.isValid)
    {
        return false;
    }

    HTTPClient httpClient;
    httpClient.begin(trackingEndpoint);
    httpClient.addHeader("Content-Type", "application/json");

    StaticJsonDocument<256> dataRecord;
    dataRecord["id"] = recordId++;
    dataRecord["device_id"] = deviceId;
    dataRecord["created_at"] = gpsData.timestamp;
    dataRecord["latitude"] = gpsData.latitude;
    dataRecord["longitude"] = gpsData.longitude;

    String jsonData;
    serializeJson(dataRecord, jsonData);

    int httpCode = httpClient.POST(jsonData);

    if (httpCode > 0)
    {
        Serial.print("GPS HTTP Code: ");
        Serial.println(httpCode);
        String response = httpClient.getString();
        Serial.println("GPS Response: " + response);
        httpClient.end();
        return true;
    }
    else
    {
        Serial.print("Error en GPS HTTP: ");
        Serial.println(httpCode);
        httpClient.end();
        return false;
    }
}

bool CommunicationHandler::sendRfidData(const RfidData &rfidData)
{
    if (!isConnected || !rfidData.isValid)
    {
        return false;
    }

    HTTPClient http;
    http.begin(rfidEndpoint);
    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<200> payload;
    payload["rfidCode"] = rfidData.rfidCode;
    payload["scanType"] = rfidData.scanType;

    String jsonData;
    serializeJson(payload, jsonData);

    int httpCode = http.POST(jsonData);

    if (httpCode > 0)
    {
        Serial.print("RFID HTTP Code: ");
        Serial.println(httpCode);
        String response = http.getString();
        Serial.println("RFID Response: " + response);
        http.end();
        return true;
    }
    else
    {
        Serial.print("Error en RFID HTTP: ");
        Serial.println(httpCode);
        http.end();
        return false;
    }
}

void CommunicationHandler::checkConnection()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("Reconectando WiFi...");
        WiFi.disconnect();
        isConnected = false;
        connectToWiFi();
    }
}

bool CommunicationHandler::isWiFiConnected() const
{
    return isConnected && (WiFi.status() == WL_CONNECTED);
}

String CommunicationHandler::getISO8601Time() const
{
    time_t now;
    struct tm timeinfo;
    char buf[30];
    time(&now);
    gmtime_r(&now, &timeinfo);
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return String(buf);
}
