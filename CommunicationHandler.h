#ifndef COMMUNICATION_HANDLER_H
#define COMMUNICATION_HANDLER_H

/**
 * @file CommunicationHandler.h
 * @brief Declares the CommunicationHandler class.
 *
 * Handles WiFi connection and HTTP communication in the Modest IoT Nano-framework.
 * Manages data transmission to remote servers for GPS tracking and RFID scans.
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

#include "CommandHandler.h"
#include "GpsSensor.h"
#include "RfidSensor.h"

class CommunicationHandler : public CommandHandler
{
private:
    String wifiSSID;
    String wifiPassword;
    String trackingEndpoint;
    String rfidEndpoint;
    String deviceId;
    int recordId;
    bool isConnected;

public:
    static const int SEND_GPS_DATA_COMMAND_ID = 20;  ///< Command to send GPS data.
    static const int SEND_RFID_DATA_COMMAND_ID = 21; ///< Command to send RFID data.
    static const int CONNECT_WIFI_COMMAND_ID = 22;   ///< Command to connect to WiFi.
    static const Command SEND_GPS_DATA_COMMAND;      ///< Predefined command for GPS transmission.
    static const Command SEND_RFID_DATA_COMMAND;     ///< Predefined command for RFID transmission.
    static const Command CONNECT_WIFI_COMMAND;       ///< Predefined command for WiFi connection.

    /**
     * @brief Constructs a CommunicationHandler.
     * @param ssid WiFi network name.
     * @param password WiFi password.
     * @param trackingUrl URL for GPS tracking endpoint.
     * @param rfidUrl URL for RFID scan endpoint.
     * @param deviceId Device identifier for tracking.
     */
    CommunicationHandler(const String &ssid, const String &password,
                         const String &trackingUrl, const String &rfidUrl,
                         const String &deviceId);

    /**
     * @brief Handles communication commands.
     * @param command The command to execute.
     */
    void handle(Command command) override;

    /**
     * @brief Connects to WiFi network.
     * @return True if connection successful, false otherwise.
     */
    bool connectToWiFi();

    /**
     * @brief Sends GPS data to tracking endpoint.
     * @param gpsData The GPS data to send.
     * @return True if successful, false otherwise.
     */
    bool sendGpsData(const GpsData &gpsData);

    /**
     * @brief Sends RFID data to scan endpoint.
     * @param rfidData The RFID data to send.
     * @return True if successful, false otherwise.
     */
    bool sendRfidData(const RfidData &rfidData);

    /**
     * @brief Checks WiFi connection status and reconnects if needed.
     */
    void checkConnection();

    /**
     * @brief Gets current WiFi connection status.
     * @return True if connected, false otherwise.
     */
    bool isWiFiConnected() const;

private:
    /**
     * @brief Generates ISO8601 timestamp.
     * @return Formatted timestamp string.
     */
    String getISO8601Time() const;
};

#endif // COMMUNICATION_HANDLER_H
