#ifndef COMMUNICATION_HANDLER_H
#define COMMUNICATION_HANDLER_H

/**
 * @file CommunicationHandler.h
 * @brief Handles WiFi communication and HTTP requests for Wokwi
 */

#include "CommandHandler.h"
#include "GPSSensor.h"
#include "RFIDSensor.h"

class CommunicationHandler : public CommandHandler
{
private:
    String wifiSSID;
    String wifiPassword;
    String trackingEndpoint;
    String iamEndpoint;
    String sensorScanEndpoint;
    String apiKey;
    String deviceId; 
    bool isConnected;
    bool isDeviceRegistered;

public:
    CommunicationHandler(const String &deviceId, const String &ssid, const String &password,
                         const String &trackingUrl, const String &iamUrl,
                         const String &sensorUrl, const String &key);

    void handle(Command command) override;
    bool connectToWiFi();
    bool registerDevice();
    bool sendSensorScan(const RFIDData &rfidData);
    bool sendGPSData(const GPSData &gpsData, const String &rfidCode);
    bool isWiFiConnected() const;
    bool isRegistered() const;
    String getMacAddress() const;
    String getDeviceId() const;
    void checkConnection();
    ~CommunicationHandler();
};

#endif // COMMUNICATION_HANDLER_H