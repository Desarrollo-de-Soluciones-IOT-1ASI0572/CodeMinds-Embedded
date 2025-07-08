/**
 * @file TrackingDevice.h
 * @brief Main tracking device class for Wokwi
 */

#ifndef TRACKING_DEVICE_H
#define TRACKING_DEVICE_H

#include "EventHandler.h"
#include "GPSSensor.h"
#include "RFIDSensor.h"
#include "CommunicationHandler.h"

class TrackingDevice : public EventHandler
{
private:
    GPSSensor *gpsSensor;
    RFIDSensor *rfidSensor;
    CommunicationHandler *commHandler;
    String deviceId;
    String assignedRFID;
    String currentRFIDCode;
    unsigned long lastUpdate;
    unsigned long updateInterval;

public:
    TrackingDevice(const String &deviceId, const String &rfidCode);

    void initialize();
    void on(Event event) override;
    void handle(Command command);
    void update();

    bool isConnected() const;
    String getMacAddress() const;
    String getAssignedRFID() const;
    String getDeviceId() const;

    ~TrackingDevice();
};

#endif // TRACKING_DEVICE_H