/**
 * @file RFIDSensor.h
 * @brief RFID sensor for Wokwi with configurable RFID code
 */

#ifndef RFID_SENSOR_H
#define RFID_SENSOR_H

#include "EventHandler.h"

struct RFIDData
{
    String rfidCode;
    bool isValid;
};

class RFIDEventIds
{
public:
    static const int RFID_DETECTED = 2;
};

class RFIDCommandIds
{
public:
    static const int SEND_RFID_DATA = 11;
};

class RFIDSensor : public EventHandler
{
private:
    RFIDData lastDetection;
    EventHandler *deviceHandler;
    unsigned long lastScan;
    unsigned long scanInterval;
    String assignedRFIDCode;  // Código RFID asignado a este dispositivo
    String deviceId;          // ID del dispositivo

public:
    RFIDSensor(int pin, const String &rfidCode, const String &deviceId, EventHandler *handler);
    void update();
    RFIDData getLastDetection() const;
    void on(Event event) override;
    void simulateCardDetection();
    String getAssignedRFID() const;
    String getDeviceId() const;
    ~RFIDSensor();
};

#endif // RFID_SENSOR_H