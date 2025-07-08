/**
 * @file RFIDSensor.cpp
 * @brief Implementation of RFID sensor for Wokwi with configurable RFID code
 */

#include "RFIDSensor.h"

RFIDSensor::RFIDSensor(int pin, const String &rfidCode, const String &deviceId, EventHandler *handler)
    : deviceHandler(handler), lastScan(0), scanInterval(5000), 
      assignedRFIDCode(rfidCode), deviceId(deviceId)
{
    lastDetection.isValid = false;
    lastDetection.rfidCode = "";

    Serial.println("RFID Sensor inicializado para Wokwi");
    Serial.print("Device ID: ");
    Serial.println(deviceId);
    Serial.print("Código RFID asignado: ");
    Serial.println(assignedRFIDCode);
}

void RFIDSensor::update()
{
    if (millis() - lastScan >= scanInterval)
    {
        simulateCardDetection();
        lastScan = millis();
    }
}

void RFIDSensor::simulateCardDetection()
{
    // Simular detección del RFID asignado a este dispositivo (30% de probabilidad)
    if (random(100) < 30)
    {
        // Usar el código RFID asignado específicamente a este dispositivo
        lastDetection.rfidCode = assignedRFIDCode;
        lastDetection.isValid = true;

        Serial.print("RFID detectado - Device: ");
        Serial.print(deviceId);
        Serial.print(" | UID: ");
        Serial.println(lastDetection.rfidCode);

        // Notificar al dispositivo principal
        Event rfidEvent(RFIDEventIds::RFID_DETECTED);
        deviceHandler->on(rfidEvent);
    }
}

RFIDData RFIDSensor::getLastDetection() const
{
    return lastDetection;
}

String RFIDSensor::getAssignedRFID() const
{
    return assignedRFIDCode;
}

String RFIDSensor::getDeviceId() const
{
    return deviceId;
}

void RFIDSensor::on(Event event)
{
    // No procesa eventos directamente
}

RFIDSensor::~RFIDSensor()
{
    // No hay recursos dinámicos que liberar
}