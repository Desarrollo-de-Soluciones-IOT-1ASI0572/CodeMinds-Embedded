/**
 * @file TrackingDevice.cpp
 * @brief Implementation of main tracking device for Wokwi
 */

#include "TrackingDevice.h"
#include "ModestIoT.h"

// Configuración para Wokwi
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
#define API_KEY "secret-api-key"

// Endpoints para Wokwi (usando host.wokwi.internal)
#define ENDPOINT_IAM "http://host.wokwi.internal:5000/api/v1/register"
#define ENDPOINT_TRACKING "http://host.wokwi.internal:5000/api/v1/tracking"
#define ENDPOINT_SENSOR_SCAN "http://host.wokwi.internal:5000/api/v1/sensor-scans"

// Pins para Wokwi
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define RFID_PIN 21

TrackingDevice::TrackingDevice(const String &deviceId, const String &rfidCode)
    : deviceId(deviceId), assignedRFID(rfidCode), lastUpdate(0), updateInterval(1000)
{
    // Inicializar componentes
    gpsSensor = new GPSSensor(GPS_RX_PIN, GPS_TX_PIN, this);
    rfidSensor = new RFIDSensor(RFID_PIN, rfidCode, deviceId, this);

    commHandler = new CommunicationHandler(deviceId, WIFI_SSID, WIFI_PASSWORD,
                                           ENDPOINT_TRACKING, ENDPOINT_IAM,
                                           ENDPOINT_SENSOR_SCAN, API_KEY);
    currentRFIDCode = rfidCode;
}

void TrackingDevice::initialize()
{
    Serial.begin(9600);
    Serial.println("=== CodeMinds Tracking Device - Wokwi Edition ===");
    Serial.print("Device ID: ");
    Serial.println(deviceId);
    Serial.print("RFID Code: ");
    Serial.println(assignedRFID);
    Serial.println("Inicializando dispositivo...");

    // Inicializar semilla para números aleatorios
    randomSeed(analogRead(0));

    // Conectar WiFi
    Command connectCmd(CommandIds::CONNECT_WIFI);
    handle(connectCmd);

    if (commHandler->isWiFiConnected())
    {
        Command registerCmd(CommandIds::REGISTER_DEVICE);
        handle(registerCmd);
    }

    Serial.println("Dispositivo listo para simulación!");
    Serial.print("Device ID: ");
    Serial.println(commHandler->getDeviceId());
}

void TrackingDevice::on(Event event)
{
    switch (event.id)
    {
    case RFIDEventIds::RFID_DETECTED:
    {
        Serial.print("RFID detectado - Device: ");
        Serial.println(deviceId);

        RFIDData rfidData = rfidSensor->getLastDetection();
        if (rfidData.isValid && commHandler->isWiFiConnected() && commHandler->isRegistered())
        {
            currentRFIDCode = rfidData.rfidCode;

            // Enviar sensor scan
            bool scanSuccess = commHandler->sendSensorScan(rfidData);
            if (scanSuccess)
            {
                Serial.println("✓ Sensor scan enviado exitosamente");
            }
            else
            {
                Serial.println("✗ Error al enviar sensor scan");
            }
        }
        break;
    }
    case GPSEventIds::GPS_DATA_READY:
    {
        Serial.print("GPS data ready - Device: ");
        Serial.println(deviceId);

        if (!currentRFIDCode.isEmpty() && commHandler->isWiFiConnected() && commHandler->isRegistered())
        {
            GPSData gpsData = gpsSensor->getLastData();
            if (gpsData.isValid)
            {
                bool trackingSuccess = commHandler->sendGPSData(gpsData, currentRFIDCode);
                if (trackingSuccess)
                {
                    Serial.println("✓ Datos GPS enviados exitosamente");
                }
                else
                {
                    Serial.println("✗ Error al enviar datos GPS");
                }
            }
        }
        break;
    }
    }
}

void TrackingDevice::handle(Command command)
{
    commHandler->handle(command);
}

void TrackingDevice::update()
{
    if (millis() - lastUpdate >= updateInterval)
    {
        // Actualizar sensores
        rfidSensor->update();
        gpsSensor->update();

        // Verificar conexión
        commHandler->checkConnection();

        lastUpdate = millis();
    }
}

bool TrackingDevice::isConnected() const
{
    return commHandler->isWiFiConnected();
}

String TrackingDevice::getAssignedRFID() const
{
    return assignedRFID;
}

String TrackingDevice::getDeviceId() const
{
    return deviceId;
}

TrackingDevice::~TrackingDevice()
{
    delete gpsSensor;
    delete rfidSensor;
    delete commHandler;
}