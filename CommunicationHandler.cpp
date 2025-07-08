/**
 * @file CommunicationHandler.cpp
 * @brief Implementation of Communication Handler for Wokwi
 */

#include "CommunicationHandler.h"
#include "ModestIoT.h"
#include "GPSSensor.h"
#include "RFIDSensor.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

CommunicationHandler::CommunicationHandler(const String &deviceId, const String &ssid, const String &password,
                                           const String &trackingUrl, const String &iamUrl,
                                           const String &sensorUrl, const String &key)
    : deviceId(deviceId), wifiSSID(ssid), wifiPassword(password), trackingEndpoint(trackingUrl),
      iamEndpoint(iamUrl), sensorScanEndpoint(sensorUrl), apiKey(key),
      isConnected(false), isDeviceRegistered(false)
{
    // Obtener MAC address para referencia
    WiFi.mode(WIFI_MODE_STA);
    
    Serial.print("Device ID: ");
    Serial.println(deviceId);
}

void CommunicationHandler::handle(Command command)
{
    switch (command.id)
    {
    case CommandIds::CONNECT_WIFI:
        connectToWiFi();
        break;
    case CommandIds::REGISTER_DEVICE:
        registerDevice();
        break;
    default:
        // Otros comandos no manejados aquí
        break;
    }
}

bool CommunicationHandler::connectToWiFi()
{
    Serial.println("Conectando a WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID, wifiPassword);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\nWiFi conectado!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());

        // Sincronizar tiempo
        configTime(0, 0, "pool.ntp.org", "time.nist.gov");
        int timeAttempts = 0;
        while (time(nullptr) < 8 * 3600 * 2 && timeAttempts < 10)
        {
            delay(500);
            Serial.print(".");
            timeAttempts++;
        }
        Serial.println("\nHora sincronizada.");

        isConnected = true;
        return true;
    }
    else
    {
        Serial.println("\nError: No se pudo conectar a WiFi");
        return false;
    }
}

bool CommunicationHandler::registerDevice()
{
    if (!isConnected)
    {
        Serial.println("Error: WiFi no conectado para registrar dispositivo");
        return false;
    }

    Serial.println("Registrando dispositivo...");

    StaticJsonDocument<128> registerDoc;
    registerDoc["device_id"] = deviceId; 

    String payload;
    serializeJson(registerDoc, payload);

    HTTPClient httpClient;
    httpClient.begin(iamEndpoint);
    httpClient.addHeader("Content-Type", "application/json");

    int responseCode = httpClient.POST(payload);
    String response = httpClient.getString();

    Serial.print("Respuesta registro dispositivo: ");
    Serial.println(responseCode);
    Serial.println(response);

    if (responseCode == 201 || responseCode == 409)
    {
        isDeviceRegistered = true;

        // Extraer API key de la respuesta si es necesario
        if (responseCode == 201)
        {
            StaticJsonDocument<256> responseDoc;
            deserializeJson(responseDoc, response);
            if (responseDoc.containsKey("api_key"))
            {
                apiKey = responseDoc["api_key"].as<String>();
                Serial.print("API Key obtenida: ");
                Serial.println(apiKey);
            }
        }

        Serial.println("Dispositivo registrado exitosamente");
        httpClient.end();
        return true;
    }
    else
    {
        Serial.println("Error al registrar dispositivo");
        httpClient.end();
        return false;
    }
}

bool CommunicationHandler::sendSensorScan(const RFIDData &rfidData)
{
    if (!isConnected || !isDeviceRegistered)
    {
        Serial.println("Error: Dispositivo no conectado o no registrado");
        return false;
    }

    StaticJsonDocument<256> scanDoc;
    scanDoc["device_id"] = deviceId;
    scanDoc["rfid_code"] = rfidData.rfidCode;

    String payload;
    serializeJson(scanDoc, payload);

    HTTPClient httpClient;
    httpClient.begin(sensorScanEndpoint);
    httpClient.addHeader("Content-Type", "application/json");
    httpClient.addHeader("X-API-Key", apiKey);

    int responseCode = httpClient.POST(payload);
    String response = httpClient.getString();

    Serial.print("Respuesta POST Sensor Scan: ");
    Serial.println(responseCode);
    Serial.println(response);

    httpClient.end();
    return responseCode == 200;
}

bool CommunicationHandler::sendGPSData(const GPSData &gpsData, const String &rfidCode)
{
    if (!isConnected || !isDeviceRegistered)
    {
        Serial.println("Error: Dispositivo no conectado o no registrado");
        return false;
    }

    StaticJsonDocument<256> dataRecord;
    dataRecord["device_id"] = deviceId;
    dataRecord["rfid_code"] = rfidCode;
    dataRecord["latitude"] = gpsData.latitude;
    dataRecord["longitude"] = gpsData.longitude;
    dataRecord["speed"] = gpsData.speed;

    String payload;
    serializeJson(dataRecord, payload);

    HTTPClient httpClient;
    httpClient.begin(trackingEndpoint);
    httpClient.addHeader("Content-Type", "application/json");
    httpClient.addHeader("X-API-Key", apiKey);

    int responseCode = httpClient.POST(payload);
    String response = httpClient.getString();

    Serial.print("Respuesta POST GPS: ");
    Serial.println(responseCode);
    Serial.println(response);

    httpClient.end();
    return responseCode == 200;
}

bool CommunicationHandler::isWiFiConnected() const
{
    return isConnected && WiFi.status() == WL_CONNECTED;
}

bool CommunicationHandler::isRegistered() const
{
    return isDeviceRegistered;
}

String CommunicationHandler::getDeviceId() const
{
    return deviceId;
}

void CommunicationHandler::checkConnection()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        isConnected = false;
        Serial.println("WiFi desconectado, reintentando...");
        connectToWiFi();
    }
}

CommunicationHandler::~CommunicationHandler()
{
    // No hay recursos dinámicos que liberar
}