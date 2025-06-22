#include "ModestIoT.h"
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

// WiFi Credentials
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""

#define ENDPOINT_URL "http://host.wokwi.internal:5000/api/v1/tracking"

// Device Identification
#define DEVICE_ID "HC2956"

// GPS Pins
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

unsigned long lastSend = 0;
int record_id = 1;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
    delay(10);
  }
  Serial.println("ESP32 Serial Monitor funcionando!");
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, 6);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");
}

String getISO8601Time()
{
  time_t now;
  struct tm timeinfo;
  char buf[30];
  time(&now);
  gmtime_r(&now, &timeinfo);
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buf);
}

void loop()
{
  while (GPSSerial.available() > 0)
  {
    char c = GPSSerial.read();
    gps.encode(c);
  }

  if (gps.location.isUpdated() && millis() - lastSend > 10000)
  {
    double lat = gps.location.lat();
    double lng = gps.location.lng();

    Serial.print("Latitud: ");
    Serial.println(lat, 6);
    Serial.print("Longitud: ");
    Serial.println(lng, 6);

    StaticJsonDocument<256> dataRecord;
    dataRecord["id"] = record_id++;
    dataRecord["device_id"] = DEVICE_ID;
    dataRecord["created_at"] = getISO8601Time();
    dataRecord["latitude"] = lat;
    dataRecord["longitude"] = lng;

    String dataRecordResource;
    serializeJson(dataRecord, dataRecordResource);

    HTTPClient httpClient;
    httpClient.begin(ENDPOINT_URL);
    httpClient.addHeader("Content-Type", "application/json");
    int httpResponseCode = httpClient.POST(dataRecordResource);

    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);

    String responseResource = httpClient.getString();
    Serial.println("Respuesta del servidor:");
    Serial.println(responseResource);

    httpClient.end();

    lastSend = millis();
  }
}