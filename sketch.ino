#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
#define ENDPOINT_URL "http://host.wokwi.internal:5000/api/v1/tracking"
#define DEVICE_ID "HC2956"

// GPS Pins
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

unsigned long lastSend = 0;
int record_id = 1;

const char *rfid_codes[] = {
    "XX01X"};

const int total_codes = sizeof(rfid_codes) / sizeof(rfid_codes[0]);
const char *server_url = "http://host.wokwi.internal:5000/api/v1/sensor-scans/create";
unsigned long scanInterval = 5000;


void setup()
{
  Serial.begin(115200);
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  connectToWiFi();
  randomSeed(analogRead(0));
}

void loop()
{
  static unsigned long lastScanTime = 0;
  
  if (millis() - lastScanTime >= scanInterval)
  {
    lastScanTime = millis();
    processScan();
  }

  getCoordinates();

  if (WiFi.status() != WL_CONNECTED)
  {
    reconnectWiFi();
  }
}

void processScan()
{
  int index = random(0, total_codes);
  const char *rfid = rfid_codes[index];

  Serial.print("Enviando RFID: ");
  Serial.println(rfid);

  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    http.begin(server_url);
    http.addHeader("Content-Type", "application/json");

    // Construir payload correctamente
    StaticJsonDocument<200> payload;
    payload["rfidCode"] = rfid;
    payload["scanType"] = "ENTRY"; // Campo debe coincidir con el esperado por Flask

    String jsonData;
    serializeJson(payload, jsonData);

    int httpCode = http.POST(jsonData);

    if (httpCode > 0)
    {
      String response = http.getString();
      Serial.print("HTTP Code: ");
      Serial.println(httpCode);
      Serial.print("Response: ");
      Serial.println(response);
    }
    else
    {
      Serial.print("Error en HTTP: ");
      Serial.println(httpCode);
    }

    http.end();
  }
}

void connectToWiFi()
{
  Serial.print("Conectando a WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConectado!");
}

void reconnectWiFi()
{
  Serial.println("Reconectando WiFi...");
  WiFi.disconnect();
  connectToWiFi();
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

void getCoordinates() {
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
