#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""

const char *rfid_codes[] = {
    "XX01X"};

const int total_codes = sizeof(rfid_codes) / sizeof(rfid_codes[0]);
const char *server_url = "http://host.wokwi.internal:5000/api/v1/sensor-scans/create";
unsigned long scanInterval = 5000;

void setup()
{
  Serial.begin(115200);
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