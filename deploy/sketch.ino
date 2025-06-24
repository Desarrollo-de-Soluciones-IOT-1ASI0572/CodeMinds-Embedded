#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <SPI.h>
#include <MFRC522.h>

// WiFi Credentials
#define WIFI_SSID "Wifi123"
#define WIFI_PASSWORD "ola12345"

#define ENDPOINT_URL "http://192.168.140.159:5001/api/v1/tracking"
#define RFID_URL "https://671b0977acf9aa94f6ac5d8e.mockapi.io/api/v1/attendance"

// Device Identification
#define DEVICE_ID "HC2956"

// GPS Pins
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// RFID Pins
#define RST_PIN 22
#define SS_PIN 5

TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

unsigned long lastSend = 0;
int record_id = 1;


void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado. Intentando reconectar...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, 6);
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
      delay(500);
      Serial.print(".");
      retries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi reconectado!");
    } else {
      Serial.println("\nNo se pudo reconectar a WiFi.");
    }
  }
}

void setupNTP() {
  // Configurar NTP
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("Configurando tiempo NTP...");
  
  // Esperar hasta que se sincronice el tiempo
  time_t now = 0;
  int timeout = 0;
  while (now < 1000000000 && timeout < 20) { // Esperar max 20 segundos
    time(&now);
    delay(1000);
    timeout++;
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Tiempo NTP configurado!");
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
    delay(10);
  }
  Serial.println("ESP32 Serial Monitor funcionando!");
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  connectToWiFi();
  setupNTP();

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

  void connectToWiFi()
  {
    WiFi.mode(WIFI_STA);
    Serial.print("Conectando a WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }

    Serial.println("\nConectado!");
  }

  void loop()
  {
    checkWiFiConnection();

    getCoordinates();
    if (WiFi.status() != WL_CONNECTED)
    {
      checkWiFiConnection();
    }

  }

  void getCoordinates() {
    // Leer todos los datos disponibles del GPS
    while (GPSSerial.available() > 0) {
      char c = GPSSerial.read();
      if (gps.encode(c)) {
        // Debug: mostrar información del GPS
        Serial.print("Satellites: ");
        Serial.println(gps.satellites.value());
        Serial.print("HDOP: ");
        Serial.println(gps.hdop.hdop());
      }
    }

    // Verificar si tenemos una ubicación válida y ha pasado suficiente tiempo
    if (gps.location.isValid() && gps.location.isUpdated() && 
        gps.satellites.value() >= 4 && millis() - lastSend > 5000) {
      
      double lat = gps.location.lat();
      double lng = gps.location.lng();

      Serial.println("=== GPS DATA ===");
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());
      Serial.print("HDOP: ");
      Serial.println(gps.hdop.hdop());
      Serial.print("Latitud: ");
      Serial.println(lat, 6);
      Serial.print("Longitud: ");
      Serial.println(lng, 6);
      Serial.print("Altitude: ");
      Serial.println(gps.altitude.meters());
      Serial.print("Speed: ");
      Serial.println(gps.speed.kmph());
      Serial.println("===============");

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
    } else if (gps.location.isValid()) {
      // GPS tiene datos pero no cumple condiciones para enviar
      Serial.print("GPS válido pero no enviando - Satellites: ");
      Serial.print(gps.satellites.value());
      Serial.print(", Time since last send: ");
      Serial.println(millis() - lastSend);
    } else {
      // GPS no tiene datos válidos
      static unsigned long lastDebug = 0;
      if (millis() - lastDebug > 10000) { // Debug cada 10 segundos
        Serial.println("Esperando señal GPS...");
        Serial.print("Caracteres procesados: ");
        Serial.println(gps.charsProcessed());
        Serial.print("Sentences with fix: ");
        Serial.println(gps.sentencesWithFix());
        Serial.print("Failed checksum: ");
        Serial.println(gps.failedChecksum());
        lastDebug = millis();
      }
    }
  }
