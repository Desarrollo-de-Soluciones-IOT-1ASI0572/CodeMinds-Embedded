/**
 * @file sketch.ino
 * @brief ESP32 Tracking Device using ModestIoT Framework
 *
 * Este sketch implementa un dispositivo de rastreo utilizando programación orientada a objetos
 * con el framework ModestIoT. Combina lectura RFID y GPS para enviar datos a un servidor.
 *
 * @author CodeMinds
 * @date July 05, 2025
 * @version 2.0
 */

#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <SPI.h>
#include <MFRC522.h>

// ================== FRAMEWORK MODEST IOT ==================
// Event Handler Framework
struct Event
{
  int id;
  explicit Event(int eventId) : id(eventId) {}
  bool operator==(const Event &other) const { return id == other.id; }
};

class EventHandler
{
public:
  virtual void on(Event event) = 0;
  virtual ~EventHandler() = default;
};

// Command Handler Framework
struct Command
{
  int id;
  explicit Command(int commandId) : id(commandId) {}
  bool operator==(const Command &other) const { return id == other.id; }
};

class CommandHandler
{
public:
  virtual void handle(Command command) = 0;
  virtual ~CommandHandler() = default;
};

// ================== CONFIGURACIÓN ==================
// WiFi Credentials
#define WIFI_SSID "Wifi123"
#define WIFI_PASSWORD "ola12345"
#define API_KEY "secret-api-key"

// Endpoints
#define ENDPOINT_IAM "http://192.168.120.118:5000/api/v1/register"
#define ENDPOINT_TRACKING "http://192.168.120.118:5000/api/v1/tracking"

// GPS Pins
#define GPS_RX_PIN 16 // RX2 (GPIO16)
#define GPS_TX_PIN 17 // TX2 (GPIO17)

// RFID Pins
#define RST_PIN 22 // Pin RST personalizable (ej. GPIO 22)
#define SS_PIN 21  // Pin SS (SDA) personalizable (ej. GPIO 21)

// ================== ESTRUCTURAS DE DATOS ==================
struct GPSData
{
  double latitude;
  double longitude;
  bool isValid;
  String timestamp;
};

struct RFIDData
{
  String rfidCode;
  bool isValid;
};

// ================== EVENTOS Y COMANDOS ==================
class EventIds
{
public:
  static const int GPS_DATA_READY = 1;
  static const int RFID_DETECTED = 2;
  static const int DATA_SENT = 3;
};

class CommandIds
{
public:
  static const int SEND_GPS_DATA = 10;
  static const int SEND_RFID_DATA = 11;
  static const int CONNECT_WIFI = 12;
};

// ================== DECLARACIÓN DE CLASES ==================

// Forward declarations
class RealGPSSensor;
class RealRFIDSensor;
class CommunicationHandler;
class EduGoTrackingDevice;

// ================== CLASE SENSOR GPS ==================
class RealGPSSensor : public EventHandler
{
private:
  TinyGPSPlus gps;
  HardwareSerial *gpsSerial;
  GPSData lastData;
  EventHandler *deviceHandler;

public:
  RealGPSSensor(int rxPin, int txPin, EventHandler *handler);
  void update();
  GPSData getLastData() const;
  void on(Event event) override;
  ~RealGPSSensor();
};

// ================== SENSOR RFID ==================
class RealRFIDSensor : public EventHandler
{
private:
  MFRC522 *rfidReader;
  RFIDData lastDetection;
  EventHandler *deviceHandler;

public:
  RealRFIDSensor(int ssPin, int rstPin, EventHandler *handler);
  void update();
  RFIDData getLastDetection() const;
  void on(Event event) override;
  ~RealRFIDSensor();
};

// ================== MANEJADOR DE COMUNICACIÓN ==================
class CommunicationHandler : public CommandHandler
{
private:
  String wifiSSID;
  String wifiPassword;
  String trackingEndpoint;
  String rfidEndpoint;
  String apiKey;
  bool isConnected;

public:
  CommunicationHandler(const String &ssid, const String &password,
                       const String &trackingUrl, const String &rfidUrl,
                       const String &key);
  void handle(Command command) override;
  bool connectToWiFi();
  bool sendRFIDData(const RFIDData &rfidData);
  bool sendGPSData(const GPSData &gpsData, const String &deviceId);
  bool isWiFiConnected() const;
  void checkConnection();
};

// ================== DISPOSITIVO PRINCIPAL ==================
class EduGoTrackingDevice : public EventHandler, public CommandHandler
{
private:
  RealGPSSensor *gpsSensor;
  RealRFIDSensor *rfidSensor;
  CommunicationHandler *commHandler;
  String currentDeviceId;
  unsigned long lastUpdate;
  unsigned long updateInterval;

public:
  EduGoTrackingDevice();
  void initialize();
  void on(Event event) override;
  void handle(Command command) override;
  void update();
  ~EduGoTrackingDevice();
};

// ================== IMPLEMENTACIÓN DE MÉTODOS ==================

// Implementación RealGpsSensor
RealGPSSensor::RealGPSSensor(int rxPin, int txPin, EventHandler *handler) : deviceHandler(handler)
{
  gpsSerial = new HardwareSerial(2);
  gpsSerial->begin(9600, SERIAL_8N1, rxPin, txPin);
}

void RealGPSSensor::update()
{
  while (gpsSerial->available() > 0)
  {
    if (gps.encode(gpsSerial->read()) && gps.location.isUpdated())
    {
      lastData.latitude = gps.location.lat();
      lastData.longitude = gps.location.lng();
      lastData.isValid = gps.location.isValid();

      if (lastData.isValid)
      {
        Serial.print("GPS - Latitud: ");
        Serial.print(lastData.latitude, 6);
        Serial.print(", Longitud: ");
        Serial.println(lastData.longitude, 6);

        // Notificar al dispositivo principal
        Event gpsEvent(EventIds::GPS_DATA_READY);
        deviceHandler->on(gpsEvent);
      }
    }
  }
}

GPSData RealGPSSensor::getLastData() const
{
  return lastData;
}

void RealGPSSensor::on(Event event)
{
  // No procesa eventos directamente
}

RealGPSSensor::~RealGPSSensor()
{
  delete gpsSerial;
}

// Implementación RealRfidSensor
RealRFIDSensor::RealRFIDSensor(int ssPin, int rstPin, EventHandler *handler) : deviceHandler(handler)
{
  rfidReader = new MFRC522(ssPin, rstPin);
  SPI.begin();
  rfidReader->PCD_Init();
  Serial.println("RFID Reader inicializado");
}

void RealRFIDSensor::update()
{
  if (!rfidReader->PICC_IsNewCardPresent() || !rfidReader->PICC_ReadCardSerial())
  {
    return;
  }

  String uid = "";
  for (byte i = 0; i < rfidReader->uid.size; i++)
  {
    if (rfidReader->uid.uidByte[i] < 0x10)
      uid += "0";
    uid += String(rfidReader->uid.uidByte[i], HEX);
  }
  uid.toUpperCase();

  lastDetection.rfidCode = uid;
  lastDetection.isValid = true;

  Serial.print("RFID detectado - UID: ");
  Serial.println(uid);

  // Notificar al dispositivo principal
  Event rfidEvent(EventIds::RFID_DETECTED);
  deviceHandler->on(rfidEvent);

  rfidReader->PICC_HaltA();
  rfidReader->PCD_StopCrypto1();
}

RFIDData RealRFIDSensor::getLastDetection() const
{
  return lastDetection;
}

void RealRFIDSensor::on(Event event)
{
  // No procesa eventos directamente
}

RealRFIDSensor::~RealRFIDSensor()
{
  delete rfidReader;
}

// Implementación RealCommunicationHandler
CommunicationHandler::CommunicationHandler(const String &ssid, const String &password,
                                           const String &trackingUrl, const String &rfidUrl,
                                           const String &key)
    : wifiSSID(ssid), wifiPassword(password), trackingEndpoint(trackingUrl),
      rfidEndpoint(rfidUrl), apiKey(key), isConnected(false) {}

void CommunicationHandler::handle(Command command)
{
  switch (command.id)
  {
  case CommandIds::CONNECT_WIFI:
    connectToWiFi();
    break;
  case CommandIds::SEND_GPS_DATA:
    // Comando manejado externamente
    break;
  case CommandIds::SEND_RFID_DATA:
    // Comando manejado externamente
    break;
  }
}

bool CommunicationHandler::connectToWiFi()
{
  Serial.println("Conectando a WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID, wifiPassword);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Sincronizar tiempo
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  while (time(nullptr) < 8 * 3600 * 2)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nHora sincronizada.");

  isConnected = true;
  return true;
}

bool CommunicationHandler::sendRFIDData(const RFIDData &rfidData)
{
  if (!isConnected)
    return false;

  StaticJsonDocument<256> rfidRecord;
  rfidRecord["rfid_code"] = rfidData.rfidCode;

  String payload;
  serializeJson(rfidRecord, payload);

  HTTPClient httpClient;
  httpClient.begin(rfidEndpoint);
  httpClient.addHeader("Content-Type", "application/json");

  int responseCode = httpClient.POST(payload);
  String response = httpClient.getString();

  Serial.print("Respuesta POST RFID: ");
  Serial.println(responseCode);
  Serial.println(response);

  httpClient.end();
  return responseCode == 200;
}

bool CommunicationHandler::sendGPSData(const GPSData &gpsData, const String &deviceId)
{
  if (!isConnected)
    return false;

  StaticJsonDocument<256> dataRecord;
  dataRecord["device_id"] = deviceId;
  dataRecord["latitude"] = gpsData.latitude;
  dataRecord["longitude"] = gpsData.longitude;

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

void CommunicationHandler::checkConnection()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    isConnected = false;
    Serial.println("WiFi desconectado, reintentando...");
    connectToWiFi();
  }
}

// Implementación EduGoTrackingDevice
EduGoTrackingDevice::EduGoTrackingDevice() : lastUpdate(0), updateInterval(1000)
{
  // Inicializar componentes
  gpsSensor = new RealGPSSensor(GPS_RX_PIN, GPS_TX_PIN, this);
  rfidSensor = new RealRFIDSensor(SS_PIN, RST_PIN, this);
  commHandler = new CommunicationHandler(WIFI_SSID, WIFI_PASSWORD,
                                         ENDPOINT_TRACKING, ENDPOINT_IAM, API_KEY);
  currentDeviceId = "";
}

void EduGoTrackingDevice::initialize()
{
  Serial.begin(9600);
  Serial.println("=== EduGo Tracking Device v2.0 ===");
  Serial.println("Inicializando dispositivo con ModestIoT Framework...");

  // Conectar WiFi
  Command connectCmd(CommandIds::CONNECT_WIFI);
  handle(connectCmd);

  Serial.println("Dispositivo listo!");
}

void EduGoTrackingDevice::on(Event event)
{
  switch (event.id)
  {
  case EventIds::RFID_DETECTED:
  {
    Serial.println("Evento: RFID detectado");
    RFIDData rfidData = rfidSensor->getLastDetection();
    if (rfidData.isValid && commHandler->isWiFiConnected())
    {
      currentDeviceId = rfidData.rfidCode; // Usar RFID como device ID
      commHandler->sendRFIDData(rfidData);
    }
    break;
  }
  case EventIds::GPS_DATA_READY:
  {
    Serial.println("Evento: Datos GPS listos");
    if (!currentDeviceId.isEmpty())
    {
      GPSData gpsData = gpsSensor->getLastData();
      if (gpsData.isValid && commHandler->isWiFiConnected())
      {
        commHandler->sendGPSData(gpsData, currentDeviceId);
      }
    }
    break;
  }
  }
}

void EduGoTrackingDevice::handle(Command command)
{
  commHandler->handle(command);
}

void EduGoTrackingDevice::update()
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

EduGoTrackingDevice::~EduGoTrackingDevice()
{
  delete gpsSensor;
  delete rfidSensor;
  delete commHandler;
}

// ================== INSTANCIA GLOBAL ==================
EduGoTrackingDevice *trackingDevice;

// ================== SETUP Y LOOP ==================
void setup()
{
  trackingDevice = new EduGoTrackingDevice();
  trackingDevice->initialize();
}

void loop()
{
  trackingDevice->update();
}
