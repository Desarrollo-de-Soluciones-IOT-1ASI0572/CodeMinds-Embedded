/**
 * @file sketch.ino
 * @brief Main Arduino sketch using ModestIoT framework for GPS tracking and RFID scanning.
 *
 * This sketch demonstrates object-oriented programming with the ModestIoT framework,
 * implementing a complete tracking device with GPS and RFID capabilities using
 * events and commands for clean separation of concerns.
 *
 * @author Angel Velasquez (ModestIoT Framework)
 * @date March 22, 2025
 * @version 0.1
 */

#include "ModestIoT.h"

// Pin definitions
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define RFID_PIN 21

// Network configuration
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
#define DEVICE_ID "HC2956"

// Server endpoints
#define TRACKING_ENDPOINT "http://host.wokwi.internal:5000/api/v1/tracking"
#define RFID_ENDPOINT "http://host.wokwi.internal:5000/api/v1/sensor-scans/create"

// Global tracking device instance
TrackingDevice *trackingDevice;

void setup()
{
  Serial.begin(115200);
  Serial.println("=== ModestIoT Tracking Device ===");

  // Initialize random seed
  randomSeed(analogRead(0));

  // Create tracking device with all configuration
  trackingDevice = new TrackingDevice(
      GPS_RX_PIN, GPS_TX_PIN, // GPS pins
      RFID_PIN,               // RFID pin  
      WIFI_SSID,              // WiFi credentials
      WIFI_PASSWORD,
      TRACKING_ENDPOINT, // Server endpoints
      RFID_ENDPOINT,
      DEVICE_ID // Device identifier
  );

  // Initialize the device
  trackingDevice->initialize();

  Serial.println("=== Setup Complete ===");
}

void loop()
{
  // Update the tracking device (handles all sensors and communication)
  trackingDevice->update();

  // Small delay to prevent overwhelming the system
  delay(100);
}
