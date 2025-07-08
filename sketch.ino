/**
 * @file sketch.ino
 * @brief Simulación de 10 dispositivos EduGo
 */

#include "TrackingDevice.h"

// 10 dispositivos con sus códigos RFID específicos
TrackingDevice* devices[10];

void setup() {
    Serial.begin(9600);
    Serial.println("Inicializando 10 dispositivos EduGo...");
    
    // Crear los 10 dispositivos con solo deviceId y rfidCode
    devices[0] = new TrackingDevice("EduGo-Device-01", "A3B7C921");
    devices[1] = new TrackingDevice("EduGo-Device-02", "4F2D8E6A");
    devices[2] = new TrackingDevice("EduGo-Device-03", "D1943FA2");
    devices[3] = new TrackingDevice("EduGo-Device-04", "7C3A1B8F");
    devices[4] = new TrackingDevice("EduGo-Device-05", "E50864D0");
    devices[5] = new TrackingDevice("EduGo-Device-06", "6A92FE13");
    devices[6] = new TrackingDevice("EduGo-Device-07", "09BC71E4");
    devices[7] = new TrackingDevice("EduGo-Device-08", "3F84A65C");
    devices[8] = new TrackingDevice("EduGo-Device-09", "B1E7D93A");
    devices[9] = new TrackingDevice("EduGo-Device-10", "527CAB89");
    
    // Inicializar todos los dispositivos
    for (int i = 0; i < 10; i++) {
        devices[i]->initialize();
        delay(1000);
    }
    
    Serial.println("Dispositivos inicializados");
}

void loop() {
    // Actualizar todos los dispositivos
    for (int i = 0; i < 10; i++) {
        devices[i]->update();
    }
    
    delay(2000);
}