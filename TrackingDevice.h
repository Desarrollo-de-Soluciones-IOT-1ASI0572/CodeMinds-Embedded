#ifndef TRACKING_DEVICE_H
#define TRACKING_DEVICE_H

/**
 * @file TrackingDevice.h
 * @brief Declares the TrackingDevice class.
 *
 * A complete IoT device in the Modest IoT Nano-framework that combines GPS tracking
 * and RFID scanning capabilities with network communication. Demonstrates the integration
 * of sensors, actuators, and communication in a single device.
 *
 * @author Angel Velasquez
 * @date March 22, 2025
 * @version 0.1
 */

/*
 * This file is part of the Modest IoT Nano-framework (C++ Edition).
 * Copyright (c) 2025 Angel Velasquez
 *
 * Licensed under the Creative Commons Attribution-NoDerivatives 4.0 International (CC BY-ND 4.0).
 * You may use, copy, and distribute this software in its original, unmodified form, provided
 * you give appropriate credit to the original author (Angel Velasquez) and include this notice.
 * Modifications, adaptations, or derivative works are not permitted.
 *
 * Full license text: https://creativecommons.org/licenses/by-nd/4.0/legalcode
 */

#include "Device.h"
#include "GpsSensor.h"
#include "RfidSensor.h"
#include "CommunicationHandler.h"
#include <Arduino.h>

class TrackingDevice : public Device
{
private:
    GpsSensor *gpsSensor;
    RfidSensor *rfidSensor;
    CommunicationHandler *commHandler;

    unsigned long lastUpdate;
    unsigned long updateInterval;

public:
    /**
     * @brief Constructs a TrackingDevice with all necessary components.
     * @param gpsRxPin GPS RX pin.
     * @param gpsTxPin GPS TX pin.
     * @param rfidPin RFID sensor pin.
     * @param wifiSSID WiFi network name.
     * @param wifiPassword WiFi password.
     * @param trackingUrl GPS tracking endpoint URL.
     * @param rfidUrl RFID scan endpoint URL.
     * @param deviceId Device identifier.
     */
    TrackingDevice(int gpsRxPin, int gpsTxPin, int rfidPin,
                   const String &wifiSSID, const String &wifiPassword,
                   const String &trackingUrl, const String &rfidUrl,
                   const String &deviceId);

    /**
     * @brief Handles events from sensors (GPS data, RFID detection).
     * @param event The event to process.
     */
    void on(Event event) override;

    /**
     * @brief Handles commands for device control.
     * @param command The command to execute.
     */
    void handle(Command command) override;

    /**
     * @brief Initializes the device and all its components.
     */
    void initialize();

    /**
     * @brief Updates all sensors and handles communication.
     * Should be called regularly in the main loop.
     */
    void update();

    /**
     * @brief Gets the GPS sensor instance.
     * @return Pointer to the GPS sensor.
     */
    GpsSensor *getGpsSensor() const;

    /**
     * @brief Gets the RFID sensor instance.
     * @return Pointer to the RFID sensor.
     */
    RfidSensor *getRfidSensor() const;

    /**
     * @brief Gets the communication handler instance.
     * @return Pointer to the communication handler.
     */
    CommunicationHandler *getCommunicationHandler() const;

    virtual ~TrackingDevice();
};

#endif // TRACKING_DEVICE_H
