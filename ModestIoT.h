#ifndef MODEST_IOT_H
#define MODEST_IOT_H

/**
 * @file ModestIoT.h
 * @brief ModestIoT Framework - Core framework includes
 *
 * Framework orientado a objetos para dispositivos IoT con arquitectura
 * basada en eventos y comandos (CQRS-inspired)
 *
 * @author CodeMinds (Adapted for Wokwi simulation)
 * @date July 07, 2025
 * @version 2.1
 */

#include <Arduino.h>
#include "EventHandler.h"
#include "CommandHandler.h"

// ================== GENERAL COMMAND IDS ==================
class CommandIds
{
public:
    static const int CONNECT_WIFI = 12;
    static const int REGISTER_DEVICE = 13;
};

#endif // MODEST_IOT_H
