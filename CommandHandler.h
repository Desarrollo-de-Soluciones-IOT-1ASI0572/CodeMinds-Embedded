#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

/**
 * @file CommandHandler.h
 * @brief Command handling system for ModestIoT Framework
 *
 * Define las estructuras y clases base para el manejo de comandos
 * en el framework ModestIoT.
 *
 * @author CodeMinds (Wokwi Implementation)
 * @date July 07, 2025
 * @version 2.1
 */

#include <Arduino.h>

// ================== COMMAND SYSTEM ==================
// Command Structure
struct Command
{
    int id;
    explicit Command(int commandId) : id(commandId) {}
    bool operator==(const Command &other) const { return id == other.id; }
};

// Command Handler Interface
class CommandHandler
{
public:
    virtual void handle(Command command) = 0;
    virtual ~CommandHandler() = default;
};

#endif // COMMAND_HANDLER_H
