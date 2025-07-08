#ifndef EVENT_HANDLER_H
#define EVENT_HANDLER_H

/**
 * @file EventHandler.h
 * @brief Event handling system for ModestIoT Framework
 *
 * Define las estructuras y clases base para el manejo de eventos
 * en el framework ModestIoT.
 *
 * @author CodeMinds (Wokwi Implementation)
 * @date July 07, 2025
 * @version 2.1
 */

#include <Arduino.h>

// ================== EVENT SYSTEM ==================
// Event Structure
struct Event
{
    int id;
    explicit Event(int eventId) : id(eventId) {}
    bool operator==(const Event &other) const { return id == other.id; }
};

// Event Handler Interface
class EventHandler
{
public:
    virtual void on(Event event) = 0;
    virtual ~EventHandler() = default;
};

#endif // EVENT_HANDLER_H
