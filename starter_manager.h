//starter_manager.h
#ifndef STARTER_MANAGER_H
#define STARTER_MANAGER_H

#include "ecu_type.h"
#include "ecu_config.h"
#include "ecu_state.h"

typedef struct {
    bool enabled;
    bool relay_output;         // status aktual output ke pin
    uint32_t last_trigger_ms;
    uint16_t timeout_ms;
} StarterManager_t;

void StarterManager_Init(StarterManager_t* sm);
void StarterManager_Update(StarterManager_t* sm, ECU_Core_t* ecu);
void StarterManager_SetOutput(bool on);  // manipulasi pin GPIO

#endif // STARTER_MANAGER_H
