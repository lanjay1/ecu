//actuator_manager.h
#ifndef ACTUATOR_MANAGER_H
#define ACTUATOR_MANAGER_H

#include "ecu_types.h"

ECU_Status ActuatorManager_Init(void);
ECU_Status ActuatorManager_UpdateAll(ActuatorOutput_t* actuator_output);
ECU_Status ActuatorManager_SetFuelInjection(float injection_time);
ECU_Status ActuatorManager_SetIgnitionAdvance(float advance_degrees);
// Tambahkan fungsi untuk aktuator lainnya

#endif // ACTUATOR_MANAGER_H