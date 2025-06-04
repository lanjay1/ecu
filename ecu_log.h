//ecu_log.h
#ifndef ECU_LOG_H
#define ECU_LOG_H

#include <stdint.h>

void ECU_Log(const char* format, ...);
void ECU_TriggerWarning(uint16_t code);

#endif
