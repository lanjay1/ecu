//event_logger.c
#include "event_logger.h"
#include "stm32h7xx_hal.h"

void EventLogger_Init(EventLogger_t* logger) {
    logger->head = 0;
    logger->count = 0;
}

void EventLogger_Add(EventLogger_t* logger, LogLevel_t level, uint16_t code, const char* message) {
    LogEntry_t entry = {
        .timestamp = HAL_GetTick(),
        .level = level,
        .code = code,
        .message = {0}
    };
    strncpy(entry.message, message, sizeof(entry.message) - 1);

    logger->entries[logger->head] = entry;
    logger->head = (logger->head + 1) % LOG_MAX_ENTRIES;
    if (logger->count < LOG_MAX_ENTRIES) logger->count++;
}
void ECU_LogEvents(void) {
    if (SensorData.RPM > ECUSettings.rev_limit) {
        EventLogger_Add(EVENT_TYPE_WARNING, "RPM limit exceeded");
    }
    if (SensorData.CLT > ECUSettings.clt_max) {
        EventLogger_Add(EVENT_TYPE_ERROR, "Coolant temp overheat");
    }
    if (ECUState.launch_active) {
        EventLogger_Add(EVENT_TYPE_INFO, "Launch control active");
    }
    if (ECUState.qs_cut_active) {
        EventLogger_Add(EVENT_TYPE_INFO, "Quickshift triggered");
    }
}
