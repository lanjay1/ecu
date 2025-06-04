//event_logger.h
#pragma once
#include <stdint.h>

#define LOG_MAX_ENTRIES  100

typedef enum {
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR,
    LOG_CRITICAL
} LogLevel_t;

typedef struct {
    uint32_t timestamp;
    LogLevel_t level;
    uint16_t code;
    char message[32];
} LogEntry_t;

typedef struct {
    LogEntry_t entries[LOG_MAX_ENTRIES];
    uint16_t head;
    uint16_t count;
} EventLogger_t;

void EventLogger_Init(EventLogger_t* logger);
void EventLogger_Add(EventLogger_t* logger, LogLevel_t level, uint16_t code, const char* message);
void EventLogger_DumpToFlash(EventLogger_t* logger);