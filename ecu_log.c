// ecu_log.c
#include "ecu_log.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "event_logger.h"  // Jika ingin log ke sistem
#include "data_logger.h"   // Opsional, jika ingin log data juga
// Ganti ini sesuai sistem Anda
extern EventLogger_t ecu_logger;

// Fungsi log teks bebas
void ECU_Log(const char* format, ...) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Output ke terminal/debug
    printf("[LOG] %s\n", buffer);

    // Simpan sebagai event log juga
    EventLogger_Add(&ecu_logger, LOG_INFO, 0, buffer);
}

// Fungsi warning — bisa untuk duty overflow, sensor fail, dsb
void ECU_TriggerWarning(uint16_t code) {
    char msg[64];
    snprintf(msg, sizeof(msg), "Warning code: %u", code);

    printf("[WARNING] %s\n", msg);

    // Catat ke event logger
    EventLogger_Add(&ecu_logger, LOG_WARNING, code, msg);
}
