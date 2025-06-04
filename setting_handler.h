//settings_handler.h
#ifndef SETTINGS_HANDLER_H
#define SETTINGS_HANDLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Handle_Settings_Command(uint8_t cmd, const char* json);

#ifdef __cplusplus
}
#endif

#endif // SETTINGS_HANDLER_H
