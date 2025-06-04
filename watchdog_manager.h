//watchdog_manager.h
#pragma once
#include "stm32h7xx_hal.h"

typedef enum {
    WDG_IWDG,       // Independent Watchdog (basic timeout)
    WDG_WWDG,       // Window Watchdog (timeout within window)
    WDG_DUAL        // Kedua watchdog aktif
} WDG_Mode;

typedef struct {
    WDG_Mode mode;
    uint32_t timeout_ms;    // Timeout IWDG
    uint32_t window_ms;     // Window untuk WWDG
    bool is_initialized;
} WDG_Handle_t;

// Public API
void WDG_Init(WDG_Handle_t* hwdg);
void WDG_Refresh(void);
void WDG_Enable(bool enable);
void WDG_SetTimeout(uint32_t timeout_ms);
void WDG_ForceReset(void);
bool WDG_IsESP32Alive(void);