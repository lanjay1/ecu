//watchdog_manager.c
#include "watchdog_manager.h"
#include "stm32h7xx_hal_iwdg.h"
#include "data_logger.h"

#define ESP32_TIMEOUT_MS 500  // Timeout 500ms untuk ESP32

static uint32_t last_esp32_ping = 0;
static bool esp32_online = true;
static WDG_Handle_t wdg_handle;

void WDG_Init(WDG_Handle_t* hwdg) {
    if (hwdg->mode == WDG_IWDG || hwdg->mode == WDG_DUAL) {
        hiwdg.Instance = IWDG1;
        hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
        hiwdg.Init.Reload = (hwdg->timeout_ms * 32) / 256; // LSI ~32kHz
        HAL_IWDG_Init(&hiwdg);
    }

    if (hwdg->mode == WDG_WWDG || hwdg->mode == WDG_DUAL) {
        hwwdg.Instance = WWDG1;
        hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
        hwwdg.Init.Window = hwdg->window_ms;
        hwwdg.Init.Counter = hwdg->timeout_ms;
        HAL_WWDG_Init(&hwwdg);
    }
    if (hwdg->recovery_state != NULL) {
        memcpy(hwdg->recovery_state, &system_state, sizeof(ECU_State_t));
    }

    wdg_handle = *hwdg;
    wdg_handle.is_initialized = true;
}

void WDG_Refresh(void) {
    if (wdg_handle.mode == WDG_IWDG || wdg_handle.mode == WDG_DUAL) {
        HAL_IWDG_Refresh(&hiwdg);
    }
    if (wdg_handle.mode == WDG_WWDG || wdg_handle.mode == WDG_DUAL) {
        HAL_WWDG_Refresh(&hwwdg);
    }
}
void WDG_Enable(bool enable) {
    if (enable) {
        if (!wdg_handle.is_initialized) {
            WDG_Init(&wdg_handle);  // Aktifkan ulang jika belum aktif
        }
    } else {
        // STM32 IWDG tidak bisa dimatikan secara hardware
        // Kita bisa mematikan hanya secara logika
        wdg_handle.is_initialized = false;
    }
}

void WDG_SetTimeout(uint32_t timeout_ms) {
    wdg_handle.timeout_ms = timeout_ms;

    // Terapkan ulang timeout jika watchdog aktif
    if (wdg_handle.is_initialized) {
        WDG_Init(&wdg_handle);
    }
}
void WDG_ESP32_Sync(void) {
    // Implementasi nyata ping ke ESP32
    uint8_t ping_cmd = 0xAA;
    if (HAL_SPI_Transmit(&hspi2, &ping_cmd, 1, 100) == HAL_OK) {
        last_esp32_ping = HAL_GetTick();
    } else {
        DataLogger_LogWarning("ESP32 ping failed");
    }
}

bool WDG_IsESP32Alive(void) {
    return (HAL_GetTick() - last_esp32_ping) < ESP32_TIMEOUT_MS;
}

void WDG_ForceReset(void) {
    NVIC_SystemReset();
}
void ECU_CheckESP32Status(void) {
    if (!WDG_IsESP32Alive()) {
        if (esp32_online) {
            EventLogger_Add(EVENT_TYPE_WARNING, "ESP32 disconnected → Limp mode");
            esp32_online = false;
        }
        ECUState.quickshift_enabled = false;
        ECUState.backfire_enabled = false;
        ECUState.tuning_locked = true;
    } else {
        if (!esp32_online) {
            EventLogger_Add(EVENT_TYPE_INFO, "ESP32 reconnected");
            esp32_online = true;
            ECUState.tuning_locked = false;
        }
    }
}