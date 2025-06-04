//starter_manager.c
#include "starter_manager.h"
#include "utils.h"      // HAL_GetTick
#include "stm32h7xx_hal.h"  // Pastikan HAL_GPIO_xx tersedia
// GPIO konfigurasi sesuai board Anda
#define STARTER_RELAY_GPIO_PORT   GPIOB
#define STARTER_RELAY_PIN         GPIO_PIN_5

void StarterManager_SetOutput(bool on) {
    if (on) {
        HAL_GPIO_WritePin(STARTER_RELAY_GPIO_PORT, STARTER_RELAY_PIN, GPIO_PIN_RESET);  // Aktif low
    } else {
        HAL_GPIO_WritePin(STARTER_RELAY_GPIO_PORT, STARTER_RELAY_PIN, GPIO_PIN_SET);
    }
}

void StarterManager_Init(StarterManager_t* sm) {
    if (!sm) return;
    sm->enabled = true;
    sm->relay_output = false;
    sm->last_trigger_ms = 0;
    sm->timeout_ms = 3000;

    StarterManager_SetOutput(false);  // Matikan starter saat init
}

void StarterManager_Update(StarterManager_t* sm, ECU_Core_t* ecu) {
    if (!sm || !ecu || !sm->enabled) return;
    uint32_t now = HAL_GetTick();

    // ⛔️ Delay lockout antar percobaan
    static uint32_t last_start_time = 0;
    if (now - last_start_time < 3000) {
        return;  // Starter diblok selama 3 detik antar percobaan
    }
    bool start_conditions_met = 
        (sensors->rpm < 300) &&
        (sensors->throttle_position < 5.0f) &&
        (sensors->clutch_switch || sensors->neutral_switch);

    if (start_conditions_met) {
        HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_SET);
        last_start_time = now;  // ⏱️ Update waktu percobaan
        EventLogger_Log("Starter: Active");
    } else {
        HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_RESET);
    }

    bool clutch_in = ecu->sensors.clutch_switch;
    bool neutral   = ecu->sensors.neutral_switch;
    bool running   = ecu->state == ECU_STATE_RUNNING;
    float rpm      = ecu->sensors.rpm;
    float tps      = ecu->sensors.tps;

    bool start_allowed = (clutch_in || neutral) && !running && rpm < 300 && tps < 5.0f;
    
    if (ecu->sensors.rpm > 500 && ecu->starter.active) {
    Starter_Disable();
}
    if (start_allowed) {
        StarterManager_SetOutput(true);
        sm->relay_output = true;
        sm->last_trigger_ms = HAL_GetTick();
    }

    // Matikan jika timeout
    if (sm->relay_output && (HAL_GetTick() - sm->last_trigger_ms > sm->timeout_ms)) {
        StarterManager_SetOutput(false);
        sm->relay_output = false;
    }

    // Tambahan keamanan: matikan kalau mesin hidup
    if (running && sm->relay_output) {
        StarterManager_SetOutput(false);
        sm->relay_output = false;
    }
    if (HAL_GPIO_ReadPin(START_BUTTON_GPIO, START_BUTTON_PIN) == GPIO_PIN_SET) {
    if (ecu->state.sensors.rpm > 500) {
        // Jika tombol starter masih ditekan padahal mesin hidup, jangan aktifkan ulang
        ecu->starter.locked = true;
    }
}

}
