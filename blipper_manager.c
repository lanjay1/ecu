//blipper_manager.c
#include "blipper_manager.h"
#include "ecu_core.h"
#include "timer.h"

typedef enum {
    BLIPPER_STATE_IDLE,
    BLIPPER_STATE_ACTIVE
} BlipperState_t;

static uint32_t blipper_duration = 100;
static uint8_t blipper_intensity = 60;
static BlipperState_t current_blipper_state = BLIPPER_STATE_IDLE;
static uint32_t blip_activation_time = 0;

void Blipper_init(void) {
    // Init PWM or throttle actuator
}

void Blipper_update(uint16_t rpm, bool is_downshift) {
    switch (current_blipper_state) {
        case BLIPPER_STATE_IDLE:
            if (is_downshift) {
                // Asumsi ECU_set_throttle_percent adalah non-blocking (mengatur target untuk TBW)
                ECU_set_throttle_percent(blipper_intensity);
                blip_activation_time = HAL_GetTick(); // atau millis()
                current_blipper_state = BLIPPER_STATE_ACTIVE;
            }
            break;

        case BLIPPER_STATE_ACTIVE:
            if ((HAL_GetTick() - blip_activation_time) >= blipper_duration) {
                ECU_set_throttle_percent(0); // Kembalikan throttle ke idle atau sesuai permintaan pedal
                current_blipper_state = BLIPPER_STATE_IDLE;
            } else if (!is_downshift) {
                // Jika kondisi downshift tidak lagi terpenuhi (misalnya, kopling dilepas cepat)
                // Mungkin ingin segera membatalkan blip
                ECU_set_throttle_percent(0);
                current_blipper_state = BLIPPER_STATE_IDLE;
            }
            break;
    }
}

void Blipper_set_duration(uint32_t ms) {
    blipper_duration = ms;
}

void Blipper_set_intensity(uint8_t percent) {
    blipper_intensity = percent;
}
