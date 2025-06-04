//vva_manager.c
#include "vva_manager.h"
#include "ecu_config.h"
#include "gpio_driver.h" // assumed GPIO interface

#define VVA_ACTIVATION_RPM 6500
#define VVA_THROTTLE_THRESHOLD 40

static bool vva_state = false;

void VVA_init(void) {
    gpio_set_output(GPIO_VVA);
    gpio_write(GPIO_VVA, false);
}

void VVA_update(uint16_t rpm, uint16_t throttle) {
    bool should_enable = (rpm >= VVA_ACTIVATION_RPM && throttle > VVA_THROTTLE_THRESHOLD);
    if (should_enable != vva_state) {
        VVA_setState(should_enable);
    }
    if (!vva_active && rpm >= VVA_ON_RPM && tps >= VVA_ON_TPS) {
    vva_active = true;
    VVA_setState(true);
} else if (vva_active && (rpm <= VVA_OFF_RPM || tps <= VVA_OFF_TPS)) {
    vva_active = false;
    VVA_setState(false);
}

}

void VVA_setState(bool enable) {
    vva_state = enable;
    gpio_write(GPIO_VVA, enable);
}

bool VVA_getState(void) {
    return vva_state;
}