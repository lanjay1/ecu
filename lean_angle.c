//lean_angle.c
#include "lean_angle.h"
#include "imu_driver.h" // MPU6050 or similar
#include "ecu_config.h"

#define LEAN_ANGLE_THRESHOLD 55
static bool lean_cut_active = false;

void LeanAngle_init(void) {
    imu_init();
}

void LeanAngle_update(void) {
    int16_t angle = imu_get_angle();
    lean_cut_active = (angle > LEAN_ANGLE_THRESHOLD);
}

bool LeanAngle_isCutActive(void) {
    return lean_cut_active;
}