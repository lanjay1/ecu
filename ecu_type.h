//ecu_type.h
#ifndef ECU_TYPES_H
#define ECU_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ECU_OK = 0,
    ECU_ERROR,
    ECU_PARAM_ERROR,
    ECU_NOT_READY
} ECU_Status;

typedef struct {
    float rpm;
    float throttle_position;
    float map_sensor;
    float coolant_temp;
    float intake_air_temp;
    float oxygen_sensor;
    float battery_voltage;
    // Tambahkan sensor lainnya sesuai kebutuhan
} SensorData_t;

typedef struct {
    float ignition_advance;
    float fuel_injection;
    float idle_control;
    bool fan_control;
    // Tambahkan aktuator lainnya sesuai kebutuhan
} ActuatorOutput_t;

#endif // ECU_TYPES_H