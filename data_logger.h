//data_logger.h
#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "ecu_types.h"

#define LOG_BUFFER_SIZE 1024

typedef enum {
    LOG_MODE_CIRCULAR,
    LOG_MODE_TRIGGERED
} LogMode_t;

typedef struct {
    uint8_t buffer[LOG_BUFFER_SIZE];
    uint16_t write_index;
    bool logging_active;
    LogMode_t mode;
    uint32_t sample_rate; // Hz
} DataLogger_t;

ECU_Status DataLogger_Init(DataLogger_t* logger, LogMode_t mode, uint32_t sample_rate);
ECU_Status DataLogger_Start(DataLogger_t* logger);
ECU_Status DataLogger_Stop(DataLogger_t* logger);
ECU_Status DataLogger_LogData(DataLogger_t* logger, SensorData_t* sensors, ActuatorOutput_t* actuators);
ECU_Status DataLogger_ReadData(DataLogger_t* logger, uint8_t* output, uint16_t len);
ECU_Status DataLogger_Trigger(DataLogger_t* logger, bool condition);
ECU_Status DataLogger_WriteEntry(DataLogger_t* logger, const SensorData_t* sensors, const ActuatorOutput_t* actuators);

#endif // DATA_LOGGER_H