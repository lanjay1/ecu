//data_logger.c
#include "data_logger.h"
#include <string.h>  // memcpy

ECU_Status DataLogger_Init(DataLogger_t* logger, LogMode_t mode, uint32_t sample_rate) {
    if (!logger) return ECU_ERROR;
    logger->write_index = 0;
    logger->logging_active = false;
    logger->mode = mode;
    logger->sample_rate = sample_rate;
    memset(logger->buffer, 0, LOG_BUFFER_SIZE);
    return ECU_OK;
}

ECU_Status DataLogger_Start(DataLogger_t* logger) {
    if (!logger) return ECU_ERROR;
    logger->logging_active = true;
    return ECU_OK;
}

ECU_Status DataLogger_Stop(DataLogger_t* logger) {
    if (!logger) return ECU_ERROR;
    logger->logging_active = false;
    return ECU_OK;
}

ECU_Status DataLogger_LogData(DataLogger_t* logger, SensorData_t* sensors, ActuatorOutput_t* actuators) {
    if (!logger || !logger->logging_active) return ECU_ERROR;

    uint16_t data_len = sizeof(SensorData_t) + sizeof(ActuatorOutput_t);
    
    if (data_len > LOG_BUFFER_SIZE) {
        // logger->logging_active = false; // Matikan logging jika data terlalu besar
        return ECU_ERROR; // Data terlalu besar untuk buffer
    }

    // Cek overflow dulu
    if (logger->write_index + data_len > LOG_BUFFER_SIZE) {
        if (logger->mode == LOG_MODE_CIRCULAR) {
            logger->write_index = 0;  // Wrap kembali ke awal
        } else {
            logger->logging_active = false; // Optional: otomatis stop jika penuh
            return ECU_ERROR;
        }
    }

    // Copy data ke buffer log
    memcpy(&logger->buffer[logger->write_index], sensors, sizeof(SensorData_t));
    logger->write_index += sizeof(SensorData_t);

    memcpy(&logger->buffer[logger->write_index], actuators, sizeof(ActuatorOutput_t));
    logger->write_index += sizeof(ActuatorOutput_t);

    return ECU_OK;
}

ECU_Status DataLogger_ReadData(DataLogger_t* logger, uint8_t* output, uint16_t len) {
    if (!logger || !output) return ECU_ERROR;

    uint16_t read_len = (len < logger->write_index) ? len : logger->write_index;
    memcpy(output, logger->buffer, read_len);

    return ECU_OK;
}

ECU_Status DataLogger_Trigger(DataLogger_t* logger, bool condition) {
    if (!logger) return ECU_ERROR;
    if (logger->mode == LOG_MODE_TRIGGERED && condition) {
        logger->logging_active = true;
    }
    return ECU_OK;
}
ECU_Status DataLogger_WriteEntry(DataLogger_t* logger, const SensorData_t* sensors, const ActuatorOutput_t* actuators) {
    return DataLogger_LogData(logger, sensors, actuators);
}
