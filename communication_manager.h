//communication_manager.h
#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include "ecu_types.h"
#include <stdint.h>
#include <stdbool.h>

// Timeout and retry configuration
#define CAN_TIMEOUT_MS          1000    // 1 second CAN timeout
#define CAN_RETRY_THRESHOLD     3       // 3 missed cycles before failure
#define ESP32_HEARTBEAT_TIMEOUT 500     // 500ms ESP32 timeout
#define COMM_BUFFER_SIZE        64

// CAN IDs for R15 V3
#define CAN_ID_ECU_DATA      0x201
#define CAN_ID_DASH          0x202
#define CAN_ID_DIAGNOSTIC    0x7DF

typedef enum {
    COMM_CAN_500K = 0,  // CAN 500kbps (R15 standard)
    COMM_UART_115K,
    COMM_SPI_8M
} CommProtocol_t;

typedef enum {
    COMM_DEVICE_CAN,
    COMM_DEVICE_ESP32,
    COMM_DEVICE_DASH
} CommDevice_t;

// Communication status structure
typedef struct {
    bool is_connected;
    uint32_t last_activity;
    uint8_t retry_count;
} CommStatus_t;

// Initialization and configuration
ECU_Status CommunicationManager_Init(CommProtocol_t protocol);

// Data transmission
ECU_Status CommunicationManager_SendData(uint8_t* data, uint16_t len);
ECU_Status CommunicationManager_ReceiveData(uint8_t* buffer, uint16_t len);

// CAN-specific functions
ECU_Status Comm_Send_CAN(uint32_t id, uint8_t* data, uint8_t len);
ECU_Status Comm_Receive_CAN(uint32_t* id, uint8_t* data, uint8_t* len);
ECU_Status Comm_Configure_CAN(uint32_t baudrate);

// Heartbeat and timeout management
void CommManager_UpdateActivity(CommDevice_t device);
void CommManager_HealthCheck(void);
void Comm_Timeout_Handler(CommDevice_t device);
void Comm_LostConnection(CommDevice_t device, ECU_Core_t* ecu);
void sendFirmwareToSTM32(const std::vector<uint8_t>& firmware);



// ESP32 specific functions
void Comm_ProcessESP32Heartbeat(void);
bool Comm_IsESP32Alive(void);

#endif // COMMUNICATION_MANAGER_H