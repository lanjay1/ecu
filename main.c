//main.c
#include "main.h"
#include "ecu_core.h"
#include "sensor_manager.h"
#include "actuator_manager.h"
#include "communication_manager.h"
#include "watchdog_manager.h"
#include "map_storage.h"
#include "ecu_tuning.h"
#include "starter_manager.h"
#include "idle_control.h"
#include "dbw_learning.h"

ECU_Core_t ecu;
IdleControl_t idle_control;
StarterManager_t starter_manager;
DBW_Learning_t dbw_learning;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();  // atau sesuai kebutuhan Anda
    MX_USART1_UART_Init();
    MX_SPI2_Init();  // jika pakai SPI ke ESP32
    MX_TIM1_Init();  // Timer injector, ign, dsb

    ECU_Init(&ecu);
    ADC_DMA_Init();
    SensorManager_Init();
    ActuatorManager_Init();
    CommunicationManager_Init();
    WatchdogManager_Init();
    IdleControl_Init(&idle_control);
    StarterManager_Init(&starter_manager);
    DBW_Learning_Init(&dbw_learning);

    // Load map dari flash (kalau ada)
    ECU_MapData_t map;
    if (MapStorage_Load(&map)) {
        MapStorage_ApplyToSystem(&map);
    } else {
        ECU_Tuning_LoadDefaultMaps();
        TPS_SetCalibration(0.5f, 4.5f);
        ECU_SetIdleTarget(1300.0f);
    }

    while (1) {
        ECU_Update(&ecu);                            // Pusat loop kontrol ECU
        IdleControl_Update(&idle_control, &ecu);     // Atur throttle idle jika DBW
        StarterManager_Update(&starter_manager, &ecu); // Aktifkan starter jika aman
        DBW_Learning_Update(&dbw_learning, TPS_GetRaw()); // Pantau posisi throttle saat belajar
        CommunicationManager_Update(&ecu);           // UART/SPI parsing command
        WatchdogManager_Kick();                      // Reset internal watchdog STM32
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        // 1ms update routine
        SensorManager_Update();
        ActuatorManager_Update();
    }
}

