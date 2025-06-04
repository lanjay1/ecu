//Ignition_hw.c
#include "stm32h7xx_hal.h"
#include "ignition_calch.h"
#include "timers.h"  // For timer definitions
#include "thresholds.h"
// Hardware Configuration
#define COIL_PIN         GPIO_PIN_5
#define COIL_PORT        GPIOA
#define MAX_DWELL_MS     4.0f    // Protection against coil overheating
#define MIN_DWELL_MS     1.0f    // Minimum required charge time
#define TIMER_CLOCK_HZ   84000000 // APB1 timer clock frequency

extern TIM_HandleTypeDef htim3; // Timer untuk advance (sinkron dengan engkol)
extern TIM_HandleTypeDef htim4; // Timer untuk dwell

// Static function prototypes
static void set_coil_dwell_timer(uint32_t dwell_us);

void Spark_Trigger(float advance_deg, float dwell_ms) 
{
    // Input validation
    dwell_ms = (dwell_ms > MAX_DWELL_MS) ? MAX_DWELL_MS : 
              (dwell_ms < MIN_DWELL_MS) ? MIN_DWELL_MS : dwell_ms;

    // Convert to microseconds for higher precision
    uint32_t dwell_us = (uint32_t)(dwell_ms * MS_TO_US);
    
    // 1. Set advance angle using timer
    HAL_Ignition_Delay(advance_deg);
    
    // 2. Charge coil with precise timing
    HAL_GPIO_WritePin(COIL_PORT, COIL_PIN, GPIO_PIN_SET);
    set_coil_dwell_timer(dwell_us);
    
    // 3. Spark firing happens in TIM3 IRQ handler
}

static void set_coil_dwell_timer(uint32_t dwell_us)
{
    // Configure TIM4 for dwell time control
    TIM_HandleTypeDef htim4 = {
        .Instance = TIM4,
        .Init = {
            .Prescaler = (TIMER_CLOCK_HZ/1000000) - 1, // 1MHz (1us/tick)
            .CounterMode = TIM_COUNTERMODE_UP,
            .Period = dwell_us,
            .ClockDivision = TIM_CLOCKDIVISION_DIV1
        }
    };
    
    HAL_TIM_Base_Init(&htim4);
    HAL_TIM_RegisterCallback(&htim4, HAL_TIM_PERIOD_ELAPSED_CB_ID, coil_dwell_elapsed);
    HAL_TIM_Base_Start_IT(&htim4);
}

// Timer interrupt callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        // End of dwell period - fire spark
        HAL_GPIO_WritePin(COIL_PORT, COIL_PIN, GPIO_PIN_RESET);
        HAL_TIM_Base_Stop_IT(htim);
    }
}

void HAL_Ignition_Delay(float degrees) 
{
    // Convert degrees to timer ticks (360° = full cycle)
    uint32_t ticks = (uint32_t)((degrees / 360.0f) * IGNITION_TIMER_PERIOD);
    
    // Update TIM3 compare register
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ticks);
    HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
}