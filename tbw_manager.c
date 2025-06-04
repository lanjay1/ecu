//tbw_manager.c
#include "tbw_manager.h"
#include "thresholds.h"

// Static variables
static void (*error_callback)(TBW_Error_t) = NULL;

static void Default_Error_Callback(TBW_Error_t error) {
    // Basic error handling - can be overridden by TBW_SetErrorCallback()
}

ECU_Status TBW_Calibrate(TBW_Handle_t* h, float min, float max) {
    if(!h) return ECU_PARAM_ERROR;
    if(max <= min) return ECU_PARAM_ERROR;
    
    h->tps_min_voltage = min;
    h->tps_max_voltage = max;
    h->is_calibrated = true;
    
    return ECU_OK;
}

void TBW_Init(TBW_Handle_t* h, TIM_HandleTypeDef* htim, ADC_HandleTypeDef* hadc) {
    if(!h || !htim || !hadc) return;
    
    h->tim = htim;
    h->adc = hadc;
    h->is_calibrated = false;
    h->error_callback = Default_Error_Callback;
    
    // PWM Setup (1000-2000μs pulse width)
    htim->Instance->ARR = PWM_CYCLE_US; // 50Hz frequency
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    
    // ADC Setup (12-bit, 3.3V)
    HAL_ADC_Start_DMA(hadc, (uint32_t*)&tps_raw, 1);
}

void TBW_SetErrorCallback(void (*callback)(TBW_Error_t)) {
    error_callback = callback ? callback : Default_Error_Callback;
}

void TBW_Update(TBW_Handle_t* h, float tps_input, TBW_Mode mode) {
    // 1. Validate inputs
    if(!h || !h->is_calibrated) {
        if(h && h->tim) h->tim->Instance->CCR1 = TBW_MIN_PULSE_US;
        if(error_callback) error_callback(TBW_ERROR_SENSOR);
        return;
    }

    // 2. Clamp input (0.0 - 1.0)
    tps_input = CLAMP(tps_input, 0.0f, 1.0f);

    // 3. Calculate output based on mode
    float throttle_output;
    switch(mode) {
        case TBW_MODE_SPORT:
            throttle_output = tps_input * (1.8f - TPS_THRESHOLD_BACKFIRE * tps_input);
            break;
            
        case TBW_MODE_SPORT_PLUS:
            throttle_output = powf(tps_input, 0.6f);
            break;
            
        default: // TBW_MODE_COMFORT
            throttle_output = tps_input;
            
            if(mode > TBW_MODE_SPORT_PLUS && error_callback) {
                error_callback(TBW_ERROR_INVALID_MODE);
            }
    }

    // 4. Convert to pulse width (1000-2000μs)
    uint16_t pulse = TBW_MIN_PULSE_US + 
                   (uint16_t)(throttle_output * (TBW_MAX_PULSE_US - TBW_MIN_PULSE_US));
    
    // Overflow protection
    if(pulse > TBW_MAX_PULSE_US) {
        pulse = TBW_MAX_PULSE_US;
        if(error_callback) error_callback(TBW_ERROR_OVERFLOW);
    }
    
    // 5. Apply to hardware
    __HAL_TIM_SET_COMPARE(h->tim, TIM_CHANNEL_1, pulse);
}

float TBW_GetTPS(TBW_Handle_t* h) {
    if(!h || !h->is_calibrated) return 0.0f;
    
    // Read and normalize ADC value (0.0-1.0)
    float voltage = (float)tps_raw * 3.3f / 4095.0f;
    return CLAMP((voltage - h->tps_min_voltage) / 
                (h->tps_max_voltage - h->tps_min_voltage), 0.0f, 1.0f);
}