//adc_manager.h
#pragma once
#include "stm32h7xx_hal.h"

#define ADC_MAX_CHANNELS  6

typedef struct {
    ADC_HandleTypeDef* hadc;
    uint16_t dma_buffer[ADC_MAX_CHANNELS];
    uint8_t num_channels;
    bool is_calibrated;
} ADC_Manager_t;

void ADC_Init(ADC_Manager_t* adc, ADC_HandleTypeDef* hadc, uint8_t num_channels);
void ADC_Start(ADC_Manager_t* adc);
float ADC_ReadVoltage(ADC_Manager_t* adc, uint8_t channel);