//adc_manager.c
#include "adc_manager.h"
#define ADC_CHANNEL_COUNT 5
#define ADC_VREF 3.3f
#define ADC_RESOLUTION_MAX_VALUE 4095.0f // Untuk 12-bit ADC

void ADC_Init(ADC_Manager_t* adc, ADC_HandleTypeDef* hadc, uint8_t num_channels) {
    adc->hadc = hadc;
    adc->num_channels = (num_channels > ADC_MAX_CHANNELS) ? ADC_MAX_CHANNELS : num_channels;
    // adc->is_calibrated = false; // Dihapus dari sini, akan di-set berdasarkan hasil kalibrasi

    if (HAL_ADCEx_Calibration_Start(adc->hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        adc->is_calibrated = false;
        // Tambahkan penanganan error jika kalibrasi gagal, misal log error
    } else {
        adc->is_calibrated = true; // Set true jika kalibrasi berhasil
    }
}

void ADC_Start(ADC_Manager_t* adc) {
    HAL_ADC_Start_DMA(adc->hadc, (uint32_t*)adc->dma_buffer, adc->num_channels);
}

float ADC_ReadVoltage(ADC_Manager_t* adc, uint8_t channel) {
    if (channel >= adc->num_channels) return 0.0f;
    return (adc->dma_buffer[channel] * ADC_VREF) / ADC_RESOLUTION_MAX_VALUE;
}