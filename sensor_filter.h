//sensor_filter.h
#pragma once

typedef enum {
    FILTER_NONE,        // No filter
    FILTER_MOVING_AVG,  // Moving average
    FILTER_LOWPASS,     // Exponential low-pass
    FILTER_MEDIAN       // Median filter
} FilterType_t;

typedef struct {
    FilterType_t type;
    float alpha;        // Untuk low-pass filter (0.1 - 0.9)
    uint8_t window_size;// Untuk moving average/median
    float* buffer;      // Buffer untuk moving/median
    uint8_t index;
} SensorFilter_t;

void SensorFilter_Init(SensorFilter_t* filter, FilterType_t type, uint8_t window_size, float alpha);
float SensorFilter_Apply(SensorFilter_t* filter, float new_value);
void SensorFilter_Reset(SensorFilter_t* filter);