//sensor_filter.c
#include "sensor_filter.h"
#include <stdlib.h>   // Untuk malloc()
#include <string.h>   // Untuk memset()
#include <stdint.h>   // Untuk uint8_t

// Helper: swap untuk median sort
static void swap(float* a, float* b) {
    float temp = *a;
    *a = *b;
    *b = temp;
}

// Helper: bubble sort (cukup karena ukuran window kecil)
static void sort_array(float* array, uint8_t size) {
    for (uint8_t i = 0; i < size - 1; i++) {
        for (uint8_t j = 0; j < size - i - 1; j++) {
            if (array[j] > array[j + 1]) {
                swap(&array[j], &array[j + 1]);
            }
        }
    }
}

void SensorFilter_Init(SensorFilter_t* filter, FilterType_t type, uint8_t window_size, float alpha) {
    filter->type = type;
    filter->alpha = alpha;
    filter->window_size = window_size;
    filter->index = 0;

    filter->buffer = NULL;

    if (type == FILTER_MOVING_AVG || type == FILTER_MEDIAN || type == FILTER_LOWPASS) {
        filter->buffer = (float*)malloc(window_size * sizeof(float));
        if (filter->buffer) {
            memset(filter->buffer, 0, window_size * sizeof(float));
        }
    }
}

float SensorFilter_Apply(SensorFilter_t* filter, float new_value) {
    if (!filter->buffer) return new_value;

    switch (filter->type) {
        case FILTER_LOWPASS:
            *filter->buffer = (filter->alpha * new_value) + ((1 - filter->alpha) * (*filter->buffer));
            return *filter->buffer;

        case FILTER_MOVING_AVG:
            filter->buffer[filter->index++] = new_value;
            if (filter->index >= filter->window_size) filter->index = 0;

            float sum = 0;
            for (uint8_t i = 0; i < filter->window_size; i++) {
                sum += filter->buffer[i];
            }
            return sum / filter->window_size;

        case FILTER_MEDIAN: {
            filter->buffer[filter->index++] = new_value;
            if (filter->index >= filter->window_size) filter->index = 0;

            float temp[filter->window_size];
            memcpy(temp, filter->buffer, filter->window_size * sizeof(float));
            sort_array(temp, filter->window_size);

            if (filter->window_size % 2 == 0) {
                uint8_t mid = filter->window_size / 2;
                return (temp[mid - 1] + temp[mid]) / 2.0f;
            } else {
                return temp[filter->window_size / 2];
            }
        }

        default:
            return new_value;
    }
}

void SensorFilter_Reset(SensorFilter_t* filter) {
    if (filter->buffer) {
        memset(filter->buffer, 0, filter->window_size * sizeof(float));
    }
    filter->index = 0;
}