// speed_output.h
#ifndef SPEED_OUTPUT_H
#define SPEED_OUTPUT_H

#include <stdint.h>

void SpeedOutput_Init(void);
void SpeedOutput_Update(uint32_t vehicle_speed_kph);

#endif // SPEED_OUTPUT_H
