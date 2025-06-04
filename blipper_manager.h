//blipper_manager.h
#ifndef BLIPPER_MANAGER_H
#define BLIPPER_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

void Blipper_init(void);
void Blipper_update(uint16_t rpm, bool is_downshift);
void Blipper_set_duration(uint32_t ms);
void Blipper_set_intensity(uint8_t percent);

#endif
