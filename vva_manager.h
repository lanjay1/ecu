//vva_manager.h
#ifndef VVA_MANAGER_H
#define VVA_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

void VVA_init(void);
void VVA_update(uint16_t rpm, uint16_t throttle);
void VVA_setState(bool enable);
bool VVA_getState(void);

#endif // VVA_MANAGER_H