//dbw_learning.h
#ifndef DBW_LEARNING_H
#define DBW_LEARNING_H

#include "ecu_types.h"
#include <stdbool.h>

typedef struct {
    bool enabled;
    bool complete;
    float min_pos;
    float max_pos;
    uint32_t last_update_ms;
} DBW_Learning_t;

void DBW_Learning_Init(DBW_Learning_t* learning);
void DBW_Learning_Start(DBW_Learning_t* learning);
void DBW_Learning_Update(DBW_Learning_t* learning, float raw_position);
bool DBW_Learning_IsComplete(const DBW_Learning_t* learning);

#endif // DBW_LEARNING_H
