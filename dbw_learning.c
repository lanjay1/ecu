//dbw_learning.c
#include "dbw_learning.h"
#include "utils.h"  // for HAL_GetTick()

#define LEARNING_DURATION_MS   1500

void DBW_Learning_Init(DBW_Learning_t* learning) {
    if (!learning) return;
    learning->enabled = false;
    learning->complete = false;
    learning->min_pos = 1.0f;
    learning->max_pos = 0.0f;
    learning->last_update_ms = 0;
}

void DBW_Learning_Start(DBW_Learning_t* learning) {
    if (!learning) return;
    learning->enabled = true;
    learning->complete = false;
    learning->min_pos = 1.0f;
    learning->max_pos = 0.0f;
    learning->last_update_ms = HAL_GetTick();
}

void DBW_Learning_Update(DBW_Learning_t* learning, float raw_position) {
    if (!learning || !learning->enabled) return;

    if (raw_position < learning->min_pos)
        learning->min_pos = raw_position;
    if (raw_position > learning->max_pos)
        learning->max_pos = raw_position;

    if ((HAL_GetTick() - learning->last_update_ms) >= LEARNING_DURATION_MS) {
        learning->enabled = false;
        learning->complete = true;
    }
}

bool DBW_Learning_IsComplete(const DBW_Learning_t* learning) {
    return learning && learning->complete;
}
