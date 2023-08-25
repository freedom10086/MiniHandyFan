#ifndef KEY_H
#define KEY_H

#include "esp_types.h"
#include "esp_event.h"


ESP_EVENT_DECLARE_BASE(FAN_PUSH_KEY_EVENT);

#define PUSH_KEY_NUM 9

/**
 * key click event
 */
typedef enum {
    PUSH_KEY_SHORT_CLICK,
    PUSH_KEY_LONG_CLICK,
} key_event_id_t;

void key_init();

#endif