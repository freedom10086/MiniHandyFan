#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "key.h"
#include "fan_common.h"
#include "event_common.h"

#define TAG "keyboard"
#define KEY_CLICK_MIN_GAP 30
#define KEY_LONG_PRESS_TIME_GAP 400

ESP_EVENT_DEFINE_BASE(FAN_PUSH_KEY_EVENT);

//static TaskHandle_t xTaskToNotify = NULL;
//const UBaseType_t xArrayIndex = 0;

static QueueHandle_t event_queue;
static esp_timer_handle_t timers[1];
static uint32_t tick_count[1];
static uint32_t key_up_tick_count[1];
static uint32_t time_diff_ms, tick_diff, key_up_tick_diff;
static bool ignore_long_pressed[1];

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(event_queue, &gpio_num, NULL);
}

static void timer_callback(void *arg) {
    uint8_t index = (int) arg;
    ESP_LOGI(TAG, "timer call back timer for %d", index);
    // still down
    if (gpio_get_level(PUSH_KEY_NUM) == 0) {
        time_diff_ms = pdTICKS_TO_MS(xTaskGetTickCount() - tick_count[index]);
        ESP_LOGI(TAG, "timer call back time_diff_ms %ld", time_diff_ms);
        if (time_diff_ms >= KEY_LONG_PRESS_TIME_GAP - 10) {
            // long pressed
            ignore_long_pressed[index] = true;
            ESP_LOGI(TAG, "key %d long press detect by timer", PUSH_KEY_NUM);
            common_post_event_data(FAN_PUSH_KEY_EVENT,
                                   PUSH_KEY_LONG_CLICK,
                                   &time_diff_ms,
                                   sizeof(time_diff_ms));
        }
    }
}

static void key_task_entry(void *arg) {

    //xTaskToNotify = xTaskGetCurrentTaskHandle();
    event_queue = xQueueCreate(10, sizeof(gpio_num_t));

    gpio_num_t clicked_gpio;
    esp_timer_create_args_t timer_args_1 = {
            .arg = (void *) 0,
            .callback = &timer_callback,
            .name = "pushkey"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args_1, &timers[0]));

    while (1) {
        if (xQueueReceive(event_queue, &clicked_gpio, portMAX_DELAY)) {
            // vTaskDelay(pdMS_TO_TICKS(2));
            uint8_t index = 0;
            if (gpio_get_level(clicked_gpio) == 0) {
                // key down
                //ESP_LOGI(TAG, "key %d click down detect...", clicked_gpio);
                if (!esp_timer_is_active(timers[index])) {
                    ignore_long_pressed[index] = false;
                    ESP_LOGI(TAG, "start timer for %d", index);
                    esp_timer_start_once(timers[index], KEY_LONG_PRESS_TIME_GAP * 1000);
                }
            } else if (gpio_get_level(clicked_gpio) == 1) {
                // key up
                //ESP_LOGI(TAG, "key %d click up detect...", clicked_gpio);
                tick_diff = xTaskGetTickCount() - tick_count[index];
                time_diff_ms = pdTICKS_TO_MS(tick_diff);

                key_up_tick_diff = xTaskGetTickCount() - key_up_tick_count[index];
                if (time_diff_ms > KEY_LONG_PRESS_TIME_GAP) {
                    if (ignore_long_pressed[index]) {
                        ESP_LOGI(TAG, "key %d long press, ignore", clicked_gpio);
                    } else {
                        ESP_LOGI(TAG, "key %d long press", clicked_gpio);
                        common_post_event_data(FAN_PUSH_KEY_EVENT,
                                          PUSH_KEY_LONG_CLICK,
                                          &time_diff_ms,
                                          sizeof(time_diff_ms));
                    }
                } else if (pdTICKS_TO_MS(key_up_tick_diff) > KEY_CLICK_MIN_GAP) {
                    if (esp_timer_is_active(timers[index])) {
                        esp_timer_stop(timers[index]);
                    }

                    ESP_LOGI(TAG, "key %d short press", clicked_gpio);
                    common_post_event_data(FAN_PUSH_KEY_EVENT,
                                      PUSH_KEY_SHORT_CLICK,
                                      &time_diff_ms,
                                      sizeof(time_diff_ms));
                } else {
                    ESP_LOGW(TAG, "key up to quickly gpio:%d, time diff:%ldms", clicked_gpio,
                             pdTICKS_TO_MS(key_up_tick_diff));
                }

                key_up_tick_count[index] = xTaskGetTickCount();
            }

            tick_count[index] = xTaskGetTickCount();
        }
    }

    esp_timer_stop(timers[0]);
    esp_timer_stop(timers[1]);
    esp_timer_delete(timers[0]);
    esp_timer_delete(timers[1]);
    vTaskDelete(NULL);
}

void key_init() {
    TaskHandle_t tsk_hdl;
    /* Create NMEA Parser task */
    BaseType_t err = xTaskCreate(
            key_task_entry,
            "key_task",
            2048,
            NULL,
            uxTaskPriorityGet(NULL),
            &tsk_hdl);
    if (err != pdTRUE) {
        ESP_LOGE(TAG, "create key detect task failed");
    }

    ESP_LOGI(TAG, "keyboard task init OK");

    gpio_config_t io_config = {
            .pin_bit_mask = (1ull << PUSH_KEY_NUM),
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_ANYEDGE,
            .pull_up_en = 1,
            .pull_down_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));

    //ESP_LOGI(TAG, "io %d, level %d", KEY_1_NUM, gpio_get_level(KEY_1_NUM));
    //ESP_LOGI(TAG, "io %d, level %d", KEY_2_NUM, gpio_get_level(KEY_2_NUM));

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(PUSH_KEY_NUM, gpio_isr_handler, (void *) PUSH_KEY_NUM);
}