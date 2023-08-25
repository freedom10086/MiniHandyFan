#include <stdio.h>
#include <inttypes.h>
#include <esp_lcd_panel_vendor.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_sleep.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

#include "key.h"
#include "battery.h"
#include "fan_common.h"
#include "oled.h"
#include "esp_lvgl_port.h"

#define FAN_PWR_IO 3

#define EXT_IO_1 4
#define EXT_IO_2 5
#define EXT_IO_3 6

#define BDC_ENCODER_GPIO_A            2
#define BDC_ENCODER_GPIO_B            8
#define BDC_ENCODER_PCNT_HIGH_LIMIT   1000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1000

#define TAG "fan_main"

esp_event_loop_handle_t event_loop_handle;
RTC_DATA_ATTR static uint32_t boot_count = 0;
static TaskHandle_t x_update_notify_handl = NULL;


void fan_pwr_on();

void fan_pwr_off();

void encoder_task_entry(void *args);

void lcd_task_start();

static void fan_speed_task(void *args);

void enter_deep_sleep(int sleep_ts);

void init_fan_event_loop();

void app_main(void) {
    boot_count++;
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
        ESP_LOGI(TAG, "wake up by cause  %d", cause);
    }
    printf("Hello world!, boot count %ld\n", boot_count);

    init_fan_event_loop();

    ESP_LOGI(TAG, "Config Fan Power IO");
    gpio_config_t fan_pwr_io_config = {
            .pin_bit_mask = (1ull << FAN_PWR_IO),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = 0,
            .pull_down_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&fan_pwr_io_config));
    fan_pwr_on();

    // key init
    ESP_LOGI(TAG, "Initialize rotary encoder");
    key_init();
    //rotary_encoder_init();

    // init battery
    battery_init();

    xTaskCreate(fan_speed_task, "fan_speed_task", 3072, NULL,
                uxTaskPriorityGet(NULL), NULL);

    xTaskCreate(encoder_task_entry, "encoder_task", 3072, NULL,
                uxTaskPriorityGet(NULL), NULL);

    lcd_task_start();
}

void fan_pwr_on() {
    gpio_set_level(FAN_PWR_IO, 1);
}

void fan_pwr_off() {
    gpio_set_level(FAN_PWR_IO, 0);
}

static void application_task(void *args) {
    while (1) {
        esp_err_t err = esp_event_loop_run(event_loop_handle, portMAX_DELAY);
        if (err != ESP_OK) {
            break;
        }
    }

    ESP_LOGE(TAG, "suspended task for loop %p", event_loop_handle);
    vTaskSuspend(NULL);
}

static void fan_speed_task(void *args) {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .timer_num        = LEDC_TIMER_0,
            .duty_resolution  = LEDC_TIMER_13_BIT, // Set duty resolution to 13 bits,
            .freq_hz          = (5000),  // Set output frequency at 5 kHz
            .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = LEDC_CHANNEL_0,
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = EXT_IO_3,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    x_update_notify_handl = xTaskGetCurrentTaskHandle();
    static uint32_t ulNotificationCount;
    uint32_t continue_time_out_count = 0;

    while (1) {
        // xTaskGenericNotify(x_update_notify_handl, 0, *full_update, eIncrement, NULL);
        ulNotificationCount = ulTaskGenericNotifyTake(0, pdTRUE, pdMS_TO_TICKS(60000));
        if (ulNotificationCount > 0) { // may > 1 more data ws send
            continue_time_out_count = 0;
        } else {
            /* The call to ulTaskNotifyTake() timed out. */
            continue_time_out_count += 1;
        }

        // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (4095)));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    }
}

static void key_click_event_handler(void *event_handler_arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "rev key click event %ld", event_id);
    // finally pass here
    switch (event_id) {
        case PUSH_KEY_SHORT_CLICK:
            break;
        case PUSH_KEY_LONG_CLICK:
            break;
        default:
            break;
    }

    // if page not handle key click event here handle
    ESP_LOGI(TAG, "no page handler key click event %ld", event_id);
}

void init_fan_event_loop() {
    // create event loop
    esp_event_loop_args_t loop_args = {
            .queue_size = 16,
            .task_name = NULL // no task will be created
    };

    // Create the event loops
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &event_loop_handle));

    ESP_LOGI(TAG, "starting application task");
    // Create the application task with the same priority as the current task
    xTaskCreate(application_task, "application_task", 3072, NULL,
                uxTaskPriorityGet(NULL), NULL);

    // key click event
    esp_event_handler_register_with(event_loop_handle,
                                    FAN_PUSH_KEY_EVENT, ESP_EVENT_ANY_ID,
                                    key_click_event_handler, NULL);
}

static bool
notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    lv_disp_t *disp = (lv_disp_t *) user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}

void lcd_task_start() {
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = EXT_IO_1,
            .scl_io_num = EXT_IO_2,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = LCD_PIXEL_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .control_phase_bytes = 1,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .dc_bit_offset = 6,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t) I2C_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
            .bits_per_pixel = 1,
            .reset_gpio_num = -1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
            .io_handle = io_handle,
            .panel_handle = panel_handle,
            .buffer_size = 64 * 32,
            .double_buffer = true,
            .hres = 64,
            .vres = 32,
            .monochrome = true,
            .rotation = {
                    .swap_xy = false,
                    .mirror_x = false,
                    .mirror_y = false,
            }
    };
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);
    /* Register done callback for IO */
    const esp_lcd_panel_io_callbacks_t cbs = {
            .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, disp);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    ESP_LOGI(TAG, "Display LVGL Scroll Text");
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(label, "Miny Handy Fan");
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

static bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t) user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

void encoder_task_entry(void *args) {
    ESP_LOGI(TAG, "Init encoder driver to decode rotary signal");
    pcnt_unit_config_t unit_config = {
            .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
            .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
            .flags.accum_count = true, // enable counter accumulation
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    pcnt_chan_config_t chan_a_config = {
            .edge_gpio_num = BDC_ENCODER_GPIO_A,
            .level_gpio_num = BDC_ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
            .edge_gpio_num = BDC_ENCODER_GPIO_B,
            .level_gpio_num = BDC_ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));

    pcnt_event_callbacks_t cbs = {
            .on_reach = pcnt_on_reach,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    // Report counter value
    int pulse_count = 0;
    int event_count = 0;
    static int last_pulse_count = 0;

    while (1) {
        if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) {
            ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
        } else {
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
            int real_pulses = pulse_count - last_pulse_count;
            last_pulse_count = pulse_count;

            ESP_LOGI(TAG, "Pulse count: %d, Real Pulse: %d", pulse_count, real_pulses);
        }
    }
}

void enter_deep_sleep(int sleep_ts) {
    if (sleep_ts > 0) {
        esp_sleep_enable_timer_wakeup(sleep_ts * 1000000);
    }
    //esp_sleep_enable_ext1_wakeup(1 << KEY_1_NUM, ESP_EXT1_WAKEUP_ALL_LOW);
#ifdef SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    const gpio_config_t config = {
            .pin_bit_mask = 1 << GPIO_NUM_0,
            .mode = GPIO_MODE_INPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(1 << GPIO_NUM_0, ESP_GPIO_WAKEUP_GPIO_LOW));
#else
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
    gpio_pullup_en(GPIO_NUM_0);
    gpio_pulldown_dis(GPIO_NUM_0);
#endif
    fan_pwr_off();

    ESP_LOGI(TAG, "enter deep sleep mode, sleep %ds", sleep_ts);
    esp_deep_sleep_start();
}