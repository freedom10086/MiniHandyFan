//
// Created by yang on 2023/8/23.
//

#ifndef RAGEFAN_OLED_H
#define RAGEFAN_OLED_H

#include <soc/gpio_num.h>
#include "stdio.h"
#include "oledfont.h"

#define LCD_PIXEL_CLOCK_HZ      (400 * 1000)
#define I2C_HOST                0
// ssd1315
#define LCD_I2C_ADDR            0x3c
#define LCD_WIDTH               64
#define LCD_HEIGHT              32
#define I2C_MASTER_TIMEOUT_MS   100

void oled_init(gpio_num_t sda, gpio_num_t scl);

void oled_set_pos(uint8_t x, uint8_t y);

void oled_draw_bitmap(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *bmp);

void olde_draw_char(uint8_t x, uint8_t y, const char chr, uint8_t sizey);

void oled_draw_string(uint8_t x, uint8_t y, const char *chr, uint8_t sizey);

void oled_display_on(void);

void oled_display_off(void);

void oled_clear(void);

void oled_reverse_color(uint8_t reverse);

void oled_display_rotate(uint8_t rotate);

#endif
