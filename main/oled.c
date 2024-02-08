#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oled.h"
#include "esp_log.h"
#include "stdio.h"
#include "driver/i2c_master.h"

#define TAG "oled"

#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF
#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA
#define SSD1306_SETVCOMDETECT 0xDB
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9
#define SSD1306_SETMULTIPLEX 0xA8
#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10
#define SSD1306_SETSTARTLINE 0x40
#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR 0x22
#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8
#define SSD1306_SEGREMAP 0xA0
#define SSD1306_CHARGEPUMP 0x8D
#define SSD1306_EXTERNALVCC 0x01
#define SSD1306_SWITCHCAPVCC 0x02

//Scrolling constants
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_DEACTIVATE_SCROLL 0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A

static i2c_master_dev_handle_t dev_handle;

static esp_err_t i2c_write_cmd(uint8_t cmd) {
    int ret;
    uint8_t write_buf[2] = {0x00, cmd};
    ret = i2c_master_transmit(dev_handle,
                                     write_buf, sizeof(write_buf),
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "write i2c cmd failed, %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t i2c_write_data(uint8_t data) {
    int ret;
    uint8_t write_buf[2] = {0x40, data};
    ret = i2c_master_transmit(dev_handle,
                                     write_buf,
                                     sizeof(write_buf),
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "write i2c data failed, %s", esp_err_to_name(ret));
    }
    return ret;
}

static void lcd_i2c_init(gpio_num_t sda, gpio_num_t scl) {
    ESP_LOGI(TAG, "Initialize I2C bus");

    int i2c_master_port = I2C_HOST;
    i2c_master_bus_config_t i2c_mst_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = i2c_master_port,
            .scl_io_num = scl,
            .sda_io_num = sda,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus_handle;
    esp_err_t iic_err = i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);
    if (iic_err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialized failed %d %s", iic_err, esp_err_to_name(iic_err));
    } else {
        ESP_LOGI(TAG, "I2C initialized successfully");
    }

    i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = LCD_I2C_ADDR,
            .scl_speed_hz = 100000,
    };

    esp_err_t err = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "I2C add device failed");
    }
}

void oled_init(gpio_num_t sda, gpio_num_t scl) {
    lcd_i2c_init(sda, scl);
    vTaskDelay(pdMS_TO_TICKS(100));

    i2c_write_cmd(SSD1306_DISPLAYOFF); /*display off*/
    //i2c_write_cmd(0x00); /*set lower column address*/
    //i2c_write_cmd(0x12); /*set higher column address*/
    i2c_write_cmd(0x00); /*set display start line*/
    i2c_write_cmd(0xB0); /*set page address*/
    i2c_write_cmd(0x81); /*contract control*/
    i2c_write_cmd(0x4f); /*128*/
    i2c_write_cmd(0xA1); /*set segment remap*/
    i2c_write_cmd(SSD1306_NORMALDISPLAY); /*normal / reverse*/
    i2c_write_cmd(SSD1306_SETMULTIPLEX); /*multiplex ratio*/
    i2c_write_cmd(0x1F); /*duty = 1/32*/
    i2c_write_cmd(0xC8); /*Com scan direction*/
    i2c_write_cmd(SSD1306_SETDISPLAYOFFSET); /*set display offset*/
    i2c_write_cmd(0x00);
    i2c_write_cmd(0xD5); /*set osc division*/
    i2c_write_cmd(0x80);
    i2c_write_cmd(0xD9); /*set pre-charge period*/
    i2c_write_cmd(0x1f);
    i2c_write_cmd(0xDA); /*set COM pins*/
    i2c_write_cmd(0x12);
    i2c_write_cmd(0xdb); /*set vcomh*/
    i2c_write_cmd(0x40);
    i2c_write_cmd(0x8d); /*set charge pump enable*/
    i2c_write_cmd(0x14);

    oled_clear();
    i2c_write_cmd(SSD1306_DISPLAYON); /*display ON*/
}

void oled_set_pos(uint8_t x, uint8_t y) {
    x += 32;
    i2c_write_cmd(0xb0 + y);
    i2c_write_cmd(((x & 0xf0) >> 4) | 0x10);
    i2c_write_cmd((x & 0x0f));
}

void oled_draw_bitmap(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *bmp) {
    uint16_t j = 0;
    uint8_t i, m;
    height = height / 8 + ((height % 8) ? 1 : 0);
    for (i = 0; i < height; i++) {
        oled_set_pos(x, i + y);
        for (m = 0; m < width; m++) {
            i2c_write_data(bmp[j++]);
        }
    }
}

//在指定位置显示一个字符
//x:0~63
//y:0~31
//sizey:选择字体 6x8  8x16
void olde_draw_char(uint8_t x, uint8_t y, const char chr, uint8_t sizey) {
    uint8_t c = 0, sizex = sizey / 2, temp;
    uint16_t i = 0, size1;
    if (sizey == 8)size1 = 6;
    else size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * (sizey / 2);
    c = chr - ' ';//得到偏移后的值
    oled_set_pos(x, y);
    for (i = 0; i < size1; i++) {
        if (i % sizex == 0 && sizey != 8) oled_set_pos(x, y++);
        if (sizey == 8) {
            temp = asc2_0806[c][i];
            i2c_write_data(temp);//6X8字号
        } else if (sizey == 16) {
            temp = asc2_1608[c][i];
            i2c_write_data(temp);//8x16字号
        } else return;
    }
}

//显示一个字符号串
void oled_draw_string(uint8_t x, uint8_t y, const char *chr, uint8_t sizey) {
    uint8_t j = 0;
    while (chr[j] != '\0') {
        olde_draw_char(x, y, chr[j++], sizey);
        if (sizey == 8)x += 6;
        else x += sizey / 2;
    }
}

//开启OLED显示
void oled_display_on(void) {
    i2c_write_cmd(0X8D);  //SET DCDC命令
    i2c_write_cmd(0X14);  //DCDC ON
    i2c_write_cmd(SSD1306_DISPLAYON);  //DISPLAY ON
}

//关闭OLED显示
void oled_display_off(void) {
    i2c_write_cmd(0X8D);  //SET DCDC命令
    i2c_write_cmd(0X10);  //DCDC OFF
    i2c_write_cmd(SSD1306_DISPLAYOFF);  //DISPLAY OFF
}

//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void oled_clear(void) {
    uint8_t i, n;
    for (i = 0; i < 4; i++) {
        i2c_write_cmd(0xb0 + i);  //设置页地址（0~7）
        i2c_write_cmd(0x00);      //设置显示位置—列低地址
        i2c_write_cmd(0x12);      //设置显示位置—列高地址
        for (n = 0; n < 64; n++) {
            i2c_write_data(0x00);
        }
    } //更新显示
}

//反显函数
void oled_reverse_color(uint8_t reverse) {
    if (!reverse) i2c_write_cmd(0xA6);//正常显示
    else i2c_write_cmd(0xA7);//反色显示
}

//屏幕旋转180度
void oled_display_rotate(uint8_t rotate) {
    if (!rotate) {
        i2c_write_cmd(0xC8);//正常显示
        i2c_write_cmd(0xA1);
    } else {
        i2c_write_cmd(0xC0);//反转显示
        i2c_write_cmd(0xA0);
    }
}