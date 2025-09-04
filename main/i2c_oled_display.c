/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "lvgl.h"
#include "i2c_oled_display.h"
#include "sensor_data.h"
#include "my_screens/my_screens.h"

#if CONFIG_LCD_CONTROLLER_SH1107
#include "esp_lcd_sh1107.h"
#else
#include "esp_lcd_panel_vendor.h"
#endif

static const char *OLED_DISP = "OLED_DISPLAY";
static const char *OLED_INIT = "OLED_INIT";


#define I2C_BUS_PORT  0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// The pixel number in horizontal and vertical
#if CONFIG_LCD_CONTROLLER_SSD1306
#define LCD_H_RES                       CONFIG_LCD_CONTROLLER_WIDTH
#define LCD_V_RES                       CONFIG_SSD1306_HEIGHT
#elif CONFIG_LCD_CONTROLLER_SH1107
#define LCD_H_RES              64
#define LCD_V_RES              128
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define EXAMPLE_LVGL_TICK_PERIOD_MS    5
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define LVGL_TASK_PRIORITY     1
#define EXAMPLE_LVGL_PALETTE_SIZE      8
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1000 / CONFIG_FREERTOS_HZ

// To use LV_COLOR_FORMAT_I1, we need an extra buffer to hold the converted data
static uint8_t oled_buffer[LCD_H_RES * LCD_V_RES / 8];
// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
static _lock_t lvgl_api_lock;

//extern void example_lvgl_demo_ui(lv_disp_t *disp);

// Globals (or file-scope statics)
static i2c_master_bus_handle_t i2c_bus;
static esp_lcd_panel_io_handle_t io_handle;
static esp_lcd_panel_handle_t panel_handle;

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io_panel, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);

    // This is necessary because LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette. Skip the palette here
    // More information about the monochrome, please refer to https://docs.lvgl.io/9.2/porting/display.html#monochrome-displays
    px_map += EXAMPLE_LVGL_PALETTE_SIZE;

    uint16_t hor_res = lv_display_get_physical_horizontal_resolution(disp);
    int x1 = area->x1;
    int x2 = area->x2;
    int y1 = area->y1;
    int y2 = area->y2;

    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            /* The order of bits is MSB first
                        MSB           LSB
               bits      7 6 5 4 3 2 1 0
               pixels    0 1 2 3 4 5 6 7
                        Left         Right
            */
            bool chroma_color = (px_map[(hor_res >> 3) * y  + (x >> 3)] & 1 << (7 - x % 8));

            /* Write to the buffer as required for the display.
            * It writes only 1-bit for monochrome displays mapped vertically.*/
            uint8_t *buf = oled_buffer + hor_res * (y >> 3) + (x);
            if (chroma_color) {
                (*buf) &= ~(1 << (y % 8));
            } else {
                (*buf) |= (1 << (y % 8));
            }
        }
    }
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, oled_buffer);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void lvgl_port_task(void *arg)
{
    static const char *LVGL_TAG = "LVGL Task";
    ESP_LOGI(LVGL_TAG, "Init SSD1306 & LVGL");
    ssd1306_bus_init();
    ssd1306_panel_init();
    ssd1306_LVGL_init();
    create_temp_hum_screen();
//    lvgl_demo_ui(lvgl_disp);

    ESP_LOGI(LVGL_TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;

    // Now run the periodic handler with vTaskDelayUntil
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
//        ESP_LOGI(LVGL_TAG, "Loop");
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, EXAMPLE_LVGL_TASK_MIN_DELAY_MS);
        // in case of lvgl display not ready yet
        time_till_next_ms = MIN(time_till_next_ms, EXAMPLE_LVGL_TASK_MAX_DELAY_MS);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(time_till_next_ms));        
//        usleep(1000 * time_till_next_ms);
    }
}

void start_lvgl_task(void)
{
    #ifndef CONFIG_LCD_CONTROLLER_NONE
        ESP_LOGI(OLED_DISP, "Create LVGL task");
        xTaskCreate(lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);
    #endif
}

void oled_scroll_text(void) {

    ESP_LOGI(OLED_INIT, "Display LVGL Scroll Text");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    _lock_acquire(&lvgl_api_lock);
    lvgl_scroll_text(lvgl_disp);
    _lock_release(&lvgl_api_lock);
}

void lvgl_scroll_text(lv_display_t *disp)
{
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(label, "Why are my balls itchy?");
    /* Size of the screen (if you use rotation 90 or 270, please use lv_display_get_vertical_resolution) */
    lv_obj_set_width(label, lv_display_get_horizontal_resolution(disp));
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
}


/*
void create_dashboard(lv_obj_t *screen, const char *initial_text, istruct sensor_reading_t *new_data,
                      const lv_img_dsc_t *therm_img, const lv_img_dsc_t *hum_img) {
    // Main container
    lv_obj_t *main_cont = lv_obj_create(screen);
    lv_obj_set_size(main_cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_layout(main_cont, LV_LAYOUT_FLEX_COL);
    lv_obj_set_flex_align(main_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    // Top scrolling text
    scrolling_label = lv_label_create(main_cont);
    lv_label_set_text(scrolling_label, initial_text);
    lv_label_set_long_mode(scrolling_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_width(scrolling_label, LV_PCT(100));
    lv_obj_set_style_text_font(scrolling_label, &lv_font_montserrat_12, 0);

    // Bottom horizontal row for icons + values
    lv_obj_t *icon_row = lv_obj_create(main_cont);
    lv_obj_set_width(icon_row, LV_PCT(100));
    lv_obj_set_layout(icon_row, LV_LAYOUT_FLEX_ROW);
    lv_obj_set_flex_align(icon_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    // Temperature container
    lv_obj_t *temp_cont = lv_obj_create(icon_row);
    lv_obj_set_layout(temp_cont, LV_LAYOUT_FLEX_ROW);
    lv_obj_set_flex_align(temp_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    lv_obj_t *temp_img = lv_img_create(temp_cont);
    lv_img_set_src(temp_img, therm_img);

    temp_label = lv_label_create(temp_cont);
    lv_label_set_text_fmt(temp_label, "%d°C", temp);

    // Humidity container
    lv_obj_t *hum_cont = lv_obj_create(icon_row);
    lv_obj_set_layout(hum_cont, LV_LAYOUT_FLEX_ROW);
    lv_obj_set_flex_align(hum_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    lv_obj_t *hum_img_obj = lv_img_create(hum_cont);
    lv_img_set_src(hum_img_obj, hum_img);

    hum_label = lv_label_create(hum_cont);
    lv_label_set_text_fmt(hum_label, "%d%%", hum);
}
*/

void update_readings(sensor_reading_t new_data) {
    if(temp_label) lv_label_set_text_fmt(temp_label, "%.1f°C", new_data.temperature[CURRENT]);
    if(hum_label)  lv_label_set_text_fmt(hum_label, "%.1f%%", new_data.humidity[CURRENT]);
}

void ssd1306_bus_init(void) {
    // Configure and install I2C bus
    ESP_LOGI(OLED_INIT, "Initialize I2C bus");
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_1,
        .sda_io_num = CONFIG_OLED_I2C_MASTER_SDA,
        .scl_io_num = CONFIG_OLED_I2C_MASTER_SCL,
        .glitch_ignore_cnt = 7,
//        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));
}

void ssd1306_panel_init(void) {
    ESP_LOGI(OLED_INIT, "Install panel IO");
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = CONFIG_OLED_I2C_ADDRESS,
        .scl_speed_hz = CONFIG_OLED_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = CONFIG_OLED_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = CONFIG_OLED_PARAM_BITS, // According to SSD1306 datasheet
#if CONFIG_LCD_CONTROLLER_SSD1306
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
        .dc_bit_offset = 0,                     // According to SH1107 datasheet
        .flags =
        {
            .disable_control_phase = 1,
        }
#endif
    };
    esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle);

    ESP_LOGI(OLED_INIT, "Install SSD1306 panel driver");

    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = CONFIG_OLED_PIN_NUM_RST,
    };
    #if CONFIG_LCD_CONTROLLER_SSD1306
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh1107(io_handle, &panel_config, &panel_handle));
#endif
}

void ssd1306_LVGL_init(void) {
    ESP_LOGI(OLED_INIT, "Reset SSD1306 panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));    // mirror X, mirror Y)

#if CONFIG_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif

    ESP_LOGI(OLED_INIT, "Initialize LVGL");
    ESP_LOGI(OLED_INIT, "lv_init");
    lv_init();
    // create a lvgl display
    ESP_LOGI(OLED_INIT, "lv_display_create");
    lvgl_disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    // associate the i2c panel handle to the display
    ESP_LOGI(OLED_INIT, "lv_display_set_user_data");
    lv_display_set_user_data(lvgl_disp, panel_handle);
    // create draw buffer
    void *buf = NULL;
    ESP_LOGI(OLED_INIT, "Allocate separate LVGL draw buffers");
    // LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette.
    size_t draw_buffer_sz = LCD_H_RES * LCD_V_RES / 8 + EXAMPLE_LVGL_PALETTE_SIZE;
    buf = heap_caps_calloc(1, draw_buffer_sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(buf);

    // LVGL9 suooprt new monochromatic format.
    lv_display_set_color_format(lvgl_disp, LV_COLOR_FORMAT_I1);
    // initialize LVGL draw buffers
    lv_display_set_buffers(lvgl_disp, buf, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_FULL);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(lvgl_disp, example_lvgl_flush_cb);

    ESP_LOGI(OLED_INIT, "Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    /* Register done callback */
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, lvgl_disp);

    ESP_LOGI(OLED_INIT, "Use esp_timer as LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));
}
