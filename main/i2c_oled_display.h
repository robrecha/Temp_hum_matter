// SSD1306_drv.h
#pragma once  // include guard

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include "sensor_data.h"

static lv_obj_t *scrolling_label;
static lv_obj_t *temp_label;
static lv_obj_t *hum_label;

static lv_display_t *lvgl_disp;

void oled_scroll_text(void);
void lvgl_scroll_text(lv_display_t *disp);
/*void create_dashboard(lv_obj_t *screen, const char *initial_text, struct sensor_reading_t *new_data,
                      const lv_img_dsc_t *therm_img, const lv_img_dsc_t *hum_img);*/
void create_temp_hum_screen(void);
void update_readings(sensor_reading_t new_data);
void start_lvgl_task(void);
static void lvgl_port_task(void *arg);
void ssd1306_bus_init(void);
void ssd1306_panel_init(void);
void ssd1306_LVGL_init(void);

#ifdef __cplusplus
}
#endif