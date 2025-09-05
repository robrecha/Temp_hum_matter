#include <esp_log.h>
#include "my_screens/my_screens.h"
#include "../my_images/my_images.h" // Include the header file for image declarations
#include "i2c_oled_display.h"
#include "lvgl.h"

// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
static _lock_t lvgl_api_lock;

static const char *TAG_SCREEN = "Screens";

//Create screen object
lv_obj_t *temp_hum_scr;

void create_temp_hum_screen(void) {
    // Create a new screen
    temp_hum_scr = lv_obj_create(NULL);
    ESP_LOGI(TAG_SCREEN, "Display LVGL Scroll Text");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    _lock_acquire(&lvgl_api_lock);
    Scroll_text();      // Scroll text -----------------

    // Temperature image widget -----------------
    lv_obj_t *img_temp = lv_image_create(temp_hum_scr);
    lv_image_set_src(img_temp, &temperature_icon_32);   // Reference to the declared image
    lv_obj_set_pos(img_temp, 0, 20);                    // Temperature icon at 0 px right, middle height
    // Temperature value label ------------------ 
    temp_label_internal = lv_label_create(temp_hum_scr);        // Temperature label (to the right of icon)
   // lv_obj_t *temp_label = lv_label_create(scr);        // Temperature label (to the right of icon)
    lv_label_set_text(temp_label_internal, "23.4°C");
    lv_obj_align_to(temp_label_internal, img_temp, LV_ALIGN_OUT_RIGHT_MID, 2, 0);    // ^ aligns label to the right side of icon, with +2px gap

    // Humidity image widget
    lv_obj_t *img_hum = lv_image_create(temp_hum_scr);
    lv_image_set_src(img_hum, &humidity_icon_32);       // Reference to the declared image
    lv_obj_set_pos(img_hum, 70, 20);                    // Humidity icon at 70 px right, middle height
    // Humidity value label  
    hum_label_internal = lv_label_create(temp_hum_scr);
    lv_label_set_text(hum_label_internal, "55%");
    lv_obj_align_to(hum_label_internal, img_hum, LV_ALIGN_OUT_RIGHT_MID, 2, 0);

    // Load the screen
    lv_scr_load(temp_hum_scr);
    _lock_release(&lvgl_api_lock);
}

void update_temp_hum(void *param) {
    sensor_reading_t * data = (sensor_reading_t *) param;
    if(temp_label_internal) lv_label_set_text_fmt(temp_label_internal, "%.1f°C", data->temperature[CURRENT]);
    if(hum_label_internal)  lv_label_set_text_fmt(hum_label_internal, "%.1f%%", data->humidity[CURRENT]);
    free(data);  // cleanup allocated memory
}

void Scroll_text(void) {
    lv_obj_t *label = lv_label_create(temp_hum_scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(label, "Lucci's Sausage Party!");
    /* Size of the screen (if you use rotation 90 or 270, please use lv_display_get_vertical_resolution) */
    lv_obj_set_width(label, lv_display_get_horizontal_resolution(lvgl_disp));
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
}
