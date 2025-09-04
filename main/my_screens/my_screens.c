#include "my_screens/my_screens.h"
#include "../my_images/my_images.h" // Include the header file for image declarations   

void create_temp_hum_screen(void) {
    // Create a new screen
    lv_obj_t *scr = lv_obj_create(NULL);

    // Temperature image widget
    lv_obj_t *img_temp = lv_image_create(scr);
    lv_image_set_src(img_temp, &temperature_icon_32);   // Reference to the declared image
    lv_obj_set_pos(img_temp, 0, 20);                    // Temperature icon at 0 px right, middle height
    // Temperature value label  
    lv_obj_t *temp_label = lv_label_create(scr);        // Temperature label (to the right of icon)
    lv_label_set_text(temp_label, "23.4°C");
    lv_obj_align_to(temp_label, img_temp, LV_ALIGN_OUT_RIGHT_MID, 2, 0);    // ^ aligns label to the right side of icon, with +2px gap

    // Humidity image widget
    lv_obj_t *img_hum = lv_image_create(scr);
    lv_image_set_src(img_hum, &humidity_icon_32);       // Reference to the declared image
    lv_obj_set_pos(img_hum, 70, 20);                    // Humidity icon at 70 px right, middle height
    // Humidity value label  
    lv_obj_t *hum_label = lv_label_create(scr);
    lv_label_set_text(hum_label, "55%");
    lv_obj_align_to(hum_label, img_hum, LV_ALIGN_OUT_RIGHT_MID, 2, 0);

    // Load the screen
    lv_scr_load(scr);
}
