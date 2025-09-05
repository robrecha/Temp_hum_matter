#ifndef MY_SCREENS_H
#define MY_SCREENS_H

#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

// Function Declarations
void create_temp_hum_screen(void);
void update_temp_hum(void *param);
void Scroll_text(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // MY_SCREENS_H