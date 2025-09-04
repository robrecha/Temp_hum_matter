#ifndef MY_IMAGES_H
#define MY_IMAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"   

// Declare the image descriptor
LV_IMG_DECLARE(temperature_icon_32);   // 'temperature_icon' must match the variable name in temperature_icon.c
LV_IMG_DECLARE(humidity_icon_32);      // 'humidity_icon' must match the variable name in humidity_icon.c

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // MY_IMAGES_H