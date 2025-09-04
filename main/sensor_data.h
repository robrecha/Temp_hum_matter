#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float temperature[2];
    float humidity[2];
} sensor_reading_t;

typedef enum {
    SENSOR0 = 0,        // 0
    SENSOR1
} Sensor_numbers_t;


// General
typedef enum {
    CURRENT = 0,        // 0
    PREVIOUS
} change_tracking_t;

/* Declare the global queue handle */
extern QueueHandle_t AHT0_sensor_queue;
extern QueueHandle_t AHT1_sensor_queue;

#ifdef __cplusplus
} // extern "C"
#endif

#endif // SENSOR_DATA_H